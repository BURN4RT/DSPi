# I2S Clock-Slave Input Mode Specification

*Last updated: 2026-07-06*

This document specifies the I2S clock-slave input mode: behavior, wiring, the
complete host-facing control surface (vendor commands, notifications, bulk
parameter and persistence formats), and the integration patterns an app or
LLM needs to support the feature. It complements
`Documentation/Features/i2s_input_spec.md`, which describes the default
clock-master architecture; everything there still applies when the clock mode
is MASTER.

---

## 1. Concept

The DSPi has two I2S clock modes, selected by a single persisted setting:

| Mode | Value | Who drives BCK/LRCLK | Rate authority |
|---|---|---|---|
| MASTER (default) | 0 | DSPi (PIO side-set or I2S output clock master) | Host-selected via `REQ_SET_INPUT_RATE` (0xED) |
| SLAVE | 1 | External device; DSPi's BCK/LRCLK pins become inputs | Auto-detected from the external LRCLK |

In SLAVE mode the DSPi handles audio data only: it clocks input data in and
output data out on clock edges supplied by the external master. This allows
superior external clocking and compatibility with I2S sources that only
operate as clock masters (many ADCs, USB-to-I2S interfaces).

The mode is only meaningful while the input source is I2S
(`REQ_SET_INPUT_SOURCE` = 2). With any other input source the setting is
dormant: it is stored, reported by GET, and takes effect at the next switch
into I2S. **While a non-I2S source is selected, the DSPi reverts to driving
the clock pins whenever an I2S output slot exists** (see section 8).

Everything below is identical on RP2040 and RP2350 except where noted (ADAT
is RP2350-only).

### Clock domains in slave mode

| Output | Clocking in slave mode |
|---|---|
| I2S output slots | Edge-slaved: a wait-driven PIO program shifts data bits directly on the external BCK/LRCLK edges (bit-exact lock, no servo) |
| SPDIF output slots | Internally clocked, rate-matched to the external clock by a servo (measured-rate loop plus consumer-fill trim) |
| ADAT output (RP2350) | Same servo divider as SPDIF (identical 256 x Fs PIO clock) |
| PDM sub output | Nominal rate, not servoed (identical to SPDIF-input mode behavior) |
| MCK output | Forced off (a locally generated MCK would be asynchronous to the external BCK/LRCLK, which is invalid for downstream converters); re-enabled per its stored config when leaving slave mode |

Inter-slot sample alignment is preserved: synchronized output starts are
gated on an external LRCLK edge and all output slots (both types) start in a
single `pio_enable_sm_mask_in_sync` write, with the edge-slaved I2S program
discarding exactly one frame so I2S and SPDIF slots emit the same sample
index in the same frame window.

---

## 2. Hardware and wiring

### Pins

The clock pins are the same configurable pair used in master mode:

- BCK = `i2s_bck_pin` (`REQ_SET/GET_I2S_BCK_PIN`, 0xC2/0xC3; default GPIO 14)
- LRCLK = BCK + 1 (hardware constraint, default GPIO 15)

In slave mode both are inputs. Input data pins (`REQ_SET/GET_I2S_RX_PIN`,
0xF1/0xF2) and I2S output slot data pins (0x7C/0x7D) are unchanged.

### External master requirements

- **BCK must be 64 x Fs** (32 BCK per channel, 24-in-32 standard I2S with the
  1-bit delay). Masters running 32 x Fs or 48 x Fs frames produce garbage;
  there is no runtime format detection.
- **Standard LRCLK polarity** (low = left). Inverted-WS masters silently swap
  channels.
- Supported detected rates: **44100, 48000, 96000 Hz** (snap tolerance 2%).
  Other rates (32k, 88.2k, 176.4k, 192k) never lock; outputs stay muted and
  the status reports ACQUIRING with the raw measured rate.

### Usage example

External USB-to-I2S interface (master-only) feeding two I2S DACs and two
SPDIF DACs:

1. Interface BCK out -> DSPi GPIO 14 (BCK, now input); LRCLK out -> GPIO 15.
2. Interface DATA out -> DSPi I2S RX data pin (default GPIO 1).
3. Two output slots set to I2S; their data pins -> the I2S DACs' data inputs.
   The DACs' BCK/LRCLK inputs connect to the interface's master outputs
   (all devices share the one external clock pair).
4. Two output slots left as SPDIF -> the SPDIF DACs.
5. Enable slave mode, set the input source to I2S, play.

### Dual-driver warning (important)

Whenever the DSPi is NOT in slave-clocked I2S operation (input source is USB
or SPDIF, or clock mode is MASTER) and an I2S output slot exists or the I2S
input is active, the DSPi drives BCK/LRCLK itself. If the external master is
still wired and driving, both devices drive the same lines (bounded by pad
current limits, but not healthy). Apps should warn users to avoid selecting
other input sources or master mode while an external master is hard-wired,
or wire the clock lines through small series resistors. A short muted
transient of this kind also occurs at boot before a slave-mode startup
preset fully applies, and during live mode/preset transitions; the firmware
minimizes but cannot eliminate it while the user has both devices wired.

---

## 3. Runtime behavior

### Lock state machine

Slave mode adds a SPDIF-style lock state machine, exposed via
`REQ_GET_I2S_SLAVE_STATUS` (0x8A) and NOTIFY event 0x09:

| State | Value | Meaning |
|---|---|---|
| INACTIVE | 0 | Not in slave role (mode is MASTER, input is not I2S, or the input is stopped, e.g. across a rate change or flash write) |
| ACQUIRING | 1 | Measuring the external clocks; no lock yet; outputs muted |
| RELOCKING | 2 | Clocks lost or rate changed; outputs muted, waiting |
| LOCKED | 3 | Locked to a supported rate; audio flowing (after prefill) |

Mechanics:

- The rate is measured from the RX DMA word rate (2 words per frame of
  external LRCLK) over ~32 ms windows. Two consecutive agreeing windows are
  required to lock, so lock takes ~64-100 ms after valid clocks appear.
- Clock loss is declared after 5 ms without data words. Behavior is **mute
  and wait**: outputs are muted and drained; nothing plays until valid
  clocks return, then the device re-locks, re-prefills, and resumes.
  Downstream SPDIF/ADAT receivers lose lock during the wait.
- An external rate change (e.g. the source switching 48 -> 96 kHz) is
  detected as a lock drop; the device re-acquires, retunes the whole
  pipeline to the new rate, and resumes automatically. Expect roughly
  100-300 ms of mute for the full re-lock plus prefill.
- A long-window (8-16 s) measurement refines the servo rate reference to
  ~0.1 ppm, which keeps ADAT rate-locked even with no SPDIF slot to observe.

### Rate reporting

- `REQ_GET_INPUT_RATE` (0xEE) still returns `{current pipeline Hz, selected
  I2S Hz}`. In slave mode the pipeline Hz follows the detected rate once
  locked; the "selected" value is the stored master-mode preference.
- `REQ_SET_INPUT_RATE` (0xED) in slave mode stores the value (and notifies)
  but does NOT change the live rate; the stored rate applies when the device
  returns to master mode.
- The status packet (0x8A) carries both the snapped locked rate and the raw
  measured rate.

### Output type switching

Output slots remain freely switchable between SPDIF and I2S (0xC0) while
slave mode is active. The type switch rebuilds the affected slots with the
correct clocking (edge-slaved program for I2S) and re-runs the muted
prefill/synchronized start, exactly like a type switch in any other mode.

---

## 4. Vendor commands

All three new commands work over USB vendor control transfers and the
UART/I2C control interfaces (standard dispatcher framing).

### REQ_SET_I2S_CLOCK_MODE (0x88), OUT

Payload: 1 byte, 0 = master, 1 = slave. Values > 1 are ignored.

Deferred apply: the main loop performs the transition (input restart,
output-slot clocking rebuild, MCK policy). If the input source is not I2S
the value is recorded and applies at the next switch into I2S. Setting the
current mode again is a no-op. A `PARAM_CHANGED` notification for the
`input_config.i2s_clock_mode` wire field is emitted when the change is
actually applied, not at SET time.

### REQ_GET_I2S_CLOCK_MODE (0x89), IN

Returns 1 byte: the LIVE mode. A SET that has not yet been applied by the
main loop is not reflected (window is a few main-loop iterations; track the
apply via the PARAM_CHANGED notification if exact sequencing matters).

### REQ_GET_I2S_SLAVE_STATUS (0x8A), IN

Returns the 16-byte `I2sSlaveStatusPacket` (little-endian):

| Offset | Size | Field | Meaning |
|---|---|---|---|
| 0 | 1 | state | I2sSlaveState (0-3, table above) |
| 1 | 1 | clock_mode | Live mode (0/1); lets one call answer both questions |
| 2 | 1 | lock_count | Locks since boot (saturates at 255) |
| 3 | 1 | loss_count | Losses since boot (saturates at 255) |
| 4 | 4 | detected_rate | Snapped Hz (44100/48000/96000); 0 unless LOCKED |
| 8 | 4 | measured_hz | Raw measured external rate, rounded Hz; 0 when no clocks |
| 12 | 4 | reserved | Zero |

Poll this for diagnostics UI (e.g. 2-10 Hz while an I2S settings page is
open). For state transitions prefer NOTIFY event 0x09.

### Interactions with existing commands

| Command | Slave-mode behavior |
|---|---|
| 0xE0 SET_INPUT_SOURCE | Unchanged; slave clocking engages/disengages when I2S becomes/stops being the source |
| 0xED SET_INPUT_RATE | Stored + notified only; no live effect until master mode |
| 0xC2 SET_I2S_BCK_PIN | Unchanged rules (rejected while any slot is I2S); a change restarts the input on the new pins |
| 0xC4 SET_MCK_ENABLE | Stored; MCK output remains forced off while slave mode is live and resumes per config on exit |
| 0xF1/0xF3 RX data pins / channel count | Unchanged (multichannel 4/6/8-ch input works in slave mode on RP2350; all pairs sample against the external clocks) |

---

## 5. Notifications

### NOTIFY_EVT_I2S_SLAVE_STATE (0x09)

Pushed on every slave lock-state transition. v2 packet, 9 bytes:

| Byte | Value |
|---|---|
| 0 | 0x02 (v2 version) |
| 1 | 0x09 (event id) |
| 2 | flags (0) |
| 3 | sequence number |
| 4 | state (I2sSlaveState) |
| 5-8 | detected rate, Hz, little-endian (0 unless the state is LOCKED) |

Note this packet is 9 bytes where most discrete v2 events are 8; parsers
must use per-event lengths (as with `PARAM_CHANGED`).

Expected sequences an app will observe:

- Enable + lock: `ACQUIRING -> LOCKED(rate)`
- Clock loss and recovery: `RELOCKING -> INACTIVE -> ACQUIRING -> LOCKED`
  (the INACTIVE blip is the internal receiver restart that re-frames the
  state machines; treat RELOCKING/INACTIVE/ACQUIRING uniformly as "waiting
  for clocks")
- External rate change: same as clock loss, then `LOCKED(new rate)`; a
  pipeline rate change also fires the usual recompute-driven notifications
- Leaving slave mode or switching source: `INACTIVE`

### PARAM_CHANGED

The mode itself is mirrored into the bulk-params shadow, so applying it
emits a standard `PARAM_CHANGED` (event 0x02) carrying the wire offset of
`input_config.i2s_clock_mode` with a 1-byte value. Rate changes ripple the
usual `INPUT_FORMAT` (0x05) and related events exactly as SPDIF rate changes
do.

---

## 6. Persistence and bulk parameters

### Wire format (REQ_GET/SET_ALL_PARAMS, 0xA0/0xA1)

`WIRE_FORMAT_VERSION` = 18. `WireInputConfig` (16-byte section, unchanged
size and packet total of 5872 bytes):

| Byte | Field |
|---|---|
| 0 | input_source |
| 1 | spdif_rx_pin |
| 2 | i2s_rx_pin (pair 0) |
| 3 | i2s_input_rate (encoded: 0=44100, 1=48000, 2=96000) |
| 4 | i2s_input_channels (2/4/6/8; 0 = absent) |
| 5-7 | i2s_rx_pin_ext[3] (pairs 1-3) |
| 8-9 | spdif_rx_pin_ext[2] (SPDIF inputs 2/3) |
| 10 | spdif_rx_enabled_ext_p1 (SPDIF 2/3 enable mask + 1) |
| 11 | **i2s_clock_mode (0=master, 1=slave; new in V21)** |
| 12-15 | reserved |

Apply-side semantics match the vendor SET: validated (<= 1), deferred to the
main loop, no-op when unchanged. Pre-V21 payloads are rejected wholesale by
the existing strict version check, as with every wire bump.

### Preset slots

`SLOT_DATA_VERSION` = 28 appends one byte, `i2s_clock_mode`, to the slot
tail. Pre-V28 slots load with the mode defaulting to master. The mode is
saved by `REQ_SAVE_PRESET` like the rest of the IO config and restored on
preset load / boot, honoring `output_config_mode`:

- WITH_PRESET: the slot value (device-global directory value as fallback).
- INDEPENDENT: the device-global directory value (saved via
  `REQ_SAVE_OUTPUT_CONFIG`); directory format V13 grew the device-global
  `FlashOutputConfig` by this byte with an automatic V12 -> V13 migration.

A preset load that changes the mode while I2S is live applies it through the
same deferred main-loop transition (one extra muted reset after the load).

---

## 7. App integration patterns

### Enabling slave mode

1. `SET_I2S_CLOCK_MODE(1)` (0x88).
2. If not already: `SET_INPUT_SOURCE(2)` (0xE0).
3. Watch NOTIFY 0x09 (or poll 0x8A) for LOCKED; display the detected rate.
4. Audio starts automatically after lock + prefill. No further host action.
5. Persist with `REQ_SAVE_PRESET` as usual.

### Status UI guidance

- Map ACQUIRING/RELOCKING/INACTIVE (while mode=slave and source=I2S) to a
  single "waiting for external clock" indication with the raw `measured_hz`
  as a diagnostic (0 means no clocks at all; a nonzero value that never
  locks means an unsupported rate or wrong BCK ratio).
- In slave mode, gray out the sample-rate selector (0xED) or relabel it
  "rate used in master mode"; show the detected rate as read-only.
- Hide/disable MCK controls while slave mode is live (values are kept but
  the output is forced off).
- The lock/loss counters are cumulative since boot; deltas indicate cable or
  source stability problems.

### Sequencing detail

GET 0x89 reflects the mode only after the deferred apply; if the app needs a
positive confirmation, wait for the PARAM_CHANGED on
`input_config.i2s_clock_mode` (or the first 0x09 event) after the SET.

---

## 8. Edge cases and known limitations

- **Non-I2S sources re-drive the clock pins.** By design the slave clocking
  applies only while I2S is the input source (there is no way to keep other
  sources' outputs on the external clock domain without resampling). See the
  dual-driver warning in section 2.
- **Supported rates are 44.1/48/96 kHz** (the pipeline's global set). An
  88.2/176.4/192 kHz master never locks; outputs stay muted.
- **BCK must be 64 x Fs and LRCLK polarity standard.** No runtime detection
  of other framings.
- **Extreme DMA-interrupt outages can slip edge-slaved I2S output framing.**
  The edge-slaved TX program free-runs between synchronized starts; a DMA
  service outage longer than the joined TX FIFO (roughly 80 microseconds at
  48 kHz) would slip its framing against the external clock until the next
  synchronized restart (any prefill/reset re-frames it). This mirrors the
  pre-existing exposure of the internal data-only I2S slave program and has
  the same practical headroom; flash operations already suspend and restart
  the pipeline around their blackout.
- **Mute-and-wait on clock loss** means downstream SPDIF/ADAT DACs lose lock
  while external clocks are absent (the deliberate design choice; there is
  no internal-clock fallback).
- **Servo residual on exit.** Returning to master mode or another source
  restores nominal clocking through the standard rate-change path; the
  ppm-level servo trim on SPDIF dividers normalizes exactly as it does when
  leaving SPDIF input mode.
- **RP2040 vs RP2350**: identical behavior; RP2040 is limited to the single
  stereo input pair and has no ADAT, as in master mode.

---

## 9. Implementation map (for firmware developers)

| Piece | Location |
|---|---|
| Mode globals, pending flags, `i2s_slave_mode_active()` | `firmware/DSPi/audio_input.h/.c` |
| RX external role, rate measurement, lock FSM, servo, status | `firmware/DSPi/i2s_input.c` (`i2s_slave_*`), `i2s_input.h` |
| Edge-slaved TX PIO program | `firmware/pico-extras/src/rp2_common/pico_audio_i2s_multi/audio_i2s_dataout_extclk.pio` |
| Library external-clock plumbing, program patch/reload, two-phase `enable_sync_prepare` | `audio_i2s_multi.c/.h` (and `audio_spdif.c/.h` for the SPDIF prepare half) |
| LRCLK-gated combined output start, main-loop slave block, deferred mode-change handler, type-switch clocking rebuild | `firmware/DSPi/main.c` |
| ADAT servo handoff | `firmware/DSPi/adat_output.c` (`adat_output_servo_divider`, resync divider pull) |
| Vendor handlers 0x88-0x8A, 0xED gate | `firmware/DSPi/vendor_commands.c`, `config.h` |
| NOTIFY event 0x09 (push + formatter) | `firmware/DSPi/notify.c/.h` |
| Persistence (slot V28, directory V13, wire V21) | `firmware/DSPi/flash_storage.c`, `bulk_params.h/.c` |
