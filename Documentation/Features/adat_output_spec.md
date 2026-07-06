# ADAT Bulk Output

*Last updated: 2026-07-06*

## Overview

The ADAT bulk output streams all 8 main output channels (post-matrix, post
per-output EQ/crossover, post gain/volume/mute, post delay) as a single ADAT
lightpipe signal on one GPIO. It is intended as a "bulk" monitoring/expansion
tap of the finalized output channels, running concurrently with the four
SPDIF/I2S output slots and PDM; enabling it changes nothing about the
existing outputs.

| Property | Value |
|---|---|
| Platform | RP2350 only (RP2040 lacks 8 output channels) |
| Channels | 8 (output channels 1..8, the SPDIF slot channels) |
| Sample rates | 44.1 kHz and 48 kHz only |
| Bit depth | 24-bit (same clamp + scale as the SPDIF outputs) |
| Rates above 48 kHz | Stream auto-suspends; auto-resumes when the rate returns |
| ADAT user bits | Always 0 (no timecode/MIDI/S-MUX signalling) |
| S/MUX | Not supported (future work) |
| Default GPIO | 12 (`PICO_ADAT_PIN`) |
| Signal | 3.3 V logic NRZI at 256 x Fs; drive an optical TOSLINK transmitter rated for ~12.3 Mbit/s |

## Signal path and alignment

The tap point is after every per-output processing stage and after the same
gain composite the SPDIF/I2S slots receive (host volume x master volume x
preset mute x per-output matrix gain, including mute and delay). What goes to
ADAT channel N is bit-identical in level to what goes to slot output N.

Alignment guarantees:

- ADAT cannot drift against the output slots. Its PIO clock is 256 x Fs, the
  same as SPDIF TX, and its clock divider is the identical value: nominal in
  USB/I2S modes, and in SPDIF-input mode the clock servo (which trims the
  output dividers to track the source) writes ADAT the same divider it writes
  the SPDIF slots.
- ADAT runs a constant 96-sample lead cushion (2 ms at 48 kHz) relative to the
  slot outputs. The offset is fixed and is re-established at every synchronized
  output restart (rate change, preset load, output type/pin change, input
  source switch).
- During host underruns, ADAT inserts silence slaved one-for-one to the slot-0
  DMA starvation counter, so the ADAT-to-slot offset survives underruns
  exactly while a USB stream is open.
- Corner case: if the host closes its USB stream while a non-USB input
  (SPDIF/I2S) keeps playing, a transient output underrun in that state can
  shift ADAT by up to one 48-sample chunk relative to the slots until the next
  pipeline restart re-canonicalizes the offset.

## Vendor commands

Commands follow the standard vendor EP0 conventions (also reachable over the
UART and I2C control transports through the shared dispatcher). All SET
commands carry their payload in `wValue` and return a single status byte from
the shared `PIN_CONFIG_*` set (config.h): `0x00` SUCCESS, `0x01` INVALID_PIN,
`0x02` PIN_IN_USE, `0x03` INVALID_OUTPUT, `0x05` INVALID_PARAM.

| Command | ID | Direction | wValue | Response |
|---|---|---|---|---|
| `REQ_SET_ADAT_ENABLE` | 0xCA | SET | 0 = disable, 1 = enable (low byte) | 1 status byte |
| `REQ_GET_ADAT_ENABLE` | 0xCB | GET | - | 1 byte: configured enable (0/1) |
| `REQ_SET_ADAT_PIN` | 0xCC | SET | GPIO number (low byte) | 1 status byte |
| `REQ_GET_ADAT_PIN` | 0xCD | GET | - | 1 byte: configured GPIO |
| `REQ_GET_ADAT_STATUS` | 0xCE | GET | - | 8-byte `AdatStatus` |

Behavioral notes:

- `REQ_SET_ADAT_ENABLE` with the current value is a no-op success. Enabling
  validates the configured pin first (INVALID_PIN / PIN_IN_USE on conflict).
  Disabling always succeeds.
- `REQ_SET_ADAT_PIN` validates the GPIO (23..25 and out-of-range rejected,
  conflicts with any owned pin rejected) and may be issued while enabled; the
  stream is re-routed under a muted restart. `wValue = 0` resets to the
  platform default pin (matching the flash/bulk "0 = default" convention).
- Config changes apply from the main loop inside the same muted
  pipeline-restart bracket used by output type/pin switches; expect the brief
  global mute-and-resync that any output reconfiguration produces.
- On RP2040 both SETs return `0x03` (INVALID_OUTPUT); the GETs return zeros.
- While ADAT is enabled its GPIO is owned: every other pin-assignment command
  rejects that pin with PIN_IN_USE, and vice versa.

### `AdatStatus` (REQ_GET_ADAT_STATUS, 8 bytes, packed, little-endian)

| Offset | Type | Field | Meaning |
|---|---|---|---|
| 0 | u8 | `enabled` | Configured enable (persisted intent) |
| 1 | u8 | `active` | Stream currently running |
| 2 | u8 | `pin` | Configured data GPIO |
| 3 | u8 | `rate_ok` | Current sample rate is 44.1/48 kHz |
| 4 | u16 | `resync_count` | Stream restarts since boot (every pipeline reset increments this; a growing number during steady playback is normal only if resets occur) |
| 6 | u16 | `slip_count` | Emergency local resyncs since boot; should stay 0, nonzero indicates a stalled main loop or DMA fault |

`enabled=1, active=0, rate_ok=0` is the auto-suspended state (rate above
48 kHz); the stream resumes automatically, no host action needed.

## Notify event

`NOTIFY_EVT_ADAT_STATE` (0x08), pushed on every stream start/stop including
rate-policy auto-suspend/resume. 8-byte v2 packet:

```
[0] 0x02 (version)   [1] 0x08 (event)   [2] 0 (flags)   [3] seq
[4] enabled          [5] active         [6] pin         [7] 0
```

Apps that track ADAT state should consume this instead of polling 0xCE.
Config changes made via 0xCA/0xCC additionally produce the usual
`PARAM_CHANGED` events for the `adat_config` wire fields.

## Bulk params (REQ_GET/SET_ALL_PARAMS)

`WIRE_FORMAT_VERSION` is now **17** (was 16). Hosts must fetch the new layout;
apply rejects any other version. The new section is the final member of
`WireBulkParams`:

```c
typedef struct __attribute__((packed)) {
    uint8_t enabled;      // 0/1 configured enable
    uint8_t pin;          // data GPIO; 0 = platform default (12)
    uint8_t reserved[6];  // future (user bits, S/MUX); write 0
} WireAdatConfig;         // 8 bytes; sizeof(WireBulkParams) = 5872
```

Collect returns the live config (zeros on RP2040). Apply clamps `enabled`,
maps `pin == 0` to the default, rejects invalid/conflicting pins by keeping
the live pin, and defers the hardware apply exactly like the vendor commands.

## Persistence

ADAT config persists identically to the other physical IO config (output
types, I2S BCK/MCK, SPDIF RX pin) and honors `output_config_mode`
(REQ 0x98/0x99):

- **WITH_PRESET (1, default):** `adat_enabled`/`adat_pin` are stored in each
  preset slot (`SLOT_DATA_VERSION` 23) and applied on preset load. Slots saved
  before V23 leave the current device-level ADAT config untouched.
- **INDEPENDENT (0):** ADAT config lives in the device-global directory block
  (`PresetDirectory` V8, `FlashOutputConfig`), is applied at boot, and preset
  load / factory reset never touch it. `REQ_SAVE_OUTPUT_CONFIG` (0x52) stores
  the live ADAT config with the rest of the IO block.

`adat_pin == 0` in flash means "unset, use the platform default". Old
firmware versions' presets and directories migrate automatically; a V23
preset loads on old firmware as its version-appropriate prefix (ADAT fields
ignored).

## App integration patterns

Enable flow:

1. (Optional) `REQ_SET_ADAT_PIN` if the hardware uses a non-default GPIO.
2. `REQ_SET_ADAT_ENABLE` wValue=1; check the status byte.
3. Wait for `NOTIFY_EVT_ADAT_STATE` with `active=1` (or poll 0xCE). If
   `rate_ok=0` the device is above 48 kHz; the stream starts when the rate
   drops, with another 0x08 event.
4. Persist: in WITH_PRESET mode save the preset (0x91); in INDEPENDENT mode
   send `REQ_SAVE_OUTPUT_CONFIG` (0x52).

Rate handling: apps do not need to manage the 48 kHz restriction. Selecting
96 kHz suspends the stream (event with `active=0, enabled=1`); returning to
44.1/48 kHz resumes it automatically.

Platform detection: check the platform ID (RP2350 = 1) from the device ident
before exposing ADAT UI, or treat `0xCA` returning INVALID_OUTPUT as "not
available".

Level mapping: ADAT sample N is the same 24-bit value the SPDIF outputs
carry: float clamp to [-1, +1] then scale by 8388607.

## Wire format (for receiver/analyzer implementers)

Each sample period is one 256-bit frame, NRZI encoded (a 1 is a level
transition), bit clock 256 x Fs (12.288 MHz at 48 kHz):

```
[1] [0000000000] [1] [u3 u2 u1 u0]          16-bit sync pad, user bits = 0
then for each channel 0..7 (30 bits each):
[1][b23..b20] [1][b19..b16] [1][b15..b12] [1][b11..b8] [1][b7..b4] [1][b3..b0]
```

Samples are MSB-first, two's complement 24-bit. The 10-zero run is the only
place the stream is transition-free for more than 4 bits, which is what
receivers lock onto. The encoder was verified by exhaustive host-side
decode/NRZI/zero-run analysis (4096-frame corpus including both full-scale
extremes); no hardware ADAT receiver has confirmed interop yet.

## Firmware internals (summary)

- Engine: `firmware/DSPi/adat_output.c`; NRZI is encoded on the CPU (prefix-XOR
  per word with line-level state carried across frames), so the PIO program is
  a single `out pins, 1` at 256 x Fs and the clkdiv is identical to the SPDIF
  TX divider (servo-tracked in SPDIF-input mode). DMA channels 13 (data) + 14
  (control) run a free-running 896-frame (28 KB) ring with an IRQ-less chained
  wrap.
- The DSP pushes stuffed frames from `process_input_block()` after the slot
  buffer gives; the blocking give bounds the ring lead, so overwrite of
  unplayed frames is structurally impossible while the DMA runs.
- Rate policy hooks in `perform_rate_change()`; stream restarts in
  `complete_pipeline_reset()` Phase 6 and `enable_outputs_in_sync()`.
- RP2040 compiles the engine out entirely (zero RAM/flash cost).
