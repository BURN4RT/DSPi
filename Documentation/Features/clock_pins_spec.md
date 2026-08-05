# I2S Clock-Pin Mode Specification (Unified / Split BCK+LRCLK Pairs)

*Last updated: 2026-07-10 (initial version)*

This document specifies the I2S clock-pin mode feature: what it does, the
complete host-facing control surface (vendor commands, validation rules,
notifications, bulk-parameter and persistence formats), and the integration
patterns an app developer or LLM needs to support it. It complements
`Documentation/Features/i2s_slave_input_spec.md` (the I2S clock-slave input
mode this feature extends) and `Documentation/Features/i2s_output_spec.md`.

---

## 1. Concept

The DSPi has two I2S *clock modes* (`REQ_SET_I2S_CLOCK_MODE`, 0x88):

- MASTER (0, default): the device drives BCK/LRCLK as outputs.
- SLAVE (1): an external master drives BCK/LRCLK; the device's pins are inputs.

Historically both clock modes used one GPIO pair. The *clock-pin mode*
selects whether that stays true:

| Pin mode | Value | Master clock mode uses | Slave clock mode uses |
|---|---|---|---|
| UNIFIED (default, legacy) | 0 | `i2s_bck_pin` (+1 for LRCLK) | the same pair |
| SPLIT | 1 | `i2s_bck_pin` (+1) | `i2s_bck_pin_slave` (+1) |

UNIFIED is convenient when one connector serves both roles. SPLIT exists for
boards that route the two roles to different physical connectors, e.g. a test
PCB that must exercise master-mode outputs and an external-master input
without rewiring; it also removes the dual-driver contention window described
in `i2s_slave_input_spec.md` section 2 (with SPLIT, an always-wired external
master never shares a pad with the device-driven master pair).

Key invariants:

- **LRCLK is always BCK + 1** in both pairs (PIO side-set hardware
  constraint). Each pair is stored/configured as a single BCK GPIO.
- The slave pair is **fully dormant in UNIFIED mode**: it is stored and
  reported, but nothing ever drives, reads, or reserves it.
- In SPLIT mode **both pairs are reserved**: validation refuses to assign RX
  data pins, control-interface pins, or the DAC-mute pin onto either pair,
  and the two pairs must not overlap each other.
- The firmware resolves the pair the hardware actually uses through one
  helper (`i2s_effective_bck_pin()`): the slave pair if and only if
  `pin mode == SPLIT && clock mode == SLAVE`, else the master pair.

Identical on RP2040 and RP2350 except the defaults below.

## 2. Default pins

| | BCK | LRCLK |
|---|---|---|
| Master/unified pair (both platforms) | GPIO 14 | GPIO 15 |
| Slave pair, RP2040 | GPIO 12 | GPIO 13 |
| Slave pair, RP2350 | GPIO 26 | GPIO 27 |

The slave defaults are the nearest sequential GPIO pair to 14/15 that no
other feature defaults to on that platform. On RP2040, 12/13 are free (ADAT
is RP2350-only and the RP2040 MCK default is GPIO 21). On RP2350 everything
adjacent to 14/15 is a default for some feature (13 MCK, 12 ADAT, 11 DAC
mute, 10 PDM, 9-6 SPDIF out, 5 SPDIF RX, 4-1 I2S RX data, 16/17 UART, 18/19
I2C, 20/21 optional SPDIF inputs 2/3), so 26/27 is the closest clean pair.

## 3. Vendor commands

All commands work over USB vendor control transfers and the UART/I2C control
interfaces (standard dispatcher framing). The pin-setting commands live on
the GET-style path (wValue-carried, 1-byte status/data response), like the
other pin commands.

### REQ_SET_I2S_CLOCK_PIN_MODE (0xFE), IN, returns 1 status byte

`wValue` = 0 (UNIFIED) or 1 (SPLIT). Response: one `PIN_CONFIG_*` byte.

| Status | Value | When |
|---|---|---|
| PIN_CONFIG_SUCCESS | 0x00 | Applied (or no-op: already in this mode) |
| PIN_CONFIG_PIN_IN_USE | 0x02 | Entering SPLIT while the slave pair overlaps the master pair or either slave GPIO is owned by another subsystem (RX data pin, fixed peripheral, control interface, DAC mute, Control Surfaces binding) |
| PIN_CONFIG_OUTPUT_ACTIVE | 0x04 | The change would move the live effective pair (clock mode is, or is pending, SLAVE) while at least one output slot is I2S |
| PIN_CONFIG_INVALID_PARAM | 0x05 | wValue > 1 |

Semantics:

- Entering SPLIT is validated **even when dormant** (clock mode MASTER):
  the slave pair must be conflict-free at enable time, because a later
  clock-mode flip (0x88) adopts it without rechecking. If the enable is
  rejected with PIN_IN_USE, move the slave pair first (0xC2 role 1) or free
  the conflicting assignment, then retry.
- Leaving SPLIT needs no pin checks (the master pair is reserved in every
  mode, so nothing else can occupy it).
- If the change moves the live effective pair with only the I2S *input*
  running (no I2S output slots), it applies immediately and the input
  restarts on the new pair via the usual deferred muted restart.
- If clock mode is MASTER the set is a pure stored-state change; nothing
  live moves (master clocking uses the master pair in both pin modes).
- With I2S output slots active and slave clocking in force the set is
  rejected (OUTPUT_ACTIVE). Workaround sequence: switch the I2S slots to
  SPDIF (0xC0), change the pin mode, switch back.

### REQ_GET_I2S_CLOCK_PIN_MODE (0xFF), IN, returns 1 byte

Returns the live pin mode (0/1). Applied synchronously by 0xFE, so unlike
0x89 there is no deferred-apply reporting window.

### REQ_SET_I2S_BCK_PIN (0xC2), IN, returns 1 status byte; extended

`wValue = (role << 8) | GPIO`:

- role 0 = master/unified pair. **Legacy hosts that send a bare GPIO in
  wValue get role 0 implicitly and see unchanged behavior.**
- role 1 = slave pair. Storable at any time (including while UNIFIED, where
  it is dormant); useful to stage the pair before enabling SPLIT.

| Status | When |
|---|---|
| PIN_CONFIG_SUCCESS | Applied or no-op (pin already set to that value) |
| PIN_CONFIG_INVALID_PIN | GPIO or GPIO+1 invalid (out of range, or 23-25 reserved) |
| PIN_CONFIG_INVALID_OUTPUT | role > 1 |
| PIN_CONFIG_OUTPUT_ACTIVE | The pair being moved is the one live I2S output slots clock against: role 0 while any slot is I2S (except in SPLIT + slave clocking, where the master pair is dormant and the set is allowed); role 1 while any slot is I2S in SPLIT + slave clocking |
| PIN_CONFIG_PIN_IN_USE | GPIO or GPIO+1 owned by another subsystem, or the pair would overlap the other pair (role 1 always checks against the master pair; role 0 checks against the slave pair only in SPLIT mode, so the dormant slave default never constrains legacy UNIFIED-mode hosts) |

If the input is running on the pair being changed, the set arms a deferred
muted input restart on the new pins (identical to legacy 0xC2 behavior).

### REQ_GET_I2S_BCK_PIN (0xC3), IN, returns 1 byte; extended

`wValue` = role (0 = master/unified pair, 1 = slave pair). Returns that
pair's BCK GPIO; LRCLK is that value + 1. Invalid roles return 0. Legacy
hosts sending wValue = 0 read the master pin exactly as before.

### Interactions with existing commands

| Command | Behavior with clock-pin modes |
|---|---|
| 0x88 SET_I2S_CLOCK_MODE | Unchanged wire semantics. In SPLIT mode a master/slave flip also swaps the effective pair; the deferred main-loop transition rebuilds input and output clocking on the correct pair automatically. |
| 0xC0 SET_OUTPUT_TYPE | Unchanged. I2S slots always clock on the effective pair at (re)build time. |
| 0xF1/0xF3 RX data pins / channel count | Unchanged wire semantics. Validation rejects data pins on either clock pair while SPLIT (both pairs claimed), only the master pair while UNIFIED. |
| 0xF5/0xF7 UART/I2C config, DAC mute (0xEA), Control Surfaces bindings | Same rule: their pin validation treats both pairs as claimed in SPLIT mode. |
| 0xC4-0xC9 MCK | Unaffected; MCK is independent of the clock pairs (and forced off in slave clocking as before). |

## 4. Notifications

Both settings are mirrored into the bulk-params shadow, so applying them
emits standard `PARAM_CHANGED` (event 0x02) notifications carrying the wire
offset and 1-byte value:

- Pin mode: offset of `i2s_config.clock_pin_mode_p1`, value is the **+1
  encoding** (1 = unified, 2 = split), matching what 0xA0 returns.
- Slave BCK: offset of `i2s_config.bck_pin_slave`, plain GPIO value.
- Master BCK (role 0): offset of `i2s_config.bck_pin`, unchanged from legacy.

Within the `WireI2SConfig` section the byte positions are: `output_types[4]`
at 0-3, `bck_pin` 4, `mck_pin` 5, `mck_enabled` 6, `mck_multiplier` 7,
`clock_pin_mode_p1` 8, `bck_pin_slave` 9, reserved 10-15. Hosts using the
SDK's `WireBulkParams` struct get the absolute offsets from `offsetof`.

## 5. Bulk parameters (REQ_GET/SET_ALL_PARAMS, 0xA0/0xA1)

`WIRE_FORMAT_VERSION` is **unchanged** (18): the two fields claim former
reserved bytes of the 16-byte `WireI2SConfig` section with absent-value
sentinels, so payloads from older hosts (zeros there) apply cleanly.

| Field | Encoding | SET semantics |
|---|---|---|
| `clock_pin_mode_p1` | 0 = absent (keep live), 1 = unified, 2 = split | Installed live when valid; decoded value must be <= 1 |
| `bck_pin_slave` | 0 = absent (keep live), else BCK GPIO | Installed only if the GPIO pair is valid, collision-free against fixed peripherals, and non-overlapping with the master pair installed by the same payload; otherwise the live pin is kept (same keep-live-on-reject model as `bck_pin`) |

The RX data-pin set inside the same payload is validated against the clock
pairs that will actually be installed (including these two fields), not the
pre-apply live values. If the effective input pair moves, the apply arms the
usual deferred muted input restart.

**Feature detection:** read 0xA0 and check `clock_pin_mode_p1 != 0`.
Firmware without this feature reports 0 there (it was a reserved byte).
Alternatively probe GET 0xFF; older firmware errors/stalls on the unknown
command.

## 6. Persistence

Both settings are part of the physical IO block and follow the existing
**output-config mode** selection (`REQ_SET/GET_OUTPUT_CONFIG_MODE`,
0x98/0x99):

- WITH_PRESET (default): saved into the active preset by `REQ_SAVE_PRESET`
  and restored on preset load, with the device-global directory value as the
  baseline for presets that predate the fields.
- INDEPENDENT: the live values survive preset loads; snapshot them
  device-globally with `REQ_SAVE_OUTPUT_CONFIG` (0x52); restored at boot.

Formats:

- **Preset slot V29** (`SLOT_DATA_VERSION` 29) tail-appends two bytes:
  `i2s_clock_pin_mode` (plain 0/1; 0 = unified) and `i2s_bck_pin_slave`
  (0 = unset, falls back to the device baseline then the platform default).
  Pre-V29 slots load with unified + default, byte-identical to their old
  behavior.
- **Directory V14** grows the device-global `FlashOutputConfig` from 29 to
  31 bytes with the same two fields; a V13 to V14 migration (and the whole
  older chain) fills them with 0 = unified/unset automatically.
- A preset/bulk restore that changes these fields while I2S is live applies
  them through the same deferred muted transitions as every other IO field;
  inter-slot output alignment is preserved by the synchronized restart paths.

Stored values are validated on apply exactly like the master BCK pin
(invalid or conflicting stored pins are rejected and the live/known-good
value is kept), so a corrupted or cross-platform image cannot install a
broken clock pair.

## 7. App integration patterns

### Enabling SPLIT mode

1. Optionally read 0xC3 role 1 to show the current slave pair; move it first
   with 0xC2 role 1 if the default conflicts with your wiring.
2. `SET_I2S_CLOCK_PIN_MODE(1)` (0xFE). Handle PIN_IN_USE by prompting the
   user to free the conflicting pin or choose another pair; handle
   OUTPUT_ACTIVE by offering to temporarily switch I2S output slots to SPDIF.
3. Persist: `REQ_SAVE_PRESET` (WITH_PRESET) or `REQ_SAVE_OUTPUT_CONFIG`
   (INDEPENDENT), as for any IO change.

### UI guidance

- Present the mode as a radio choice ("Shared clock pins" / "Separate pins
  per clock mode") with the slave-pair picker enabled only in SPLIT mode.
- Always display LRCLK as read-only BCK+1 (both pickers).
- When SPLIT is active, annotate both pairs as reserved in any pin-map UI;
  when UNIFIED, show the slave pair as stored-but-inactive.
- After a SET, update state from the PARAM_CHANGED notifications (or re-GET
  0xFF / 0xC3) rather than assuming success; every SET can be rejected with
  a status byte.

### Sequencing detail

0xFE and 0xC2 apply their stored value synchronously (the status byte is
authoritative); only the hardware restart is deferred by a few main-loop
iterations. This differs from 0x88 (clock mode), whose GET reflects the
change only after the deferred apply.

## 8. Edge cases and known limitations

- **Both pairs must be sequential GPIO pairs** (LRCLK = BCK + 1). There is
  no independent LRCLK assignment in either mode.
- **The pairs may not overlap** (including LRCLK adjacency: bases exactly 1
  apart collide). Enforced when setting the slave pin (always), the master
  pin (in SPLIT mode), enabling SPLIT, and on every stored-config apply.
- **In UNIFIED mode the dormant slave pin constrains nothing.** A legacy
  host can freely place the master pair or other functions on GPIOs 12/13
  (RP2040) / 26/27 (RP2350); the conflict, if any, is caught when SPLIT is
  enabled.
- **GPIO 26/27 are ADC-capable pins** (RP2350 default slave pair). Boards
  using them for analog must move the slave pair.
- **Rejection while outputs run is by design** (same rule as the legacy BCK
  pin): moving a pair that live I2S output slots clock against would break
  inter-slot sample alignment, which is inviolable. Use the
  SPDIF-switch-change-switch-back sequence.
- **The 0x88 clock-mode flip validates nothing new**: it adopts whatever
  pair is effective. That is safe because every path that could make the
  slave pair effective (0xFE enable, 0xC2 role 1, stored-config apply)
  validated it first.

## 9. Implementation map (for firmware developers)

| Piece | Location |
|---|---|
| Mode/pin globals (`i2s_clock_pin_mode`, `i2s_bck_pin_slave`) | `firmware/DSPi/usb_audio.c` (defs), `audio_input.h` (externs, enum) |
| `i2s_effective_bck_pin()`, `i2s_clock_pin_claimed()` | `firmware/DSPi/audio_input.h` (inline) |
| Defaults (`PICO_I2S_BCK_PIN_SLAVE`) and command IDs (0xFE/0xFF) | `firmware/DSPi/config.h` |
| Vendor handlers (0xC2/0xC3 role indexing, 0xFE/0xFF) | `firmware/DSPi/vendor_commands.c` |
| Effective-pair consumers: TX `clock_pin_base`, rebuild change detection | `firmware/DSPi/main.c` (`process_type_switches`) |
| Effective-pair consumers: RX start snapshot, active-pin query | `firmware/DSPi/i2s_input.c` |
| Pin-claim validation (both pairs in SPLIT) | `vendor_commands.c` (`is_pin_in_use`, `check_i2s_rx_pin`, `ctrl_iface_check_pin`, `i2s_rx_pin_set_acceptable`), `dac_hw_mute.c` |
| Persistence (slot V29, directory V14, frozen v13 structs, migration) | `firmware/DSPi/flash_storage.c` |
| Wire fields + collect/apply | `firmware/DSPi/bulk_params.h` / `bulk_params.c` |
