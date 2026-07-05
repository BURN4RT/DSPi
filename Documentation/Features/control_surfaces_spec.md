# Control Surfaces (User-Wired Physical Controls and Indicators)

*Firmware capability format version: 1*
*Config (flash) version: 1*

This document is the complete, self-contained specification for the DSPi
Control Surfaces feature: user-wired push buttons, toggle switches,
potentiometers, quadrature rotary encoders, and indicator LEDs on spare GPIOs,
configured over vendor commands `0x84`-`0x87`. It is written for a host-app
developer (e.g. DSPi Console) or an LLM adding Control Surfaces support to an
app, or extending the firmware feature itself. No DSPi source is required to
implement a host client.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Overview and design rationale

### 1.1 The Verb-Type-Noun-Parameter-GPIO user model

A **binding** attaches one physical component to one firmware parameter through
one operation, on one or two GPIOs. The user thinks in five parts:

| Concept | Meaning | Example |
|---------|---------|---------|
| **Verb** | What the user does (turn, press, flip, slide) | "turn" |
| **Type** | The component wired up (`CsType`) | rotary encoder |
| **Noun** | The firmware parameter driven or shown (`CsNoun`) | master volume |
| **Parameter** | The operation applied (`CsAction`) plus its value/step/range | +1 dB per detent |
| **GPIO** | The pin(s) the component occupies | GPIO 27/28 |

The **Verb is not carried on the wire**; it is derived from the **Type**. A
potentiometer can only "adjust", an encoder can only "step", a switch can only
"follow", and so on. Encoding the verb separately would let a host send an
impossible combination (a pot that "toggles"); deriving it from the type makes
those states unrepresentable. The wire therefore carries Type, Noun, Action,
flags, pins, and numeric operands; the host builds its verb-oriented UI from the
capability tables (section 4).

### 1.2 Why apply goes through the shared vendor dispatcher

Every control action is applied by dispatching **the same vendor command a host
would send**, through `vendor_dispatch_set` / `vendor_dispatch_get` with source
`CTRL_SOURCE_GPIO`. Turning the master-volume encoder one detent calls exactly
the `REQ_SET_MASTER_VOLUME` handler that a USB or UART host would reach. This
buys three things for free:

- **Zero duplication.** No parameter-apply logic is re-implemented in the
  Control Surfaces engine. Validation, clamping, deferred pipeline-safe apply
  (e.g. preset load stops SPDIF RX and fences Core 1), and output-slot alignment
  all come from the existing handler. The engine only resolves the target value.
- **Free host notifications.** The dispatch is tagged `PARAM_SRC_GPIO` (value 5
  in the notify `ParamSource` enum), so any change a knob makes emits the normal
  `PARAM_CHANGED` notification to connected hosts through the existing notify
  protocol (USB EP `0x83`, UART type-`0x40` frames), letting a UI track a
  physical control in real time with no extra path.
- **Transport-neutral safety.** The engine runs from main-loop context and gets
  the same `CTRL_DISPATCH_BUSY` back-pressure a USB control SET in flight would
  cause, so it never races the USB stack.

### 1.3 Why the config is device-global (not per-preset)

The binding table is persisted **device-global** in the preset directory (V7),
not inside any preset. Wiring is a **board-level** fact: which physical control
sits on which GPIO does not change when the listener switches EQ profiles.
Consequently the config:

- **Survives preset changes.** Loading preset 3 does not rewire the knobs.
- **Survives factory reset** (the audio state resets; the wiring does not),
  exactly like `dac_hw_mute` and the UART/I2C control-interface config.
- Is **not** part of `WireBulkParams`; the bulk wire-format version is unchanged
  by this feature.

An all-zero blob (every slot `CS_TYPE_NONE`) is the idle state; a fresh or
factory-reset device needs no special seeding.

---

## 2. Wire reference (byte-by-byte)

All structures are `__attribute__((packed))`, little-endian, no padding. The
same `CsBinding` bytes appear on the wire (the `REQ_SET`/`GET_CS_BINDING`
payload) and in flash.

### 2.1 Fixed-point dB convention

Continuous (dB) fields use **signed 8.8 fixed point**: `1.0 dB = 256`. So
`-20 dB = -5120 (0xEC00)`, `+0 dB = 0`, `-0.5 dB = -128`. Bool and enum fields
are plain integers (0/1 for bool, 0..N-1 for enum).

Pots quantize their output to the **nearest half-dB** before dispatching: the
filtered ADC position maps onto the dB span, is rounded to half-dB units
(`round(dB * 2)`), and only re-dispatches when that half-dB value changes. This
gives smooth, jitter-free knob behavior without flooding the dispatcher.

### 2.2 `CsBinding` (16 bytes)

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `type` | `CsType` (0-5); `0` = slot cleared |
| 1 | 1 | `noun` | `CsNoun` (0-8) |
| 2 | 1 | `action` | `CsAction` (0-8) |
| 3 | 1 | `flags` | `CS_FLAG_*` bitfield (see 6); bits above `0x04` are reserved and rejected with `CS_STATUS_INVALID_VALUE` |
| 4 | 1 | `gpio[0]` | primary GPIO |
| 5 | 1 | `gpio[1]` | second GPIO (encoders); `0xFF` (`CS_GPIO_UNUSED`) otherwise |
| 6 | 2 | `reserved[2]` | write 0 |
| 8 | 2 | `value` (int16) | `CS_ACT_SET` target / `CS_ACT_IND_EQUALS` comparand (8.8 dB for continuous nouns; plain int for bool/enum) |
| 10 | 2 | `step` (int16) | `STEP`/`INC`/`DEC` size; `0` = default (1 dB = 256, or 1 enum step); 8.8 dB for continuous |
| 12 | 2 | `range_min` (int16) | pot span low end, 8.8 dB; both range fields `0` = the noun's full range |
| 14 | 2 | `range_max` (int16) | pot span high end, 8.8 dB |

### 2.3 `CsFlashConfig` (132 bytes)

The device-global persisted blob. Not sent by any single command; it is what
the firmware stores and what `REQ_GET_ALL_PARAMS` does **not** contain.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `version` | `CS_CONFIG_VERSION` (1) |
| 1 | 3 | `reserved[3]` | 0 |
| 4 | 128 | `bindings[8]` | eight 16-byte `CsBinding` records |

### 2.4 `CsTypeDesc` (4 bytes) and `CsCapsHeader` (28 bytes)

`CsTypeDesc`:

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this component can drive |
| 2 | 1 | `pin_count` | GPIOs consumed (1 or 2) |
| 3 | 1 | `pin_class` | `CS_PINCLASS_ANY` (0) or `CS_PINCLASS_ADC` (1) |

`CsCapsHeader` (returned by `REQ_GET_CS_CAPS` with `wValue = 0xFFFF`):

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `caps_version` | capability format version (1) |
| 1 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (8) |
| 2 | 1 | `type_count` | `CS_TYPE_COUNT` (6); the type table has this many entries, indexed by `CsType` |
| 3 | 1 | `noun_count` | `CS_NOUN_COUNT` (9) |
| 4 | 24 | `types[6]` | six `CsTypeDesc`, one per `CsType` including index 0 (`NONE`, all-zero) |

Total `4 + 4*6 = 28` bytes.

### 2.5 `CsNounDesc` (8 bytes)

Returned by `REQ_GET_CS_CAPS` with `wValue = noun index` (0-8).

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `kind` | `CS_KIND_CONTINUOUS` (0), `CS_KIND_BOOL` (1), `CS_KIND_ENUM` (2) |
| 1 | 1 | `enum_count` | valid enum values `0..enum_count-1` (`CS_KIND_ENUM` only) |
| 2 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this noun accepts |
| 4 | 2 | `min_q8` (int16) | continuous range low end, 8.8 dB |
| 6 | 2 | `max_q8` (int16) | continuous range high end, 8.8 dB |

### 2.6 `CsStatusPacket` (12 bytes)

Returned by `REQ_GET_CS_STATUS`.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `last_status` | result of the most recent `REQ_SET_CS_BINDING` (`PIN_CONFIG_*` / `CS_STATUS_*`) |
| 1 | 1 | `last_slot` | slot the last SET targeted |
| 2 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (8) |
| 3 | 1 | `active_mask` | bit N = binding N is live |
| 4 | 8 | `slot_status[8]` | per-slot apply status; `PIN_CONFIG_SUCCESS` (0) when live, else the failure code |

---

## 3. Command reference (`0x84`-`0x87`)

| Command | Code | Dir | wValue | wLength / payload | Response |
|---------|------|-----|--------|-------------------|----------|
| `REQ_SET_CS_BINDING` | `0x84` | OUT | slot (0-7) | 16-byte `CsBinding` | none (deferred; poll `0x87`) |
| `REQ_GET_CS_BINDING` | `0x85` | IN | slot (0-7) | - | 16-byte `CsBinding` (live) |
| `REQ_GET_CS_CAPS` | `0x86` | IN | `0xFFFF` = header+types; else noun index | - | 28-byte `CsCapsHeader` or 8-byte `CsNounDesc` |
| `REQ_GET_CS_STATUS` | `0x87` | IN | - | - | 12-byte `CsStatusPacket` |

### 3.1 Transport note (UART / I2C)

Every one of these commands works identically over USB, UART, and I2C (the
transport-neutral dispatcher). On the external transports, the direction bit
matters: `0x85`, `0x86`, `0x87` are IN (GET-type) frames; `0x84` is an OUT
(SET-type) frame carrying the 16-byte payload. Commands whose only parameter is
`wValue` and that produce a response live on the GET path; the three GETs here
already are GETs, so no special handling is needed.

### 3.2 Deferred-apply model (`REQ_SET_CS_BINDING`)

`REQ_SET_CS_BINDING` does **not** apply synchronously. The USB/transport handler
only validates the slot index and latches the 16-byte binding into a pending
buffer, then returns immediately (no response payload). The main loop then:

1. Runs `control_surfaces_apply_binding` (releases the slot's old pins, validates
   the new binding, claims the new pins, seeds runtime state).
2. Records the result in `last_status` / `last_slot`.
3. **Persists the whole binding table to flash only on `PIN_CONFIG_SUCCESS`**, so
   a rejected SET can never clobber a good stored config.

On accept, the handler immediately records `CS_STATUS_PENDING` (`0x16`) in
`last_status` / `last_slot`; the main-loop apply then overwrites it with the
real result. The host learns the outcome by polling `REQ_GET_CS_STATUS` until
`last_status != CS_STATUS_PENDING` for the slot named in `last_slot`. A short
poll (a few ms) is ample; the apply is a directory sector flash write.

If the slot index in `wValue` is `>= 8`, the handler records
`CS_STATUS_INVALID_SLOT` immediately (no main-loop round trip needed) and no
apply is queued.

### 3.3 Status codes

`0x00`-`0x05` reuse the shared `PIN_CONFIG_*` namespace (config.h); Control
Surfaces extends it from `0x10`.

| Code | Name | Meaning |
|------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | applied (or slot cleared) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | pin out of range, or an encoder's two pins are equal |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | pin already claimed by another peripheral or binding |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | (shared namespace; not produced by CS) |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | (shared namespace; not produced by CS) |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | (shared namespace; not produced by CS) |
| `0x10` | `CS_STATUS_INVALID_SLOT` | slot index >= 8 |
| `0x11` | `CS_STATUS_INVALID_TYPE` | `type` >= `CS_TYPE_COUNT` |
| `0x12` | `CS_STATUS_INVALID_NOUN` | `noun` >= `CS_NOUN_COUNT` |
| `0x13` | `CS_STATUS_INVALID_ACTION` | `action` invalid, or not allowed for this type+noun |
| `0x14` | `CS_STATUS_INVALID_VALUE` | `value` / `step` / `range` out of bounds for the noun, or unknown `flags` bits set |
| `0x15` | `CS_STATUS_PIN_NOT_ADC` | a pot was assigned a non-ADC GPIO (not 26-28) |
| `0x16` | `CS_STATUS_PENDING` | SET accepted; the main-loop apply has not run yet (poll again) |

---

## 4. Capability / validity model

A binding is valid iff its `action` bit is set in **both** the type's action mask
and the noun's action mask, its numeric operands are in range for the noun's
kind, and its pins pass the shared conflict checks. Validation order:
type -> noun -> action-in-range -> action-allowed-by-type-AND-noun ->
value/step/range bounds -> pins.

> **Hosts MUST build their UI from `REQ_GET_CS_CAPS` at runtime, not from
> hardcoded tables.** The firmware serves these exact tables, so a host that
> reads them can never disagree with the device about which combinations are
> legal, and new types/nouns/actions in a future firmware appear in the UI with
> no host change. The tables below are the current (version 1) values, shown so
> an integrator can reason about them; treat them as illustrative, not as a
> substitute for reading the caps at connect time.

### 4.1 Reading the caps responses

- `REQ_GET_CS_CAPS`, `wValue = 0xFFFF` -> 28-byte `CsCapsHeader`: version, limits,
  and the 6-entry type table (indexed by `CsType`).
- `REQ_GET_CS_CAPS`, `wValue = 0..8` (a noun index) -> 8-byte `CsNounDesc` for
  that noun. An out-of-range noun STALLs (USB) / returns ERROR (UART/I2C).

### 4.2 Type table (`CsType` -> allowed actions, pins, class)

| `CsType` | Value | Allowed actions | `actions` mask | Pins | Pin class |
|----------|-------|-----------------|----------------|------|-----------|
| `CS_TYPE_NONE` | 0 | (none; clears the slot) | `0x0000` | 0 | ANY |
| `CS_TYPE_BUTTON` | 1 | `INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER` | `0x00BC` | 1 | ANY |
| `CS_TYPE_SWITCH` | 2 | `FOLLOW` | `0x0040` | 1 | ANY |
| `CS_TYPE_POT` | 3 | `ADJUST` | `0x0001` | 1 | **ADC** |
| `CS_TYPE_ENCODER` | 4 | `STEP` | `0x0002` | 2 | ANY |
| `CS_TYPE_LED` | 5 | `IND_EQUALS` | `0x0100` | 1 | ANY |

Action bit positions (`CS_ACT_BIT(a) = 1 << a`): `ADJUST`=0, `STEP`=1, `INC`=2,
`DEC`=3, `TOGGLE`=4, `SET`=5, `FOLLOW`=6, `TRIGGER`=7, `IND_EQUALS`=8.

### 4.3 Noun table (`CsNoun` -> kind, enum count, range, allowed actions)

| `CsNoun` | Value | Kind | enum_count | min_q8 / max_q8 | Allowed actions | `actions` mask |
|----------|-------|------|------------|-----------------|-----------------|----------------|
| `CS_NOUN_USER_VOLUME` | 0 | CONTINUOUS | - | -15360 / 0 (-60..0 dB) | `ADJUST`,`STEP`,`INC`,`DEC`,`SET` | `0x002F` |
| `CS_NOUN_MASTER_VOLUME` | 1 | CONTINUOUS | - | -32512 / 0 (-127..0 dB) | `ADJUST`,`STEP`,`INC`,`DEC`,`SET` | `0x002F` |
| `CS_NOUN_USER_MUTE` | 2 | BOOL | - | 0 / 0 | `TOGGLE`,`SET`,`FOLLOW`,`IND_EQUALS` | `0x0170` |
| `CS_NOUN_LOUDNESS` | 3 | BOOL | - | 0 / 0 | `TOGGLE`,`SET`,`FOLLOW`,`IND_EQUALS` | `0x0170` |
| `CS_NOUN_CROSSFEED` | 4 | BOOL | - | 0 / 0 | `TOGGLE`,`SET`,`FOLLOW`,`IND_EQUALS` | `0x0170` |
| `CS_NOUN_LEVELLER` | 5 | BOOL | - | 0 / 0 | `TOGGLE`,`SET`,`FOLLOW`,`IND_EQUALS` | `0x0170` |
| `CS_NOUN_PRESET` | 6 | ENUM | 10 | - | `STEP`,`INC`,`DEC`,`SET`,`IND_EQUALS` | `0x012E` |
| `CS_NOUN_INPUT_SOURCE` | 7 | ENUM | 3 | - | `STEP`,`INC`,`DEC`,`SET`,`IND_EQUALS` | `0x012E` |
| `CS_NOUN_CLIP` | 8 | BOOL | - | 0 / 0 | `TRIGGER`,`IND_EQUALS` | `0x0180` |

The *effective* legal action set for a (type, noun) pair is the bitwise AND of
its two masks. Example: an encoder (`STEP` only) on `USER_MUTE` (bool, no
`STEP`) has an empty intersection and is rejected with
`CS_STATUS_INVALID_ACTION`.

---

## 5. Nouns reference

Each noun maps to an existing vendor command; the engine resolves an absolute
target and dispatches it.

| Noun | Underlying command | Value semantics |
|------|--------------------|-----------------|
| `USER_VOLUME` | `REQ_SET_USER_VOLUME` (`0xDA`, float dB) | The user/OS-slider volume (`audio_state.volume`), shared with UAC1; range -60..0 dB (`CENTER_VOLUME_INDEX = 60`). |
| `MASTER_VOLUME` | `REQ_SET_MASTER_VOLUME` (`0xD2`, float dB) | Device output ceiling; range -127..0 dB. The `-128 dB` mute sentinel is **not reachable** from a pot or encoder (the engine floors the live read at -127 and the range stops there); use a `SET`/`FOLLOW` on a bool noun for mute. |
| `USER_MUTE` | `REQ_SET_USER_MUTE` (`0xDC`, uint8 0/1) | User mute, shared with UAC1. |
| `LOUDNESS` | `REQ_SET_LOUDNESS` (`0x58`, uint8 0/1) | Loudness compensation enable. |
| `CROSSFEED` | `REQ_SET_CROSSFEED` (`0x5E`, uint8 0/1) | Crossfeed enable. |
| `LEVELLER` | `REQ_SET_LEVELLER_ENABLE` (`0xB4`, uint8 0/1) | Volume leveller enable. |
| `PRESET` | `REQ_PRESET_LOAD` (`0x91`, GET, wValue = slot) | Active preset 0-9. Uses the deferred, pipeline-safe load path a host would use. **Stepping moves across OCCUPIED slots only** (empty slots are skipped; an empty device is a no-op). |
| `INPUT_SOURCE` | `REQ_SET_INPUT_SOURCE` (`0xE0`, uint8) | `0` = USB, `1` = SPDIF, `2` = I2S (`enum_count = 3`). |
| `CLIP` | `REQ_CLEAR_CLIPS` (`0x83`, GET) | Any-channel clip latch. As an **LED** (`IND_EQUALS`, value 1): lit while any clip bit is set; stays lit until cleared. As a **button** (`TRIGGER`): a press clears the clip flags (same as a host `REQ_CLEAR_CLIPS`). |

### 5.1 Enum stepping detail

For `STEP`/`INC`/`DEC` on enum nouns, `CS_FLAG_WRAP` makes the ends wrap; without
it the value clamps at 0 and `enum_count-1`. For `PRESET`, stepping searches
outward in the requested direction for the next occupied slot (honoring wrap),
so a "next preset" encoder skips empty slots.

Enum nouns apply **deferred** in the firmware (a preset load or input switch
takes tens of ms). The engine therefore steps from a **target shadow**, the
last dispatched target, until the live value confirms it (or a 500 ms timeout
drops it), so spinning an encoder several detents advances several presets
rather than re-targeting the same next slot from a stale current value.

### 5.2 Continuous stepping / adjust detail

- `INC`/`DEC`/`STEP` on a continuous noun add/subtract `step` dB (default 1 dB),
  clamped to `[min_q8, max_q8]`.
- `ADJUST` (pot) maps the knob position across `[min_q8, max_q8]`, or across
  `[range_min, range_max]` when either range field is non-zero (a custom span,
  e.g. a volume knob limited to -30..0 dB).

---

## 6. Electrical / wiring reference

### 6.1 Buttons and switches (1 GPIO)

- **Default wiring is active-low with the internal pull-up**: wire the component
  between the GPIO and GND. Idle reads high (1), pressed/closed reads low (0);
  the engine inverts this to a logical level (1 = pressed/on).
- **`CS_FLAG_INVERT`** selects active-high with the internal pull-down: wire the
  component between the GPIO and 3V3.
- **Debounce**: a level must be stable for **10 ms** (10 ticks at 1 kHz) before it
  is accepted.
- **Button** (`CS_TYPE_BUTTON`): edge-triggered. The bound action fires on the
  press edge (logical 0 -> 1). The first debounced level seen after a claim is a
  sync, not a press, so a button held at boot does not fire spuriously.
- **Switch** (`CS_TYPE_SWITCH`, `FOLLOW` only): level-follow. The bound bool
  tracks the switch position, **including at boot** (boot-sync): the first stable
  read is dispatched as an absolute value, so the firmware state matches the
  physical switch immediately.

### 6.2 LEDs (1 GPIO)

- Driven **active-high** by default: the GPIO is an output, high = lit.
- **`CS_FLAG_INVERT`** = active-low (drive low = lit), for LEDs wired to 3V3
  through a resistor.
- The pin is initialized to the "off" state at claim and only re-driven when the
  indicated condition (`noun value == value`) changes, so there is no per-tick
  GPIO churn.

### 6.3 Rotary encoders (2 GPIOs)

- **2-pin incremental quadrature.** `gpio[0]` = channel A, `gpio[1]` = channel B;
  the two pins must differ. Both pins use the internal pull-up (or pull-down under
  `CS_FLAG_INVERT`), so wire the common terminal to GND (or 3V3).
- **One detent = 4 quarter-steps** (a full A/B cycle). The engine decodes with a
  standard 16-entry quadrature transition table and credits one `STEP` per detent.
  Invalid two-bit jumps decode as no movement (a skipped noisy sample).
- **`CS_FLAG_REVERSE`** flips the direction.

### 6.4 Potentiometers / faders (ADC GPIO only)

- **Only GPIO 26, 27, 28** (ADC channels 0-2, on both RP2040 and RP2350). GPIO 29
  is the board's VSYS/3 monitor and is excluded. A pot on any other GPIO is
  rejected with `CS_STATUS_PIN_NOT_ADC`.
- **Full-range or custom span.** With `range_min == range_max == 0`, the knob
  spans the noun's full dB range; otherwise it spans `[range_min, range_max]`
  (8.8 dB, must satisfy `range_min < range_max` and lie inside the noun range).
- **`CS_FLAG_REVERSE`** inverts the direction (CW = down).
- **Conditioning**: the 12-bit ADC reading is smoothed with an EMA (shift 3),
  gated by a **12-count deadband** (~0.3%) so electrical jitter does not
  re-dispatch, and quantized to the nearest half-dB.
- **Immediate takeover with boot sync.** At claim the engine seeds the filter and
  waits ~**50 ms** (50 ticks) for the EMA to settle, then takes the knob's
  physical position as the value (an absolute dispatch). There is no
  "pick-up"/"catch" behavior: the knob is authoritative as soon as it settles.

### 6.5 Poll budget

The tick runs at **1 kHz**. A detented encoder needs 4 samples per detent, so the
poll comfortably tracks a fast hand-spin (~250 quarter-steps/s -> ~60 detents/s).
Pots are read **one ADC conversion per tick, round-robin** across active pots, so
with the maximum useful three pots each is sampled every ~3 ms; ample for a fader.

---

## 7. Runtime behavior details

### 7.1 BUSY retry latch (absolute targets, no double-toggle)

Actions resolve to an **absolute target** at event time, never a relative "toggle
again". If a dispatch returns `CTRL_DISPATCH_BUSY` (a USB control SET is
mid-flight), the engine latches the resolved target and retries it on the next
tick, before sampling new events. Because the target is absolute, a retry can
never double-apply (e.g. a toggle cannot flip twice). Rapid encoder detents
accumulate correctly across a BUSY stall: the base for the next step is the
latched pending target, not the stale live value.

### 7.2 Boot bring-up order and pin collisions

`control_surfaces_init()` runs **last** in `core0_init()`, after `preset_boot_load`,
all audio/output pin claims, `notify_init`, and the UART/I2C control interfaces,
so pin-conflict checks are truthful and dispatched writes see initialized state.
For each stored non-empty binding it calls `control_surfaces_apply_binding`. If a
binding's pins now collide with something claimed earlier (a moved output pin, an
enabled control interface, etc.):

- The binding is **kept down** (inactive) but its config is **preserved** in the
  live table, so it round-trips unchanged and re-activates once the conflict
  clears.
- The failure code appears in that slot's `slot_status[]` (readable via
  `REQ_GET_CS_STATUS`); the `active_mask` bit stays 0.

Pot and switch boot-sync dispatches do not fire from `init`; they happen from the
first main-loop ticks.

### 7.3 Notification behavior

- **Parameter changes a control makes DO notify.** Each dispatch is tagged
  `PARAM_SRC_GPIO` (5), so hosts see the normal `PARAM_CHANGED` event and can
  reflect a physical knob live.
- **Binding-config changes do NOT notify.** `REQ_SET_CS_BINDING` only does
  GPIO/flash work; there is no `PARAM_CHANGED` for the binding table itself. A
  host that just wrote a binding should **re-read** it (`REQ_GET_CS_BINDING`) and
  poll `REQ_GET_CS_STATUS` for the result; it will not receive a push.

---

## 8. App integration patterns

All multi-byte fields little-endian. `slot` is 0-7.

### 8.1 Enumerate capabilities (do this at connect)

1. `GET 0x86, wValue=0xFFFF` -> 28-byte `CsCapsHeader`. Read `type_count`,
   `noun_count`, `max_bindings`, and the six `CsTypeDesc` entries.
2. For each noun `n` in `0 .. noun_count-1`: `GET 0x86, wValue=n` -> 8-byte
   `CsNounDesc`. Cache kind, enum_count, range, and the accepted-action mask.
3. Build the picker UI: for each type, offer the nouns whose action mask
   intersects the type's action mask; offer only the intersecting actions.

### 8.2a Configure: rotary encoder on GPIO 27/28, master volume, 1 dB/detent

Binding fields: `type = ENCODER (4)`, `noun = MASTER_VOLUME (1)`,
`action = STEP (1)`, `flags = 0`, `gpio = {27, 28}`, `value = 0`,
`step = 256` (1 dB in 8.8), `range = {0, 0}` (full range).

16-byte `CsBinding` (hex):

```
04 01 01 00 1B 1C 00 00 00 00 00 01 00 00 00 00
```

(`1B`=27, `1C`=28; `step` bytes `00 01` = 256.) Send:
`SET 0x84, wValue=<slot>, payload=<16 bytes>`, then poll
`GET 0x87` until `last_slot == slot` and read `last_status` (expect `0x00`).

### 8.2b Configure: LED on GPIO 20 indicating loudness is on

Binding fields: `type = LED (5)`, `noun = LOUDNESS (3)`,
`action = IND_EQUALS (8)`, `flags = 0` (active-high), `gpio = {20, 0xFF}`,
`value = 1` (lit while loudness == 1).

16-byte `CsBinding` (hex):

```
05 03 08 00 14 FF 00 00 01 00 00 00 00 00 00 00
```

(`14`=20, `FF`=unused second pin; `value` bytes `01 00` = 1.) Send the same way.
For an active-low LED, set `flags = 0x01` (`CS_FLAG_INVERT`).

### 8.3 Clear a binding

Send a binding with `type = 0` (`CS_TYPE_NONE`) to the slot; the rest of the
bytes are ignored. This releases the slot's pins and marks it idle.

```
SET 0x84, wValue=<slot>, payload = 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

`last_status` returns `PIN_CONFIG_SUCCESS`.

### 8.4 Read back and display live status

- `GET 0x85, wValue=<slot>` -> the live 16-byte `CsBinding` for editing/display.
- `GET 0x87` -> `CsStatusPacket`: `active_mask` tells you which slots are live;
  `slot_status[slot]` gives per-slot health (0 = ok, else a failure code from
  section 3.3, e.g. a boot pin collision).
- To reflect live control activity (a knob being turned), subscribe to the
  notification stream and watch for `PARAM_CHANGED` events with
  `source == PARAM_SRC_GPIO` (5).

---

## 9. Extension guide (firmware)

The wire format is future-proof because hosts read caps at runtime; adding
capability does not break existing hosts.

### 9.1 Add a new noun

1. Append a value to `CsNoun` (never renumber; `CS_NOUN_COUNT` grows).
2. Add a `CsNounDesc` row in `s_noun_desc[]` (kind, enum_count, action mask,
   dB range) in control_surfaces.c.
3. Add a `case` in `cs_noun_get()` (live read) and in `cs_try_dispatch()` (map to
   the underlying vendor command). For a deferred/pipeline-safe apply, dispatch
   through `vendor_dispatch_get` like `PRESET`/`CLIP` do.
4. Document it in section 5 here.

### 9.2 Add a new action

1. Append to `CsAction` (never renumber; `CS_ACT_COUNT` grows). Bit position must
   stay `< 16` (the masks are `uint16`).
2. Add the bit to the relevant type masks (`s_caps.types[]`) and the noun action
   groups (`CS_CONTROL_ACTS` / `CS_BOOL_ACTS` / `CS_ENUM_ACTS` or a per-noun mask).
3. Implement the behavior in the tick/press handlers and any value-bounds check in
   `cs_validate()`.

### 9.3 Add a new component type (e.g. a character display)

1. Append to `CsType` (never renumber; `CS_TYPE_COUNT` grows). Add a `CsTypeDesc`
   row: its action mask (indicate-class actions for a display), pin count (respect
   the `CS_MAX_BINDINGS` and per-tick ADC budget), and pin class.
2. Handle it in `cs_claim_pins`, `cs_release_pins`, `cs_seed_runtime`, and the
   `control_surfaces_tick()` dispatch switch.
3. No wire-format change is needed: `CsBinding` already carries type/pins/value,
   and hosts pick up the new type from the caps header automatically.

### 9.4 Versioning rules

- **Append-only** enum values; **never renumber** an existing `CsType`, `CsNoun`,
  or `CsAction` (they are wire- and flash-persistent).
- Bump `CS_CONFIG_VERSION` **only** when the `CsFlashConfig` byte layout changes
  (adding an enum value does not). A stored blob with a higher version than the
  firmware understands is ignored (feature stays idle) rather than misread.
- Bump `caps_version` when the descriptor semantics change so hosts can adapt.
- A `CsFlashConfig` **layout** change also needs a directory version bump: add a
  new `PresetDirectory_vN` snapshot, extend `load_directory()` with an
  N-1 -> N migration, bump `DIR_VERSION_CURRENT`, and extend
  `dir_sanitize_cs_config()` if new fields need bounds checks (see flash_storage.c
  for the V6 -> V7 pattern).

---

## 10. Limits and constraints

- **8 bindings** (`CS_MAX_BINDINGS`). Slots are independent; a slot holds one
  component.
- **Pin conflicts** use the shared `ctrl_iface_check_pin()`: a pin must be a valid
  GPIO, not already claimed by an output, MCK, SPDIF/I2S RX, DAC hardware-mute, a
  UART/I2C control interface, or another live binding. The **I2S clock pair is
  always treated as reserved** (whether or not I2S is currently running), matching
  the other control-interface pin checks. `control_surfaces_owns_pin()` is wired
  into `pin_used_by_fixed_peripheral()`, so no other subsystem can take a live CS
  pin.
- **One ADC conversion per tick** (round-robin), so pots share the single ADC
  fairly without stalling the 1 kHz loop.
- **No PIO and no GPIO IRQs** are used; the engine is pure main-loop polling. It is
  a cheap no-op when no binding is active.
- **RP2040 XIP placement**: `control_surfaces.c.o` executes from flash (XIP) on
  RP2040 via the copy-to-RAM linker script, keeping the RAM image within the
  264 KB budget. It contains no `DSP_TIME_CRITICAL` code and never runs inside the
  IRQs-off flash-write window, so XIP execution is safe.
- **Identical on both platforms**: same 8 bindings, same ADC pins 26-28, same
  behavior on RP2040 and RP2350.
- **Output-slot alignment is unaffected.** Every apply goes through the existing,
  known-safe vendor handlers (e.g. preset load's deferred pipeline-safe path), so
  the inviolable inter-slot phase-alignment guarantee is preserved; the Control
  Surfaces engine adds no new audio-path or reset behavior of its own.
