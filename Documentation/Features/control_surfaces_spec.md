# Control Surfaces (User-Wired Physical Controls and Indicators)

*Firmware capability format version: 2*
*Config (flash) version: 2*
*Directory version: 10*

This document is the complete, self-contained specification for the DSPi
Control Surfaces feature: user-wired push buttons, toggle switches,
potentiometers, quadrature rotary encoders, indicator LEDs, and PWM-dimmed
LEDs on spare GPIOs, configured over vendor commands `0x84`-`0x87` plus
`0x8B`/`0x8C` (per-slot names). It is
written for a host-app developer (e.g. DSPi Console) or an LLM adding Control
Surfaces support to an app, or extending the firmware feature itself. No DSPi
source is required to implement a host client.

Format v2 supersedes the v1 (16-byte binding, 8 slots, 9 nouns) format
described by earlier revisions of this document; see section 11 for the
compatibility story.

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
| **Parameter** | The operation applied (`CsAction`) plus its event/value/step/range/target | +1 dB per detent |
| **GPIO** | The pin(s) the component occupies | GPIO 27/28 |

The **Verb is not carried on the wire**; it is derived from the **Type**. A
potentiometer can only "adjust", an encoder can only "step", a switch can only
"follow", and so on. Encoding the verb separately would let a host send an
impossible combination (a pot that "toggles"); deriving it from the type makes
those states unrepresentable. The wire therefore carries Type, Noun, Action,
flags, event, target/index, pins, and numeric operands; the host builds its
verb-oriented UI from the capability tables (section 4).

v2 adds four orthogonal concepts to this model:

- **Events** (buttons only): one physical button can carry several bindings,
  distinguished by gesture: short press, long press (>= 500 ms), double press
  (two taps within 350 ms). Section 6.1.
- **Targets**: nouns that address a channel (per-input preamp, per-output
  gain/mute/enable, per-channel clip/level) or a channel plus a filter band
  (per-filter frequency/gain/Q/type/bypass) carry the address in the binding's
  `target` / `index` bytes. Section 4.4.
- **Units**: continuous nouns declare a unit (dB, Hz, Q, percent). Frequency
  and Q step and map logarithmically; dB and percent linearly. Section 2.1.
- **Acceleration and repeat**: encoders can accelerate on fast rotation;
  INC/DEC buttons can auto-repeat while held. Sections 6.3, 6.1.

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

Per-filter nouns use a read-modify-write: the engine copies the live filter
recipe, replaces the one field the binding drives, and dispatches the full
packet through `REQ_SET_EQ_PARAM`. The EQ handlers share a single deferred
apply slot, so the engine additionally treats "an EQ apply is still owed" as
BUSY and retries next tick rather than overwriting a pending write.

### 1.3 Why the config is device-global (not per-preset)

The binding table is persisted **device-global** in the preset directory (V9),
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

The per-slot **names** (section 3.4) follow the same reasoning and live in the
same directory (V10), as slot metadata alongside the binding table rather than
inside it: a name describes the physical control ("Sub Level"), so it survives
binding edits and slot clears, and may be set before a binding exists.

---

## 2. Wire reference (byte-by-byte)

All structures are `__attribute__((packed))`, little-endian, no padding. The
same `CsBinding` bytes appear on the wire (the `REQ_SET`/`GET_CS_BINDING`
payload) and in flash.

### 2.1 Units and fixed-point conventions

Every continuous noun declares a **unit** in its descriptor (`CsNounDesc.unit`).
The unit fixes both the wire encoding of `value`/`range_min`/`range_max` and
the stepping law:

| Unit | Value | Encoding of value/range | Stepping law | `step` field encoding | Default step | Pot/LED mapping |
|------|-------|-------------------------|--------------|------------------------|--------------|-----------------|
| `CS_UNIT_NONE` | 0 | plain integer (bool 0/1, enum 0..N-1) | one position | (unused) | 1 position | n/a |
| `CS_UNIT_DB` | 1 | signed 8.8 dB (1 dB = 256) | linear | 8.8 dB | 1 dB | linear |
| `CS_UNIT_HZ` | 2 | plain integer Hz | logarithmic | 8.8 octaves | 1/12 octave | logarithmic |
| `CS_UNIT_Q` | 3 | 8.8 Q (Q 0.707 = 181) | logarithmic | 8.8 octaves | 1/12 octave | logarithmic |
| `CS_UNIT_PERCENT` | 4 | 8.8 percent (1 % = 256) | linear | 8.8 percent | 1 % | linear |

Logarithmic stepping multiplies: one detent scales the value by
`2^(step_octaves)`, so a frequency knob moves in musically even ratios and a
Q knob sweeps 0.1 to 10 in even multiplicative steps. `step = 256` is one
octave per detent; `step = 0` selects the default 1/12 octave.

Pots and dispatch quantization: linear units quantize to **half a unit**
(half-dB, half-percent); log units quantize to **1/24 octave** above the
noun's minimum. A pot only re-dispatches when the quantized value changes,
giving smooth, jitter-free knob behavior without flooding the dispatcher.

### 2.2 `CsBinding` (24 bytes)

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `type` | `CsType` (0-6); `0` = slot cleared |
| 1 | 1 | `noun` | `CsNoun` (0-34) |
| 2 | 1 | `action` | `CsAction` (0-11) |
| 3 | 1 | `flags` | `CS_FLAG_*` bitfield (see 2.2.1); unknown bits are rejected with `CS_STATUS_INVALID_VALUE` |
| 4 | 1 | `gpio[0]` | primary GPIO |
| 5 | 1 | `gpio[1]` | second GPIO (encoders); `0xFF` (`CS_GPIO_UNUSED`) otherwise |
| 6 | 1 | `event` | `CsEvent` (buttons: 0 = press, 1 = long, 2 = double); MUST be 0 for other types |
| 7 | 1 | `target` | channel address for targeted nouns (section 4.4); 0 otherwise |
| 8 | 1 | `index` | filter band for `CS_TARGET_DSP_BAND` nouns; 0 otherwise |
| 9 | 1 | `reserved` | write 0 (rejected non-zero) |
| 10 | 2 | `value` (int16) | `SET`/`MOMENTARY` target, `IND_EQUALS`/`IND_ABOVE` comparand; unit-encoded (2.1) |
| 12 | 2 | `step` (int16) | `STEP`/`INC`/`DEC` size; `0` = the unit default (2.1) |
| 14 | 2 | `range_min` (int16) | pot / `IND_LEVEL` span low end; both range fields `0` = the noun's full range |
| 16 | 2 | `range_max` (int16) | pot / `IND_LEVEL` span high end |
| 18 | 6 | `reserved2[6]` | write 0 (rejected non-zero) |

#### 2.2.1 Flags

| Bit | Name | Applies to | Meaning |
|-----|------|-----------|---------|
| `0x01` | `CS_FLAG_INVERT` | inputs | active-high with pull-down (default is active-low with pull-up) |
| | | LEDs | drive low = lit (PWM LED: inverted duty) |
| `0x02` | `CS_FLAG_REVERSE` | pot/encoder | invert direction |
| `0x04` | `CS_FLAG_WRAP` | enum STEP/INC/DEC | wrap around the ends (INC + WRAP = a "cycle" button) |
| `0x08` | `CS_FLAG_ACCEL` | encoder only | fast rotation multiplies the step (section 6.3) |
| `0x10` | `CS_FLAG_REPEAT` | button INC/DEC, press event only | auto-repeat while held (section 6.1) |

`CS_FLAG_ACCEL` on a non-encoder and `CS_FLAG_REPEAT` on anything but a
button INC/DEC are rejected with `CS_STATUS_INVALID_VALUE`.

### 2.3 `CsFlashConfig` (388 bytes)

The device-global persisted blob. Not sent by any single command; it is what
the firmware stores and what `REQ_GET_ALL_PARAMS` does **not** contain.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `version` | `CS_CONFIG_VERSION` (2) |
| 1 | 3 | `reserved[3]` | 0 |
| 4 | 384 | `bindings[16]` | sixteen 24-byte `CsBinding` records |

### 2.4 `CsTypeDesc` (4 bytes) and `CsCapsHeader` (32 bytes)

`CsTypeDesc`:

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this component can drive |
| 2 | 1 | `pin_count` | GPIOs consumed (1 or 2) |
| 3 | 1 | `pin_class` | `CS_PINCLASS_ANY` (0) or `CS_PINCLASS_ADC` (1) |

`CsCapsHeader` (returned by `REQ_GET_CS_CAPS` with `wValue = 0xFFFF`):

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `caps_version` | capability format version (2) |
| 1 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (16) |
| 2 | 1 | `type_count` | `CS_TYPE_COUNT` (7); the type table has this many entries, indexed by `CsType` |
| 3 | 1 | `noun_count` | `CS_NOUN_COUNT` (35) |
| 4 | 28 | `types[7]` | seven `CsTypeDesc`, one per `CsType` including index 0 (`NONE`, all-zero) |

Total `4 + 4*7 = 32` bytes.

### 2.5 `CsNounDesc` (12 bytes)

Returned by `REQ_GET_CS_CAPS` with `wValue = noun index` (0-34).

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `kind` | `CS_KIND_CONTINUOUS` (0), `CS_KIND_BOOL` (1), `CS_KIND_ENUM` (2) |
| 1 | 1 | `enum_count` | valid enum values `0..enum_count-1` (`CS_KIND_ENUM` only) |
| 2 | 2 | `actions` (uint16) | `CS_ACT_BIT` mask this noun accepts; **0 = noun unavailable on this platform** (e.g. `ADAT_ACTIVE` on RP2040) |
| 4 | 2 | `min_q` (int16) | continuous range low end, unit-encoded (2.1) |
| 6 | 2 | `max_q` (int16) | continuous range high end |
| 8 | 1 | `unit` | `CS_UNIT_*` (2.1) |
| 9 | 1 | `target_kind` | `CS_TARGET_*` (4.4) |
| 10 | 1 | `target_count` | valid `target` values `0..target_count-1`; 0 when untargeted |
| 11 | 1 | `dflags` | `CS_NDF_DEFERRED` (`0x01`): apply is deferred; the engine steps from a target shadow (7.2) |

### 2.6 `CsStatusPacket` (22 bytes)

Returned by `REQ_GET_CS_STATUS`.

| Off | Size | Field | Meaning |
|----|------|-------|---------|
| 0 | 1 | `last_status` | result of the most recent `REQ_SET_CS_BINDING` (`PIN_CONFIG_*` / `CS_STATUS_*`) |
| 1 | 1 | `last_slot` | slot the last SET targeted |
| 2 | 1 | `max_bindings` | `CS_MAX_BINDINGS` (16) |
| 3 | 1 | `reserved` | 0 |
| 4 | 2 | `active_mask` (uint16) | bit N = binding N is live |
| 6 | 16 | `slot_status[16]` | per-slot apply status; `PIN_CONFIG_SUCCESS` (0) when live, else the failure code |

---

## 3. Command reference (`0x84`-`0x87`, `0x8B`-`0x8C`)

| Command | Code | Dir | wValue | wLength / payload | Response |
|---------|------|-----|--------|-------------------|----------|
| `REQ_SET_CS_BINDING` | `0x84` | OUT | slot (0-15) | 24-byte `CsBinding` | none (deferred; poll `0x87`) |
| `REQ_GET_CS_BINDING` | `0x85` | IN | slot (0-15) | - | 24-byte `CsBinding` (live) |
| `REQ_GET_CS_CAPS` | `0x86` | IN | `0xFFFF` = header+types; else noun index | - | 32-byte `CsCapsHeader` or 12-byte `CsNounDesc` |
| `REQ_GET_CS_STATUS` | `0x87` | IN | - | - | 22-byte `CsStatusPacket` |
| `REQ_SET_CS_NAME` | `0x8B` | OUT | slot (0-15) | 1-32 byte name | none (deferred; poll `0x87`) |
| `REQ_GET_CS_NAME` | `0x8C` | IN | slot (0-15) | - | 32-byte NUL-terminated name |

(`0x88`-`0x8A` are reserved for another feature branch.)

### 3.1 Transport note (UART / I2C)

Every one of these commands works identically over USB, UART, and I2C (the
transport-neutral dispatcher). On the external transports, the direction bit
matters: `0x85`, `0x86`, `0x87`, `0x8C` are IN (GET-type) frames; `0x84` and
`0x8B` are OUT (SET-type) frames carrying their payload.

### 3.2 Deferred-apply model (`REQ_SET_CS_BINDING`)

`REQ_SET_CS_BINDING` does **not** apply synchronously. The USB/transport handler
only validates the slot index and payload length and latches the 24-byte binding
into a pending buffer, then returns immediately (no response payload). The main
loop then:

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

If the slot index in `wValue` is `>= 16`, the handler records
`CS_STATUS_INVALID_SLOT` immediately (no main-loop round trip) and no apply is
queued. A payload shorter than 24 bytes (e.g. a v1 host still sending 16)
records `CS_STATUS_INVALID_VALUE` immediately. A SET arriving while a previous
SET is still queued for the main-loop apply is dropped and records
`CS_STATUS_BUSY` (`0x1B`); a host that polls for the PENDING result before
sending the next SET (as this section prescribes) never sees it.

### 3.3 Status codes

`0x00`-`0x05` reuse the shared `PIN_CONFIG_*` namespace (config.h); Control
Surfaces extends it from `0x10`.

| Code | Name | Meaning |
|------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | applied (or slot cleared) |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | pin out of range, or an encoder's two pins are equal |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | pin already claimed by another peripheral or a non-shareable binding |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | (shared namespace; not produced by CS) |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | (shared namespace; not produced by CS) |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | (shared namespace; not produced by CS) |
| `0x10` | `CS_STATUS_INVALID_SLOT` | slot index >= 16 |
| `0x11` | `CS_STATUS_INVALID_TYPE` | `type` >= `CS_TYPE_COUNT` |
| `0x12` | `CS_STATUS_INVALID_NOUN` | `noun` >= `CS_NOUN_COUNT` |
| `0x13` | `CS_STATUS_INVALID_ACTION` | `action` invalid, or not allowed for this type+noun (including a platform-disabled noun, whose action mask is 0) |
| `0x14` | `CS_STATUS_INVALID_VALUE` | `value` / `step` / `range` out of bounds, unknown `flags` bits, misused ACCEL/REPEAT flag, non-zero reserved bytes, or INVERT mismatch on a shared button pin |
| `0x15` | `CS_STATUS_PIN_NOT_ADC` | a pot was assigned a non-ADC GPIO (not 26-28) |
| `0x16` | `CS_STATUS_PENDING` | SET accepted; the main-loop apply has not run yet (poll again) |
| `0x17` | `CS_STATUS_INVALID_TARGET` | `target`/`index` out of range for the noun (or non-zero on an untargeted noun) |
| `0x18` | `CS_STATUS_INVALID_EVENT` | bad `event` value, an event on a non-button, or MOMENTARY/REPEAT bound to a non-press event |
| `0x19` | `CS_STATUS_PWM_CONFLICT` | PWM LED collides with another PWM LED on the same PWM slice+channel |
| `0x1A` | `CS_STATUS_EVENT_IN_USE` | another button binding already has this GPIO+event pair |
| `0x1B` | `CS_STATUS_BUSY` | SET dropped; a previous binding SET is still queued for apply (retry after polling) |
| `0x1C` | `CS_STATUS_FLASH_ERROR` | name SET accepted but the directory persist failed |

### 3.4 Per-slot names (`REQ_SET_CS_NAME` / `REQ_GET_CS_NAME`)

Each of the 16 slots carries a device-persistent user label (`CS_NAME_LEN` =
32 bytes, NUL-terminated; same convention as preset and channel names), so an
app on any host, or an external MCU on UART/I2C, can show what a given control
is for ("Sub Level", "Mute All") without out-of-band knowledge.

Semantics:

- **Names are slot metadata, independent of the binding.** Setting or clearing
  a binding leaves the slot's name untouched, and a name may be set for a slot
  that has no binding yet. Clear a name explicitly by sending a payload of one
  NUL byte.
- **Persistence** is device-global in the preset directory (V10), next to the
  binding table; names survive preset changes and factory reset. All-zero =
  unnamed, so a fresh directory needs no seeding.
- `REQ_SET_CS_NAME` uses the **same deferred model** as the binding SET
  (section 3.2) because the persist is a directory-sector flash write: the
  handler validates the slot, latches the name, records `CS_STATUS_PENDING`
  in `last_status`/`last_slot`, and the main loop persists and overwrites the
  status with `PIN_CONFIG_SUCCESS` (or `CS_STATUS_FLASH_ERROR`). Binding and
  name SETs share the single status channel; poll `REQ_GET_CS_STATUS` until
  the PENDING result resolves before sending the next SET of either kind.
  A name SET arriving while a previous name SET is still queued records
  `CS_STATUS_BUSY`; an empty payload records `CS_STATUS_INVALID_VALUE`.
  Payloads longer than 31 characters are truncated.
- `REQ_GET_CS_NAME` is synchronous and reads the directory's RAM cache (no
  flash access); it always returns 32 bytes with guaranteed NUL termination.
- Names are **not** part of `WireBulkParams` and emit **no notification**;
  a host that cares about cross-host renames re-reads the 16 names on
  connect (16 GETs, one round trip each).

---

## 4. Capability / validity model

A binding is valid iff its `action` bit is set in **both** the type's action mask
and the noun's action mask, its `event` obeys the button rules, its
`target`/`index` address an existing channel/band, its numeric operands are in
range for the noun's kind and unit, and its pins pass the conflict checks.
Validation order: type -> noun -> action-in-range -> flags/reserved ->
action-allowed-by-type-AND-noun -> event -> target/index -> value/step/range ->
pins -> PWM slice.

> **Hosts MUST build their UI from `REQ_GET_CS_CAPS` at runtime, not from
> hardcoded tables.** The firmware serves these exact tables, so a host that
> reads them can never disagree with the device about which combinations are
> legal, and new types/nouns/actions in a future firmware appear in the UI with
> no host change. The tables below are the current (version 2) values, shown so
> an integrator can reason about them; treat them as illustrative, not as a
> substitute for reading the caps at connect time.

### 4.1 Reading the caps responses

- `REQ_GET_CS_CAPS`, `wValue = 0xFFFF` -> 32-byte `CsCapsHeader`: version, limits,
  and the 7-entry type table (indexed by `CsType`).
- `REQ_GET_CS_CAPS`, `wValue = 0..34` (a noun index) -> 12-byte `CsNounDesc` for
  that noun. An out-of-range noun STALLs (USB) / returns ERROR (UART/I2C).
- A noun whose `actions == 0` is unavailable on this platform (currently only
  `ADAT_ACTIVE` on RP2040); grey it out.

### 4.2 Type table (`CsType` -> allowed actions, pins, class)

| `CsType` | Value | Allowed actions | `actions` mask | Pins | Pin class |
|----------|-------|-----------------|----------------|------|-----------|
| `CS_TYPE_NONE` | 0 | (none; clears the slot) | `0x0000` | 0 | ANY |
| `CS_TYPE_BUTTON` | 1 | `INC`, `DEC`, `TOGGLE`, `SET`, `TRIGGER`, `MOMENTARY` | `0x02BC` | 1 | ANY |
| `CS_TYPE_SWITCH` | 2 | `FOLLOW` | `0x0040` | 1 | ANY |
| `CS_TYPE_POT` | 3 | `ADJUST` | `0x0001` | 1 | **ADC** |
| `CS_TYPE_ENCODER` | 4 | `STEP` | `0x0002` | 2 | ANY |
| `CS_TYPE_LED` | 5 | `IND_EQUALS`, `IND_ABOVE` | `0x0500` | 1 | ANY |
| `CS_TYPE_LED_PWM` | 6 | `IND_EQUALS`, `IND_ABOVE`, `IND_LEVEL` | `0x0D00` | 1 | ANY |

Action bit positions (`CS_ACT_BIT(a) = 1 << a`): `ADJUST`=0, `STEP`=1, `INC`=2,
`DEC`=3, `TOGGLE`=4, `SET`=5, `FOLLOW`=6, `TRIGGER`=7, `IND_EQUALS`=8,
`MOMENTARY`=9, `IND_ABOVE`=10, `IND_LEVEL`=11.

### 4.3 Noun table (`CsNoun` -> kind, unit, range, target, allowed actions)

Action-mask groups used below:

- **CONT-RW** `0x0C2F` = `ADJUST | STEP | INC | DEC | SET | IND_ABOVE | IND_LEVEL`
- **BOOL-RW** `0x0370` = `TOGGLE | SET | FOLLOW | MOMENTARY | IND_EQUALS`
- **ENUM-RW** `0x012E` = `STEP | INC | DEC | SET | IND_EQUALS`
- **BOOL-RO / ENUM-RO** `0x0100` = `IND_EQUALS`
- **CONT-RO** `0x0C00` = `IND_ABOVE | IND_LEVEL`

| `CsNoun` | Value | Kind | Unit | Range / enum_count | Target | Actions |
|----------|-------|------|------|--------------------|--------|---------|
| `USER_VOLUME` | 0 | CONT | DB | -60..0 dB | - | CONT-RW |
| `MASTER_VOLUME` | 1 | CONT | DB | -127..0 dB | - | CONT-RW |
| `USER_MUTE` | 2 | BOOL | - | - | - | BOOL-RW |
| `LOUDNESS` | 3 | BOOL | - | - | - | BOOL-RW |
| `CROSSFEED` | 4 | BOOL | - | - | - | BOOL-RW |
| `LEVELLER` | 5 | BOOL | - | - | - | BOOL-RW |
| `PRESET` | 6 | ENUM | - | 10 | - | ENUM-RW, deferred |
| `INPUT_SOURCE` | 7 | ENUM | - | 3 (USB/SPDIF/I2S) | - | ENUM-RW, deferred |
| `CLIP` | 8 | BOOL | - | - | - | `TRIGGER \| IND_EQUALS` (`0x0180`) |
| `EQ_BYPASS` | 9 | BOOL | - | 1 = bypassed | - | BOOL-RW |
| `LG_SYNC` | 10 | BOOL | - | - | - | BOOL-RW |
| `CROSSFEED_PRESET` | 11 | ENUM | - | 4 (default/chu-moy/meier/custom) | - | ENUM-RW |
| `CROSSFEED_ITD` | 12 | BOOL | - | - | - | BOOL-RW |
| `LEVELLER_AMOUNT` | 13 | CONT | PERCENT | 0..100 % | - | CONT-RW |
| `LEVELLER_SPEED` | 14 | ENUM | - | 3 (slow/medium/fast) | - | ENUM-RW |
| `LEVELLER_LOOKAHEAD` | 15 | BOOL | - | - | - | BOOL-RW |
| `PREAMP` | 16 | CONT | DB | -24..+24 dB | INPUT_CH | CONT-RW |
| `OUTPUT_GAIN` | 17 | CONT | DB | -60..+12 dB | OUTPUT_CH | CONT-RW |
| `OUTPUT_MUTE` | 18 | BOOL | - | - | OUTPUT_CH | BOOL-RW |
| `OUTPUT_ENABLE` | 19 | BOOL | - | - | OUTPUT_CH | BOOL-RW |
| `FILTER_FREQ` | 20 | CONT | HZ | 20..20000 Hz | DSP_BAND | CONT-RW, deferred |
| `FILTER_GAIN` | 21 | CONT | DB | -24..+24 dB | DSP_BAND (PEQ only) | CONT-RW, deferred |
| `FILTER_Q` | 22 | CONT | Q | 0.1..10 | DSP_BAND (PEQ only) | CONT-RW, deferred |
| `FILTER_TYPE` | 23 | ENUM | - | 11 (PEQ `FilterType` 0-10) | DSP_BAND (PEQ only) | ENUM-RW, deferred |
| `FILTER_BYPASS` | 24 | BOOL | - | 1 = bypassed | DSP_BAND | BOOL-RW, deferred |
| `SIGGEN` | 25 | BOOL | - | 1 = running | - | BOOL-RW |
| `DAC_MUTE_TEST` | 26 | BOOL | - | - | - | `TRIGGER` (`0x0080`) |
| `CLIP_CH` | 27 | BOOL | - | read-only | DSP_CH | BOOL-RO |
| `LEVEL` | 28 | CONT | DB | -60..0 dB, read-only | DSP_CH | CONT-RO |
| `SPDIF_LOCK` | 29 | BOOL | - | read-only | - | BOOL-RO |
| `SAMPLE_RATE` | 30 | ENUM | - | 3 (0=44.1k, 1=48k, 2=96k), read-only | - | ENUM-RO |
| `USB_STREAMING` | 31 | BOOL | - | read-only | - | BOOL-RO |
| `ADAT_ACTIVE` | 32 | BOOL | - | read-only; **RP2350 only** (mask 0 on RP2040) | - | BOOL-RO |
| `LG_PRESENT` | 33 | BOOL | - | read-only | - | BOOL-RO |
| `LG_MUTED` | 34 | BOOL | - | read-only | - | BOOL-RO |

The *effective* legal action set for a (type, noun) pair is the bitwise AND of
its two masks. Example: an encoder (`STEP` only) on `USER_MUTE` (bool, no
`STEP`) has an empty intersection and is rejected with
`CS_STATUS_INVALID_ACTION`.

### 4.4 Targets

`CsNounDesc.target_kind` says what the binding's `target` byte addresses;
`target_count` gives the valid range (it reflects the platform: RP2350 has 8
input channels and 9 outputs, RP2040 has 2 and 5).

| `target_kind` | Value | `target` means | `index` means |
|---------------|-------|----------------|----------------|
| `CS_TARGET_NONE` | 0 | (must be 0) | (must be 0) |
| `CS_TARGET_INPUT_CH` | 1 | input channel 0..N-1 | (must be 0) |
| `CS_TARGET_OUTPUT_CH` | 2 | output channel 0..N-1 | (must be 0) |
| `CS_TARGET_DSP_CH` | 3 | DSP channel (inputs first, then outputs) | (must be 0) |
| `CS_TARGET_DSP_BAND` | 4 | DSP channel | filter band |

Valid bands for `CS_TARGET_DSP_BAND`: PEQ bands `0..channel_band_counts-1`
(currently 10 per channel), plus, for `FILTER_FREQ` and `FILTER_BYPASS` only,
the crossover bands `20..23` **on output channels only** (matching the EQ
handler's addressing; see crossover_filters_spec.md). `FILTER_GAIN`, `FILTER_Q`
and `FILTER_TYPE` are PEQ-only. Anything else is `CS_STATUS_INVALID_TARGET`.

---

## 5. Nouns reference

Each noun maps to an existing vendor command; the engine resolves an absolute
target and dispatches it.

| Noun | Underlying command | Value semantics |
|------|--------------------|-----------------|
| `USER_VOLUME` | `REQ_SET_USER_VOLUME` (`0xDA`, float dB) | The user/OS-slider volume, shared with UAC1; range -60..0 dB. |
| `MASTER_VOLUME` | `REQ_SET_MASTER_VOLUME` (`0xD2`, float dB) | Device output ceiling; range -127..0 dB. The `-128 dB` mute sentinel is **not reachable** from a pot or encoder; use a mute noun. |
| `USER_MUTE` | `REQ_SET_USER_MUTE` (`0xDC`, uint8 0/1) | User mute, shared with UAC1. |
| `LOUDNESS` | `REQ_SET_LOUDNESS` (`0x58`) | Loudness compensation enable. |
| `CROSSFEED` | `REQ_SET_CROSSFEED` (`0x5E`) | Crossfeed enable. |
| `LEVELLER` | `REQ_SET_LEVELLER_ENABLE` (`0xB4`) | Volume leveller enable. |
| `PRESET` | `REQ_PRESET_LOAD` (`0x91`, GET, wValue = slot) | Active preset 0-9, deferred pipeline-safe load. **Stepping moves across OCCUPIED slots only.** |
| `INPUT_SOURCE` | `REQ_SET_INPUT_SOURCE` (`0xE0`) | 0 = USB, 1 = SPDIF, 2 = I2S. |
| `CLIP` | `REQ_CLEAR_CLIPS` (`0x83`, GET) | Any-channel clip latch. LED (`IND_EQUALS`, value 1): lit while any clip bit is set. Button (`TRIGGER`): clears all clip flags. |
| `EQ_BYPASS` | `REQ_SET_BYPASS` (`0x46`) | Master EQ bypass; value 1 = EQ bypassed. |
| `LG_SYNC` | `REQ_SET_LG_SOUND_SYNC_ENABLE` (`0xE6`) | LG Sound Sync enable. |
| `CROSSFEED_PRESET` | `REQ_SET_CROSSFEED_PRESET` (`0x60`) | Crossfeed voicing 0-3; INC+WRAP makes a preset-cycle button. |
| `CROSSFEED_ITD` | `REQ_SET_CROSSFEED_ITD` (`0x66`) | Crossfeed inter-aural time delay enable. |
| `LEVELLER_AMOUNT` | `REQ_SET_LEVELLER_AMOUNT` (`0xB6`, float) | Leveller strength 0..100 %. |
| `LEVELLER_SPEED` | `REQ_SET_LEVELLER_SPEED` (`0xB8`) | 0 slow, 1 medium, 2 fast. |
| `LEVELLER_LOOKAHEAD` | `REQ_SET_LEVELLER_LOOKAHEAD` (`0xBC`) | Lookahead enable (toggling resets the leveller delay line; that is the handler's documented behavior, not a CS side effect). |
| `PREAMP` | `REQ_SET_PREAMP_CH` (`0xD0`, wValue = `target`, float dB) | Per-input-channel preamp, -24..+24 dB bindable span. |
| `OUTPUT_GAIN` | `REQ_SET_OUTPUT_GAIN` (`0x74`, wValue = `target`, float dB) | Per-output gain, -60..+12 dB bindable span. |
| `OUTPUT_MUTE` | `REQ_SET_OUTPUT_MUTE` (`0x76`, wValue = `target`) | Per-output soft mute. |
| `OUTPUT_ENABLE` | `REQ_SET_OUTPUT_ENABLE` (`0x72`, wValue = `target`) | Per-output enable. Subject to the PDM/EQ-worker Core 1 interlock; a blocked enable is silently skipped by the handler (the switch/LED will show the true state). |
| `FILTER_FREQ` | `REQ_SET_EQ_PARAM` (`0x42`, RMW) | Per-filter frequency, 20..20000 Hz, log stepping. Valid on PEQ bands and (outputs only) crossover bands 20-23; a sub-crossover frequency knob is `target = <sub output>`, `index = 20`. |
| `FILTER_GAIN` | `REQ_SET_EQ_PARAM` (RMW) | Per-filter gain, -24..+24 dB. PEQ bands only. |
| `FILTER_Q` | `REQ_SET_EQ_PARAM` (RMW) | Per-filter Q, 0.1..10, log stepping. PEQ bands only. |
| `FILTER_TYPE` | `REQ_SET_EQ_PARAM` (RMW) | PEQ `FilterType` 0-10 (flat, peaking, low-shelf, high-shelf, low-pass, high-pass, notch, all-pass, all-pass-1, low-shelf-1, high-shelf-1). PEQ bands only. |
| `FILTER_BYPASS` | `REQ_SET_BAND_BYPASS` (`0xD8`, wValue = `(target << 8) \| index`) | Per-band user bypass; value 1 = bypassed. PEQ and (outputs only) crossover bands. |
| `SIGGEN` | `REQ_SIGGEN_CONTROL` (`0xA6`, GET, wValue = start/stop) | Test signal generator run state (uses the applied `SiggenConfig`). 1 = running (any non-idle state). `MOMENTARY` gives hold-to-test. |
| `DAC_MUTE_TEST` | `REQ_TEST_DAC_HW_MUTE` (`0xEC`, GET) | `TRIGGER` pulses the DAC hardware mute for ~1 s so an installer can verify pin and polarity by ear. |
| `CLIP_CH` | (read-only) | Per-channel clip latch bit (`target` = DSP channel). Clearing is global via the `CLIP` noun. |
| `LEVEL` | (read-only) | Per-channel peak meter in dB (-60..0), `target` = DSP channel. Drives `IND_ABOVE` (signal-present LED) and `IND_LEVEL` (PWM meter LED). |
| `SPDIF_LOCK` | (read-only) | 1 while the SPDIF receiver is locked. |
| `SAMPLE_RATE` | (read-only) | Enum for `IND_EQUALS`: LED lit while the pipeline rate equals 44.1/48/96 kHz. An unrecognised rate matches nothing. |
| `USB_STREAMING` | (read-only) | 1 while USB is the active input and the host stream is running. |
| `ADAT_ACTIVE` | (read-only, RP2350) | 1 while the ADAT output is streaming at a supported rate. |
| `LG_PRESENT` | (read-only) | 1 while an LG Sound Sync source is detected. |
| `LG_MUTED` | (read-only) | 1 while an LG source is present and reports muted. |

### 5.1 Enum stepping detail

For `STEP`/`INC`/`DEC` on enum nouns, `CS_FLAG_WRAP` makes the ends wrap; without
it the value clamps at 0 and `enum_count-1`. **`INC` + `WRAP` is the canonical
"cycle" button** (cycle inputs, cycle crossfeed presets, cycle presets). For
`PRESET`, stepping searches outward in the requested direction for the next
occupied slot (honoring wrap), so a "next preset" button skips empty slots.

### 5.2 Continuous stepping / adjust detail

- `INC`/`DEC`/`STEP` on a continuous noun move by `step` (unit default when 0),
  linearly for dB/percent and multiplicatively for Hz/Q, clamped to the noun
  range. Encoder acceleration multiplies the number of steps (6.3).
- `ADJUST` (pot) maps the knob position across the noun's full range, or across
  `[range_min, range_max]` when either field is non-zero (a custom span,
  e.g. a volume knob limited to -30..0 dB, or a sub-crossover knob limited to
  40..200 Hz). Log-unit nouns map exponentially so knob travel is perceptually
  even.

### 5.3 Deferred nouns and the target shadow

Nouns flagged `CS_NDF_DEFERRED` (`PRESET`, `INPUT_SOURCE`, all `FILTER_*`)
apply asynchronously in the firmware (a preset load takes tens of ms; EQ writes
land on the next main-loop pass). The engine steps such nouns from a **target
shadow**, the last dispatched target, until the live value confirms it (or a
500 ms timeout drops it), so spinning an encoder several detents advances
several steps rather than re-targeting from a stale read.

The `FILTER_*` nouns additionally treat "an EQ apply is still owed" as BUSY
(the EQ handlers share one deferred packet slot); the engine retries next tick,
so rapid detents on two different filter knobs cannot overwrite each other.

---

## 6. Electrical / wiring reference

### 6.1 Buttons (1 GPIO): gestures, sharing, repeat

- **Default wiring is active-low with the internal pull-up**: wire the button
  between the GPIO and GND. **`CS_FLAG_INVERT`** selects active-high with the
  internal pull-down.
- **Debounce**: a level must be stable for **10 ms** before it is accepted.
- **Events.** Each button binding names the gesture that fires it
  (`CsBinding.event`): `PRESS` (0), `LONG` (1, held >= 500 ms), `DOUBLE` (2,
  two taps with the second press within 350 ms of the first release).
- **Pin sharing.** Several button bindings may share one GPIO, one per event
  (the same GPIO+event pair twice is `CS_STATUS_EVENT_IN_USE`). All bindings
  on a shared pin must agree on `CS_FLAG_INVERT` (`CS_STATUS_INVALID_VALUE`
  otherwise). Sharing with any non-button binding is `PIN_CONFIG_PIN_IN_USE`.
- **Gesture disambiguation** (per pin, automatic from which events are bound):
  - Only `PRESS` bound: fires on the press edge (immediate).
  - `LONG` also bound: `PRESS` fires on release before the 500 ms threshold;
    `LONG` fires once at the threshold while still held.
  - `DOUBLE` also bound: a second press inside the window fires `DOUBLE`; a
    lone tap fires `PRESS` when the window expires (the unavoidable cost of
    double-press detection is that latency on single presses).
  - A long hold of the second tap of a double does not additionally fire `LONG`.
- **Hold-to-repeat** (`CS_FLAG_REPEAT`, `INC`/`DEC` on the `PRESS` event):
  after 400 ms held, the action repeats at 12.5 Hz. Runtime-suppressed on pins
  that also carry `LONG` or `DOUBLE` bindings (the hold belongs to the gesture
  decoder there).
- **`MOMENTARY`** (press event only): on press the noun is set to `value`; on
  release it is restored to the value captured at the press. Both directions
  are absolute dispatches (7.1). Engages on the press edge regardless of other
  gestures on the pin; combining it with `LONG`/`DOUBLE` bindings on the same
  pin is legal but rarely what you want.
- A button held at boot (or when its binding is applied) is a sync, not a
  press; nothing fires, including `LONG`, until it is released and pressed
  again.

### 6.2 Switches (1 GPIO)

- `FOLLOW` only: the bound bool tracks the switch position, **including at
  boot** (boot-sync): the first stable read is dispatched as an absolute value,
  so the firmware state matches the physical switch immediately.
- Wiring/debounce as buttons; no events.

### 6.3 Rotary encoders (2 GPIOs)

- **2-pin incremental quadrature.** `gpio[0]` = channel A, `gpio[1]` = channel B;
  the two pins must differ. Both pins use the internal pull-up (or pull-down
  under `CS_FLAG_INVERT`), so wire the common terminal to GND (or 3V3).
- **One detent = 4 quarter-steps** (a full A/B cycle), decoded with a standard
  16-entry transition table; invalid two-bit jumps decode as no movement.
- **`CS_FLAG_REVERSE`** flips the direction.
- **Acceleration** (`CS_FLAG_ACCEL`, encoders only): the per-detent step count
  multiplies with rotation speed, measured as the gap between detents:
  >= 128 ms x1, < 128 ms x2, < 64 ms x4, < 32 ms x8. Enum nouns never
  accelerate (always one position per detent). Recommended for volume and
  frequency encoders; leave off for fine-trim duties.

### 6.4 Potentiometers / faders (ADC GPIO only)

- **Only GPIO 26, 27, 28** (ADC channels 0-2, on both RP2040 and RP2350). GPIO 29
  is the board's VSYS/3 monitor and is excluded. A pot on any other GPIO is
  rejected with `CS_STATUS_PIN_NOT_ADC`.
- **Full-range or custom span.** With `range_min == range_max == 0`, the knob
  spans the noun's full range; otherwise it spans `[range_min, range_max]`
  (unit-encoded, must satisfy `range_min < range_max` and lie inside the noun
  range).
- **`CS_FLAG_REVERSE`** inverts the direction (CW = down).
- **Conditioning**: the 12-bit ADC reading is smoothed with an EMA (shift 3),
  gated by a **12-count deadband** (~0.3%) so electrical jitter does not
  re-dispatch, and quantized per the noun's unit (half-dB / half-percent /
  1/24 octave).
- **Immediate takeover with boot sync.** At claim the engine seeds the filter
  and waits ~50 pot-service rounds for the EMA to settle (~50 ms with one pot;
  scaled by the round-robin, so ~150 ms with three), then takes the knob's
  physical position as the value (an absolute dispatch). There is no
  "pick-up"/"catch" behavior: the knob is authoritative as soon as it settles.
  **The knob owns its parameter**: shortly after boot (or binding apply) it
  overrides whatever value the preset or a host loaded, so wire pots only to
  parameters the panel should own.

### 6.5 LEDs (1 GPIO)

- Driven **active-high** by default: the GPIO is an output, high = lit.
  **`CS_FLAG_INVERT`** = active-low (drive low = lit), for LEDs wired to 3V3
  through a resistor.
- Actions: `IND_EQUALS` (lit while noun value == `value`) and `IND_ABOVE`
  (lit while noun value >= `value`, continuous nouns; e.g. a signal-present
  LED is `LEVEL` + `IND_ABOVE` with `value = -45 dB`).
- The pin is initialized to "off" at claim and only re-driven on change.
  Indicator evaluation is decimated to **every 8 ms**, staggered across slots.

### 6.6 PWM LEDs (1 GPIO, hardware PWM)

- `CS_TYPE_LED_PWM` drives the LED from the pin's hardware PWM slice
  (wrap 4095 at sysclk/16, a ~2 kHz carrier). Nothing else in the firmware
  uses PWM, so all slices are free; still, two PWM LEDs must not land on the
  same **slice+channel output** (e.g. GPIO 0 and GPIO 16 are both slice 0
  channel A on RP2040); that is rejected with `CS_STATUS_PWM_CONFLICT`.
  Two PWM LEDs on the same slice but different channels (e.g. GPIO 0 and 1)
  are fine.
- Actions: `IND_LEVEL` (brightness follows the noun value across the noun
  range or the custom `[range_min, range_max]` span, with a squared
  perceptual curve; a per-channel VU-style meter LED is `LEVEL` +
  `IND_LEVEL`), plus `IND_EQUALS`/`IND_ABOVE` as full-on/off.
- **`CS_FLAG_INVERT`** inverts the duty cycle for active-low wiring.
- Refresh is decimated like plain LEDs (8 ms); brightness changes are applied
  only when the computed level changes.

### 6.7 Poll budget

The tick runs at **1 kHz**. A detented encoder needs 4 samples per detent, so the
poll comfortably tracks a fast hand-spin (~250 quarter-steps/s -> ~60 detents/s).
Pots are read **one ADC conversion per tick, round-robin** across active pots, so
with the maximum useful three pots each is sampled every ~3 ms; ample for a fader.
Buttons debounce per **pin group**, not per binding, so sharing costs nothing.

---

## 7. Runtime behavior details

### 7.1 BUSY retry latch (absolute targets, no double-toggle)

Actions resolve to an **absolute target** at event time, never a relative "toggle
again". If a dispatch returns `CTRL_DISPATCH_BUSY` (a USB control SET is
mid-flight, or a shared deferred-apply slot is owed), the engine latches the
resolved target and retries it on the next tick, before sampling new events.
Because the target is absolute, a retry can never double-apply (e.g. a toggle
cannot flip twice; a `MOMENTARY` release restores exactly the captured value).
Rapid encoder detents accumulate correctly across a BUSY stall: the base for
the next step is the latched pending target, not the stale live value.

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

### 7.4 Concurrent writers

A physical filter knob and a host editing the same filter both funnel through
the shared EQ handler; the last write wins per full packet. The engine's RMW
reads the live recipe at event time and its BUSY guard prevents it from
overwriting a not-yet-applied host write, but interleaved host and knob edits
of two different fields of the same band within one apply window resolve to
the later packet. This matches the existing multi-host behavior of the EQ
command surface.

---

## 8. App integration patterns

All multi-byte fields little-endian. `slot` is 0-15. Field offsets per 2.2:
`type,noun,action,flags @0-3`, `gpio @4-5`, `event,target,index,rsv @6-9`,
`value @10`, `step @12`, `range_min @14`, `range_max @16`, `reserved2 @18-23`.

### 8.1 Enumerate capabilities (do this at connect)

1. `GET 0x86, wValue=0xFFFF` -> 32-byte `CsCapsHeader`. Read `type_count`,
   `noun_count`, `max_bindings`, and the seven `CsTypeDesc` entries.
2. For each noun `n` in `0 .. noun_count-1`: `GET 0x86, wValue=n` -> 12-byte
   `CsNounDesc`. Cache kind, unit, enum_count, range, target kind/count,
   dflags, and the accepted-action mask. Skip nouns with `actions == 0`.
3. Build the picker UI: for each type, offer the nouns whose action mask
   intersects the type's action mask; offer only the intersecting actions.
   For targeted nouns, offer a channel (and band) picker sized by
   `target_count` (and the device's band layout).

### 8.2a Rotary encoder on GPIO 27/28, master volume, 1 dB/detent, accelerated

`type=ENCODER(4)`, `noun=MASTER_VOLUME(1)`, `action=STEP(1)`,
`flags=CS_FLAG_ACCEL(0x08)`, `gpio={27,28}`, `step=256`.

```
04 01 01 08 1B 1C 00 00 00 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00
```

Send `SET 0x84, wValue=<slot>, payload=<24 bytes>`, then poll `GET 0x87` until
`last_slot == slot` and `last_status != 0x16` (expect `0x00`).

### 8.2b LED on GPIO 20 indicating loudness is on

`type=LED(5)`, `noun=LOUDNESS(3)`, `action=IND_EQUALS(8)`, `gpio={20,0xFF}`,
`value=1`.

```
05 03 08 00 14 FF 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2c One button, three functions (GPIO 16): mute / next preset / input cycle

Three bindings sharing GPIO 16, distinguished by event:

Slot A, short press toggles user mute (`event=PRESS`):
```
01 02 04 00 10 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```
Slot B, long press steps to the next occupied preset with wrap
(`event=LONG(1)`, `action=INC(2)`, `flags=WRAP(0x04)`):
```
01 06 02 04 10 FF 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```
Slot C, double press cycles the input source
(`event=DOUBLE(2)`, `action=INC(2)`, `flags=WRAP(0x04)`, `noun=INPUT_SOURCE(7)`):
```
01 07 02 04 10 FF 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2d Sub-crossover frequency pot on GPIO 26, 40..200 Hz

RP2350 example: sub output is DSP channel 12 (8 inputs + output index 4),
crossover band 20. `type=POT(3)`, `noun=FILTER_FREQ(20=0x14)`,
`action=ADJUST(0)`, `target=12(0x0C)`, `index=20(0x14)`,
`range_min=40(0x0028)`, `range_max=200(0x00C8)`.

```
03 14 00 00 1A FF 00 0C 14 00 00 00 00 00 28 00 C8 00 00 00 00 00 00 00
```

### 8.2e PWM meter LED on GPIO 21 following output 1's level

RP2350 example: output 1 is DSP channel 8. `type=LED_PWM(6)`,
`noun=LEVEL(28=0x1C)`, `action=IND_LEVEL(11=0x0B)`, `target=8`.

```
06 1C 0B 00 15 FF 00 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
```

### 8.2f Hold-to-test: momentary siggen button on GPIO 17

`type=BUTTON(1)`, `noun=SIGGEN(25=0x19)`, `action=MOMENTARY(9)`, `value=1`.

```
01 19 09 00 11 FF 00 00 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00
```

Configure the signal first (`REQ_SIGGEN_SET_CONFIG`); the button starts and
stops whatever config is applied.

### 8.3 Clear a binding

Send a binding with `type = 0` (`CS_TYPE_NONE`) to the slot; the rest of the
bytes are ignored. This releases the slot's pins and marks it idle.

```
SET 0x84, wValue=<slot>, payload = 24 x 00
```

`last_status` returns `PIN_CONFIG_SUCCESS`.

### 8.4 Read back and display live status

- `GET 0x85, wValue=<slot>` -> the live 24-byte `CsBinding` for editing/display.
- `GET 0x87` -> `CsStatusPacket`: `active_mask` (uint16) tells you which slots
  are live; `slot_status[slot]` gives per-slot health (0 = ok, else a failure
  code from section 3.3, e.g. a boot pin collision).
- To reflect live control activity (a knob being turned), subscribe to the
  notification stream and watch for `PARAM_CHANGED` events with
  `source == PARAM_SRC_GPIO` (5).

---

## 9. Extension guide (firmware)

The engine (control_surfaces.c) is noun-agnostic; parameter-specific code lives
in control_surfaces_nouns.c. Hosts read caps at runtime, so adding capability
does not break existing hosts.

### 9.1 Add a new noun

1. Append a value to `CsNoun` (never renumber; `CS_NOUN_COUNT` grows).
2. Add a `CsNounDesc` row in `cs_noun_table[]` (kind, unit, action mask, range,
   target kind/count, dflags) in control_surfaces_nouns.c.
3. Add a `case` in `cs_noun_get()` (live read, natural units) and in
   `cs_noun_dispatch()` (map to the underlying vendor command). For a
   deferred/pipeline-safe apply, dispatch through `vendor_dispatch_get` like
   `PRESET` does, and set `CS_NDF_DEFERRED` so the engine uses a target shadow.
   If the apply shares a pending slot with other commands (like the EQ packet),
   return `false` (BUSY) while that slot is owed.
4. If the noun is targeted, extend `cs_noun_validate_target()` if the generic
   kinds do not cover it.
5. Document it in section 5 here.

### 9.2 Add a new action

1. Append to `CsAction` (never renumber; `CS_ACT_COUNT` grows). Bit position must
   stay `< 16` (the masks are `uint16`).
2. Add the bit to the relevant type masks (`s_caps.types[]` in
   control_surfaces.c) and the noun action groups (control_surfaces_nouns.c).
3. Implement the behavior in the tick/gesture handlers and any value-bounds
   check in `cs_validate()`.

### 9.3 Add a new component type (e.g. a character display)

1. Append to `CsType` (never renumber; `CS_TYPE_COUNT` grows). Add a `CsTypeDesc`
   row: its action mask, pin count, and pin class.
2. Handle it in `cs_claim_pins`, `cs_release_pins`, `cs_seed_runtime`, and the
   `control_surfaces_tick()` dispatch switch.
3. No wire-format change is needed: `CsBinding` already carries
   type/pins/event/target/value, and hosts pick up the new type from the caps
   header automatically.

### 9.4 Versioning rules

- **Append-only** enum values; **never renumber** an existing `CsType`, `CsNoun`,
  `CsAction`, or `CsEvent` (they are wire- and flash-persistent).
- Bump `CS_CONFIG_VERSION` **only** when the `CsFlashConfig` byte layout changes
  (adding an enum value does not). A stored blob with a higher version than the
  firmware understands is ignored (feature stays idle) rather than misread.
- Bump `caps_version` when the descriptor semantics change so hosts can adapt.
- A `CsFlashConfig` **layout** change also needs a directory version bump: add a
  new `PresetDirectory_vN` snapshot, extend `load_directory()` with an
  N-1 -> N migration, bump `DIR_VERSION_CURRENT`, and extend
  `dir_sanitize_cs_config()` if new fields need bounds checks (see
  flash_storage.c for the V8 -> V9 pattern, which is exactly this feature's
  v1 -> v2 config migration).

---

## 10. Limits and constraints

- **16 bindings** (`CS_MAX_BINDINGS`). Slots are independent; a slot holds one
  component (or one gesture of a shared button).
- **Pin conflicts** use the shared `ctrl_iface_check_pin()`: a pin must be a valid
  GPIO, not already claimed by an output, MCK, SPDIF/I2S RX, ADAT, DAC
  hardware-mute, a UART/I2C control interface, or another live binding (the one
  sanctioned overlap is button-to-button sharing per 6.1). The **I2S clock pair
  is always treated as reserved**, matching the other control-interface pin
  checks. `control_surfaces_owns_pin()` is wired into
  `pin_used_by_fixed_peripheral()`, so no other subsystem can take a live CS pin.
- **One ADC conversion per tick** (round-robin), so pots share the single ADC
  fairly without stalling the 1 kHz loop.
- **No PIO and no GPIO IRQs** are used; the engine is pure main-loop polling. It
  is a cheap no-op when no binding is active. PWM LEDs use the otherwise unused
  hardware PWM slices.
- **Indicator evaluation is decimated** to 8 ms and staggered across slots, so
  many LEDs (including status-struct reads and the meter dB conversion) stay
  cheap on the RP2040's software float.
- **RP2040 XIP placement**: control_surfaces.c and control_surfaces_nouns.c
  execute from flash (XIP); they contain no `DSP_TIME_CRITICAL` code and never
  run inside the IRQs-off flash-write window.
- **Platform differences** are carried entirely by the caps tables:
  `target_count` reflects the platform's channel counts, and `ADAT_ACTIVE`
  has an empty action mask on RP2040. Everything else is identical on RP2040
  and RP2350.
- **Output-slot alignment is unaffected.** Every apply goes through the existing,
  known-safe vendor handlers (e.g. preset load's deferred pipeline-safe path), so
  the inviolable inter-slot phase-alignment guarantee is preserved; the Control
  Surfaces engine adds no new audio-path or reset behavior of its own.

---

## 11. v1 -> v2 compatibility

- **Stored configs migrate automatically.** A device with a V8 directory (v1
  16-byte bindings, 8 slots) migrates on first boot: the 8 bindings carry
  over with `event = PRESS`, `target = index = 0`; slots 8-15 are empty. All
  v1 nouns, actions, flags, and status codes kept their numeric values.
  A V9 directory migrates to V10 by appending the (empty) name block; the
  bindings are untouched.
- **The control wire format is NOT backward compatible.** `REQ_SET_CS_BINDING`
  requires 24 bytes (a 16-byte payload gets `CS_STATUS_INVALID_VALUE`);
  `REQ_GET_CS_BINDING`, `REQ_GET_CS_CAPS`, and `REQ_GET_CS_STATUS` responses
  grew. Hosts must read `caps_version` (2) and use the v2 sizes.
- v1's `IND_EQUALS`-only LED bindings behave identically after migration; the
  `CLIP` noun's semantics are unchanged.
