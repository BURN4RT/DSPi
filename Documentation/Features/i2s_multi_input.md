# DSPi Multichannel I2S Input - App Developer Guide

*Last updated: 2026-06-29*

This document is the single, self-contained reference an app developer needs to
add support for DSPi's I2S input channel modes (2 / 4 / 6 / 8 channels). It
covers the working principles, every relevant vendor command, the unified
channel model, validation rules, persistence, host notifications, and concrete
workflow patterns. No prior knowledge of the firmware internals is assumed.

If you only read one section, read **"Quick start"** then **"App workflow
patterns"**.

---

## 1. What this feature is

DSPi can take its audio from one of three input sources: USB (UAC1), S/PDIF, or
**I2S**. When the source is I2S, the device samples an external I2S transmitter
(typically an ADC) on a shared bit-clock / word-clock pair plus one serial-data
line per stereo pair. This guide is about the **multichannel** I2S modes: 2, 4,
6, or 8 input channels, captured as 1 to 4 stereo pairs that are guaranteed to be
sample-aligned with each other.

Key facts up front:

- **Platform:** multichannel (4/6/8) is **RP2350 only**. RP2040 supports I2S
  input but is **stereo-only** (2 channels / 1 pair).
- **Channel counts:** `2`, `4`, `6`, `8` (i.e. 1 to 4 stereo pairs).
- **Format:** 24-bit audio in 32-bit I2S frames, 44.1 / 48 / 96 kHz; the device
  is the clock master (or follows the I2S output clock if an output slot is I2S;
  see "Clock roles").
- **Pins:** one shared BCK (bit clock) + LRCLK (word clock) pair, plus one
  independently-assignable serial-data GPIO per stereo pair.
- **Unified channel model:** each active input channel (0..7) is a first-class
  DSP channel with its own parametric EQ, preamp, metering, and name; all inputs
  feed an 8 x N routing matrix to the outputs.

---

## 2. Quick start

The minimal sequence to get 8-channel I2S input running (RP2350):

```
1. Set channel count:        REQ_SET_I2S_INPUT_CHANNELS  wValue = 8
2. Assign the 3 extra pins:  REQ_SET_I2S_RX_PIN  wValue = (1<<8)|gpio1
                             REQ_SET_I2S_RX_PIN  wValue = (2<<8)|gpio2
                             REQ_SET_I2S_RX_PIN  wValue = (3<<8)|gpio3
   (pair 0's data pin and the shared BCK pin already have valid defaults)
3. Switch the source to I2S: REQ_SET_INPUT_SOURCE  payload = 2  (INPUT_SOURCE_I2S)
4. (optional) Select rate:   REQ_SET_INPUT_RATE   payload = 48000
5. Route inputs to outputs:  REQ_SET_MATRIX_ROUTE  (per crosspoint)
```

Each step returns a status byte where relevant; check it (see "Validation &
errors"). After step 3 the device emits a `NOTIFY_EVT_INPUT_FORMAT` push so your
UI can relayout to the active channel count.

Recommended order is **count first, then pins** (the default extra pins are
distinct and free, so raising the count never fails on a stale pin), but both
orders work.

---

## 3. Working principles

### 3.1 Clock + data topology

I2S input uses one **shared clock pair** for every stereo pair:

- **BCK** (bit clock) on `i2s_bck_pin`.
- **LRCLK** (word/frame clock) on `i2s_bck_pin + 1` (always BCK+1; not separately
  assignable).
- One **serial-data** line per stereo pair, on `i2s_rx_pin[0..3]`, each
  independently assignable.

```
            +-------------------- BCK  (i2s_bck_pin)
            |  +----------------- LRCLK (i2s_bck_pin + 1)
   external |  |   data0 ------>  pair 0 -> input channels 0,1
   ADC(s)   |  |   data1 ------>  pair 1 -> input channels 2,3
            |  |   data2 ------>  pair 2 -> input channels 4,5
            |  |   data3 ------>  pair 3 -> input channels 6,7
```

All four data lines are clocked by the one BCK/LRCLK pair, so the external
transmitter(s) must share that clock.

### 3.2 Clock roles (who drives BCK/LRCLK)

- **Master role (default):** no output slot is configured as I2S, so the device
  generates BCK/LRCLK and the external ADC slaves to them. This is the normal
  case for an ADC front-end.
- **Slave role:** at least one output slot is I2S, so the I2S *output* clock
  master drives BCK/LRCLK and the input samples against those same pads.

You do not select the role; the firmware derives it from the output
configuration. It matters only in that BCK/LRCLK are shared between I2S input
and I2S output.

### 3.3 Sample alignment guarantee

When you select 4/6/8 channels, the pairs are started on a single
synchronized cycle and advance in lockstep on the shared clock, so the
2/4/6/8 channels are **sample-aligned** (no inter-channel skew). This is a hard
guarantee of the firmware; you can rely on phase coherence across all input
channels for multichannel imaging, sub integration, and measurement.

### 3.4 Unified channel model

DSPi treats input channels as first-class DSP channels:

- Channel indices `0..NUM_INPUT_CHANNELS-1` are **inputs** (0..7 on RP2350,
  0..1 on RP2040).
- Channel indices `CH_OUT_1..NUM_CHANNELS-1` are **outputs**
  (`CH_OUT_1 == NUM_INPUT_CHANNELS`).
- Each input channel has its own **parametric EQ** (up to `MAX_BANDS` = 12
  bands), **preamp gain**, **peak meter**, and **name**.
- All active inputs feed an **N x M matrix mixer** (8 x 9 on RP2350) whose
  crosspoints route any input to any output with gain + polarity.

So "4-channel I2S input" means input channels 0..3 are live, each independently
processable, then mixed to the outputs through the matrix.

### 3.5 What is bypassed in multichannel mode

Some DSP blocks are inherently stereo and are **bypassed when more than 2 input
channels are active** (whether the multichannel source is I2S or USB):

- **Loudness compensation**
- **Volume leveller**
- **Crossfeed**

In multichannel mode each input gets only its own per-channel PEQ + preamp, then
the matrix. Per-output PEQ, crossover, gain, and delay still apply normally on
the output side. If your app exposes loudness/leveller/crossfeed, grey them out
(or mark them inactive) whenever the active input count is greater than 2.

---

## 4. Platform support

| Capability | RP2040 | RP2350 |
|---|---|---|
| I2S input | Yes (stereo) | Yes |
| I2S input channels | 2 only | 2 / 4 / 6 / 8 |
| Max stereo pairs (`I2S_RX_MAX_PAIRS`) | 1 | 4 |
| Input channels (`NUM_INPUT_CHANNELS`) | 2 | 8 |
| Output channels (`NUM_OUTPUT_CHANNELS`) | 5 (4 S/PDIF + PDM) | 9 (8 S/PDIF + PDM) |

On RP2040, `REQ_SET_I2S_INPUT_CHANNELS` accepts only `2`; `4/6/8` are rejected.
`REQ_SET_I2S_RX_PIN` accepts only pair `0`. Write your app to read the supported
maximum from the device rather than hard-coding 8 (see "Capability detection").

---

## 5. Hardware / wiring

- Wire the external transmitter's BCK to `i2s_bck_pin` and its LRCLK to
  `i2s_bck_pin + 1`.
- Wire each ADC's serial-data output to the GPIO you assign for that pair
  (`i2s_rx_pin[0..3]`).
- Standard I2S framing: 24-bit data, MSB first, 1-bit delay, left channel when
  LRCLK is low (matches the device's I2S output framing).
- The data pins are inputs only; the device never drives them.

**Defaults (firmware power-on / factory):**

| Item | Default GPIO |
|---|---|
| BCK (`i2s_bck_pin`) | 14 (LRCLK = 15) |
| Pair 0 data (`i2s_rx_pin[0]`) | 4 |
| Pair 1 data (`i2s_rx_pin[1]`) | 16 |
| Pair 2 data (`i2s_rx_pin[2]`) | 17 |
| Pair 3 data (`i2s_rx_pin[3]`) | 18 |
| Channel count | 2 |
| Rate | 48000 |

The pair 1..3 defaults (16/17/18) are placeholders. Multichannel input requires
wiring one ADC data line per pair, so always let the user assign the extra data
pins to match their board, then save a preset.

---

## 6. Transport: how to send a command

All commands are **USB control transfers on the vendor interface**:

- `bmRequestType`: vendor type (bits 6:5 = `10`). Direction bit 7 = `1` (IN) for
  GET / commands that return data, `0` (OUT) for SET.
- `bRequest`: the opcode (e.g. `0xF3`).
- `wValue`: per-command (see each command below).
- `wIndex`: vendor interface number (the firmware routes all vendor-type control
  transfers regardless of `wIndex`).
- Data stage: the payload (SET) or the response (GET).

In `pyusb` terms:

```python
# SET (host -> device), OUT
dev.ctrl_transfer(0x40, bRequest, wValue, wIndex, payload_bytes)
# GET (device -> host), IN, request `length` bytes
resp = dev.ctrl_transfer(0xC0, bRequest, wValue, wIndex, length)
```

`0x40` = OUT|vendor|device, `0xC0` = IN|vendor|device. Use the recipient your
existing DSPi integration already uses; the device does not filter on recipient.

---

## 7. Vendor command reference (I2S-input relevant)

Status-returning SET commands return a single byte from the `PIN_CONFIG_*` set
(see "Validation & errors"). `0x00` = success.

### 7.1 Channel count

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_I2S_INPUT_CHANNELS` | `0xF3` | channel count `2/4/6/8` | returns 1 status byte |
| `REQ_GET_I2S_INPUT_CHANNELS` | `0xF4` | 0 | returns 1 byte: active count |

Setting the count (de)allocates the stereo pairs and, if I2S is the active
source, restarts the input so all pairs re-sync. A count *increase* validates
each newly-activated pair's data pin first (valid GPIO, no peripheral / clock /
cross-pair conflict) and is **rejected** if any clashes; fix the offending pin
then retry. RP2040 accepts only `2`.

### 7.2 Per-pair data pins

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_I2S_RX_PIN` | `0xF1` | `(pair << 8) | gpio` | returns 1 status byte |
| `REQ_GET_I2S_RX_PIN` | `0xF2` | `pair` (0..3) | returns 1 byte: that pair's GPIO |

`pair` is the stereo-pair index 0..3 (high byte of `wValue`); the low byte is the
GPIO number. A bare `wValue = gpio` (high byte 0) addresses pair 0, which keeps
older two-channel hosts working unchanged.

The firmware keeps all four data pins mutually distinct and clear of the clocks:
a SET is rejected if the GPIO is invalid, is the BCK or LRCLK pin, is already
used by another peripheral (an output, MCK, the S/PDIF RX pin, or the DAC-mute
pin), or is already another I2S pair's data pin. Pair 0 alone (stereo) hot-swaps
its pin; any higher pair, or a multichannel config, restarts the input so every
pair re-syncs.

### 7.3 BCK (clock) pin

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_I2S_BCK_PIN` | `0xC2` | new BCK GPIO | returns 1 status byte |
| `REQ_GET_I2S_BCK_PIN` | `0xC3` | 0 | returns 1 byte: BCK GPIO |

LRCLK is always BCK+1, so both BCK and BCK+1 must be valid GPIOs. The SET is
rejected while an I2S *output* is active (`PIN_CONFIG_OUTPUT_ACTIVE`) and on a
peripheral conflict. Changing BCK affects both I2S input and I2S output (shared
clock).

### 7.4 Input source

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_INPUT_SOURCE` | `0xE0` | 0 | payload: 1 byte `InputSource` |
| `REQ_GET_INPUT_SOURCE` | `0xE1` | 0 | returns 1 byte `InputSource` |

`InputSource`: `0` = USB, `1` = S/PDIF, `2` = I2S. Switching to I2S brings up the
input on the configured pins/count/rate and runs a glitch-free prefill handshake
before unmuting.

### 7.5 Input rate

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_INPUT_RATE` | `0xED` | 0 | payload: `uint32` Hz (44100 / 48000 / 96000) |
| `REQ_GET_INPUT_RATE` | `0xEE` | 0 | returns 2 x `uint32`: {current Hz, selected I2S Hz} |

In I2S mode the device is the rate authority; set the rate to match your ADC.
The change brackets the input cleanly (mute, divider update, re-sync).

### 7.6 Status + metering

| Command | Opcode | wValue | Response |
|---|---|---|---|
| `REQ_GET_STATUS` | `0x50` | `9` | combined status packet (see below) |
| `REQ_GET_STATUS` | `0x50` | `23` | `uint32`: live active input channel count |

The **combined packet** (`wValue = 9`) is the one to poll for meters + the live
input count in a single round-trip. Layout:

```
offset 0 .. NUM_CHANNELS*2-1 : peaks[NUM_CHANNELS], uint16 LE each
                               (inputs are indices 0..NUM_INPUT_CHANNELS-1)
+0  : cpu0_load   (uint8)
+1  : cpu1_load   (uint8)
+2..+5 : clip_flags (uint32 LE)
+6  : active_input_channels (uint8)   <-- 2/4/6/8, source-aware
```

Total length: RP2350 = 17*2 + 7 = 41 bytes; RP2040 = 7*2 + 7 = 21 bytes.

`active_input_channels` (the `+6` byte, and the scalar `wValue == 23`) is
**source-aware**: it reports the I2S count when I2S is the active source, the USB
alt count for USB, and 2 for S/PDIF. Use it to drive your channel-strip layout;
do not assume it equals the I2S count unless I2S is the active source.

Input peak meters are `peaks[0 .. active_input_channels-1]`.

### 7.7 Per-input preamp

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_PREAMP_CH` | `0xD0` | channel index (0..7) | payload: `float` dB |
| `REQ_GET_PREAMP_CH` | `0xD1` | channel index | returns `float` dB |

Per-input-channel preamp gain, applied at capture before EQ/matrix. Use this to
trim individual ADC inputs. (Legacy `REQ_SET_PREAMP` / `0x44` sets all channels;
`REQ_GET_PREAMP` / `0x45` returns channel 0.)

### 7.8 Per-input parametric EQ

| Command | Opcode | Payload (`EqParamPacket`, 16 bytes) |
|---|---|---|
| `REQ_SET_EQ_PARAM` | `0x42` | see struct |
| `REQ_GET_EQ_PARAM` | `0x43` | wValue selects channel/band; returns the packet |

```c
typedef struct __attribute__((packed)) {
    uint8_t channel;   // 0..7 = inputs (unified model)
    uint8_t band;      // 0..(per-channel PEQ count-1) for PEQ
    uint8_t type;      // FilterType (see below)
    uint8_t bypass;    // exactly 1 = bypassed; anything else = active
    float   freq;      // Hz
    float   Q;
    float   gain_db;
} EqParamPacket;       // 16 bytes
```

`FilterType`: `0` FLAT, `1` PEAKING, `2` LOWSHELF, `3` HIGHSHELF, `4` LOWPASS,
`5` HIGHPASS, `6` NOTCH, `7` ALLPASS, `8` ALLPASS1 (first-order), `9` LOWSHELF1,
`10` HIGHSHELF1. (Crossover band types are output-channel-only; see the crossover
spec.) PEQ on input channels works exactly as on any channel; just set
`channel` to the input index 0..7.

### 7.9 Matrix routing (inputs -> outputs)

| Command | Opcode | Payload (`MatrixRoutePacket`, 12 bytes) |
|---|---|---|
| `REQ_SET_MATRIX_ROUTE` | `0x70` | see struct |
| `REQ_GET_MATRIX_ROUTE` | `0x71` | returns crosspoint state |

```c
typedef struct __attribute__((packed)) {
    uint8_t input;        // 0..NUM_INPUT_CHANNELS-1 (0..7 on RP2350)
    uint8_t output;       // 0..NUM_OUTPUT_CHANNELS-1 (0..8 on RP2350)
    uint8_t enabled;      // 0 or 1
    uint8_t phase_invert; // 0 or 1
    float   gain_db;      // up to +12 dB
} MatrixRoutePacket;      // 12 bytes
```

The matrix is how multichannel input reaches the outputs. With 8 inputs you have
an 8 x 9 grid of crosspoints; enable the ones you need (e.g. input 0 -> S/PDIF 1
L, input 2 -> S/PDIF 2 L, etc.). Output indices: `0..7` are the 4 S/PDIF stereo
slots (L,R interleaved), `8` is PDM (RP2350).

### 7.10 Channel names

| Command | Opcode | wValue | Payload / Response |
|---|---|---|---|
| `REQ_SET_CHANNEL_NAME` | `0x9B` | channel index | 1..32 byte UTF-8 name |
| `REQ_GET_CHANNEL_NAME` | `0x9C` | channel index | returns 32 bytes |

Default input names are source-aware: with I2S active the inputs default to
`"I2S L"`, `"I2S R"`, `"I2S 3"` ... `"I2S 8"`. They switch automatically when the
source changes (only names you have not customized). Custom names persist with
the preset.

### 7.11 Bulk parameter transfer (snapshot / restore)

| Command | Opcode | Notes |
|---|---|---|
| `REQ_GET_ALL_PARAMS` | `0xA0` | read the entire DSP state in one control transfer |
| `REQ_SET_ALL_PARAMS` | `0xA1` | write the entire DSP state in one control transfer |

The bulk `WireInputConfig` section carries the I2S input config:
`i2s_rx_pin` (pair 0), `i2s_input_rate`, `i2s_input_channels`, and
`i2s_rx_pin_ext[3]` (pairs 1..3). Per-field `0 = absent / keep the live value`.
On restore, the I2S pin set + BCK are validated as a unit and an inconsistent
config is rejected rather than applied (the live config is retained), so a
round-tripped or hand-built bulk blob can never bring two state machines up on
one GPIO or on a clock pin. Prefer the dedicated commands above for interactive
edits and bulk only for full snapshot/restore.

---

## 8. Validation & errors

Status-returning SET commands return one byte:

| Value | Name | Meaning |
|---|---|---|
| `0x00` | `PIN_CONFIG_SUCCESS` | applied |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | not a usable GPIO, or an invalid channel count |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | conflicts with another peripheral / clock / I2S pair |
| `0x03` | `PIN_CONFIG_INVALID_OUTPUT` | no such pair, or count exceeds the platform max |
| `0x04` | `PIN_CONFIG_OUTPUT_ACTIVE` | BCK change rejected while an I2S output is active |

Pin rules the firmware enforces (so your app does not have to, but should mirror
for good UX):

- A GPIO is invalid if it is reserved (UART/power/LED) or out of range for the
  platform.
- An I2S data pin may not equal the BCK or LRCLK pin (checked even while I2S is
  inactive, because they coexist once the input runs).
- The four data pins must be mutually distinct.
- A data pin may not equal an output pin, MCK, the S/PDIF RX pin, or the DAC-mute
  pin that is currently in use.
- Raising the channel count validates each newly-activated pair's pin and rejects
  the increase if any clashes.

Recommended UX: when a SET returns non-zero, surface the reason and keep your UI
in sync by issuing the matching GET (the device did not change state on a
rejected SET).

---

## 9. Persistence

The I2S input config participates in DSPi's normal persistence:

- **Per preset (flash):** channel count, all four data pins, BCK pin, rate, and
  every per-input PEQ/preamp/name/matrix crosspoint are captured when you save a
  preset and restored on load. Saving a preset is how a user makes a wired
  multichannel setup permanent.
- **Device-global (flash directory):** in INDEPENDENT output-config mode the
  physical IO (pins, BCK, count, rate) is stored once and applied at boot.
- **Bulk (`WireInputConfig`):** for full host-driven snapshot/restore.

You do not need to manage versions; just use `REQ_PRESET_SAVE` / load and the
bulk commands. Restores are validated (see 7.11) so a stale or cross-device
config degrades gracefully (the bad fields are ignored) rather than producing a
broken or clashing setup.

---

## 10. Host notifications (push)

DSPi pushes asynchronous events so your UI can stay live without polling. The
ones relevant here:

| Event | ID | When it fires |
|---|---|---|
| `NOTIFY_EVT_INPUT_FORMAT` | `0x05` | active input channel count changes |
| `NOTIFY_EVT_PARAM_CHANGED` | `0x02` | any single parameter changed (offset + bytes) |

`NOTIFY_EVT_INPUT_FORMAT` fires whenever the **active** count changes: a USB alt
change, an **input-source switch** (e.g. into I2S), and a **live I2S channel-count
change**. Its payload carries the new count. Treat it as "relayout your channel
strips now"; re-read the count (or take it from the event) and rebuild the view.

`NOTIFY_EVT_PARAM_CHANGED` carries a wire offset + value for the field that
changed (e.g. an I2S pin, the count, a crosspoint). Use it to keep multiple
clients or views consistent. The dedicated SETs above also emit these so a
second app sees your edits.

If your app does not subscribe to notifications, polling `REQ_GET_STATUS`
(`wValue = 9`) for meters already includes the live count, so you will pick up
changes on the next poll.

---

## 11. App workflow patterns

### 11.1 Capability detection (do this first)

Do not hard-code 8 channels. Probe what the device supports rather than assuming:

```
# Attempt the count you want; the status byte tells you if the part supports it.
status = SET_I2S_INPUT_CHANNELS(8)
if status == PIN_CONFIG_INVALID_OUTPUT:
    # stereo-only part (RP2040): cap the UI at 2 channels / 1 pair
    max_channels = 2
elif status == PIN_CONFIG_SUCCESS:
    max_channels = 8
```

`PIN_CONFIG_INVALID_OUTPUT` from a count SET means "this part cannot do that many
pairs"; treat it as the capability signal. (You can also distinguish the model
from the USB product string / descriptor if you prefer not to issue a probing
SET.)

### 11.2 Enable a multichannel I2S setup

```
# 1. Pick the channel count
status = SET_I2S_INPUT_CHANNELS(count)         # 2/4/6/8
assert status == SUCCESS

# 2. Assign data pins for the active pairs (pair 0 + (count/2 - 1) extras)
for pair in range(count // 2):
    status = SET_I2S_RX_PIN((pair << 8) | gpio_for[pair])
    if status != SUCCESS: show_error(pair, status); abort()

# 3. (optional) BCK pin + rate
SET_I2S_BCK_PIN(bck_gpio)                       # only if changing from default
SET_INPUT_RATE(48000)

# 4. Make I2S the active source
SET_INPUT_SOURCE(2)                             # INPUT_SOURCE_I2S
# -> NOTIFY_EVT_INPUT_FORMAT arrives; relayout to `count` strips

# 5. Route inputs to outputs
for (inp, outp, gain) in routing:
    SET_MATRIX_ROUTE(inp, outp, enabled=1, gain_db=gain)

# 6. Persist
REQ_PRESET_SAVE(slot)
```

### 11.3 Relayout on the fly

```
on NOTIFY_EVT_INPUT_FORMAT(count):
    rebuild_channel_strips(count)          # show inputs 0..count-1
    grey_out_stereo_only_fx = (count > 2)  # loudness/leveller/crossfeed
# (or poll REQ_GET_STATUS wValue=9 and read the +6 byte)
```

### 11.4 Per-input processing

```
# Trim input 2 by -3 dB, add a 1 kHz bell on input 2 band 0:
SET_PREAMP_CH(channel=2, dB=-3.0)
SET_EQ_PARAM(channel=2, band=0, type=PEAKING, freq=1000, Q=1.0, gain_db=+4)
# Name it:
SET_CHANNEL_NAME(channel=2, "Sub L")
```

### 11.5 Meter the inputs

```
loop:
    packet = REQ_GET_STATUS(wValue=9)
    count  = packet[NUM_CHANNELS*2 + 6]
    for ch in range(count):
        peak = u16le(packet, ch*2)
        draw_meter(ch, peak)
```

### 11.6 Changing count live

You can change the count while I2S is the active source; the device restarts the
input transparently and re-syncs. Just issue `SET_I2S_INPUT_CHANNELS(new)` and
relayout on the resulting `NOTIFY_EVT_INPUT_FORMAT`. A *raise* whose new pins
clash is rejected (fix the pins first); a *lower* always succeeds.

---

## 12. Gotchas and edge cases

- **RP2040 is stereo-only.** Cap your UI; `4/6/8` and pairs `1..3` are rejected.
- **Extra data pins need real wiring.** The 16/17/18 defaults are placeholders;
  assign them to your board before enabling 4/6/8, then save a preset.
- **Order tip.** Set the count first, then the pins. The defaults are distinct
  and free, so a raise never trips on a stale pin; if you set pins first while
  the count is still 2, the extra pairs are inactive and not yet validated
  against peripherals until you raise the count.
- **Stereo-only FX bypass.** Loudness, leveller, and crossfeed are inactive
  whenever the active input count is greater than 2 (by design); reflect that in
  the UI.
- **Source-aware count.** `active_input_channels` follows the *active* source.
  After switching away from I2S it reports the new source's count, not the I2S
  count you configured.
- **BCK is shared with I2S output.** Changing BCK is rejected while an I2S output
  is active; the data pins must always avoid BCK and LRCLK.
- **Alignment is guaranteed.** You do not need to (and cannot) phase-align the
  channels yourself; the firmware starts all pairs in sync.
- **A rejected SET does not change state.** Re-issue the matching GET to resync
  your UI after a non-zero status.

---

## 13. Quick reference

### Opcodes

| Opcode | Command | wValue | Data |
|---|---|---|---|
| `0xF3` | SET I2S input channels | 2/4/6/8 | -> status |
| `0xF4` | GET I2S input channels | 0 | <- count (1 B) |
| `0xF1` | SET I2S RX pin | (pair<<8)\|gpio | -> status |
| `0xF2` | GET I2S RX pin | pair | <- gpio (1 B) |
| `0xC2` | SET I2S BCK pin | gpio | -> status |
| `0xC3` | GET I2S BCK pin | 0 | <- gpio (1 B) |
| `0xE0` | SET input source | 0 | -> 1 B (2=I2S) |
| `0xE1` | GET input source | 0 | <- 1 B |
| `0xED` | SET input rate | 0 | -> u32 Hz |
| `0xEE` | GET input rate | 0 | <- 2x u32 |
| `0x50` | GET status | 9 / 23 | <- packet / u32 count |
| `0xD0` | SET preamp (per ch) | channel | -> float dB |
| `0xD1` | GET preamp (per ch) | channel | <- float dB |
| `0x42` | SET EQ param | - | -> EqParamPacket |
| `0x43` | GET EQ param | ch/band | <- EqParamPacket |
| `0x70` | SET matrix route | - | -> MatrixRoutePacket |
| `0x71` | GET matrix route | - | <- crosspoint |
| `0x9B` | SET channel name | channel | -> 1..32 B |
| `0x9C` | GET channel name | channel | <- 32 B |
| `0xA0` | GET all params | 0 | <- bulk blob |
| `0xA1` | SET all params | 0 | -> bulk blob |

### Enums / constants

```
InputSource:   USB=0  SPDIF=1  I2S=2
Channel count: 2 / 4 / 6 / 8   (1..4 stereo pairs)
Pairs:         pair p -> input channels 2p, 2p+1
PIN_CONFIG:    SUCCESS=0  INVALID_PIN=1  PIN_IN_USE=2  INVALID_OUTPUT=3  OUTPUT_ACTIVE=4
NOTIFY_EVT:    PARAM_CHANGED=0x02  INPUT_FORMAT=0x05
RP2350:        NUM_INPUT_CHANNELS=8  NUM_OUTPUT_CHANNELS=9  pairs<=4
RP2040:        NUM_INPUT_CHANNELS=2  NUM_OUTPUT_CHANNELS=5  pairs=1
```

---

*See also: `current_architecture.md` (I2S Input section) for firmware internals,
and the per-channel preamp, crossover, interrupt-interface, and master-volume
specs for the shared subsystems referenced here.*
