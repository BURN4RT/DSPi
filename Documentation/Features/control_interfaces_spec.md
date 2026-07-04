# External Control Interfaces (UART and I2C Target)

*Firmware wire-protocol version: 1*

This document is the complete, self-contained specification for driving a DSPi
from an external microcontroller over UART or over the I2C target (slave)
interface. It is written for a firmware integrator building an ESP32, STM32,
Arduino, or SBC controller; no DSPi source is required to implement a client.

Writing style note: this doc avoids em-dashes per project convention.

---

## 1. Overview

### 1.1 One command surface, three transports

DSPi exposes a single vendor-command surface. Historically it was reachable only
over USB (EP0 vendor control transfers). It is now also reachable over two
external transports through a transport-neutral orchestrator inside the firmware:

- **USB** (`CTRL_SOURCE_USB`): the original EP0 path.
- **UART** (`CTRL_SOURCE_UART`): an asynchronous serial link, 3.3V logic.
- **I2C target** (`CTRL_SOURCE_I2C`): the device acts as an I2C slave; an
  external controller is the bus master.

Every transport parses its own wire framing into the same
`bRequest` / `wValue` / `wIndex` / `wLength` shape and hands it to the same
dispatcher (`vendor_dispatch_get` / `vendor_dispatch_set`). The consequence that
matters most for integrators:

> **Full parity.** Every vendor command works identically over USB, UART, and
> I2C: EQ, matrix routing, presets, master volume, input selection, the bulk
> get/set of all parameters (`0xA0` / `0xA1`), and even bootloader entry
> (`0xF0`). Any command added to the firmware in the future is automatically
> available on all three transports with nothing per-transport to implement.

The full opcode catalogue lives in `Documentation/commands.md`. This spec covers
only the framing, status, and behavior that are specific to the external
transports.

### 1.2 The one exception: USB-only self-configuration

Two commands are refused on the external transports:

| Command | Code | Meaning |
|---------|------|---------|
| `REQ_SET_UART_CONFIG` | `0xF5` | Configure the UART control interface |
| `REQ_SET_I2C_CONFIG`  | `0xF7` | Configure the I2C control interface |

An attempt to send either of these over UART or I2C is rejected with a
`BLOCKED` status (never applied). This is deliberate: an external controller can
read the interface config (`0xF6` / `0xF8`) and status (`0xF9`), but it can never
reconfigure, move the pins of, change the baud/address of, or disable the very
transport it is talking on. It cannot lock itself out. Interface configuration
is a one-time setup step performed over USB.

Everything else, including the two GET-config commands and the status readback,
is available on every transport.

### 1.3 Ship state

Both interfaces ship **disabled** by default. A fresh device (or one after a
factory reset) has no UART or I2C control activity and holds none of their pins.
A one-time USB setup enables and pins each interface; the choice persists across
reboots and survives factory reset.

---

## 2. Configuration over USB

Configuration is done with five vendor commands. All five are reachable over
USB; only the two GETs and the status readback are reachable over UART/I2C.

| Command | Code | Dir | Payload / Response |
|---------|------|-----|--------------------|
| `REQ_SET_UART_CONFIG`       | `0xF5` | OUT | 8-byte `UartCtrlConfig` (USB only) |
| `REQ_GET_UART_CONFIG`       | `0xF6` | IN  | 8-byte `UartCtrlConfig` (live) |
| `REQ_SET_I2C_CONFIG`        | `0xF7` | OUT | 8-byte `I2cCtrlConfig` (USB only) |
| `REQ_GET_I2C_CONFIG`        | `0xF8` | IN  | 8-byte `I2cCtrlConfig` (live) |
| `REQ_GET_CTRL_IFACE_STATUS` | `0xF9` | IN  | 8-byte `CtrlIfaceStatus` |

### 2.1 Structures

All fields little-endian, packed, no padding.

```c
// UART control interface configuration (8 bytes).  Framing is fixed 8N1
// (the wire CRC16 covers integrity, so parity adds nothing); only the baud
// rate is configurable.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;   // 0 = off, 1 = on
    uint8_t  tx_pin;    // GPIO carrying a UARTx TX mux; pin % 4 == 0
    uint8_t  rx_pin;    // GPIO carrying a UARTx RX mux; pin % 4 == 1, same instance
    uint8_t  reserved;  // 0
    uint32_t baud;      // 9600 .. 1000000
} UartCtrlConfig;

// I2C target control interface configuration (8 bytes).  Bus speed and
// clock-stretch behavior are controller-side properties for a target and
// are not stored here.
typedef struct __attribute__((packed)) {
    uint8_t  enabled;   // 0 = off, 1 = on
    uint8_t  sda_pin;   // GPIO carrying an I2Cx SDA mux; pin % 2 == 0
    uint8_t  scl_pin;   // GPIO carrying an I2Cx SCL mux; pin % 2 == 1, same instance
    uint8_t  address;   // 7-bit target address, 0x08 .. 0x77
    uint8_t  reserved[4];
} I2cCtrlConfig;

// REQ_GET_CTRL_IFACE_STATUS response (8 bytes).
typedef struct __attribute__((packed)) {
    uint8_t uart_last_status; // PIN_CONFIG_* from the last REQ_SET_UART_CONFIG
    uint8_t uart_live;        // 1 if the UART peripheral is up and listening
    uint8_t i2c_last_status;  // PIN_CONFIG_* from the last REQ_SET_I2C_CONFIG
    uint8_t i2c_live;         // 1 if the I2C target is up and listening
    uint8_t proto_version;    // external wire-protocol version, currently 1
    uint8_t reserved[3];      // 0
} CtrlIfaceStatus;
```

Both interfaces are pin-constrained by the RP2040/RP2350 GPIO mux: a UART TX pin
must be a GPIO whose function-2 mux is that UART instance's TX (`pin % 4 == 0`),
its RX pin must be the same instance's RX (`pin % 4 == 1`); an I2C SDA pin must be
even and its SCL pin the next odd GPIO on the same I2C instance. The firmware
validates this at apply time.

### 2.2 Defaults

Populated on fresh flash, on the V5-to-V6 directory migration, and whenever a
stored config fails a bounds check on load.

| Field | UART default | I2C default |
|-------|--------------|-------------|
| enabled | 0 (off) | 0 (off) |
| TX / SDA pin | GPIO 16 | GPIO 18 |
| RX / SCL pin | GPIO 17 | GPIO 19 |
| baud | 115200 | n/a |
| address | n/a | 0x42 |

Baud range: 9600 .. 1000000. I2C address range: 0x08 .. 0x77 (7-bit).

### 2.3 Status codes returned by the SET commands

`REQ_SET_UART_CONFIG` / `REQ_SET_I2C_CONFIG` validate and apply, returning a
1-byte `PIN_CONFIG_*` code (shared with the S/PDIF, I2S, and MCK pin commands):

| Code | Name | Meaning |
|------|------|---------|
| `0x00` | `PIN_CONFIG_SUCCESS` | Applied and persisted |
| `0x01` | `PIN_CONFIG_INVALID_PIN` | A pin is out of range or lacks the required mux function |
| `0x02` | `PIN_CONFIG_PIN_IN_USE` | A pin is already claimed by an output, RX, MCK, DAC-mute, or the other control interface |
| `0x05` | `PIN_CONFIG_INVALID_PARAM` | A non-pin field is out of range (baud below 9600 / above 1000000, or I2C address outside 0x08..0x77) |

The same code is echoed back in `CtrlIfaceStatus.uart_last_status` /
`i2c_last_status` so a controller (or a later boot) can read the last outcome.

### 2.4 Apply and persist semantics

A `REQ_SET_*_CONFIG` over USB is accepted in the USB ISR but the real work is
**deferred to the main loop**, because it touches GPIO/IRQ state and, on success,
writes flash. The main loop does, in order:

1. **Live apply.** Tear down the current interface first (so its own pins are not
   seen as in-use), then validate the new config and, if valid and enabled, bring
   the peripheral up on the new pins. On a validation failure the previous config
   is restored best-effort and the interface is left as it was.
2. **Persist only on success.** The new config is written to the preset directory
   **only when the live apply returned `PIN_CONFIG_SUCCESS`**. A rejected SET
   never clobbers a good stored config.

Because the persist is a flash write, expect the usual ~45 ms interrupt blackout
(see 5.2). Interface config is device-level, not part of any preset slot and not
part of `WireBulkParams`; the bulk wire-format version is unchanged by this
feature.

### 2.5 Notification to the USB host

Configuration is done over USB, so the host already knows it happened. Separately,
when a control command arriving over UART or I2C changes a parameter, the device
emits its normal change notification to the USB host (notification endpoint) with
a source tag identifying the transport:

| Source tag | Value |
|------------|-------|
| `PARAM_SRC_UART` | 8 |
| `PARAM_SRC_I2C`  | 9 |

A USB host monitoring the notification endpoint therefore sees changes made by an
external controller, correctly attributed.

### 2.6 Boot behavior

At boot, after the preset directory is loaded, the firmware reads the persisted
`UartCtrlConfig` / `I2cCtrlConfig` and brings each enabled interface up. Bring-up
happens **after** the output, RX, MCK, and DAC-mute pins are claimed, so a stored
control config whose pins now collide with the current output wiring is quietly
kept **down**: `enabled` stays 1 in the stored config, but the peripheral does not
start and `CtrlIfaceStatus.*_live` reads 0. The host can detect this divergence by
comparing the config's `enabled` against the live flag from `0xF9`.

---

## 3. UART Transport

### 3.1 Electrical and link parameters

- **Logic level:** 3.3V. Do not drive the RX pin above 3.3V; level-shift a 5V host.
- **Framing:** fixed **8N1** (8 data bits, no parity, 1 stop bit). Parity is
  intentionally not offered; the wire CRC16 provides integrity end to end.
- **Baud:** configurable 9600 .. 1000000; default 115200.
- **Direction:** full duplex, but the protocol is strictly request/response;
  the device only transmits in reply to a request (see 3.6 for the reserved
  notification channel).

### 3.2 Frame formats

Every frame begins with the sync byte `0xA5` followed by a type byte. All
multi-byte fields are little-endian. `bReq` is the vendor command ID; `wVal`,
`wIdx`, `wLen` are the 16-bit request fields.

```
Request SET   (host -> device):
  A5 01 bReq wValL wValH wIdxL wIdxH wLenL wLenH  payload[wLen]  crcL crcH

Request GET   (host -> device):
  A5 02 bReq wValL wValH wIdxL wIdxH wLenL wLenH                 crcL crcH
      (no payload; wLen caps the response size, 0 = full/uncapped)

Response SET  (device -> host):
  A5 81 status lenL lenH                                          crcL crcH
      (len is always 0)

Response GET  (device -> host):
  A5 82 status lenL lenH payload[len]                             crcL crcH
      (payload present only when status == OK)
```

Type byte summary:

| Type | Meaning |
|------|---------|
| `0x01` | SET request |
| `0x02` | GET request |
| `0x40` | Reserved for future device-initiated notifications (not implemented) |
| `0x81` | SET response |
| `0x82` | GET response |

Any unrecognized type byte, or bytes seen while not synced, are treated as noise
and dropped; the parser resynchronizes on the next `0xA5`.

`wIdx` is passed straight through to the dispatcher. Over USB it carries the
vendor interface number (2); over UART it is not an interface selector and can be
sent as `0x0000` for every command except those few that genuinely use `wIndex`
as a parameter. `wValue` still carries per-command data (channel, slot, packed
fields) exactly as over USB.

### 3.3 CRC16 details

- **Algorithm:** CRC16-CCITT-FALSE.
- **Polynomial:** `0x1021`.
- **Initial value:** `0xFFFF`.
- **No input reflection, no output reflection, no final XOR.**
- **Coverage:** every byte **after** the sync byte, i.e. the type byte through the
  last payload byte inclusive. The sync byte `0xA5` is **not** included.
- **On the wire:** transmitted little-endian (low byte first, then high byte).
- Accumulated incrementally byte by byte in both directions.

Reference check: CRC16-CCITT-FALSE over the ASCII string `123456789` is `0x29B1`.
Use this to validate a client implementation before wiring it to hardware.

### 3.4 Status codes (response byte)

Byte 0 of every SET/GET response payload area (the `status` field) is one of the
shared `CTRL_STATUS_*` codes:

| Code | Name | Meaning / client action |
|------|------|-------------------------|
| `0x00` | `CTRL_STATUS_OK` | Success. On a GET, `len` bytes of payload follow. |
| `0x01` | `CTRL_STATUS_BUSY` | The device could not service the request yet (a USB control SET was in flight). Retry the whole request shortly. |
| `0x02` | `CTRL_STATUS_ERROR` | Unknown command or a bad parameter; the handler rejected it. |
| `0x03` | `CTRL_STATUS_BLOCKED` | USB-only command (`0xF5` / `0xF7`) attempted over this transport. |
| `0x04` | `CTRL_STATUS_BULK_LOCKED` | The shared bulk buffer is owned by another transport. Retry shortly. |
| `0x05` | `CTRL_STATUS_CRC_ERROR` | The request failed CRC16; it was not dispatched. Resend. |
| `0x06` | `CTRL_STATUS_OVERSIZE` | A non-bulk payload exceeded 64 bytes. |
| `0x07` | `CTRL_STATUS_FRAME_ERROR` | The frame was malformed or truncated. |

### 3.5 Timing, timeouts, and the one-request-in-flight rule

- **One request in flight.** Send one request, wait for its response, then send
  the next. The device parses, dispatches from the main loop, and replies; it
  does not queue pipelined requests. Overlapping requests is undefined.
- **Dispatch happens from the main loop**, not the ISR. The ISR only moves bytes
  into a ring; nothing here ever blocks or busy-waits, and the audio pipeline is
  never touched. Response latency is therefore one main-loop iteration under
  normal conditions, longer if a flash write is in progress.
- **Timeouts.** Use generous per-request timeouts. During a flash write the
  device is deaf for ~45 ms with interrupts off; inbound UART bytes sent in that
  window can be lost (see 5.2). A robust client uses a timeout of at least a few
  hundred milliseconds and retries on timeout or on `BUSY` / `BULK_LOCKED` / a
  CRC error.

### 3.6 Future notifications

Frame type `0x40` is reserved for device-initiated asynchronous notification
frames (for example, to push a parameter change or a preset-load event to the
UART controller without polling). Version 1 is **request/response only**; the
device never sends unsolicited frames today. A forward-compatible client should
already treat a leading `0xA5 0x40 ...` frame as "ignore for now" rather than a
protocol error. Until then, poll the relevant GET commands for state.

### 3.7 Bulk transfer notes (0xA0 / 0xA1)

`REQ_GET_ALL_PARAMS` (`0xA0`) and `REQ_SET_ALL_PARAMS` (`0xA1`) move the entire
`WireBulkParams` blob (roughly 3.7 KB; exact size is the wire-format version's
anchor) in a single framed transfer, with the whole payload carried in one GET
response / one SET request payload. These are large; budget the wire time:

| Baud | Approx. time for a ~3.7 KB bulk frame (8N1, ~10 bits/byte) |
|------|-----------------------------------------------------------|
| 9600 | ~3.8 s |
| 115200 | ~0.32 s |
| 460800 | ~0.08 s |
| 1000000 | ~0.037 s |

For interactive control prefer the targeted per-parameter commands; use bulk only
for a full snapshot/restore. The bulk buffer is a single shared resource across
USB/UART/I2C; if another transport holds it you get `BULK_LOCKED` and should
retry. Run bulk transfers at a high baud rate to keep the buffer-hold window
short.

---

## 4. I2C Transport

### 4.1 Electrical and bus parameters

- **Role:** target (slave) only. The DSPi never initiates bus traffic; an
  external controller is always the master.
- **Addressing:** 7-bit. Configurable address 0x08 .. 0x77; default 0x42.
- **Speed:** up to **400 kHz** (Fast-mode). The device does not restrict the SCL
  rate itself; keep the master at or below 400 kHz.
- **Pull-ups:** the device enables the pad's weak internal pull-ups as a safety
  net, but they are too weak for reliable bus edges. Fit proper **external
  pull-ups** (typically 2.2k .. 4.7k to 3.3V) sized for your bus capacitance and
  speed.
- **Logic level:** 3.3V.

### 4.2 Frame formats

A request is a single I2C **write** transaction: an 8-byte header, optionally
followed by the SET payload. All multi-byte fields little-endian.

```
Request (I2C write):
  [type][bReq][wValL][wValH][wIdxL][wIdxH][wLenL][wLenH][payload...]
    type 0x01 = SET  (payload of wLen bytes follows the header)
    type 0x02 = GET  (no payload; wLen caps the response size, 0 = full)
```

A response is read back with one or more I2C **read** transactions:

```
Response (I2C read):
  [status][lenL][lenH][payload...]   then 0xFF padding beyond the response end
    status is a CTRL_STATUS_* code (same table as UART, section 3.4)
    len is 0 unless status == OK on a GET
```

There is no sync byte and no CRC on I2C; the bus provides framing and ACK-level
integrity. The header layout matches the UART frame body (type through wLen), so
the same request-builder code can serve both transports.

### 4.3 The BUSY-retry pattern and repeated-START

Request dispatch runs from the main loop, so a read issued **before** the main
loop has processed the request finds no response ready. In that case the device
returns a BUSY frame:

```
  [0x01, 0x00, 0x00]   (status = CTRL_STATUS_BUSY, len = 0)
```

The controller simply retries the read a moment later.

- **With repeated-START** (write header, repeated-START, read in one combined
  transaction): the first read typically lands in the BUSY window and returns the
  BUSY frame; a follow-up read (or a short delay then read) returns the real
  response.
- **Without repeated-START** (separate write then read transactions): issue the
  write, wait briefly, then read; if you get BUSY, poll the read until the status
  is no longer BUSY.

Either way, treat `status == 0x01` as "not ready, read again".

### 4.4 Chunked, resumable reads

The parked response survives across multiple read transactions until it is fully
consumed. A controller that cannot read the whole response in one transaction
(small FIFO, or no repeated-START) can read it in chunks: successive reads
continue where the previous one left off, and once the payload is exhausted the
device returns `0xFF` padding. Reading past the end is safe and simply yields
`0xFF`.

Starting a **new request write discards any unconsumed response**. Finish reading
a response before issuing the next request.

### 4.5 Clock stretching

The device stretches the clock only briefly, bounded to ISR latency at I2C FIFO
boundaries. It **never** stretches while waiting on application state; that is
exactly what the BUSY frame is for. A controller therefore never sees an
unbounded stretch waiting on DSP or flash work.

### 4.6 Bulk transfer notes (0xA0 / 0xA1)

Bulk get/set works over I2C too. A large GET response (~3.7 KB) is read out in
chunks per 4.4; a large SET is written as one long header+payload transaction (or
as your master's max-transfer chunks if it fragments writes). At 400 kHz a ~3.7 KB
transfer is on the order of ~80 ms of bus time plus per-transaction overhead. The
same shared-buffer lock applies: a `BULK_LOCKED` status means another transport
holds the buffer; retry. During a flash write an in-progress I2C transaction can
stall; use generous timeouts and retry (see 5.2).

---

## 5. Shared Behavior

### 5.1 CTRL_STATUS codes (both transports)

The `status` byte in every UART and I2C response uses the same set:

| Code | Name |
|------|------|
| `0x00` | `CTRL_STATUS_OK` |
| `0x01` | `CTRL_STATUS_BUSY` |
| `0x02` | `CTRL_STATUS_ERROR` |
| `0x03` | `CTRL_STATUS_BLOCKED` |
| `0x04` | `CTRL_STATUS_BULK_LOCKED` |
| `0x05` | `CTRL_STATUS_CRC_ERROR` (UART only in practice; I2C has no CRC) |
| `0x06` | `CTRL_STATUS_OVERSIZE` |
| `0x07` | `CTRL_STATUS_FRAME_ERROR` |

**What `OK` means on a SET.** `CTRL_STATUS_OK` confirms the frame was valid
and the command was dispatched; it does not certify the parameter was
applied. A recognized command whose payload fails the handler's own
validation (too short, value out of range, invalid channel index) is
silently ignored, exactly as it is over USB, and still answers `OK`.
`CTRL_STATUS_ERROR` is returned only for commands the dispatcher itself
rejects (unknown `bRequest`, or a wValue-only SET sent as a SET-type frame
instead of a GET-type frame). A client that needs positive confirmation
should follow the SET with the matching GET and compare; this readback
contract is the same one USB hosts use (see the write-to-readback map in
`Documentation/commands.md`).

### 5.2 Flash-blackout caveat

Any command that writes flash (preset save/load/delete, device-global saves,
factory reset, an interface config SET, DAC-mute config, a bulk apply that
persists) triggers a ~45 ms window with interrupts disabled on the core. During
that window:

- **UART:** inbound bytes can be dropped; a request sent into the window may be
  lost or mangled.
- **I2C:** an in-flight transaction can stall until the window ends.

External hosts should use generous per-request timeouts (hundreds of ms) and
retry. Prefer not to pipeline a flash-writing command immediately behind another
request.

### 5.3 Pin-collision model

The rule is uniform with the rest of the firmware's pin management:

- A **disabled** interface holds no pins. Turning an interface off frees its GPIOs
  for outputs, RX, MCK, or the other control interface.
- **All validation happens at enable/apply time** (or at boot bring-up). A config
  is checked for valid mux functions and for collisions against every other pin
  consumer before the peripheral is brought up.
- A **live** UART or I2C interface reserves its TX/RX or SDA/SCL pins: they
  participate in the firmware's `is_pin_in_use` check, so any other pin-assignment
  command (output pin, RX pin, MCK pin, DAC-mute pin, or the other control
  interface) that targets a live control pin is rejected with
  `PIN_CONFIG_PIN_IN_USE`.
- A persisted config whose pins **collide at boot** (because the output wiring now
  occupies them) leaves that interface **down**: it does not start and is visible
  as `*_live == 0` via `REQ_GET_CTRL_IFACE_STATUS` (`0xF9`) even though its stored
  `enabled` is 1. Re-run the USB config, or free the conflicting pin, to bring it
  up.

Because a SET reconfiguration tears the interface down before validating the new
pins, moving an interface from one valid pin pair to another does not
false-trigger a self-collision.

### 5.4 Factory-reset survival

The interface configs are device-level, stored in the preset directory (V6),
alongside the DAC-hardware-mute config. Like that config, they **survive a factory
reset**: a DSP factory reset does not wipe device-level wiring. To return the
interfaces to their disabled defaults, write disabled configs over USB.

---

## 6. Worked Examples

Common request/response bytes for setting and reading the master volume
(`REQ_SET_MASTER_VOLUME` = `0xD2`, `REQ_GET_MASTER_VOLUME` = `0xD3`). The payload
is a 4-byte IEEE 754 single-precision dB value, little-endian. For these examples
`wValue = 0x0000` and `wIndex = 0x0000`.

Target value: **-20.0 dB**. As little-endian float: `00 00 A0 C1`.

### 6.1 UART: set master volume to -20.0 dB

```
Host -> device (SET request):
  A5 01 D2 00 00 00 00 04 00 00 00 A0 C1 CF 6D
   |  |  |  |___ wValue=0  |___ wIndex=0  |___ wLen=4  |____payload____|  |__CRC16 LE
   |  |  |__ bReq = 0xD2
   |  |__ type = 0x01 (SET)
   |__ sync

Device -> host (SET response, OK):
  A5 81 00 00 00 4C 2F
   |  |  |  |___ len = 0   |__ CRC16 LE
   |  |  |__ status = 0x00 (OK)
   |  |__ type = 0x81
   |__ sync
```

### 6.2 UART: read master volume back

```
Host -> device (GET request, cap response at 4 bytes):
  A5 02 D3 00 00 00 00 04 00 B0 EB
   |  |  |  |___ wValue=0  |___ wIndex=0  |___ wLen=4 (cap)  |__ CRC16 LE
   |  |  |__ bReq = 0xD3
   |  |__ type = 0x02 (GET)
   |__ sync

Device -> host (GET response, OK, 4-byte payload = -20.0 dB):
  A5 82 00 04 00 00 00 A0 C1 AB 91
   |  |  |  |___ len = 4   |____payload____|  |__ CRC16 LE
   |  |  |__ status = 0x00 (OK)
   |  |__ type = 0x82
   |__ sync
```

### 6.3 I2C: set master volume to -20.0 dB

```
Host write to target address (SET request, 12 bytes):
  01 D2 00 00 00 00 04 00 00 00 A0 C1
   |  |  |___ wValue=0  |___ wIdx=0  |___ wLen=4  |____payload____|
   |  |__ bReq = 0xD2
   |__ type = 0x01 (SET)

Host read from target (after dispatch):
  00 00 00  FF FF ...
   |  |___ len = 0   |__ padding
   |__ status = 0x00 (OK)

  If read too soon (before the main loop dispatched):
  01 00 00     (BUSY; retry the read)
```

### 6.4 I2C: read master volume back

```
Host write to target address (GET request, 8-byte header):
  02 D3 00 00 00 00 04 00
   |  |  |___ wValue=0  |___ wIdx=0  |___ wLen=4 (cap)
   |  |__ bReq = 0xD3
   |__ type = 0x02 (GET)

Host read from target (after dispatch):
  00 04 00 00 00 A0 C1  FF FF ...
   |  |___ len = 4   |____payload____|  |__ padding
   |__ status = 0x00 (OK)

  A read landing in the BUSY window returns 01 00 00; retry.
```

---

## 7. Related Documentation

- `Documentation/commands.md` - the complete vendor-command opcode catalogue
  (all commands here are reachable over every transport).
- `Documentation/current_architecture.md` - the "External Control Interfaces
  (UART / I2C Target)" section describes the firmware-side orchestrator, the
  transport modules, and the memory/linker impact.
- `firmware/DSPi/vendor_commands.h` - the orchestrator API and `CTRL_STATUS_*`
  codes.
- `firmware/DSPi/uart_control.h`, `firmware/DSPi/i2c_control.h` - transport
  module headers with the authoritative wire-format comments.
