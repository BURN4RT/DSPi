# Chunked Bulk Params: Host-Side Migration Guide

*Last updated: 2026-07-04*

## Why this exists

At wire format V16 the bulk-params blob (`WireBulkParams`) grew to **5864
bytes**, exact-size-only. Windows cannot move that in one vendor control
transfer: WinUSB enforces a documented **4096-byte maximum** on the data
stage, and libusb's Windows backend rejects larger requests with
`LIBUSB_ERROR_INVALID_PARAM` before anything reaches the wire (GitHub issue
#62). All three Zadig driver options route through the same backend, so no
driver swap avoids it. Chrome's WebUSB on Windows binds through WinUSB and
inherits the same cap.

The firmware therefore provides chunked access to the same buffer:

| Code | Name | Direction | wValue | wLength |
|------|------|-----------|--------|---------|
| `0xA2` | `REQ_GET_ALL_PARAMS_CHUNK` | IN | byte offset | chunk size |
| `0xA3` | `REQ_SET_ALL_PARAMS_CHUNK` | OUT | byte offset | chunk size |

The single-shot `0xA0` / `0xA1` commands are unchanged and remain the right
choice on macOS, Linux, and on the UART/I2C control transports (no size cap
there; `0xA2`/`0xA3` are refused on UART/I2C with `CTRL_STATUS_BLOCKED`).
The wire payload is byte-identical to the single-shot commands; only the
transfer is split. No wire format version change.

## What a host must change

Only the transport layer of the bulk read/write helpers. Everything that
parses or builds `WireBulkParams` is untouched.

Pick a chunk size of at most 4096; **2048 is recommended** (comfortable
margin, three transfers per direction).

### Read (replaces one `0xA0` control-IN)

1. Issue control-IN `bRequest=0xA2, wValue=0, wLength=2048`. Receiving
   offset 0 makes the device snapshot the complete struct into an internal
   buffer under its bulk lock, so all chunks come from one coherent image.
2. Repeat with `wValue = 2048`, then `4096`, until you have accumulated
   the full struct. The final chunk returns short (5864 - 4096 = 1768
   bytes); the device clamps `wLength` past the end for you.
3. Concatenate in offset order. Sanity-check `header.payload_length`
   against what you received, as with `0xA0`.

You may read `header` fields from the first chunk to learn the exact total
instead of hardcoding 5864.

### Write (replaces one `0xA1` control-OUT)

1. Build the full payload exactly as for `0xA1`.
2. Issue control-OUT `bRequest=0xA3, wValue=0` with the first 2048 bytes,
   then `wValue=2048`, then `wValue=4096` with the remainder.
3. Chunks **must be sequential and non-overlapping starting at offset 0**;
   an out-of-order offset is STALLed and you must restart from 0.
4. When the final byte lands, the device applies the blob through the same
   deferred main-loop path as `0xA1` (same validation, same notify
   rebaseline). There is no separate commit command.

### Error handling and session rules

- A STALL on any chunk means the session was rejected or lost: restart the
  whole transfer from offset 0. Offset 0 always starts a fresh session.
- Do not interleave other vendor requests between chunks; any non-chunk
  vendor request tears the open session down (by design, so an abandoned
  session cannot wedge the shared buffer).
- Do not pause longer than about 3 seconds between chunks; an idle session
  is reaped and later offsets will STALL.
- A STALL at offset 0 usually means the bulk buffer is momentarily owned by
  another transport (UART/I2C bulk in flight, or a pending apply); retry
  after a short delay.
- One session at a time; starting a GET session cancels an unfinished SET
  session and vice versa.

### pyusb sketch (tools/dspi_test)

```python
TOTAL = 5864          # or read header.payload_length from the first chunk
CHUNK = 2048

def get_all_params(dev):
    data = b""
    off = 0
    while off < TOTAL:
        n = min(CHUNK, TOTAL - off)
        data += dev.ctrl_transfer(0xC0, 0xA2, wValue=off, wIndex=0,
                                  data_or_wLength=n, timeout=1000).tobytes()
        off += n
    return data

def set_all_params(dev, blob):
    assert len(blob) == TOTAL
    for off in range(0, TOTAL, CHUNK):
        chunk = blob[off:off + CHUNK]
        dev.ctrl_transfer(0x40, 0xA3, wValue=off, wIndex=0,
                          data_or_wLength=chunk, timeout=1000)
```

### Choosing single-shot vs chunked at runtime

Simplest portable policy: try `0xA0`/`0xA1` once; on
`LIBUSB_ERROR_INVALID_PARAM` (or any pre-wire size rejection), fall back to
chunked and remember the choice. Alternatively, always use chunked on
`sys.platform == "win32"` / when running under WebUSB; the chunked path
works identically on every OS, so a host may also just use it everywhere.

## Firmware reference

Implementation: `firmware/DSPi/vendor_commands.c` (search
`REQ_GET_ALL_PARAMS_CHUNK`); sessions ride the shared bulk owner lock
introduced with the external control interfaces, so USB chunking, UART/I2C
bulk streaming, and single-shot USB transfers all serialize against the one
`bulk_param_buf`.
