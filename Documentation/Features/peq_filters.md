# PEQ Filters: Host Integration Specification

*Last updated: 2026-08-04*

This document is the complete host-side reference for the DSPi parametric EQ
(PEQ) system: every filter type, every parameter, and every wire interface an
application needs to read and write PEQ bands. It covers the classic RBJ-style
types (0..10) and the Linkwitz Transform (11). Crossover filters (types 32..63,
bands 20..23) are covered separately in `crossover_filters_spec.md`; band
bypass semantics in `band_bypass_spec.md`.

---

## 1. Data Model

### 1.1 Channels

Every channel (inputs and outputs alike) has its own independent PEQ bank.
Channel indices are inputs first, then outputs:

| Platform | Inputs | Outputs | NUM_CHANNELS | Channel index map |
|----------|--------|---------|--------------|-------------------|
| RP2350   | 8 (USB/SPDIF/I2S) | 9 (8 S/PDIF + PDM sub) | 17 | 0..7 inputs, 8..16 outputs |
| RP2040   | 2 | 5 (4 S/PDIF + PDM sub) | 7 | 0..1 inputs, 2..6 outputs |

The last output channel is always the PDM subwoofer.

### 1.2 Bands

Each channel has **10 active PEQ bands, indices 0..9**. Band indices 10..19
are a reserved gap (writes are silently rejected); 20..23 address the
crossover bands (separate feature). All 10 PEQ bands are identical in
capability; there is no ordering requirement, but bands are processed in
index order (a serial cascade).

### 1.3 Band parameters

A band is described by:

| Field    | Type  | Meaning |
|----------|-------|---------|
| type     | u8    | FilterType enum, see section 2 |
| bypass   | u8    | Exactly `1` = user-bypassed; **any other value = active** (0 recommended). See `band_bypass_spec.md`. |
| freq     | f32   | Corner/centre frequency, Hz. Clamped by firmware to [10, 0.45 x Fs]. |
| Q        | f32   | Quality factor. Clamped to [0.1, 20]. |
| gain_db  | f32   | Gain in dB for gain-based types. **For the Linkwitz Transform this field carries fp in Hz** (section 3). |
| qp       | u16   | Linkwitz Transform only: target Q as fixed point `round(Qp * 512)`. `0` selects the 0.707 default. Ignored by every other type. |

Firmware clamps are applied at coefficient-computation time. Reads normally
return the value you wrote, but out-of-range freq/Q values can be clamped
into the stored recipe when all filters are recomputed (for example after a
sample-rate change), so send in-range values.

---

## 2. Filter Types

`enum FilterType` values 0..31 are the PEQ block. Never send crossover values
(32..63) to a PEQ band; the firmware flattens them defensively.

| Value | Name | Order | freq | Q | gain_db | Description |
|-------|------|-------|------|---|---------|-------------|
| 0  | FLAT | - | - | - | - | No filtering. The default state of every band. |
| 1  | PEAKING | 2 | centre | bandwidth | boost/cut dB | RBJ peaking EQ. ~0 dB gain (< 0.01 dB) is treated as flat. |
| 2  | LOWSHELF | 2 | corner | shelf slope | shelf dB | RBJ low shelf. |
| 3  | HIGHSHELF | 2 | corner | shelf slope | shelf dB | RBJ high shelf. |
| 4  | LOWPASS | 2 | corner | resonance | ignored | RBJ low-pass. |
| 5  | HIGHPASS | 2 | corner | resonance | ignored | RBJ high-pass. |
| 6  | NOTCH | 2 | centre | width | ignored | RBJ notch. |
| 7  | ALLPASS | 2 | centre | Q | ignored | RBJ second-order all-pass; phase 0 to -360 degrees, -180 at freq. |
| 8  | ALLPASS1 | 1 | corner | ignored | ignored | First-order all-pass; phase 0 to -180 degrees, -90 at freq. |
| 9  | LOWSHELF1 | 1 | corner | ignored | shelf dB | First-order (6 dB/oct) low shelf, monotonic. |
| 10 | HIGHSHELF1 | 1 | corner | ignored | shelf dB | First-order high shelf. |
| 11 | LINKWITZ_TRANSFORM | 2 | **f0** | **Q0** | **fp (Hz)** | Pole/zero bass-extension biquad, see section 3. |
| 12 | LOWPASS1 | 1 | corner | ignored | ignored | First-order (6 dB/oct) low pass; unity at DC, -3 dB at the corner, no resonance. |
| 13 | HIGHPASS1 | 1 | corner | ignored | ignored | First-order (6 dB/oct) high pass; -3 dB at the corner, no resonance. |
| 14..31 | reserved | | | | | Rejected as flat by current firmware. |

Gain-based types (1, 2, 3, 9, 10) treat |gain| < 0.01 dB as flat. Shelf gain
follows the RBJ convention `A = 10^(gain_db/40)`; the dB value you send is
the full shelf gain.

---

## 3. The Linkwitz Transform (type 11)

### 3.1 What it does

The Linkwitz Transform (LT) replaces a sealed-box woofer's measured
second-order high-pass rolloff, described by resonance frequency `f0` and
total Q `Q0`, with a new target alignment `fp` / `Qp`. It is the industry
standard tool for extending sealed-subwoofer bass response (Siegfried
Linkwitz's pole/zero equalizer).

Transfer function (analog prototype, `w0 = 2*pi*f0`, `wp = 2*pi*fp`):

```
H(s) = (s^2 + s*w0/Q0 + w0^2) / (s^2 + s*wp/Qp + wp^2)
```

Zeros cancel the driver's existing poles; new poles place the target rolloff.
DC gain is `(f0/fp)^2`; gain approaches unity above both corners. Choosing
`fp < f0` extends bass (boost below f0, up to `40*log10(f0/fp)` dB per the
squared ratio at DC); `fp > f0` attenuates it.

The firmware realizes this with each corner independently tan-prewarped, so
the digital response is exact at both f0 and fp. On RP2350 it runs on the
hybrid path: a Simper SVF when both corners are below Fs/7.5 (essentially
always, for bass frequencies), a TDF2 biquad otherwise. On RP2040 it is a
Q28 fixed-point biquad; like all sub-30 Hz filters on that platform,
coefficient quantization limits precision at the lowest frequencies.

### 3.2 Parameter mapping (important)

The band's four LT parameters map onto the standard wire fields as follows:

| LT parameter | Wire field | Units | Range (firmware clamp) |
|--------------|-----------|-------|------------------------|
| f0 (driver resonance) | `freq` | Hz | [10, 0.15 x Fs] |
| Q0 (driver Q) | `Q` | - | [0.1, 20] |
| fp (target frequency) | `gain_db` | **Hz, not dB** | [10, 0.15 x Fs]; `fp <= 0` makes the band flat |
| Qp (target Q) | `qp` u16 sidecar | Q x 512 | [0.1, 20] after decode; `0` = 0.707 default |

`qp` encoding examples: Qp 0.5 -> 256; Qp 0.707 -> 362 (or just 0); Qp 1.0
-> 512. Decode is `Qp = qp/512`. Resolution is ~0.002, far finer than any UI
step.

Because `gain_db` is repurposed, do not present it as a gain in your UI for
LT bands, and never write a dB-style value into it for this type.

### 3.3 Typical use and safety

Example: a sealed box measuring f0 = 55 Hz, Q0 = 1.1, retargeted to
fp = 25 Hz, Qp = 0.55:

- `type = 11, freq = 55.0, Q = 1.1, gain_db = 25.0, qp = 282`
- DC boost is `40*log10(55/25) = 13.7 dB`. Warn users: LT boost is real
  low-frequency gain and consumes driver excursion and amplifier headroom;
  apps should surface the implied DC boost (`40*log10(f0/fp)` dB) and
  encourage a matching preamp/master-volume reduction.

### 3.4 Round-trip and preset behavior

- `qp` persists in user presets (preset slot format V30) and round-trips
  through bulk GET/SET_ALL_PARAMS (wire format V22).
- If a band's type changes away from LT, the stored `qp` is retained by the
  single-band vendor path (harmless, unused) but zeroed by a bulk apply that
  writes a non-LT type.
- Old hosts that are unaware of `qp` and rewrite a band with a 16-byte
  `REQ_SET_EQ_PARAM` payload do not disturb the stored `qp` (see 4.1).

### 3.5 Control surfaces

LT is deliberately excluded from front-panel filter-type cycling, and the
control-surface GAIN noun is a no-op on LT bands (a dB clamp would corrupt
the fp value). FREQ and Q nouns still edit f0/Q0. Full LT configuration is
host-only.

---

## 4. Wire Interfaces

All vendor requests use the vendor control endpoint; multi-byte fields are
little-endian.

### 4.1 REQ_SET_EQ_PARAM (0x42): write one band

Payload is the 16-byte `EqParamPacket`, optionally followed by 2 bytes of
`qp` for the Linkwitz Transform:

| Offset | Size | Field | Notes |
|--------|------|-------|-------|
| 0 | 1 | channel | 0..NUM_CHANNELS-1 |
| 1 | 1 | band | 0..9 for PEQ (20..23 crossover) |
| 2 | 1 | type | FilterType |
| 3 | 1 | bypass | exactly 1 = bypassed |
| 4 | 4 | freq | f32 LE |
| 8 | 4 | Q | f32 LE |
| 12 | 4 | gain_db | f32 LE (fp in Hz for LT) |
| 16 | 2 | qp | **optional** u16 LE, Q x 512. Omitted (16-byte payload): the band's stored qp is preserved. Present (18-byte payload): stored qp is replaced. |

Always send the 18-byte form when writing LT bands. The 16-byte form remains
fully valid for all other types (and for legacy hosts).

Invalid channel/band combinations are silently ignored (no error, no STALL).
The write is asynchronous: the packet is latched and applied from the main
loop, then a parameter-write notification is emitted (section 4.5).

### 4.2 REQ_GET_EQ_PARAM (0x43): read one field

Control IN, 4-byte response. `wValue` encodes the target:

```
wValue = (channel << 8) | (band << 3) | param
```

| param | Returns | Format |
|-------|---------|--------|
| 0 | type | u32 |
| 1 | freq | f32 |
| 2 | Q | f32 |
| 3 | gain_db | f32 (fp in Hz for LT bands) |
| 4 | bypass | u32, 1 = bypassed |
| 5 | qp | u32 (low 16 bits = qp_x512; 0 for non-PEQ band indices) |

`band` is a 5-bit field (0..31), so crossover bands 20..23 are addressable
with the same request.

### 4.3 REQ_SET_BAND_BYPASS (0xD8): toggle bypass without touching params

`wValue = (channel << 8) | band`, 1-byte payload (`1` = bypass, anything
else = active). Preserves all stored parameters including `qp`.

### 4.4 Bulk transfer: REQ_GET_ALL_PARAMS (0xA0) / REQ_SET_ALL_PARAMS (0xA1)

The bulk snapshot (`WIRE_FORMAT_VERSION = 22`; also available chunked via
0xA2/0xA3, see `bulk_params_chunking.md`) carries every PEQ band as a
16-byte `WireBandParams`:

| Offset | Size | Field |
|--------|------|-------|
| 0 | 1 | type |
| 1 | 1 | bypass |
| 2 | 2 | reserved; **carries qp (u16 LE) when type = 11**, zero otherwise |
| 4 | 4 | freq f32 |
| 8 | 4 | q f32 |
| 12 | 4 | gain_db f32 (fp in Hz for LT) |

Layout is row-major `eq[channel][band]`, 12 band slots per channel (only
0..9 are active). On SET, the firmware reads the reserved bytes as `qp` for
LT bands and forces 0 for all other types. Hosts on wire format 21 or older
are rejected outright by the exact-version check; update the app's parser to
declare version 22 (no layout change, only the reserved-byte meaning).

### 4.5 Parameter-write notifications

Hosts listening on the notification endpoint receive an EQ-parameter event
after every applied band write; its payload embeds the same `WireBandParams`
layout as 4.4, including `qp` in the reserved bytes for LT bands. See
`notification_protocol_v2_spec.md`.

### 4.6 Presets

User preset save/load (vendor 0x90..0x9C, `user_presets_spec.md`) persists
all PEQ state including `qp` (slot format V30). Presets saved by older
firmware load with `qp = 0`, i.e. the 0.707 default for any LT band.

---

## 5. App Implementation Checklist

1. **Type picker**: offer types 0..11; label 11 "Linkwitz Transform".
2. **Per-type field visibility**: hide Q for types 8..10; hide gain for
   types 4..8; for type 11 show four fields (f0, Q0, fp, Qp) and no gain.
3. **Writes**: one `REQ_SET_EQ_PARAM` per band edit; 18-byte payload for LT.
   Batch restores via `REQ_SET_ALL_PARAMS` (or the chunked variants).
4. **Reads**: `REQ_GET_ALL_PARAMS` for full state (parse reserved bytes as
   qp when type = 11); `REQ_GET_EQ_PARAM` param 5 for a single qp.
5. **Validation before send**: clamp freq/f0/fp to [10, 20000] and Q/Q0/Qp
   to [0.1, 20] in the UI so what the user sees matches what the DSP runs.
6. **LT UX**: display implied DC boost `40*log10(f0/fp)` dB; warn above
   ~15 dB. Do not map any dB control onto `gain_db` for LT bands.
7. **Response plotting**: biquad coefficients for all types follow the RBJ
   cookbook; for LT use the analog prototype in 3.1 (bilinear with both
   corners prewarped if you need sample-exact curves; `tools/filter_tester/
   user_linkwitz.py` is the reference model).

---

## 6. Firmware Internals (reference)

- Coefficients: `dsp_pipeline.c: dsp_compute_coefficients()`. LT SVF mix:
  `g = tan(pi*fp/Fs)`, `k = 1/Qp`, `r = tan(pi*f0/Fs)/g`, `m0 = 1`,
  `m1 = r/Q0 - k`, `m2 = r^2 - 1`. Biquad: `b = [1 + g0/Q0 + g0^2,
  2(g0^2 - 1), 1 - g0/Q0 + g0^2]`, `a` likewise from `gp`, `Qp`.
- qp storage: `peq_qp_x512[NUM_CHANNELS][MAX_BANDS]` (dsp_pipeline.c),
  parallel to `filter_recipes` because `EqParamPacket` has no spare field.
- RAM cost: 408 bytes (RP2350) / 168 bytes (RP2040).
- Flash: `PresetSlot.peq_qp_x512`, version-gated tail append at
  SLOT_DATA_VERSION 30.
