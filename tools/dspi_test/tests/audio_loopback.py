"""
audio_loopback.py — hardware-in-the-loop output & filter tests (group "audio").

Unlike the rest of the suite (control-plane only), these tests measure real
audio: a host signal is played out the DSPi USB audio output, processed by the
DSP, and captured back from DSPi's own USB capture input (the DSPI_LOOPBACK
firmware build, which taps output slot 0 after all DSP), and the measured result
is compared against the firmware's filter math
(tools/filter_tester/compare_filter.py).

Gating: this group is EXCLUDED from the default run; enable with `--audio`
(or `--group audio`). It needs the DSPI_LOOPBACK firmware plus
`sounddevice`+`numpy`+`scipy` (`pip install sounddevice numpy scipy`). If the
deps or the capture interface are absent the tests SKIP (never hard-fail).

The capture is hardwired to output slot 0, so the target slot is slot 0; it is
still auto-probed once per session (the probe empirically confirms which slot
reaches the capture). State is mutated freely; the suite's pre-suite snapshot is
restored at the end.
"""

from __future__ import annotations

import struct
import time

from ..device import OP
from ..framework import test, Skip
from .. import audio

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None


# --- Tolerances / parameters (tunable) --------------------------------------

MAG_TOL_DB = 0.7        # max |measured - expected| magnitude error
PHASE_TOL_DEG = 6.0     # max phase-shape error (after removing fixed delay), all-pass
CORR_MIN = 0.30         # cross-correlation strength that means "signal present"
NOISE_MAX_DBFS = -100.0
RESIDUAL_MAX_DBFS = -80.0   # flat-path per-sample residual (bit-exact-ish)
GAIN_TOL_DB = 0.5           # flat-path overall gain vs unity
FS = audio.DEFAULT_FS       # 48 kHz; DSPi follows the host USB rate

# FilterType enum (firmware) by RBJ-reference name.
TYPE = {"peaking": 1, "lowshelf": 2, "highshelf": 3, "lowpass": 4, "highpass": 5,
        "notch": 6, "allpass": 7, "allpass1": 8, "lowshelf1": 9, "highshelf1": 10}
FLAT = 0
INPUT_USB = 0
TYPE_SPDIF, TYPE_I2S = 0, 1   # OutputType enum (firmware)
# Settle after an output-type switch before bit-exact measurement: the switch
# runs complete_pipeline_reset() and the I2S clock + loopback servo re-prime, so
# the first capture frames after the switch carry a transient.  2 s lets it fully
# settle so I2S meets the same single-shot bit-exact standard as S/PDIF (no retry).
TYPE_SWITCH_SETTLE_S = 2.0

# Magnitude-shaping PEQ configs: (name, rbj_name, fc, Q, gain_db). Frequencies
# straddle the RP2350 SVF/biquad boundary (Fs/7.5 ~= 6400 Hz @ 48 kHz).

PEQ_CONFIGS = [
    ("peaking_lo",    "peaking",       300.0, 2.0,   6.0),
    ("peaking_hi",    "peaking",      9000.0, 2.0,   6.0),
    ("peaking_cut",   "peaking",      1000.0, 1.0,  -6.0),
    ("lowshelf_lo",   "lowshelf",      300.0, 0.707, 6.0),
    ("lowshelf_hi",   "lowshelf",     9000.0, 0.707, 6.0),
    ("highshelf_lo",  "highshelf",     300.0, 0.707, 6.0),
    ("highshelf_hi",  "highshelf",    9000.0, 0.707, 6.0),
    ("lowpass_lo",    "lowpass",       300.0, 0.707, 0.0),
    ("lowpass_hi",    "lowpass",      9000.0, 0.707, 0.0),
    ("highpass_lo",   "highpass",      300.0, 0.707, 0.0),
    ("highpass_hi",   "highpass",     9000.0, 0.707, 0.0),
    ("notch_lo",      "notch",         300.0, 4.0,   0.0),
    ("notch_hi",      "notch",        9000.0, 4.0,   0.0),
    ("lowshelf1_lo",  "lowshelf1",     300.0, 0.707, 6.0),
    ("lowshelf1_hi",  "lowshelf1",    9000.0, 0.707, 6.0),
    ("highshelf1_lo", "highshelf1",    300.0, 0.707, 6.0),
    ("highshelf1_hi", "highshelf1",   9000.0, 0.707, 6.0),
    ("allpass1_lo",   "allpass1",      300.0, 0.707, 0.0),
    ("allpass1_hi",   "allpass1",     9000.0, 0.707, 0.0),
    ("allpass_lo",    "allpass",       300.0, 0.707, 0.0),
    ("allpass_hi",    "allpass",      9000.0, 0.707, 0.0),
]


# --- Vendor-command helpers (mirror tests/eq.py, tests/outputs.py) -----------

def _eq_packet(ch, band, ftype, freq, q, gain, bypass=0):
    return struct.pack("<BBBBfff", ch, band, ftype, bypass, freq, q, gain)


def _set_band(dev, ch, band, ftype, freq, q, gain):
    dev.set(OP.SET_EQ_PARAM, _eq_packet(ch, band, ftype, freq, q, gain))


def _route(dev, inp, out, enabled, gain_db=0.0, phase=0):
    # MatrixRoutePacket: <input, output, enabled, phase_invert, gain_db>.
    dev.set(OP.SET_MATRIX_ROUTE, struct.pack("<BBBBf", inp, out, enabled, phase, gain_db))


def _signal_amp(gain_db):
    """Pick a sweep amplitude so the post-filter peak stays well below 0 dBFS."""
    boost = 10.0 ** (max(gain_db, 0.0) / 20.0)
    return min(0.4, 0.6 / boost)


# --- Session rig (devices + auto-probed target slot), discovered once --------

_RIG = None  # dict on success, or a str reason once we know it's unavailable


def _slot_indices(profile, slot):
    """For output slot `s`: (out_l, out_r) matrix/enable/gain indices (0-based
    output index) and (ch_l, ch_r) EQ channel indices.  In the unified channel
    model the EQ channel space is inputs [0..NUM_INPUT-1] followed by outputs, so
    output EQ channels start at NUM_INPUT_CHANNELS (2 on RP2040, 8 on RP2350) —
    NOT a fixed +2."""
    out_l, out_r = 2 * slot, 2 * slot + 1
    base = profile.num_input_channels
    return out_l, out_r, base + out_l, base + out_r


def _baseline(dev, profile=None):
    """Clean, deterministic pre-conditions: USB input, EVERY gain stage at unity.

    The device master volume powers on at -20 dB (MASTER_VOL_DEFAULT_DB), and
    user volume + user mute are independent stages; all of them must be zeroed
    explicitly or absolute-level measurements read low. Per-input preamp is
    zeroed across all input channels (8 on RP2350)."""
    dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_USB)
    dev.wait_ready()
    dev.set_f32(OP.SET_MASTER_VOLUME, 0.0)   # power-on default is -20 dB
    dev.set_f32(OP.SET_USER_VOLUME, 0.0)
    dev.set_u8(OP.SET_USER_MUTE, 0)
    n_in = profile.num_input_channels if profile is not None else 2
    for ch in range(n_in):
        dev.set_f32(OP.SET_PREAMP_CH, 0.0, wvalue=ch)
    dev.wait_ready()


def _route_only(dev, profile, out_l, out_r):
    """Route USB L/R 1:1 to exactly the (out_l, out_r) pair at 0 dB and disable
    USB -> every other output, so only this output pair carries signal. Matrix
    writes apply immediately (no deferred reset)."""
    for out in range(profile.num_output_channels):
        _route(dev, 0, out, 0)
        _route(dev, 1, out, 0)
    _route(dev, 0, out_l, 1, 0.0)   # USB L -> target L
    _route(dev, 1, out_r, 1, 0.0)   # USB R -> target R


def _config_slot(dev, profile, slot, flatten_all=False, otype=TYPE_SPDIF):
    """Make `slot` the only output carrying USB audio (1:1, 0 dB), as output
    type `otype` (S/PDIF or I2S). Returns (ch_l, ch_r). With flatten_all, zero
    every band on both channels. The loopback tap is pre-encoder, so the
    captured DSP output is identical for either output type."""
    out_l, out_r, ch_l, ch_r = _slot_indices(profile, slot)
    dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(otype << 8) | slot)
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_l)
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_r)
    dev.wait_ready()
    _route_only(dev, profile, out_l, out_r)
    if flatten_all:
        for ch in (ch_l, ch_r):
            for b in range(profile.band_ceiling):
                _set_band(dev, ch, b, FLAT, 1000.0, 0.707, 0.0)
            for b in range(20, 24):          # crossover bands (output channels)
                _set_band(dev, ch, b, FLAT, 1000.0, 0.707, 0.0)
    dev.wait_ready()
    return ch_l, ch_r


def _autoprobe_slot(dev, profile, out_dev, in_dev, fs):
    """Find which output slot the DSPi capture receives. With the DSPI_LOOPBACK
    tap this is always slot 0, but each slot is still probed in ISOLATION (USB
    routed to only that slot, all others muted) so the result is confirmed
    empirically: only the slot the capture taps shows signal — otherwise an
    earlier-routed slot keeps carrying audio and every later probe would falsely
    succeed."""
    for k in range(profile.num_spdif):     # enable all S/PDIF outputs once
        ol, orr = 2 * k, 2 * k + 1
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(0 << 8) | k)
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=ol)
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=orr)
    dev.wait_ready()

    best, best_lvl = None, -200.0
    for slot in range(profile.num_spdif):
        out_l, out_r, _cl, _cr = _slot_indices(profile, slot)
        _route_only(dev, profile, out_l, out_r)
        level, _thd, strength = audio.measure_tone(out_dev, in_dev, 0, fs, 1000.0, amp=0.3)
        if strength > CORR_MIN and level > best_lvl:
            best, best_lvl = slot, level
    if best is None:
        raise Skip("no output slot reached the DSPi capture — check input source (USB) / slot-0 routing")
    return best


def _get_rig(dev, profile):
    """Discover audio devices and the target slot once; cache for the session."""
    global _RIG
    if isinstance(_RIG, str):
        raise Skip(_RIG)
    if _RIG is not None:
        return _RIG
    if np is None:
        _RIG = "numpy not installed (pip install numpy scipy sounddevice)"
        raise Skip(_RIG)
    try:
        out_dev, in_dev, info = audio.find_devices()
    except audio.AudioUnavailable as e:
        _RIG = f"audio loopback unavailable: {e}"
        raise Skip(_RIG)

    _baseline(dev, profile)
    slot = _autoprobe_slot(dev, profile, out_dev, in_dev, FS)
    ch_l, ch_r = _config_slot(dev, profile, slot, flatten_all=True)
    _RIG = {"out": out_dev, "in": in_dev, "chan": 0, "slot": slot,
            "ch_l": ch_l, "ch_r": ch_r, "fs": FS,
            "out_name": info["out"]["name"], "in_name": info["in"]["name"]}
    return _RIG


def _expected(rbj_name, fc, q, gain, fs, freqs):
    """Expected (mag_db, phase_deg) at `freqs` from the RBJ reference."""
    from tools.filter_tester.compare_filter import rbj_coefficients, eval_biquad
    coefs = rbj_coefficients(rbj_name, fc, q, gain, fs)
    w = 2.0 * np.pi * np.asarray(freqs) / fs
    H = eval_biquad(coefs, w)
    return 20.0 * np.log10(np.abs(H) + 1e-30), np.degrees(np.unwrap(np.angle(H)))


def _phase_shape_err(freqs, meas_deg, exp_deg):
    """Max phase error after removing a best-fit (delay + offset), so residual
    sub-sample alignment delay does not masquerade as a phase error."""
    d = np.asarray(meas_deg) - np.asarray(exp_deg)
    A = np.vstack([np.asarray(freqs), np.ones_like(freqs)]).T
    coef, *_ = np.linalg.lstsq(A, d, rcond=None)
    return float(np.max(np.abs(d - A @ coef)))


def _test_freqs(fs):
    return np.logspace(np.log10(40.0), np.log10(fs * 0.40), 64)


# --- Tests ------------------------------------------------------------------

@test("audio", mutating=True)
def loopback_integrity(dev, profile, chk):
    """Flat path: signal reaches the DSPi capture at unity, low noise/THD, near bit-exact."""
    rig = _get_rig(dev, profile)
    chk.note(f"out='{rig['out_name']}' in='{rig['in_name']}' slot={rig['slot']} "
             f"ch_l={rig['ch_l']} fs={rig['fs']}")
    _set_band(dev, rig["ch_l"], 0, FLAT, 1000.0, 0.707, 0.0)
    dev.wait_ready()

    amp = 0.4
    level, thd, strength = audio.measure_tone(rig["out"], rig["in"], rig["chan"],
                                              rig["fs"], 1000.0, amp=amp)
    chk.ok(strength > CORR_MIN, f"tone reaches DSPi capture (corr {strength:.2f})")
    exp_level = 20.0 * np.log10(amp / np.sqrt(2.0))
    chk.approx(level, exp_level, 1.0, f"tone level ~{exp_level:.1f} dBFS")
    chk.ok(thd < 0.1, f"THD {thd:.4f}% < 0.1%")

    noise = audio.measure_noise(rig["out"], rig["in"], rig["chan"], rig["fs"])
    chk.ok(noise < NOISE_MAX_DBFS, f"noise floor {noise:.1f} dBFS < {NOISE_MAX_DBFS}")

    resid, scale = audio.bit_exact_residual(rig["out"], rig["in"], rig["chan"], rig["fs"])
    chk.ok(resid < RESIDUAL_MAX_DBFS, f"flat-path residual {resid:.1f} dBFS < {RESIDUAL_MAX_DBFS}")
    # The S/PDIF path may invert polarity (scale < 0); that is fine for a DAC.
    # Check |scale| for unity magnitude and just report the sign.
    gain_db = 20.0 * np.log10(abs(scale) + 1e-20)
    chk.approx(gain_db, 0.0, GAIN_TOL_DB,
               f"path gain ~0 dB (|scale| {abs(scale):.4f}, polarity {'+' if scale >= 0 else '-'})")
    chk.note(f"level={level:.2f}dBFS thd={thd:.4f}% noise={noise:.1f}dBFS "
             f"residual={resid:.1f}dBFS scale={scale:.4f}")


@test("audio", mutating=True)
def loopback_allpass_phase(dev, profile, chk):
    """First-order all-pass: magnitude stays flat and the phase shape matches."""
    rig = _get_rig(dev, profile)
    fc, q = 1000.0, 0.707
    _set_band(dev, rig["ch_l"], 0, TYPE["allpass1"], fc, q, 0.0)
    dev.wait_ready()
    freqs = _test_freqs(rig["fs"])
    mag, phase, strength = audio.measure_transfer(rig["out"], rig["in"], rig["chan"],
                                                  rig["fs"], freqs, amp=0.4)
    chk.ok(strength > CORR_MIN, f"signal present (corr {strength:.2f})")
    chk.ok(float(np.max(np.abs(mag))) < 0.3, f"all-pass magnitude flat (max |{np.max(np.abs(mag)):.3f}| dB)")
    _, exp_phase = _expected("allpass1", fc, q, 0.0, rig["fs"], freqs)
    perr = _phase_shape_err(freqs, phase, exp_phase)
    chk.ok(perr < PHASE_TOL_DEG, f"all-pass phase-shape err {perr:.2f} deg < {PHASE_TOL_DEG}")
    chk.note(f"allpass1 fc={fc}: mag_flat={np.max(np.abs(mag)):.3f}dB phase_err={perr:.2f}deg")


def _make_peq_test(name, rbj_name, fc, q, gain):
    # An all-pass is flat by definition, so a magnitude-only check passes even on
    # a filter that does nothing at all.  Assert the phase shape too, or these
    # configs have no fault-detection value.
    is_allpass = rbj_name in ("allpass", "allpass1")

    def fn(dev, profile, chk):
        rig = _get_rig(dev, profile)
        _set_band(dev, rig["ch_l"], 0, TYPE[rbj_name], fc, q, gain)
        dev.wait_ready()
        freqs = _test_freqs(rig["fs"])
        mag, phase, strength = audio.measure_transfer(
            rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=_signal_amp(gain))
        chk.ok(strength > CORR_MIN, f"signal present (corr {strength:.2f})")
        exp_mag, exp_phase = _expected(rbj_name, fc, q, gain, rig["fs"], freqs)
        err = float(np.max(np.abs(mag - exp_mag)))
        chk.ok(err < MAG_TOL_DB,
               f"{name} fc={fc:g} Q={q:g} gain={gain:g}: max |mag err| {err:.3f} dB < {MAG_TOL_DB}")
        if is_allpass:
            perr = _phase_shape_err(freqs, phase, exp_phase)
            chk.ok(perr < PHASE_TOL_DEG,
                   f"{name} fc={fc:g}: phase-shape err {perr:.2f} deg < {PHASE_TOL_DEG}")
            chk.note(f"{name}: max_mag_err={err:.3f}dB phase_err={perr:.2f}deg corr={strength:.2f}")
        else:
            chk.note(f"{name}: max_mag_err={err:.3f}dB corr={strength:.2f}")
    fn.__name__ = f"peq_{name}"
    fn.__doc__ = f"PEQ {rbj_name} fc={fc:g} Q={q:g} gain={gain:g}: measured FR matches RBJ reference."
    return test("audio", mutating=True)(fn)


# Register one test per PEQ config (distinct names for per-config reporting).
for _name, _rbj, _fc, _q, _gain in PEQ_CONFIGS:
    globals()[f"peq_{_name}"] = _make_peq_test(_name, _rbj, _fc, _q, _gain)


# --- Crossover (XO) frequency response (Phase 1) ----------------------------

XO_MAG_TOL_DB = 1.0          # steeper slopes are more sensitive than mild PEQ
XO_MAG_FLOOR_DB = -60.0      # only compare where |H| is reliably measurable
XO_BAND = 20                 # first crossover band (bands 20..23, output channels)
XO_BASE = 32                 # FILTER_XOVER_FIRST = FILTER_LR2_LP

# Representative spread: families × orders × LP/HP, fc straddling Fs/7.5 (~6400 Hz).
# (name, family, order, is_hp, fc)
XO_CONFIGS = [
    ("lr2_lp_lo",  "lr",  2, False,   500.0),
    ("lr2_lp_hi",  "lr",  2, False,  9000.0),
    ("lr2_hp_lo",  "lr",  2, True,    500.0),
    ("lr2_hp_hi",  "lr",  2, True,   9000.0),
    ("lr4_lp",     "lr",  4, False,   800.0),
    ("lr4_hp",     "lr",  4, True,    800.0),
    ("lr6_hp",     "lr",  6, True,   1000.0),
    ("lr6_lp",     "lr",  6, False,  2000.0),
    ("lr8_lp",     "lr",  8, False,  5000.0),
    ("lr8_hp",     "lr",  8, True,   9000.0),
    ("bw1_lp_lo",  "bw",  1, False,   200.0),
    ("bw1_lp_hi",  "bw",  1, False,  9000.0),
    ("bw1_hp_lo",  "bw",  1, True,    200.0),
    ("bw1_hp_hi",  "bw",  1, True,   9000.0),
    ("bw2_lp",     "bw",  2, False,  1000.0),
    ("bw2_hp",     "bw",  2, True,   1000.0),
    ("bw3_lp",     "bw",  3, False,  2000.0),
    ("bw3_hp",     "bw",  3, True,   2000.0),
    ("bw4_lp",     "bw",  4, False,  3000.0),
    ("bw4_hp",     "bw",  4, True,   3000.0),
    ("bw5_lp",     "bw",  5, False,  5000.0),
    ("bw5_hp",     "bw",  5, True,   5000.0),
    ("bw6_lp",     "bw",  6, False,  7000.0),
    ("bw6_hp",     "bw",  6, True,   7000.0),
    ("bw7_lp",     "bw",  7, False,  9000.0),
    ("bw7_hp",     "bw",  7, True,   9000.0),
    ("bw8_lp",     "bw",  8, False, 10000.0),
    ("bw8_hp",     "bw",  8, True,  10000.0),
    ("bes2_lp",    "bes", 2, False,   500.0),
    ("bes2_hp",    "bes", 2, True,    500.0),
    ("bes4_lp",    "bes", 4, False,  1000.0),
    ("bes4_hp",    "bes", 4, True,   1000.0),
    ("bes6_lp",    "bes", 6, False,  4000.0),
    ("bes6_hp",    "bes", 6, True,   4000.0),
    ("bes8_lp",    "bes", 8, False,  8000.0),
    ("bes8_hp",    "bes", 8, True,   3000.0),
]


def _xo_enum(family, order, is_hp):
    """Firmware FilterType value for a crossover type (config.h, 32..63).
    Mirrors the contiguous enum: LR2/4/6/8, BW1..8, BES2/4/6/8, each LP then HP."""
    if family == "lr":
        base = XO_BASE + 2 * (2, 4, 6, 8).index(order)
    elif family == "bw":
        base = XO_BASE + 8 + 2 * (order - 1)
    elif family == "bes":
        base = XO_BASE + 24 + 2 * (2, 4, 6, 8).index(order)
    else:
        raise ValueError(family)
    return base + (1 if is_hp else 0)


def _xo_reference(family, order, is_hp, fc, fs, freqs):
    """Ground-truth complex H from scipy (same convention as
    tools/filter_tester/test_crossover.py: Butterworth/Bessel direct, LR = BW^2)."""
    try:
        import scipy.signal as sig
    except ImportError:
        raise Skip("scipy not installed (pip install scipy)")
    btype = "highpass" if is_hp else "lowpass"
    if family == "bw":
        sos = sig.butter(order, fc, btype=btype, output="sos", fs=fs)
    elif family == "bes":
        sos = sig.bessel(order, fc, btype=btype, output="sos", fs=fs, norm="mag")
    elif family == "lr":
        bw = sig.butter(order // 2, fc, btype=btype, output="sos", fs=fs)
        sos = np.vstack([bw, bw])
    else:
        raise ValueError(family)
    _, H = sig.sosfreqz(sos, worN=2.0 * np.pi * np.asarray(freqs) / fs)
    return H


def _make_xo_test(name, family, order, is_hp, fc):
    def fn(dev, profile, chk):
        rig = _get_rig(dev, profile)
        ftype = _xo_enum(family, order, is_hp)
        try:
            _set_band(dev, rig["ch_l"], 0, FLAT, 1000.0, 0.707, 0.0)   # clear any leftover PEQ band
            _set_band(dev, rig["ch_l"], XO_BAND, ftype, fc, 0.707, 0.0)
            dev.wait_ready()
            freqs = _test_freqs(rig["fs"])
            mag, _ph, _st = audio.measure_transfer(
                rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=0.4)
            exp = 20.0 * np.log10(np.abs(_xo_reference(family, order, is_hp, fc, rig["fs"], freqs)) + 1e-30)
            band = exp > XO_MAG_FLOOR_DB     # compare only the measurable region
            # Presence by passband level, not correlation: a high-pass legitimately
            # cuts most of the sweep's energy, so corr is low even on a clean capture.
            present = float(np.max(mag[band])) if band.any() else -200.0
            chk.ok(present > -20.0, f"{name}: signal reaches passband ({present:.1f} dB)")
            err = float(np.max(np.abs(mag[band] - exp[band]))) if band.any() else 0.0
            chk.ok(err < XO_MAG_TOL_DB,
                   f"{name} fc={fc:g}: max |mag err| {err:.3f} dB < {XO_MAG_TOL_DB} "
                   f"(in the >{XO_MAG_FLOOR_DB:g} dB region)")
            chk.note(f"{name}: max_mag_err={err:.3f}dB passband={present:.2f}dB")
        finally:
            _set_band(dev, rig["ch_l"], XO_BAND, FLAT, 1000.0, 0.707, 0.0)
            dev.wait_ready()
    fn.__name__ = f"xo_{name}"
    fn.__doc__ = (f"Crossover {name} fc={fc:g}: measured FR matches the scipy "
                  f"{family.upper()}{order} reference.")
    return test("audio", mutating=True)(fn)


for _c in XO_CONFIGS:
    globals()[f"xo_{_c[0]}"] = _make_xo_test(*_c)


@test("audio", mutating=True)
def xo_lr4_complementary_sum(dev, profile, chk):
    """Linkwitz-Riley LP+HP at the same fc sum to flat magnitude (the LR property).

    Measures both legs in ONE capture (shared time reference): USB L is routed to
    both target outputs, LR4 LP on the left channel, LR4 HP on the right, and
    |H_LP + H_HP| must be ~0 dB across the band. (LR4 is even-order, so the legs
    are in phase; the path's common delay/polarity cancels in the sum.)
    """
    rig = _get_rig(dev, profile)
    fc = 1000.0
    out_l, out_r, ch_l, ch_r = _slot_indices(profile, rig["slot"])
    try:
        for out in range(profile.num_output_channels):   # USB L -> both target outputs only
            _route(dev, 0, out, 0)
            _route(dev, 1, out, 0)
        _route(dev, 0, out_l, 1, 0.0)
        _route(dev, 0, out_r, 1, 0.0)
        _set_band(dev, ch_l, 0, FLAT, 1000.0, 0.707, 0.0)   # clear leftover PEQ on both legs
        _set_band(dev, ch_r, 0, FLAT, 1000.0, 0.707, 0.0)
        _set_band(dev, ch_l, XO_BAND, _xo_enum("lr", 4, False), fc, 0.707, 0.0)  # LP
        _set_band(dev, ch_r, XO_BAND, _xo_enum("lr", 4, True),  fc, 0.707, 0.0)  # HP
        dev.wait_ready()
        freqs = _test_freqs(rig["fs"])
        h_lp, h_hp, strength = audio.measure_complex_2ch(rig["out"], rig["in"], rig["fs"], freqs, amp=0.4)
        chk.ok(strength > CORR_MIN, f"signal present (corr {strength:.2f})")
        summ_db = 20.0 * np.log10(np.abs(h_lp + h_hp) + 1e-30)
        err = float(np.max(np.abs(summ_db)))
        chk.ok(err < 1.0, f"LR4 LP+HP sum flat: max |deviation from 0 dB| {err:.3f} dB < 1.0")
        chk.note(f"lr4_sum: max|sum-0dB|={err:.3f}dB corr={strength:.2f}")
    finally:
        _set_band(dev, ch_l, XO_BAND, FLAT, 1000.0, 0.707, 0.0)
        _set_band(dev, ch_r, XO_BAND, FLAT, 1000.0, 0.707, 0.0)
        dev.wait_ready()


# --- Phase 2: output-stage controls -----------------------------------------

LEVEL_TOL_DB = 0.5
MUTE_FLOOR_DBFS = -80.0


def _flatten_chain(dev, ch):
    """Flatten the PEQ band and crossover band the other tests use, so a level /
    delay / polarity measurement sees a unity filter chain on this channel."""
    _set_band(dev, ch, 0, FLAT, 1000.0, 0.707, 0.0)
    _set_band(dev, ch, XO_BAND, FLAT, 1000.0, 0.707, 0.0)


def _tone_level(dev, rig, freq=1000.0, amp=0.4):
    lvl, _thd, _st = audio.measure_tone(rig["out"], rig["in"], rig["chan"], rig["fs"], freq, amp=amp)
    return lvl


@test("audio", mutating=True)
def output_gain_level(dev, profile, chk):
    """Per-output gain: measured level tracks the set dB."""
    rig = _get_rig(dev, profile)
    out_l = _slot_indices(profile, rig["slot"])[0]
    try:
        _flatten_chain(dev, rig["ch_l"])
        dev.set_f32(OP.SET_OUTPUT_GAIN, 0.0, wvalue=out_l); dev.wait_ready()
        base = _tone_level(dev, rig)
        for g in (-6.0, -3.0, 3.0):
            dev.set_f32(OP.SET_OUTPUT_GAIN, g, wvalue=out_l); dev.wait_ready()
            chk.approx(_tone_level(dev, rig) - base, g, LEVEL_TOL_DB, f"output gain {g:+g} dB -> level delta")
        chk.note(f"output_gain base={base:.2f}dBFS")
    finally:
        dev.set_f32(OP.SET_OUTPUT_GAIN, 0.0, wvalue=out_l); dev.wait_ready()


@test("audio", mutating=True)
def output_mute_silences(dev, profile, chk):
    """Per-output mute drops the output to silence; unmute restores it."""
    rig = _get_rig(dev, profile)
    out_l = _slot_indices(profile, rig["slot"])[0]
    try:
        _flatten_chain(dev, rig["ch_l"])
        base = _tone_level(dev, rig)
        chk.ok(base > -20.0, f"signal present before mute ({base:.1f} dBFS)")
        dev.set_u8(OP.SET_OUTPUT_MUTE, 1, wvalue=out_l); dev.wait_ready()
        muted = _tone_level(dev, rig)
        chk.ok(muted < MUTE_FLOOR_DBFS, f"muted output is silent ({muted:.1f} dBFS)")
        dev.set_u8(OP.SET_OUTPUT_MUTE, 0, wvalue=out_l); dev.wait_ready()
        chk.approx(_tone_level(dev, rig), base, LEVEL_TOL_DB, "unmute restores level")
        chk.note(f"mute: base={base:.1f} muted={muted:.1f} dBFS")
    finally:
        dev.set_u8(OP.SET_OUTPUT_MUTE, 0, wvalue=out_l); dev.wait_ready()


@test("audio", mutating=True)
def level_controls(dev, profile, chk):
    """Master volume, user volume, and per-input preamp each scale level by the set dB."""
    rig = _get_rig(dev, profile)
    try:
        _flatten_chain(dev, rig["ch_l"])
        base = _tone_level(dev, rig)
        for op, label in ((OP.SET_MASTER_VOLUME, "master volume"),
                          (OP.SET_USER_VOLUME, "user volume")):
            dev.set_f32(op, -6.0); dev.wait_ready()
            chk.approx(_tone_level(dev, rig) - base, -6.0, LEVEL_TOL_DB, f"{label} -6 dB")
            dev.set_f32(op, 0.0); dev.wait_ready()
        dev.set_f32(OP.SET_PREAMP_CH, -6.0, wvalue=0); dev.wait_ready()  # input 0 feeds the captured leg
        chk.approx(_tone_level(dev, rig) - base, -6.0, LEVEL_TOL_DB, "preamp ch0 -6 dB")
        dev.set_f32(OP.SET_PREAMP_CH, 0.0, wvalue=0); dev.wait_ready()
        chk.note(f"level_controls base={base:.2f}dBFS")
    finally:
        dev.set_f32(OP.SET_MASTER_VOLUME, 0.0)
        dev.set_f32(OP.SET_USER_VOLUME, 0.0)
        dev.set_f32(OP.SET_PREAMP_CH, 0.0, wvalue=0)
        dev.wait_ready()


@test("audio", mutating=True)
def matrix_routing(dev, profile, chk):
    """Matrix crosspoint enable: a routed input reaches the output; unrouted is silent."""
    rig = _get_rig(dev, profile)
    out_l = _slot_indices(profile, rig["slot"])[0]
    try:
        _flatten_chain(dev, rig["ch_l"])
        base = _tone_level(dev, rig)
        chk.ok(base > -20.0, f"routed crosspoint passes ({base:.1f} dBFS)")
        _route(dev, 0, out_l, 0); dev.wait_ready()
        off = _tone_level(dev, rig)
        chk.ok(off < MUTE_FLOOR_DBFS, f"unrouted crosspoint is silent ({off:.1f} dBFS)")
        _route(dev, 0, out_l, 1, 0.0); dev.wait_ready()
        chk.approx(_tone_level(dev, rig), base, LEVEL_TOL_DB, "re-route restores level")
        chk.note(f"routing: on={base:.1f} off={off:.1f} dBFS")
    finally:
        _route(dev, 0, out_l, 1, 0.0); dev.wait_ready()


@test("audio", mutating=True)
def matrix_phase_invert(dev, profile, chk):
    """Matrix phase-invert flips output polarity (the fitted path-gain sign flips)."""
    rig = _get_rig(dev, profile)
    out_l = _slot_indices(profile, rig["slot"])[0]
    try:
        _flatten_chain(dev, rig["ch_l"])
        _route(dev, 0, out_l, 1, 0.0, 0); dev.wait_ready()
        _r, scale_n = audio.bit_exact_residual(rig["out"], rig["in"], rig["chan"], rig["fs"])
        _route(dev, 0, out_l, 1, 0.0, 1); dev.wait_ready()
        _r, scale_i = audio.bit_exact_residual(rig["out"], rig["in"], rig["chan"], rig["fs"])
        chk.ok(scale_n * scale_i < 0,
               f"phase-invert flips polarity (scale {scale_n:+.3f} -> {scale_i:+.3f})")
        chk.note(f"phase_invert: scale normal={scale_n:+.3f} inverted={scale_i:+.3f}")
    finally:
        _route(dev, 0, out_l, 1, 0.0, 0); dev.wait_ready()


@test("audio", mutating=True)
def output_delay(dev, profile, chk):
    """Per-output delay shifts that output by the set sample count (vs an undelayed leg)."""
    rig = _get_rig(dev, profile)
    out_l, out_r, ch_l, ch_r = _slot_indices(profile, rig["slot"])
    delay_ms = 5.0
    expect = round(delay_ms * rig["fs"] / 1000.0)   # 240 samples @ 48 kHz
    try:
        _flatten_chain(dev, ch_l); _flatten_chain(dev, ch_r)
        for out in range(profile.num_output_channels):   # USB L -> both legs only
            _route(dev, 0, out, 0)
            _route(dev, 1, out, 0)
        _route(dev, 0, out_l, 1, 0.0)
        _route(dev, 0, out_r, 1, 0.0)
        dev.set_f32(OP.SET_OUTPUT_DELAY, 0.0, wvalue=out_l)
        dev.set_f32(OP.SET_OUTPUT_DELAY, 0.0, wvalue=out_r)
        dev.wait_ready()
        lag_base, _ = audio.measure_interchannel_lag(rig["out"], rig["in"], rig["fs"])
        dev.set_f32(OP.SET_OUTPUT_DELAY, delay_ms, wvalue=out_l); dev.wait_ready()
        lag_delayed, _ = audio.measure_interchannel_lag(rig["out"], rig["in"], rig["fs"])
        delta = lag_delayed - lag_base
        chk.approx(delta, expect, 2,
                   f"output delay {delay_ms:g} ms = {expect} samples (measured {delta})")
        chk.note(f"output_delay: lag_base={lag_base} lag_delayed={lag_delayed} delta={delta} expect={expect}")
    finally:
        dev.set_f32(OP.SET_OUTPUT_DELAY, 0.0, wvalue=out_l)
        dev.set_f32(OP.SET_OUTPUT_DELAY, 0.0, wvalue=out_r)
        dev.wait_ready()


# --- Phase 3: alignment / latency stability ---------------------------------
#
# The DSPi capture taps ONE output slot (slot 0), so this verifies INTRA-slot
# L/R sample alignment and that it survives the pipeline-reset operations the
# firmware's "output slot alignment is inviolable" guarantee covers. Full
# INTER-slot alignment (the 4 S/PDIF + I2S + PDM relative to each other) needs
# multichannel / second-receiver capture and is out of scope for this rig.

ALIGN_TOL_SAMPLES = 1
INPUT_SPDIF = 1


def _lr_lag(dev, profile, rig):
    """(lag, strength) for the slot's L vs R, with the rig's standard 1:1
    routing restored and the chain flat. Identical signal feeds both legs, so
    lag ~ 0 and strength ~ 1 when L and R are sample-aligned."""
    _config_slot(dev, profile, rig["slot"])
    _flatten_chain(dev, rig["ch_l"])
    _flatten_chain(dev, rig["ch_r"])
    dev.wait_ready()
    return audio.measure_interchannel_lag(rig["out"], rig["in"], rig["fs"])


@test("audio", mutating=True)
def slot_lr_alignment(dev, profile, chk):
    """The captured S/PDIF slot's L and R channels are sample-aligned."""
    rig = _get_rig(dev, profile)
    lag, strength = _lr_lag(dev, profile, rig)
    chk.ok(strength > 0.5, f"L/R present & correlated (corr {strength:.2f})")
    chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R aligned: lag {lag} samples (<= {ALIGN_TOL_SAMPLES})")
    chk.note(f"slot_lr_alignment: lag={lag} corr={strength:.2f}")


@test("audio", mutating=True)
def alignment_after_input_switch(dev, profile, chk):
    """Input-source switch (USB -> S/PDIF -> USB) preserves L/R sample alignment."""
    rig = _get_rig(dev, profile)
    try:
        dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_SPDIF); dev.wait_ready()
        dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_USB); dev.wait_ready()
        lag, strength = _lr_lag(dev, profile, rig)
        chk.ok(strength > 0.5, f"signal restored after input switch (corr {strength:.2f})")
        chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R still aligned: lag {lag} samples")
        chk.note(f"after_input_switch: lag={lag} corr={strength:.2f}")
    finally:
        dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_USB); dev.wait_ready()


@test("audio", mutating=True)
def alignment_after_output_type_switch(dev, profile, chk):
    """Output-type switch (S/PDIF -> I2S -> S/PDIF) on the slot preserves L/R alignment."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    try:
        st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_I2S << 8) | slot); dev.wait_ready()
        if st != 0:
            raise Skip(f"output-type switch to I2S unavailable (status {st})")
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | slot); dev.wait_ready()
        lag, strength = _lr_lag(dev, profile, rig)
        chk.ok(strength > 0.5, f"signal restored after type switch (corr {strength:.2f})")
        chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R still aligned: lag {lag} samples")
        chk.note(f"after_type_switch: lag={lag} corr={strength:.2f}")
    finally:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | slot); dev.wait_ready()


# --- Phase 3b: per-output-type audio integrity ------------------------------
#
# The loopback taps slot 0's finalized DSP output BEFORE the S/PDIF or I2S
# encoder, so the captured samples are identical for either output type. That
# makes signal PRESENCE a real liveness probe for the active output path: if the
# I2S consumer is not draining slot 0's producer pool, the pool backs up, the DSP
# callback starves, and the tone vanishes from the capture. These tests measure
# real audio with slot 0 as I2S, and stress repeated type switches (which
# exercise the shared-DMA-channel teardown/re-setup path).


def _i2s_available(dev, slot):
    """True if slot can be put into I2S output mode (restores SPDIF either way)."""
    st = dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_I2S << 8) | slot)
    dev.wait_ready()
    if st != 0:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | slot)
        dev.wait_ready()
        return False
    return True


@test("audio", mutating=True)
def output_type_i2s_audio(dev, profile, chk):
    """Slot 0 as I2S output: real audio still reaches the capture at unity with
    low THD, near bit-exact, and L/R sample-aligned. Because the tap is
    pre-encoder, a dead I2S output path would stall the slot-0 producer and the
    tone would disappear; a clean tone confirms the I2S consumer is draining."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    try:
        if not _i2s_available(dev, slot):
            raise Skip("output-type switch to I2S unavailable (check BCK pin / status)")
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_I2S)
        time.sleep(TYPE_SWITCH_SETTLE_S)   # let the I2S clock + loopback servo re-prime
        amp = 0.4
        level, thd, strength = audio.measure_tone(rig["out"], rig["in"], rig["chan"],
                                                  rig["fs"], 1000.0, amp=amp)
        chk.ok(strength > CORR_MIN, f"I2S: tone reaches capture (corr {strength:.2f})")
        exp_level = 20.0 * np.log10(amp / np.sqrt(2.0))
        chk.approx(level, exp_level, 1.0, f"I2S: tone level ~{exp_level:.1f} dBFS")
        chk.ok(thd < 0.1, f"I2S: THD {thd:.4f}% < 0.1%")
        lag, _st = audio.measure_interchannel_lag(rig["out"], rig["in"], rig["fs"])
        chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"I2S: L/R aligned (lag {lag} samples <= {ALIGN_TOL_SAMPLES})")
        # Bit-exactness + unity gain of the (pre-encoder, type-agnostic) capture,
        # single-shot to the same standard as loopback_integrity.  The 2 s settle
        # after the type switch lets the I2S clock + loopback servo fully re-prime
        # so the capture is clean without any retry.
        resid, scale = audio.bit_exact_residual(rig["out"], rig["in"], rig["chan"], rig["fs"])
        chk.ok(resid < RESIDUAL_MAX_DBFS, f"I2S: flat-path residual {resid:.1f} dBFS < {RESIDUAL_MAX_DBFS}")
        gain_db = 20.0 * np.log10(abs(scale) + 1e-20)
        chk.approx(gain_db, 0.0, GAIN_TOL_DB, f"I2S: path gain ~0 dB (|scale| {abs(scale):.4f})")
        chk.note(f"i2s_audio: level={level:.2f}dBFS thd={thd:.4f}% resid={resid:.1f}dBFS "
                 f"|scale|={abs(scale):.4f} lag={lag}")
    finally:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | slot); dev.wait_ready()
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_SPDIF)
        dev.wait_ready()


@test("audio", mutating=True)
def output_type_switch_stress(dev, profile, chk):
    """Repeated SPDIF<->I2S switching on slot 0 keeps the output alive, at unity,
    and L/R sample-aligned after EVERY switch. Exercises the shared-DMA-channel
    teardown/re-setup path (audio_spdif_teardown + full SPDIF re-setup, I2S
    setup/teardown reclaiming the same channel); a double-claim panic, a stalled
    pipeline, or lost inter-leg alignment would surface here as lost signal, a
    bad lag, or a device reset (disconnect)."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    if not _i2s_available(dev, slot):
        raise Skip("output-type switch to I2S unavailable (check BCK pin / status)")
    cycles = 4
    try:
        for i in range(cycles):
            for otype, name in ((TYPE_SPDIF, "SPDIF"), (TYPE_I2S, "I2S")):
                _config_slot(dev, profile, slot, otype=otype)
                _flatten_chain(dev, rig["ch_l"])
                _flatten_chain(dev, rig["ch_r"])
                dev.wait_ready()
                lag, strength = audio.measure_interchannel_lag(rig["out"], rig["in"], rig["fs"])
                level = _tone_level(dev, rig)
                chk.ok(strength > 0.5, f"cycle {i} -> {name}: signal present (corr {strength:.2f})")
                chk.ok(level > -20.0, f"cycle {i} -> {name}: output at level ({level:.1f} dBFS)")
                chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"cycle {i} -> {name}: L/R aligned (lag {lag})")
        chk.note(f"switch_stress: {cycles}x SPDIF<->I2S, signal+level+alignment intact after every switch")
    finally:
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(TYPE_SPDIF << 8) | slot); dev.wait_ready()
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_SPDIF)
        dev.wait_ready()


# --- Phase 4: full chain / dynamics -----------------------------------------

def _flatten_master(dev, profile):
    """Flatten the stereo-bus EQ (input channels 0/1) so loudness/crossfeed/
    leveller (which sit on the stereo input bus) are measured against a flat
    baseline."""
    for ch in (0, 1):
        for b in range(profile.band_ceiling):
            _set_band(dev, ch, b, FLAT, 1000.0, 0.707, 0.0)


def _require_stereo_input(dev):
    """loudness, leveller, and crossfeed are the inherently-stereo chain; the
    firmware bypasses them whenever more than 2 input channels are active (see
    audio_pipeline.c `multichannel`).  Many hosts (e.g. macOS CoreAudio) open the
    DSPi at its advertised 8-channel maximum, which holds the device in
    multichannel mode where these effects never run — so skip, rather than fail,
    when that is the case (the effect genuinely cannot be exercised here)."""
    nin = dev.get_u32(OP.GET_STATUS, wvalue=23)   # live active USB input channel count
    if nin > 2:
        raise Skip(f"stereo-only effect not applicable: host holds {nin}-channel input "
                   f"active (loudness/leveller/crossfeed bypassed in multichannel mode)")


@test("audio", mutating=True)
def multiband_eq(dev, profile, chk):
    """Several simultaneous PEQ bands compose to the sum (in dB) of their responses."""
    rig = _get_rig(dev, profile)
    cfgs = [(0, "lowshelf", 150.0, 0.707, 6.0),
            (1, "peaking", 1000.0, 2.0, -6.0),
            (2, "highshelf", 6000.0, 0.707, 4.0)]
    try:
        for b in range(profile.band_ceiling):
            _set_band(dev, rig["ch_l"], b, FLAT, 1000.0, 0.707, 0.0)
        _set_band(dev, rig["ch_l"], XO_BAND, FLAT, 1000.0, 0.707, 0.0)
        for (b, t, fc, q, g) in cfgs:
            _set_band(dev, rig["ch_l"], b, TYPE[t], fc, q, g)
        dev.wait_ready()
        freqs = _test_freqs(rig["fs"])
        mag, _p, _s = audio.measure_transfer(rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=0.25)
        exp = np.zeros_like(freqs, dtype=float)
        for (b, t, fc, q, g) in cfgs:
            em, _ = _expected(t, fc, q, g, rig["fs"], freqs)
            exp = exp + em
        err = float(np.max(np.abs(mag - exp)))
        chk.ok(err < 0.7, f"3-band EQ FR matches sum-of-bands: max err {err:.3f} dB")
        chk.note(f"multiband_eq: max_err={err:.3f}dB")
    finally:
        for b in range(profile.band_ceiling):
            _set_band(dev, rig["ch_l"], b, FLAT, 1000.0, 0.707, 0.0)
        dev.wait_ready()


@test("audio", mutating=True)
def loudness_shape(dev, profile, chk):
    """Loudness compensation at low volume boosts bass and treble vs the mid band."""
    rig = _get_rig(dev, profile)
    _require_stereo_input(dev)
    try:
        _flatten_chain(dev, rig["ch_l"]); _flatten_master(dev, profile)
        dev.set_f32(OP.SET_USER_VOLUME, -40.0)
        dev.set_u8(OP.SET_LOUDNESS, 0); dev.wait_ready()
        freqs = _test_freqs(rig["fs"])
        off, _p, _s = audio.measure_transfer(rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=0.4)
        dev.set_u8(OP.SET_LOUDNESS, 1); dev.wait_ready()
        on, _p, _s = audio.measure_transfer(rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=0.4)
        diff = on - off
        mid = diff[int(np.argmin(np.abs(freqs - 1000.0)))]
        lf = diff[int(np.argmin(np.abs(freqs - 60.0)))] - mid
        hf = diff[int(np.argmin(np.abs(freqs - 12000.0)))] - mid
        chk.ok(lf > 2.0, f"loudness boosts bass at low volume (+{lf:.1f} dB @60Hz vs mid)")
        chk.ok(hf > 1.0, f"loudness boosts treble (+{hf:.1f} dB @12kHz vs mid)")
        chk.note(f"loudness_shape: LF +{lf:.1f} HF +{hf:.1f} dB vs mid @ -40 dB vol")
    finally:
        dev.set_u8(OP.SET_LOUDNESS, 0); dev.set_f32(OP.SET_USER_VOLUME, 0.0); dev.wait_ready()


@test("audio", mutating=True)
def crossfeed_bleed(dev, profile, chk):
    """Crossfeed mixes a (filtered, attenuated) copy of one channel into the opposite."""
    rig = _get_rig(dev, profile)
    _require_stereo_input(dev)
    try:
        _config_slot(dev, profile, rig["slot"])
        _flatten_chain(dev, rig["ch_l"]); _flatten_chain(dev, rig["ch_r"]); _flatten_master(dev, profile)
        dev.set_u8(OP.SET_CROSSFEED, 0); dev.wait_ready()
        l_off, r_off, _ = audio.measure_tone_2ch(rig["out"], rig["in"], rig["fs"], 200.0, amp=0.4, left_only=True)
        dev.set_u8(OP.SET_CROSSFEED, 1); dev.wait_ready()
        l_on, r_on, _ = audio.measure_tone_2ch(rig["out"], rig["in"], rig["fs"], 200.0, amp=0.4, left_only=True)
        chk.ok(r_off < -50.0, f"crossfeed off: opposite channel silent ({r_off:.1f} dBFS)")
        chk.ok(r_on > r_off + 15.0, f"crossfeed on: bleed into opposite channel ({r_off:.1f} -> {r_on:.1f} dBFS)")
        chk.ok(r_on < l_on - 1.0, f"bleed attenuated vs direct (R {r_on:.1f} < L {l_on:.1f} dBFS)")
        chk.note(f"crossfeed: off R={r_off:.1f} on R={r_on:.1f} direct L={l_on:.1f} dBFS")
    finally:
        dev.set_u8(OP.SET_CROSSFEED, 0); dev.wait_ready()


@test("audio", mutating=True)
def leveller_boost(dev, profile, chk):
    """The leveller lifts a sustained quiet signal, bounded by the max-gain ceiling."""
    rig = _get_rig(dev, profile)
    _require_stereo_input(dev)
    try:
        _flatten_chain(dev, rig["ch_l"]); _flatten_master(dev, profile)
        dev.set_u8(OP.SET_LEVELLER_ENABLE, 0); dev.wait_ready()
        off, _r, _t = audio.measure_tone_2ch(rig["out"], rig["in"], rig["fs"], 1000.0, dur_s=1.2, amp=0.05)
        dev.set_f32(OP.SET_LEVELLER_AMOUNT, 100.0)
        dev.set_f32(OP.SET_LEVELLER_MAX_GAIN, 12.0)
        dev.set_u8(OP.SET_LEVELLER_SPEED, 2)
        dev.set_u8(OP.SET_LEVELLER_ENABLE, 1); dev.wait_ready()
        on, _r, _t = audio.measure_tone_2ch(rig["out"], rig["in"], rig["fs"], 1000.0, dur_s=1.2, amp=0.05)
        boost = on - off
        chk.ok(boost > 3.0, f"leveller lifts quiet signal (+{boost:.1f} dB)")
        chk.ok(boost <= 13.0, f"boost within max-gain ceiling (+{boost:.1f} dB <= ~12)")
        chk.note(f"leveller: quiet off={off:.1f} on={on:.1f} boost=+{boost:.1f} dB")
    finally:
        dev.set_u8(OP.SET_LEVELLER_ENABLE, 0)
        dev.set_f32(OP.SET_LEVELLER_AMOUNT, 0.0)
        dev.wait_ready()


@test("audio", mutating=True)
def output_clip_limit(dev, profile, chk):
    """Driving the output past 0 dBFS clamps at full scale (no wrap) and raises THD."""
    rig = _get_rig(dev, profile)
    try:
        _flatten_chain(dev, rig["ch_l"])
        lvl0, thd0, _ = audio.measure_tone(rig["out"], rig["in"], rig["chan"], rig["fs"], 1000.0, amp=0.5)
        _set_band(dev, rig["ch_l"], 0, TYPE["peaking"], 1000.0, 1.0, 12.0)  # +12 dB pushes past full scale
        dev.wait_ready()
        lvl1, thd1, _ = audio.measure_tone(rig["out"], rig["in"], rig["chan"], rig["fs"], 1000.0, amp=0.5)
        chk.ok(lvl1 > lvl0 + 3.0, f"+12 dB boost takes effect ({lvl0:.1f} -> {lvl1:.1f} dBFS)")
        chk.ok(lvl1 < 0.5, f"output clamped at full scale, not wrapped ({lvl1:.2f} dBFS)")
        chk.ok(thd1 > thd0 + 1.0, f"clipping raises THD ({thd0:.3f}% -> {thd1:.3f}%)")
        chk.note(f"clip: clean {lvl0:.1f}dBFS/{thd0:.3f}% clipped {lvl1:.1f}dBFS/{thd1:.3f}%")
    finally:
        _set_band(dev, rig["ch_l"], 0, FLAT, 1000.0, 0.707, 0.0); dev.wait_ready()
