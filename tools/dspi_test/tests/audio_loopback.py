"""
audio_loopback.py — hardware-in-the-loop output & filter tests (group "audio").

Unlike the rest of the suite (control-plane only), these tests measure real
audio: a host signal is played out the DSPi USB audio output, processed by the
DSP, captured back from the Weeb Labs USBrx (wired to DSPi's S/PDIF output as a
USB audio input), and the measured result is compared against the firmware's
filter math (tools/filter_tester/compare_filter.py).

Gating: this group is EXCLUDED from the default run; enable with `--audio`
(or `--group audio`). It needs the loopback rig plus `sounddevice`+`numpy`+`scipy`
(`pip install sounddevice numpy scipy`). If the deps or devices are absent the
tests SKIP (never hard-fail).

Routing to the USBrx-connected output slot is auto-probed once per session.
State is mutated freely; the suite's pre-suite snapshot is restored at the end.
"""

from __future__ import annotations

import struct

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

# Magnitude-shaping PEQ configs: (name, rbj_name, fc, Q, gain_db). Frequencies
# straddle the RP2350 SVF/biquad boundary (Fs/7.5 ~= 6400 Hz @ 48 kHz).
PEQ_CONFIGS = [
    ("peaking_lo",  "peaking",    300.0, 2.0,   6.0),
    ("peaking_hi",  "peaking",   9000.0, 2.0,   6.0),
    ("peaking_cut", "peaking",   1000.0, 1.0,  -6.0),
    ("lowshelf",    "lowshelf",   200.0, 0.707, 6.0),
    ("highshelf",   "highshelf", 8000.0, 0.707, 6.0),
    ("lowpass",     "lowpass",   2000.0, 0.707, 0.0),
    ("highpass",    "highpass",   200.0, 0.707, 0.0),
    ("notch",       "notch",     1000.0, 4.0,   0.0),
    ("lowshelf1",   "lowshelf1",  200.0, 0.707, 6.0),
    ("highshelf1",  "highshelf1",5000.0, 0.707, 6.0),
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


def _slot_indices(slot):
    """For S/PDIF slot `s`: (out_l, out_r) matrix/enable indices and
    (ch_l, ch_r) EQ channel indices. Channels 0,1 are the master bus."""
    out_l, out_r = 2 * slot, 2 * slot + 1
    return out_l, out_r, out_l + 2, out_r + 2


def _baseline(dev):
    """Clean, deterministic pre-conditions: USB input, unity gains."""
    dev.set_u8(OP.SET_INPUT_SOURCE, INPUT_USB)
    dev.wait_ready()
    dev.set_f32(OP.SET_MASTER_VOLUME, 0.0)
    for ch in (0, 1):
        dev.set_f32(OP.SET_PREAMP_CH, 0.0, wvalue=ch)


def _route_only(dev, profile, out_l, out_r):
    """Route USB L/R 1:1 to exactly the (out_l, out_r) pair at 0 dB and disable
    USB -> every other output, so only this output pair carries signal. Matrix
    writes apply immediately (no deferred reset)."""
    for out in range(profile.num_output_channels):
        _route(dev, 0, out, 0)
        _route(dev, 1, out, 0)
    _route(dev, 0, out_l, 1, 0.0)   # USB L -> target L
    _route(dev, 1, out_r, 1, 0.0)   # USB R -> target R


def _config_slot(dev, profile, slot, flatten_all=False):
    """Make `slot` the only S/PDIF output carrying USB audio (1:1, 0 dB).
    Returns (ch_l, ch_r). With flatten_all, zero every band on both channels."""
    out_l, out_r, ch_l, ch_r = _slot_indices(slot)
    dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(0 << 8) | slot)   # 0 = SPDIF
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_l)
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_r)
    dev.wait_ready()
    _route_only(dev, profile, out_l, out_r)
    if flatten_all:
        for ch in (ch_l, ch_r):
            for b in range(profile.band_ceiling):
                _set_band(dev, ch, b, FLAT, 1000.0, 0.707, 0.0)
    dev.wait_ready()
    return ch_l, ch_r


def _autoprobe_slot(dev, profile, out_dev, in_dev, fs):
    """Find which S/PDIF slot the USBrx receives. Each slot is probed in
    ISOLATION (USB routed to only that slot, all others muted), so only the
    physically-connected slot shows signal — otherwise an earlier-routed slot
    keeps carrying audio and every later probe would falsely succeed."""
    for k in range(profile.num_spdif):     # enable all S/PDIF outputs once
        ol, orr = 2 * k, 2 * k + 1
        dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(0 << 8) | k)
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=ol)
        dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=orr)
    dev.wait_ready()

    best, best_lvl = None, -200.0
    for slot in range(profile.num_spdif):
        out_l, out_r, _cl, _cr = _slot_indices(slot)
        _route_only(dev, profile, out_l, out_r)
        level, _thd, strength = audio.measure_tone(out_dev, in_dev, 0, fs, 1000.0, amp=0.3)
        if strength > CORR_MIN and level > best_lvl:
            best, best_lvl = slot, level
    if best is None:
        raise Skip("no S/PDIF slot reached USBrx — check cabling / output routing")
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

    _baseline(dev)
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
    """Flat path: signal reaches USBrx at unity, low noise/THD, near bit-exact."""
    rig = _get_rig(dev, profile)
    chk.note(f"out='{rig['out_name']}' in='{rig['in_name']}' slot={rig['slot']} "
             f"ch_l={rig['ch_l']} fs={rig['fs']}")
    _set_band(dev, rig["ch_l"], 0, FLAT, 1000.0, 0.707, 0.0)
    dev.wait_ready()

    amp = 0.4
    level, thd, strength = audio.measure_tone(rig["out"], rig["in"], rig["chan"],
                                              rig["fs"], 1000.0, amp=amp)
    chk.ok(strength > CORR_MIN, f"tone reaches USBrx (corr {strength:.2f})")
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
    def fn(dev, profile, chk):
        rig = _get_rig(dev, profile)
        _set_band(dev, rig["ch_l"], 0, TYPE[rbj_name], fc, q, gain)
        dev.wait_ready()
        freqs = _test_freqs(rig["fs"])
        mag, _phase, strength = audio.measure_transfer(
            rig["out"], rig["in"], rig["chan"], rig["fs"], freqs, amp=_signal_amp(gain))
        chk.ok(strength > CORR_MIN, f"signal present (corr {strength:.2f})")
        exp_mag, _ = _expected(rbj_name, fc, q, gain, rig["fs"], freqs)
        err = float(np.max(np.abs(mag - exp_mag)))
        chk.ok(err < MAG_TOL_DB,
               f"{name} fc={fc:g} Q={q:g} gain={gain:g}: max |mag err| {err:.3f} dB < {MAG_TOL_DB}")
        chk.note(f"{name}: max_mag_err={err:.3f}dB corr={strength:.2f}")
    fn.__name__ = f"peq_{name}"
    fn.__doc__ = f"PEQ {rbj_name} fc={fc:g} Q={q:g} gain={gain:g}: measured FR matches RBJ reference."
    return test("audio", mutating=True)(fn)


# Register one test per PEQ config (distinct names for per-config reporting).
for _name, _rbj, _fc, _q, _gain in PEQ_CONFIGS:
    globals()[f"peq_{_name}"] = _make_peq_test(_name, _rbj, _fc, _q, _gain)
