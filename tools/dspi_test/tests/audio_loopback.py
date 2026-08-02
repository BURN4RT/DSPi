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

import os
import struct
import time

from ..device import OP, Stall, Timeout
from ..framework import test, Skip, REGISTRY
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

# --- Operating rates --------------------------------------------------------
#
# DSPi follows the host USB rate, and the loopback capture function advertises
# 44.1 and 48 kHz (usb_descriptors.c), so both are real operating points that
# have to be verified.  Running the whole matrix at both roughly doubles a
# multi-minute run for little extra signal, so the default is the full matrix at
# the primary rate plus a targeted second pass (SECONDARY_TESTS) at 44.1 kHz.
#
# The device's playback function also advertises 96 kHz on the stereo alt, but
# the capture does not and the servo's 52-frame packet ceiling cannot carry it;
# _require_rate() skips rather than measuring garbage there.
FS = audio.DEFAULT_FS       # 48 kHz, the primary rate
FS_ALT = 44100              # secondary rate

# run.py --audio-rates sets this to run the FULL matrix at each listed rate
# instead (and then no secondary subset, since it would be redundant).
_RATES_ENV = os.environ.get("DSPI_AUDIO_RATES", "").replace(",", " ").split()
RATES_OVERRIDE = [int(r) for r in _RATES_ENV] if _RATES_ENV else None
PRIMARY_RATES = RATES_OVERRIDE or [FS]
SECONDARY_RATES = [] if RATES_OVERRIDE else [FS_ALT]

# Suffix for a rate's generated test names.  Empty at the first primary rate so
# existing names (and saved baseline reports) stay comparable.
_RATE_TAGS = {48000: "48k", 44100: "44k1", 96000: "96k"}


def _rate_tag(fs):
    return "" if fs == PRIMARY_RATES[0] else "_" + _RATE_TAGS.get(fs, str(fs))

# FilterType enum (firmware) by RBJ-reference name.
TYPE = {"peaking": 1, "lowshelf": 2, "highshelf": 3, "lowpass": 4, "highpass": 5,
        "notch": 6, "allpass": 7, "allpass1": 8, "lowshelf1": 9, "highshelf1": 10,
        "lowpass1":12, "highpass1":13}

FLAT = 0
INPUT_USB = 0
# Crossover bands live above the PEQ block at wire indices
# [XOVER_BAND_BASE .. XOVER_BAND_BASE + MAX_XOVER_BANDS - 1] (config.h).
XO_BAND = 20                  # XOVER_BAND_BASE
XO_BANDS = 4                  # MAX_XOVER_BANDS
TYPE_SPDIF, TYPE_I2S = 0, 1   # OutputType enum (firmware)

# GET_STATUS sub-indices (vendor_commands.c REQ_GET_STATUS switch).
STATUS_SAMPLE_RATE = 15       # audio_state.freq, the live operating rate
STATUS_INPUT_COUNT = 23       # active_input_channel_count()

# Feature-specific constants used to neutralize a stage (firmware headers).
UPMIX_PARAM_ENABLED = 0       # upmix.h
SIGGEN_CTL_STOP_NOW = 2       # siggen.h: immediate hard stop, no fade

# The loopback capture function advertises 44.1/48 kHz only, and the servo can
# emit at most LOOPBACK_MAX_FRAMES_PER_PACKET (52) stereo frames per USB frame.
# The playback function also advertises 96 kHz, so the device can legitimately be
# running at a rate the capture cannot carry; measuring there yields garbage.
CAPTURE_MAX_RATE = 48000

# Magnitude-shaping PEQ configs: (name, rbj_name, fc, Q, gain_db). Frequencies
# straddle the RP2350 SVF/biquad boundary (Fs/7.5 ~= 6400 Hz @ 48 kHz).

PEQ_CONFIGS = [
    ("lowpass1_lo",   "lowpass1",      300.0, 0.707, 0.0),
    ("lowpass1_hi",   "lowpass1",     9000.0, 0.707, 0.0),
    ("highpass1_lo",  "highpass1",     300.0, 0.707, 0.0),
    ("highpass1_hi",  "highpass1",    9000.0, 0.707, 0.0),
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


# --- Deferred-operation barriers --------------------------------------------
#
# dev.wait_ready() proves only that the device still answers control transfers:
# it returns on the FIRST successful poll.  Output-type switches, input-source
# switches and rate changes are all DEFERRED to the main loop (the vendor
# handler just arms a flag; main.c's process_type_switches() / input-source
# handler applies it behind pipeline_reset_ready()), so wait_ready() routinely
# returns before the operation has even started.
#
# Worse, both SET handlers compare against the APPLIED state and drop the
# request when it already matches: the type switch returns success for
# `new_type == output_types[slot]`, and the input switch arms only when
# `src != active_input_source`.  A round trip issued faster than the main loop
# applies the first leg is therefore compared against stale state and silently
# swallowed, leaving the device in the intermediate mode while the test believes
# it was restored.
#
# So: poll the applied state instead.  Because these helpers never return with a
# switch still pending, the entry-time read is always accurate and an unchanged
# request is a genuine no-op (no reset, no settle).

BARRIER_TIMEOUT_S = 4.0
BARRIER_POLL_S = 0.05

# After an operation that crossed a pipeline reset the DSP callback stalls, the
# loopback ring drains, and the capture servo takes its underrun path: it emits
# silence packets until the ring refills to TARGET_FILL_FRAMES (loopback.c).
# Measuring inside that window reads a silence gap and a splice, not the DSP.
PIPELINE_SETTLE_S = 2.0


def _wait_applied(getter, expected, timeout_s=None, poll_s=None):
    """Poll `getter()` until it reports `expected`; return the last value read.

    Transient stalls/timeouts are tolerated: a deferred reset briefly disables
    the USB control IRQ, so a transfer in that window legitimately fails.

    The bounds default to None and resolve to the module constants HERE rather
    than in the signature, so raising BARRIER_TIMEOUT_S actually takes effect
    (a default argument would freeze the value at import time).
    """
    timeout_s = BARRIER_TIMEOUT_S if timeout_s is None else timeout_s
    poll_s = BARRIER_POLL_S if poll_s is None else poll_s
    deadline = time.monotonic() + timeout_s
    last = None
    while True:
        try:
            last = getter()
            if last == expected:
                return last
        except (Stall, Timeout):
            pass
        if time.monotonic() >= deadline:
            return last
        time.sleep(poll_s)


def _read_settled(getter, timeout_s=None):
    """Read a device value, tolerating a transient control-IRQ blackout left by
    an operation that is still settling.  Returns None if it never answers.

    Every entry-time read below needs this: these helpers are routinely called
    right after a previous deferred reset, and a bare GET in that window would
    escape as "unexpected STALL" and turn the test into an ERROR.
    """
    timeout_s = BARRIER_TIMEOUT_S if timeout_s is None else timeout_s
    deadline = time.monotonic() + timeout_s
    while True:
        try:
            return getter()
        except (Stall, Timeout):
            if time.monotonic() >= deadline:
                return None
            time.sleep(BARRIER_POLL_S)


def _set_output_type(dev, slot, otype):
    """Switch `slot` to output type `otype` and wait until it is APPLIED.

    Returns True on success.  A request that matches the applied type is a
    no-op and costs one GET (this is the common case: _config_slot re-asserts
    the type on every call).
    """
    cur = _read_settled(lambda: dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=slot))
    if cur is None:
        return False        # device never answered
    if cur == otype:
        return True
    if dev.get_u8(OP.SET_OUTPUT_TYPE, wvalue=(otype << 8) | slot) != 0:
        return False        # rejected outright (bad slot / type / pin conflict)
    if _wait_applied(lambda: dev.get_u8(OP.GET_OUTPUT_TYPE, wvalue=slot), otype) != otype:
        return False        # accepted but never applied
    time.sleep(PIPELINE_SETTLE_S)
    return True


def _set_input_source(dev, src):
    """Switch the input source and wait until it is APPLIED.  Returns True on
    success.  The SET returns no status and input_source_selectable() can reject
    it silently (e.g. a disabled S/PDIF 2/3), so the read-back is also the only
    way to know the switch happened at all."""
    cur = _read_settled(lambda: dev.get_u8(OP.GET_INPUT_SOURCE))
    if cur is None:
        return False
    if cur == src:
        return True
    dev.set_u8(OP.SET_INPUT_SOURCE, src)
    if _wait_applied(lambda: dev.get_u8(OP.GET_INPUT_SOURCE), src) != src:
        return False
    time.sleep(PIPELINE_SETTLE_S)
    return True


def _device_rate(dev):
    """DSPi's live operating rate (audio_state.freq), or None if it will not
    answer (see _read_settled)."""
    return _read_settled(lambda: dev.get_u32(OP.GET_STATUS, wvalue=STATUS_SAMPLE_RATE))


def _set_rate(dev, out_dev, in_dev, fs):
    """Move DSPi to `fs` and wait until it is applied.  Returns True on success.

    DSPi has no vendor command for this: it follows the host USB rate, so the
    device only moves when the host issues the UAC1 SET_CUR, which happens when
    a stream opens at the new rate.  A short quiet tone is enough to open and
    close a stream pair there.  The rate change is then deferred to the main
    loop behind pipeline_reset_ready() like every other pipeline reset
    (main.c rate_change_pending), so the applied read-back is the barrier.
    """
    if _device_rate(dev) == fs:
        return True
    try:
        audio.measure_tone(out_dev, in_dev, 0, fs, 1000.0, amp=0.1)
    except Exception:  # noqa: BLE001
        # The host backend can refuse the rate outright (PortAudio raises when
        # the device will not open there).  That is a "cannot test at this rate"
        # condition, not a harness error, so let the caller turn it into a Skip.
        return False
    got = _wait_applied(lambda: dev.get_u32(OP.GET_STATUS, wvalue=STATUS_SAMPLE_RATE), fs)
    if got != fs:
        return False
    time.sleep(PIPELINE_SETTLE_S)
    return True


def _require_rate(dev, fs):
    """Skip unless the device is actually running at `fs`.

    Every reference in this module (RBJ/scipy magnitude and phase, the test
    frequency grid, the output-delay sample count) is computed from fs, and the
    capture streams at audio_state.freq regardless of what the host asked for
    (loopback.c); a mismatch is a silent pitch error that shows up as dozens of
    unexplained magnitude failures.  One Skip is far more useful.
    """
    actual = _device_rate(dev)
    if actual is None:
        raise Skip("device would not report its sample rate (still settling?)")
    if actual != fs:
        if actual > CAPTURE_MAX_RATE:
            raise Skip(f"device at {actual} Hz: above the loopback capture's "
                       f"{CAPTURE_MAX_RATE} Hz ceiling (servo cannot carry it)")
        raise Skip(f"device at {actual} Hz, tests reference {fs} Hz "
                   f"(set the host stream rate to {fs})")
    return actual


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

_RIG = {}        # fs -> rig dict, built on first use at that rate
_RIG_FAIL = {}   # fs -> reason, once we know that rate is unusable
_DEVS = None     # (out_dev, in_dev, info) from find_devices(), rate-independent

# Rate a _rate_variant() wrapper has selected for the test currently running.
# The framework runs tests strictly serially, so a module-level current-rate is
# safe; it lets an existing test body be replayed at another rate untouched.
_ACTIVE_FS = None


def _slot_indices(profile, slot):
    """For output slot `s`: (out_l, out_r) matrix/enable/gain indices (0-based
    output index) and (ch_l, ch_r) EQ channel indices.  In the unified channel
    model the EQ channel space is inputs [0..NUM_INPUT-1] followed by outputs, so
    output EQ channels start at NUM_INPUT_CHANNELS (2 on RP2040, 8 on RP2350) —
    NOT a fixed +2."""
    out_l, out_r = 2 * slot, 2 * slot + 1
    base = profile.num_input_channels
    return out_l, out_r, base + out_l, base + out_r


def _optional(fn, default=None):
    """Run a device call that a given platform may not implement, returning
    `default` on STALL (e.g. the upmixer, which is RP2350-only)."""
    try:
        return fn()
    except Stall:
        return default


def _baseline(dev, profile):
    """Put every stage that can perturb a measurement into a known-inert state.

    The audio group runs FIRST (run.py sorts it ahead of the control-plane
    tests) against whatever preset the device booted with, so anything NOT
    zeroed here is preset-dependent: a user preset with, say, psybass or a
    per-output delay enabled would fail the bit-exact and unity-gain checks with
    no indication why.  loopback_baseline_clean verifies the result.

    Deliberately non-destructive where possible: the per-input PEQ is disabled
    with the global bypass flag rather than by rewriting every band, so the
    preset's own EQ survives the run.
    """
    if not _set_input_source(dev, INPUT_USB):
        raise Skip("could not switch the input source to USB")

    # Gain stages.  Master volume powers on at MASTER_VOL_DEFAULT_DB (-20 dB),
    # and user volume / user mute are independent stages; all must be zeroed or
    # absolute-level measurements read low.
    dev.set_f32(OP.SET_MASTER_VOLUME, 0.0)
    dev.set_f32(OP.SET_USER_VOLUME, 0.0)
    dev.set_u8(OP.SET_USER_MUTE, 0)
    for ch in range(profile.num_input_channels):
        dev.set_f32(OP.SET_PREAMP_CH, 0.0, wvalue=ch)

    # Per-input PEQ off.  bypass_master_eq gates ONLY the per-input EQ pass
    # (audio_pipeline.c PASS 2, both platforms); output-channel EQ is untouched,
    # and that is what the filter tests drive.
    dev.set_u8(OP.SET_BYPASS, 1)

    # Effects that would colour, compress, or replace the signal.  Each sits on
    # a different part of the chain, so none of them is covered by the others:
    # leveller pre-matrix, upmixer into matrix rows 2..4, crossfeed per output
    # pair, psybass pre-crossover per output, loudness post-gain per output,
    # siggen injected post-matrix.
    dev.set_u8(OP.SET_LOUDNESS, 0)
    dev.set_u8(OP.SET_CROSSFEED, 0)
    dev.set_u8(OP.SET_LEVELLER_ENABLE, 0)
    dev.set_u8(OP.SET_PSYBASS, 0)
    _optional(lambda: dev.set_f32(OP.UPMIX_SET_PARAM, 0.0, wvalue=UPMIX_PARAM_ENABLED))
    _optional(lambda: dev.get_u8(OP.SIGGEN_CONTROL, wvalue=SIGGEN_CTL_STOP_NOW))

    # Output stage at unity on EVERY output, not just the target pair: a stray
    # delay or mute on the other leg would skew an inter-leg lag measurement.
    for out in range(profile.num_output_channels):
        dev.set_f32(OP.SET_OUTPUT_GAIN, 0.0, wvalue=out)
        dev.set_u8(OP.SET_OUTPUT_MUTE, 0, wvalue=out)
        dev.set_f32(OP.SET_OUTPUT_DELAY, 0.0, wvalue=out)
    dev.wait_ready()


def _neutral_state(dev, profile):
    """Read back what _baseline() set, as [(label, expected, actual), ...].

    Entries a platform does not implement are reported with actual=None by
    _optional() and skipped by the caller."""
    checks = [
        ("input source",   INPUT_USB, _optional(lambda: dev.get_u8(OP.GET_INPUT_SOURCE))),
        ("master volume",  0.0,  _optional(lambda: dev.get_f32(OP.GET_MASTER_VOLUME))),
        ("user volume",    0.0,  _optional(lambda: dev.get_f32(OP.GET_USER_VOLUME))),
        ("user mute",      0,    _optional(lambda: dev.get_u8(OP.GET_USER_MUTE))),
        ("input EQ bypass", 1,   _optional(lambda: dev.get_u8(OP.GET_BYPASS))),
        ("loudness",       0,    _optional(lambda: dev.get_u8(OP.GET_LOUDNESS))),
        ("crossfeed",      0,    _optional(lambda: dev.get_u8(OP.GET_CROSSFEED))),
        ("leveller",       0,    _optional(lambda: dev.get_u8(OP.GET_LEVELLER_ENABLE))),
        ("psybass",        0,    _optional(lambda: dev.get_u8(OP.GET_PSYBASS))),
    ]
    for ch in range(profile.num_input_channels):
        checks.append((f"preamp ch{ch}", 0.0,
                       _optional(lambda c=ch: dev.get_f32(OP.GET_PREAMP_CH, wvalue=c))))
    for out in range(profile.num_output_channels):
        checks.append((f"output {out} gain", 0.0,
                       _optional(lambda o=out: dev.get_f32(OP.GET_OUTPUT_GAIN, wvalue=o))))
        checks.append((f"output {out} mute", 0,
                       _optional(lambda o=out: dev.get_u8(OP.GET_OUTPUT_MUTE, wvalue=o))))
        checks.append((f"output {out} delay", 0.0,
                       _optional(lambda o=out: dev.get_f32(OP.GET_OUTPUT_DELAY, wvalue=o))))
    # Upmixer: RP2350-only, so absent entries are legitimate.  Its status byte 0
    # is "processing this packet stream", which is what actually matters.
    st = _optional(lambda: dev.get(OP.UPMIX_GET_STATUS, 16))
    if st is not None:
        checks.append(("upmixer active", 0, st[0]))
    return checks


def _route_only(dev, profile, out_l, out_r):
    """Route USB L/R 1:1 to exactly the (out_l, out_r) pair at 0 dB and disable
    every other crosspoint, so only this output pair carries signal. Matrix
    writes apply immediately (no deferred reset).

    EVERY input row is cleared, not just the stereo pair: the matrix has
    num_input_channels source rows (8 on RP2350), and when the stereo upmixer is
    running rows 2..4 additionally carry derived C/Ls/Rs (audio_pipeline.c
    PASS 3).  Clearing only rows 0/1 left those rows free to feed the target
    output, which silently broke this function's isolation guarantee and
    _autoprobe_slot's "probed in isolation" claim."""
    for inp in range(profile.num_input_channels):
        for out in range(profile.num_output_channels):
            _route(dev, inp, out, 0)
    _route(dev, 0, out_l, 1, 0.0)   # USB L -> target L
    _route(dev, 1, out_r, 1, 0.0)   # USB R -> target R


_BAND_CEILING = {}   # channel -> PEQ band count, probed once per channel


def _band_ceiling(dev, ch, hi=16):
    """PEQ band count for THIS channel: the smallest band index that STALLs.

    channel_band_counts[] is per-channel and runtime-settable (dsp_pipeline.c),
    while profile.band_ceiling is probed on channel 0, an INPUT channel.  Using
    that for the output channels the filter tests drive could under-flatten and
    leave live EQ bands under every measurement."""
    if ch in _BAND_CEILING:
        return _BAND_CEILING[ch]
    ceiling = hi + 1
    for band in range(hi + 1):
        try:
            dev.get(OP.GET_EQ_PARAM, 4, wvalue=(ch << 8) | (band << 3) | 0)
        except Stall:
            ceiling = band
            break
    _BAND_CEILING[ch] = ceiling
    return ceiling


def _config_slot(dev, profile, slot, flatten_all=False, otype=TYPE_SPDIF):
    """Make `slot` the only output carrying USB audio (1:1, 0 dB), as output
    type `otype` (S/PDIF or I2S). Returns (ch_l, ch_r). With flatten_all, zero
    every band on both channels. The loopback tap is pre-encoder, so the
    captured DSP output is identical for either output type."""
    out_l, out_r, ch_l, ch_r = _slot_indices(profile, slot)
    if not _set_output_type(dev, slot, otype):
        raise Skip(f"slot {slot} could not be set to "
                   f"{'I2S' if otype == TYPE_I2S else 'S/PDIF'} output "
                   f"(check BCK pin / pin conflict)")
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_l)
    dev.set_u8(OP.SET_OUTPUT_ENABLE, 1, wvalue=out_r)
    dev.wait_ready()
    _route_only(dev, profile, out_l, out_r)
    if flatten_all:
        for ch in (ch_l, ch_r):
            for b in range(_band_ceiling(dev, ch)):
                _set_band(dev, ch, b, FLAT, 1000.0, 0.707, 0.0)
            for b in range(XO_BAND, XO_BAND + XO_BANDS):
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
        if not _set_output_type(dev, k, TYPE_SPDIF):
            raise Skip(f"could not put slot {k} into S/PDIF mode for the probe")
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


def _find_devices_once():
    """Host audio devices, discovered once (they do not depend on the rate)."""
    global _DEVS
    if _DEVS is None:
        _DEVS = audio.find_devices()
    return _DEVS


def _get_rig(dev, profile, fs=None):
    """Rig for `fs`, building it on first use at that rate and caching it.

    `fs` defaults to whatever rate the running test was registered for
    (_ACTIVE_FS), or the primary rate for the tests registered directly.
    """
    if fs is None:
        fs = _ACTIVE_FS if _ACTIVE_FS is not None else PRIMARY_RATES[0]
    if fs in _RIG_FAIL:
        raise Skip(_RIG_FAIL[fs])
    if np is None:
        _RIG_FAIL[fs] = "numpy not installed (pip install numpy scipy sounddevice)"
        raise Skip(_RIG_FAIL[fs])
    try:
        out_dev, in_dev, info = _find_devices_once()
    except audio.AudioUnavailable as e:
        _RIG_FAIL[fs] = f"audio loopback unavailable: {e}"
        raise Skip(_RIG_FAIL[fs])

    if fs in _RIG:
        # The device may have been moved since this rig was built (the previous
        # test ran at another rate, or another host client changed it), so put
        # it back before measuring rather than just failing the comparison.
        if not _set_rate(dev, out_dev, in_dev, fs):
            raise Skip(f"could not return the device to {fs} Hz "
                       f"(now at {_device_rate(dev)})")
        return _RIG[fs]


    try:
        _baseline(dev, profile)
        if not _set_rate(dev, out_dev, in_dev, fs):
            # _require_rate names the specific reason (wrong rate, or above the
            # capture's ceiling) and always raises here, since the rate cannot
            # match after _set_rate failed.
            _require_rate(dev, fs)
            raise Skip(f"host would not move the device to {fs} Hz")
        # The target slot is a property of the loopback tap, not of the rate, so
        # a second rate reuses it rather than paying for another probe sweep.
        slot = next(iter(_RIG.values()))["slot"] if _RIG else \
            _autoprobe_slot(dev, profile, out_dev, in_dev, fs)
        ch_l, ch_r = _config_slot(dev, profile, slot, flatten_all=True)
    except Skip as e:
        # Rig setup runs once per rate.  Cache the reason so the remaining tests
        # at this rate skip instantly rather than each re-running the whole
        # (multi-second, barrier-bounded) setup and re-reporting the same fault.
        _RIG_FAIL[fs] = str(e)
        raise
    _RIG[fs] = {"out": out_dev, "in": in_dev, "chan": 0, "slot": slot,
                "ch_l": ch_l, "ch_r": ch_r, "fs": fs,
                "out_name": info["out"]["name"], "in_name": info["in"]["name"]}
    return _RIG[fs]


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

# Where this module's own registrations begin, so the per-rate replay below can
# pick out exactly the tests defined here (REGISTRY is shared by every module).
_REGISTRY_MARK = len(REGISTRY)


@test("audio", mutating=True)
def loopback_baseline_clean(dev, profile, chk):
    """Every stage the measurements assume inert really is inert.

    Registered first so a baseline that did not take produces ONE diagnostic
    failure naming the offending stage, instead of ~80 downstream tests failing
    on bit-exactness, unity gain and THD with no indication of the cause. The
    audio group runs against the device's live preset, so this is the check that
    the group's central assumption actually holds."""
    rig = _get_rig(dev, profile)
    chk.note(f"out='{rig['out_name']}' in='{rig['in_name']}' slot={rig['slot']} "
             f"ch_l={rig['ch_l']} ch_r={rig['ch_r']} fs={rig['fs']}")
    bad = []
    for label, want, got in _neutral_state(dev, profile):
        if got is None:
            continue          # not implemented on this platform
        ok = abs(got - want) < 1e-3 if isinstance(want, float) else got == want
        if not ok:
            bad.append(f"{label}={got!r} (want {want!r})")
    chk.ok(not bad, "baseline not neutral: " + ", ".join(bad[:8]) +
           (f" (+{len(bad) - 8} more)" if len(bad) > 8 else ""))

    # Matrix isolation: exactly USB L -> target L and USB R -> target R are
    # enabled.  This is what makes "unrouted crosspoint is silent" and the
    # auto-probe's isolation claim meaningful.
    out_l, out_r, _cl, _cr = _slot_indices(profile, rig["slot"])
    want_on = {(0, out_l), (1, out_r)}
    stray = []
    for inp in range(profile.num_input_channels):
        for out in range(profile.num_output_channels):
            r = _optional(lambda i=inp, o=out: dev.get(OP.GET_MATRIX_ROUTE, 8,
                                                       wvalue=(i << 8) | o))
            if r is None:
                continue
            enabled = bool(r[2])
            if enabled != ((inp, out) in want_on):
                stray.append(f"in{inp}->out{out}={'on' if enabled else 'off'}")
    chk.ok(not stray, "matrix not isolated to the target pair: " + ", ".join(stray[:8]) +
           (f" (+{len(stray) - 8} more)" if len(stray) > 8 else ""))
    chk.note(f"band ceilings: ch_l={_band_ceiling(dev, rig['ch_l'])} "
             f"ch_r={_band_ceiling(dev, rig['ch_r'])} (profile says {profile.band_ceiling})")


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
        # Both legs must be confirmed applied.  Without the barrier the restore
        # is compared against a stale active_input_source, silently swallowed,
        # and the test measures with the input still on S/PDIF.
        if not _set_input_source(dev, INPUT_SPDIF):
            raise Skip("S/PDIF input not selectable (disabled, or switch never applied)")
        if not _set_input_source(dev, INPUT_USB):
            raise Skip("could not switch the input source back to USB")
        lag, strength = _lr_lag(dev, profile, rig)
        chk.ok(strength > 0.5, f"signal restored after input switch (corr {strength:.2f})")
        chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R still aligned: lag {lag} samples")
        chk.note(f"after_input_switch: lag={lag} corr={strength:.2f}")
    finally:
        _set_input_source(dev, INPUT_USB)


@test("audio", mutating=True)
def alignment_after_output_type_switch(dev, profile, chk):
    """Output-type switch (S/PDIF -> I2S -> S/PDIF) on the slot preserves L/R alignment."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    try:
        # Each leg is confirmed applied before the next is requested, so this
        # really is a S/PDIF -> I2S -> S/PDIF round trip.  Previously the
        # restore no-op'd against a stale output_types[] and the measurement
        # ran with the slot still in I2S.
        if not _set_output_type(dev, slot, TYPE_I2S):
            raise Skip("output-type switch to I2S unavailable (check BCK pin / status)")
        if not _set_output_type(dev, slot, TYPE_SPDIF):
            raise Skip("could not restore the slot to S/PDIF")
        lag, strength = _lr_lag(dev, profile, rig)
        chk.ok(strength > 0.5, f"signal restored after type switch (corr {strength:.2f})")
        chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R still aligned: lag {lag} samples")
        chk.note(f"after_type_switch: lag={lag} corr={strength:.2f}")
    finally:
        _set_output_type(dev, slot, TYPE_SPDIF)


# --- Phase 3b: per-output-type audio integrity ------------------------------
#
# The loopback taps slot 0's finalized DSP output BEFORE the S/PDIF or I2S
# encoder, so the captured samples are identical for either output type. That
# makes signal PRESENCE a real liveness probe for the active output path: if the
# I2S consumer is not draining slot 0's producer pool, the pool backs up, the DSP
# callback starves, and the tone vanishes from the capture. These tests measure
# real audio with slot 0 as I2S, and stress repeated type switches (which
# exercise the shared-DMA-channel teardown/re-setup path).


@test("audio", mutating=True)
def output_type_i2s_audio(dev, profile, chk):
    """Slot 0 as I2S output: real audio still reaches the capture at unity with
    low THD, near bit-exact, and L/R sample-aligned. Because the tap is
    pre-encoder, a dead I2S output path would stall the slot-0 producer and the
    tone would disappear; a clean tone confirms the I2S consumer is draining."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    try:
        # _set_output_type waits for the switch to be APPLIED and then settles
        # PIPELINE_SETTLE_S for the I2S clock and the loopback servo to
        # re-prime; _config_slot's own type request is then a no-op, so this is
        # a single switch rather than the probe-then-switch it used to be.
        if not _set_output_type(dev, slot, TYPE_I2S):
            raise Skip("output-type switch to I2S unavailable (check BCK pin / status)")
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_I2S)
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
        # single-shot to the same standard as loopback_integrity.
        resid, scale = audio.bit_exact_residual(rig["out"], rig["in"], rig["chan"], rig["fs"])
        chk.ok(resid < RESIDUAL_MAX_DBFS, f"I2S: flat-path residual {resid:.1f} dBFS < {RESIDUAL_MAX_DBFS}")
        gain_db = 20.0 * np.log10(abs(scale) + 1e-20)
        chk.approx(gain_db, 0.0, GAIN_TOL_DB, f"I2S: path gain ~0 dB (|scale| {abs(scale):.4f})")
        chk.note(f"i2s_audio: level={level:.2f}dBFS thd={thd:.4f}% resid={resid:.1f}dBFS "
                 f"|scale|={abs(scale):.4f} lag={lag}")
    finally:
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_SPDIF)
        dev.wait_ready()


@test("audio", mutating=True)
def output_type_switch_stress(dev, profile, chk):
    """Repeated SPDIF<->I2S switching on slot 0 keeps the output alive, at unity,
    and L/R sample-aligned after EVERY switch. Exercises the shared-DMA-channel
    teardown/re-setup path (audio_spdif_teardown + full SPDIF re-setup, I2S
    setup/teardown reclaiming the same channel); a double-claim panic, a stalled
    pipeline, or lost inter-leg alignment would surface here as lost signal, a
    bad lag, or a device reset (disconnect).

    Every switch is confirmed APPLIED before measuring (_config_slot ->
    _set_output_type), so these are real switches rather than requests swallowed
    as no-ops against a stale output_types[]."""
    rig = _get_rig(dev, profile)
    slot = rig["slot"]
    cycles = 4
    try:
        # No separate availability probe: the first I2S leg below raises Skip
        # (via _config_slot) if the slot cannot take it, and the finally still
        # restores S/PDIF.  Probing first would cost two extra pipeline resets.
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
        _config_slot(dev, profile, slot, flatten_all=True, otype=TYPE_SPDIF)
        dev.wait_ready()


# --- Phase 4: full chain / dynamics -----------------------------------------

# Input-channel EQ needs no flattening here: _baseline() disables the whole
# per-input EQ pass with the global bypass flag, which is both cheaper and
# non-destructive (the preset's own bands survive the run).  loopback_baseline_
# clean asserts the flag is still set.


def _note_input_count(dev, chk):
    """Record the live active input count alongside a chain measurement.

    Loudness, leveller and crossfeed used to be bypassed whenever more than two
    input channels were active, so these tests skipped on any host that opened
    the DSPi at its 8-channel maximum (macOS CoreAudio does).  That bypass is
    gone: crossfeed runs per output pair post-matrix, loudness runs per output
    post-gain, and the leveller is mask-driven over the active inputs, all
    input-count agnostic (audio_pipeline.c PASS 2.5 / 4.5 / 5-7).  The count is
    still worth reporting, because it selects which code path was exercised."""
    nin = dev.get_u32(OP.GET_STATUS, wvalue=STATUS_INPUT_COUNT)
    chk.note(f"active input channels: {nin}")
    return nin


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
    _note_input_count(dev, chk)
    try:
        _flatten_chain(dev, rig["ch_l"])
        # Loudness is applied per output through loudness_output_mask; a preset
        # that cleared the target's bit would make the enable a silent no-op.
        dev.set(OP.SET_LOUDNESS_MASK, struct.pack("<H", 0xFFFF))
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
    _note_input_count(dev, chk)
    try:
        _config_slot(dev, profile, rig["slot"])
        _flatten_chain(dev, rig["ch_l"]); _flatten_chain(dev, rig["ch_r"])
        # Crossfeed runs per output PAIR, selected by output_pair_mask; without
        # the target slot's bit the enable below would do nothing.
        dev.set_u8(OP.SET_CROSSFEED_OUTPUTS, 1 << rig["slot"])
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
    _note_input_count(dev, chk)
    try:
        _flatten_chain(dev, rig["ch_l"])
        # Detector and apply masks select which INPUT channels the one linked
        # gain is derived from and applied to; a preset that cleared the stereo
        # pair's bits would leave the leveller enabled but inert here.
        dev.set(OP.SET_LEVELLER_MASKS, bytes([0xFF, 0xFF]))
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


# --- Per-rate replay --------------------------------------------------------
#
# Everything above is registered once, at the first primary rate.  Two further
# passes can follow, both built by replaying existing test bodies at another
# rate (see _rate_variant):
#
#   * PRIMARY_RATES[1:] replay the FULL matrix.  Only --audio-rates produces
#     these, because doubling a multi-minute run is an explicit choice.
#   * SECONDARY_RATES replay the targeted SECONDARY_TESTS subset.  This is the
#     default 44.1 kHz pass.
#
# Variants are registered in rate order and the round-trip test comes last, so a
# run performs exactly one rate change per extra rate rather than thrashing.
#
# The subset covers the tests where the rate genuinely changes behaviour, rather
# than the whole matrix (which would roughly double the run for little signal).
#
#   * loopback_integrity: the capture servo now runs at a NON-INTEGER 44.1
#     frames per USB frame, so its fractional accumulator, not just the
#     feed-forward term, has to hold bit-exactness and unity gain.
#   * output_delay: the expected sample count is derived from fs (220 vs 240).
#   * multiband_eq and the PEQ/XO picks: the hybrid SVF/biquad boundary is
#     Fs/7.5, so it moves from 6400 Hz to 5880 Hz.  multiband_eq's 6 kHz high
#     shelf is on the SVF side at 48 kHz and the biquad side at 44.1 kHz, so the
#     same configuration covers both paths across the two rates.
#   * slot_lr_alignment: inter-leg alignment at the second rate.
#
# Rates come from run.py --audio-rates; see PRIMARY_RATES / SECONDARY_RATES.

SECONDARY_TESTS = [
    "loopback_integrity",
    "slot_lr_alignment",
    "output_delay",
    "multiband_eq",
    "xo_lr4_complementary_sum",
    "peq_lowpass1_lo", "peq_lowpass1_hi",
    "peq_peaking_lo", "peq_peaking_hi",
    "peq_highshelf_lo", "peq_highshelf_hi",
    "xo_lr4_lp", "xo_lr4_hp", "xo_bw2_lp", "xo_bes4_hp",
]


def _rate_variant(body, fs):
    """Register an existing test body to run again at `fs`.

    The body is reused untouched: the wrapper publishes the rate in _ACTIVE_FS,
    which _get_rig() picks up, so nothing in the test has to thread fs through.
    Restored in a finally so a failure cannot leak the rate into the next test.
    """
    tag = _rate_tag(fs)

    def fn(dev, profile, chk):
        global _ACTIVE_FS
        prev = _ACTIVE_FS
        _ACTIVE_FS = fs
        try:
            return body(dev, profile, chk)
        finally:
            _ACTIVE_FS = prev

    fn.__name__ = body.__name__ + tag
    first = (body.__doc__ or body.__name__).strip().split("\n")[0]
    fn.__doc__ = f"[{fs} Hz] {first}"
    return test("audio", mutating=True)(fn)


# Bodies registered above, in order: the full matrix at the first primary rate.
_PRIMARY_BODIES = [tc.fn for tc in REGISTRY[_REGISTRY_MARK:]]

for _fs in PRIMARY_RATES[1:]:
    for _body in _PRIMARY_BODIES:
        globals()[_body.__name__ + _rate_tag(_fs)] = _rate_variant(_body, _fs)

for _fs in SECONDARY_RATES:
    for _name in SECONDARY_TESTS:
        _body = globals().get(_name)
        if _body is None:      # a renamed/removed test should be loud, not silent
            raise RuntimeError(f"SECONDARY_TESTS names an unknown test: {_name}")
        globals()[_body.__name__ + _rate_tag(_fs)] = _rate_variant(_body, _fs)


@test("audio", mutating=True)
def rate_switch_round_trip(dev, profile, chk):
    """Returning to the primary rate restores signal and L/R alignment.

    Registered last so it also leaves the device on the primary rate: the
    suite's snapshot restores device CONFIG, but the operating rate is host
    driven and is not in the bulk blob, so nothing else would put it back.
    """
    primary = PRIMARY_RATES[0]
    rig = _get_rig(dev, profile, primary)
    chk.ok(_device_rate(dev) == primary, f"device back at {primary} Hz")
    lag, strength = _lr_lag(dev, profile, rig)
    chk.ok(strength > 0.5, f"signal present after the rate round trip (corr {strength:.2f})")
    chk.ok(abs(lag) <= ALIGN_TOL_SAMPLES, f"L/R still aligned: lag {lag} samples")
    # Multichannel USB alts advertise 48 kHz only (usb_descriptors.c
    # AS_MULTICH_ALT), so any 44.1 kHz pass necessarily ran on a stereo alt.
    _note_input_count(dev, chk)
    chk.note(f"rate_round_trip: rates exercised {sorted(_RIG)} lag={lag}")
