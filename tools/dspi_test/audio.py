"""
audio.py — host-side audio measurement engine for the DSPi loopback rig.

Plays a signal out the DSPi USB audio OUTPUT (host -> DSPi USB input), lets the
DSP process it, and captures DSPi's S/PDIF output via the Weeb Labs USBrx USB
audio INPUT, then extracts level / noise / THD and the filter frequency response.

The chain is single-clock: DSPi is the master, host playback is rate-slaved to
DSPi via the USB Audio feedback endpoint, and USBrx is slaved to DSPi's S/PDIF.
So the capture is a fixed-latency, bit-exact copy of DSPi's output; the latency
is recovered by cross-correlation, with no resampling. Because the whole path is
digital, measurement SNR is ~24-bit (~140 dB), so a single sweep gives a very
clean transfer function.

This module is the only place that touches PortAudio. It is import-safe even
when sounddevice/numpy are missing: the optional deps are probed at call time and
raise AudioUnavailable so the loopback tests can SKIP cleanly.

Optional deps:  pip install sounddevice numpy scipy   (macOS also: brew install portaudio)

CLI (bring-up / diagnosis):
    python3 -m tools.dspi_test.audio --list      # enumerate host audio devices
    python3 -m tools.dspi_test.audio --probe     # raw loopback tone + level/THD/noise
"""

from __future__ import annotations

import sys

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:
    import sounddevice as sd
except Exception:  # ImportError, or OSError when the PortAudio lib is absent
    sd = None

try:
    from scipy.signal import correlate as _sp_correlate
except Exception:  # scipy optional; fall back to an FFT cross-correlation
    _sp_correlate = None


# Default host-device name substrings (case-insensitive). Override via the
# functions' arguments if your OS names them differently.
DSPI_OUT_NAME = "DSPi"
USBRX_IN_NAME = "USBrx"

DEFAULT_FS = 48000
PAD_S = 0.10          # leading/trailing silence around an excitation
TAIL_S = 0.40         # extra record time after playback ends


class AudioUnavailable(Exception):
    """sounddevice/numpy or a required loopback audio device is not present."""


def _require():
    missing = []
    if np is None:
        missing.append("numpy")
    if sd is None:
        missing.append("sounddevice (and PortAudio: brew install portaudio)")
    if missing:
        raise AudioUnavailable(
            "audio loopback needs " + " + ".join(missing)
            + "  (pip install sounddevice numpy scipy)")


# ---------------------------------------------------------------------------
# Device discovery
# ---------------------------------------------------------------------------

def list_devices() -> str:
    """Human-readable enumeration of host audio devices (for --list / errors)."""
    _require()
    lines = []
    for i, d in enumerate(sd.query_devices()):
        io = f"in={d['max_input_channels']} out={d['max_output_channels']}"
        lines.append(f"  [{i:2d}] {d['name']}  ({io}, {int(d['default_samplerate'])} Hz)")
    return "\n".join(lines)


def find_devices(out_name: str = DSPI_OUT_NAME, in_name: str = USBRX_IN_NAME):
    """Locate the DSPi output and USBrx input by name substring.

    Returns (out_index, in_index, info_dict). Raises AudioUnavailable (with the
    available device list) if either is missing, so callers can SKIP.
    """
    _require()
    devs = sd.query_devices()

    def match(name, want_out):
        key = "max_output_channels" if want_out else "max_input_channels"
        return [i for i, d in enumerate(devs)
                if name.lower() in d["name"].lower() and d[key] > 0]

    out_hits = match(out_name, True)
    in_hits = match(in_name, False)
    if not out_hits or not in_hits:
        avail_out = [d["name"] for d in devs if d["max_output_channels"] > 0]
        avail_in = [d["name"] for d in devs if d["max_input_channels"] > 0]
        what = []
        if not out_hits:
            what.append(f"no output matching '{out_name}' (have: {avail_out})")
        if not in_hits:
            what.append(f"no input matching '{in_name}' (have: {avail_in})")
        raise AudioUnavailable("; ".join(what))

    out_i, in_i = out_hits[0], in_hits[0]
    return out_i, in_i, {"out": devs[out_i], "in": devs[in_i]}


# ---------------------------------------------------------------------------
# Signal generation
# ---------------------------------------------------------------------------

def _fade(x, fs, ms=5.0):
    n = int(fs * ms / 1000.0)
    if n > 0 and 2 * n < len(x):
        w = np.hanning(2 * n)
        x = x.copy()
        x[:n] *= w[:n]
        x[-n:] *= w[n:]
    return x


def make_sweep(fs, dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Exponential (log) sine sweep f1 -> f2, faded at the edges."""
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    n = int(dur_s * fs)
    t = np.arange(n) / fs
    L = dur_s / np.log(f2 / f1)
    phase = 2.0 * np.pi * f1 * L * (np.exp(t / L) - 1.0)
    return _fade(amp * np.sin(phase).astype(np.float32), fs)


def make_tone(fs, freq=1000.0, dur_s=0.5, amp=0.4):
    _require()
    n = int(dur_s * fs)
    t = np.arange(n) / fs
    return _fade(amp * np.sin(2.0 * np.pi * freq * t).astype(np.float32), fs)


# ---------------------------------------------------------------------------
# Play + record (two independent streams; single clock domain)
# ---------------------------------------------------------------------------

def play_record(excitation, fs, out_dev, in_dev,
                in_channels=2, out_channels=2, tail_s=TAIL_S, max_retries=3):
    """Play `excitation` on out_dev while recording in_channels from in_dev.

    `excitation` is mono [N] (duplicated across out_channels) or [N, out_channels].
    Returns the captured float32 array, shape [M, in_channels].

    Uses two explicit callback streams (a recording InputStream + a feeding
    OutputStream) started together, NOT sd.play()+InputStream (which does not
    sync cleanly) and NOT a combined cross-device duplex stream (CoreAudio will
    not open one reliably across two devices). The streams free-run on the shared
    DSPi clock; the caller recovers the fixed latency by cross-correlation.
    """
    _require()
    exc = np.asarray(excitation, dtype=np.float32)
    if exc.ndim == 1:
        exc = np.column_stack([exc] * out_channels)
    n = exc.shape[0]

    # Retry on an xrun: opening/closing two cross-device streams repeatedly can
    # make CoreAudio drop buffers (input overflow / output underflow), which
    # corrupts a capture. `latency="high"` makes that rare; a retry catches the
    # stragglers so a long test run stays reliable.
    last = np.zeros((0, in_channels), dtype=np.float32)
    for _attempt in range(max_retries + 1):
        pos = {"i": 0}
        rec_frames = []
        glitch = {"bad": False}

        def _out_cb(outdata, frames, time_info, status):  # noqa: ARG001
            if status.output_underflow:
                glitch["bad"] = True
            i0 = pos["i"]
            chunk = exc[i0:i0 + frames]
            m = chunk.shape[0]
            if m:
                outdata[:m] = chunk
            if m < frames:
                outdata[m:] = 0.0       # feed silence once the excitation is done
            pos["i"] = i0 + frames

        def _in_cb(indata, frames, time_info, status):  # noqa: ARG001
            if status.input_overflow:
                glitch["bad"] = True
            rec_frames.append(indata.copy())

        instream = sd.InputStream(samplerate=fs, device=in_dev, channels=in_channels,
                                  dtype="float32", latency="high", callback=_in_cb)
        outstream = sd.OutputStream(samplerate=fs, device=out_dev, channels=out_channels,
                                    dtype="float32", latency="high", callback=_out_cb)
        instream.start()                # capture first so we never miss the onset
        outstream.start()
        sd.sleep(int((n / fs + tail_s) * 1000.0))
        outstream.stop(); instream.stop()
        outstream.close(); instream.close()

        last = np.concatenate(rec_frames, axis=0) if rec_frames else last
        if not glitch["bad"] and last.shape[0] > 0:
            return last
        sd.sleep(120)                   # settle, then retry
    return last


# ---------------------------------------------------------------------------
# Alignment + analysis
# ---------------------------------------------------------------------------

def _xcorr_lag(y, ref):
    """Integer lag (samples) of `ref` within the longer `y`, by cross-correlation,
    plus a normalized peak strength in [0,1] to detect 'no signal'."""
    y = np.asarray(y, np.float64)
    ref = np.asarray(ref, np.float64)
    if _sp_correlate is not None:
        corr = _sp_correlate(y, ref, mode="valid", method="fft")
    else:
        n = len(y) - len(ref) + 1
        corr = np.array([np.dot(y[i:i + len(ref)], ref) for i in range(max(n, 0))])
    if len(corr) == 0:
        return 0, 0.0
    lag = int(np.argmax(np.abs(corr)))
    peak = float(np.abs(corr[lag]))
    # Normalize against ||ref|| * ||y-window|| for a coherence-like strength.
    seg = y[lag:lag + len(ref)]
    denom = np.linalg.norm(ref) * (np.linalg.norm(seg) + 1e-20)
    strength = peak / (denom + 1e-20)
    return lag, strength


def align(captured_ch, reference):
    """Return (aligned_segment, strength). Caller checks `strength` (a weak peak
    means no signal / wrong routing / clock fault)."""
    lag, strength = _xcorr_lag(captured_ch, reference)
    seg = np.asarray(captured_ch, np.float32)[lag:lag + len(reference)]
    if len(seg) < len(reference):  # pad if the tail was clipped
        seg = np.concatenate([seg, np.zeros(len(reference) - len(seg), np.float32)])
    return seg, strength


def dbfs(x):
    """RMS level of x in dBFS (full-scale sine = ~ -3 dBFS RMS)."""
    r = float(np.sqrt(np.mean(np.square(np.asarray(x, np.float64))))) if len(x) else 0.0
    return 20.0 * np.log10(r + 1e-20)


def measure_transfer(out_dev, in_dev, in_channel, fs, freqs,
                     dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Play a log sweep, capture `in_channel`, return (mag_db, phase_deg, strength)
    sampled at `freqs`. H = FFT(captured)/FFT(reference) over the aligned window;
    valid because DSPi applies the filter to exactly the played samples.
    """
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    sweep = make_sweep(fs, dur_s, f1, f2, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    exc = np.concatenate([pad, sweep, pad])

    cap = play_record(exc, fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (USBrx delivered no frames)")
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, strength = align(y, sweep)

    X = np.fft.rfft(sweep)
    Y = np.fft.rfft(seg)
    fbins = np.fft.rfftfreq(len(sweep), 1.0 / fs)
    H = Y / (X + 1e-12)
    mag = 20.0 * np.log10(np.abs(H) + 1e-20)
    phase = np.unwrap(np.angle(H))

    freqs = np.asarray(freqs, np.float64)
    mag_at = np.interp(freqs, fbins, mag)
    phase_at = np.degrees(np.interp(freqs, fbins, phase))
    return mag_at, phase_at, strength


def measure_complex_2ch(out_dev, in_dev, fs, freqs, dur_s=1.0, f1=20.0, f2=None, amp=0.4):
    """Play one sweep, capture both input channels, and return (H0, H1, strength)
    as complex transfer functions at `freqs`. Both channels are aligned with a
    SHARED lag (recovered from L+R, which is broadband), so their RELATIVE phase
    is valid for summing — e.g. the Linkwitz-Riley LP+HP complementary-sum test.
    A common path delay / polarity is shared by both and cancels in |H0 + H1|.
    """
    _require()
    f2 = f2 if f2 is not None else fs * 0.45
    sweep = make_sweep(fs, dur_s, f1, f2, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (USBrx delivered no frames)")
    lag, strength = _xcorr_lag(cap[:, 0].astype(np.float64) + cap[:, 1].astype(np.float64), sweep)
    X = np.fft.rfft(sweep)
    fbins = np.fft.rfftfreq(len(sweep), 1.0 / fs)
    freqs = np.asarray(freqs, np.float64)

    def _h_at(ch):
        seg = cap[lag:lag + len(sweep), ch]
        if len(seg) < len(sweep):
            seg = np.concatenate([seg, np.zeros(len(sweep) - len(seg), np.float32)])
        H = np.fft.rfft(seg) / (X + 1e-12)
        return np.interp(freqs, fbins, H.real) + 1j * np.interp(freqs, fbins, H.imag)

    return _h_at(0), _h_at(1), strength


def measure_interchannel_lag(out_dev, in_dev, fs, dur_s=0.3, amp=0.4):
    """Play a sweep, capture both channels in one go, and return (lag, strength)
    where lag = samples that channel 0 is delayed relative to channel 1 (by
    cross-correlation). Measuring both legs in ONE capture cancels per-capture
    stream-start jitter, so this reliably reads a per-output delay difference.
    """
    _require()
    sweep = make_sweep(fs, dur_s, 30.0, fs * 0.45, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        raise AudioUnavailable("no audio captured (USBrx delivered no frames)")
    a = cap[:, 0].astype(np.float64)
    b = cap[:, 1].astype(np.float64)
    if _sp_correlate is not None:
        corr = _sp_correlate(a, b, mode="full", method="fft")
    else:  # slow fallback
        corr = np.correlate(a, b, mode="full")
    lags = np.arange(-(len(b) - 1), len(a))
    k = int(np.argmax(np.abs(corr)))
    strength = float(np.abs(corr[k]) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-20))
    return int(lags[k]), strength


def measure_tone(out_dev, in_dev, in_channel, fs, freq=1000.0, dur_s=0.5, amp=0.4):
    """Play a sine, capture it, return (level_dbfs, thd_pct, strength)."""
    _require()
    tone = make_tone(fs, freq, dur_s, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, tone, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0, 100.0, 0.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, strength = align(y, tone)
    # Trim fades for a clean steady-state THD window.
    g = int(0.01 * fs)
    core = seg[g:len(seg) - g] if len(seg) > 2 * g else seg
    level = dbfs(core)
    thd = _thd_pct(core, fs, freq)
    return level, thd, strength


def measure_tone_2ch(out_dev, in_dev, fs, freq=1000.0, dur_s=0.8, amp=0.4, left_only=False):
    """Play a tone (on the LEFT output only if left_only, else both) and capture
    both input channels. Returns (level0_dbfs, level1_dbfs, thd0_pct) measured on
    the STEADY TAIL of the aligned tone (so a settling effect like the leveller
    is read at steady state, and a delayed bleed like crossfeed is still in-window).
    """
    _require()
    tone = make_tone(fs, freq, dur_s, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    sig = np.concatenate([pad, tone, pad])
    exc = np.column_stack([sig, np.zeros_like(sig) if left_only else sig])
    cap = play_record(exc, fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0, -200.0, 100.0
    lag, _ = _xcorr_lag(cap[:, 0], tone)          # ch0 always carries the (left) tone
    lo, hi = int(0.30 * len(tone)), int(0.95 * len(tone))   # steady tail of the tone

    def seg(ch):
        s = cap[lag:lag + len(tone), ch]
        if len(s) < len(tone):
            s = np.concatenate([s, np.zeros(len(tone) - len(s), np.float32)])
        return s[lo:hi]

    s0, s1 = seg(0), seg(1)
    return dbfs(s0), dbfs(s1), _thd_pct(s0, fs, freq)


def _thd_pct(x, fs, f0, n_harm=6):
    win = np.hanning(len(x))
    X = np.abs(np.fft.rfft(x * win))
    fbins = np.fft.rfftfreq(len(x), 1.0 / fs)

    def bin_energy(f):
        if f >= fs / 2:
            return 0.0
        k = int(round(f / (fs / len(x))))
        lo, hi = max(k - 2, 0), min(k + 3, len(X))
        return float(np.sum(X[lo:hi] ** 2))

    fund = bin_energy(f0)
    harm = sum(bin_energy(f0 * h) for h in range(2, n_harm + 1))
    if fund <= 0:
        return 100.0
    return 100.0 * np.sqrt(harm / fund)


def measure_noise(out_dev, in_dev, in_channel, fs, dur_s=0.4):
    """Play silence, return the captured noise floor in dBFS."""
    _require()
    cap = play_record(np.zeros(int(dur_s * fs), np.float32), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return -200.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    return dbfs(y)


def bit_exact_residual(out_dev, in_dev, in_channel, fs, dur_s=0.4, amp=0.4):
    """Flat-path fidelity: play a sweep, align the capture, fit the best scalar
    gain, and return (residual_dbfs, scale) where residual is the RMS of
    (captured - scale*reference) relative to full scale. For a clean unity
    digital path, residual sits near the 24-bit floor and scale ~ 1.0.
    """
    _require()
    sweep = make_sweep(fs, dur_s, 30.0, fs * 0.45, amp)
    pad = np.zeros(int(PAD_S * fs), np.float32)
    cap = play_record(np.concatenate([pad, sweep, pad]), fs, out_dev, in_dev)
    if cap.shape[0] == 0:
        return 0.0, 0.0
    y = cap[:, in_channel] if cap.ndim > 1 else cap
    seg, _ = align(y, sweep)
    ref = sweep.astype(np.float64)
    seg = seg.astype(np.float64)
    scale = float(np.dot(ref, seg) / (np.dot(ref, ref) + 1e-20))
    residual = seg - scale * ref
    return dbfs(residual), scale


# ---------------------------------------------------------------------------
# CLI (bring-up / diagnosis)
# ---------------------------------------------------------------------------

def _main(argv=None):
    import argparse
    ap = argparse.ArgumentParser(prog="dspi_test.audio",
                                 description="DSPi loopback audio bring-up tool")
    ap.add_argument("--list", action="store_true", help="enumerate host audio devices")
    ap.add_argument("--probe", action="store_true",
                    help="play a 1 kHz tone out DSPi, read it back from USBrx")
    ap.add_argument("--out-name", default=DSPI_OUT_NAME)
    ap.add_argument("--in-name", default=USBRX_IN_NAME)
    ap.add_argument("--fs", type=int, default=DEFAULT_FS)
    ap.add_argument("--channel", type=int, default=0, help="USBrx capture channel (0=L,1=R)")
    args = ap.parse_args(argv)

    try:
        _require()
    except AudioUnavailable as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    if args.list or not args.probe:
        print("Host audio devices:")
        print(list_devices())
        if not args.probe:
            return 0

    try:
        out_i, in_i, info = find_devices(args.out_name, args.in_name)
    except AudioUnavailable as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2
    print(f"\nDSPi output : [{out_i}] {info['out']['name']}")
    print(f"USBrx input : [{in_i}] {info['in']['name']}")
    print(f"Sample rate : {args.fs} Hz   capture channel: {args.channel}\n")

    print("NOTE: this plays a tone to DSPi but does not configure DSPi routing —")
    print("      route USB -> the USBrx-connected output first, or use the test suite.\n")

    level, thd, strength = measure_tone(out_i, in_i, args.channel, args.fs, 1000.0)
    noise = measure_noise(out_i, in_i, args.channel, args.fs)
    print(f"1 kHz tone : level {level:7.2f} dBFS   THD {thd:6.3f}%   "
          f"corr {strength:4.2f}")
    print(f"noise floor: {noise:7.2f} dBFS")
    if strength < 0.2:
        print("\n  ⚠ weak correlation — no signal reaching USBrx. Check cabling, "
              "DSPi input source (USB), output enable/routing, and that the tone "
              "is routed to the USBrx-connected slot.")
    return 0


if __name__ == "__main__":
    sys.exit(_main())
