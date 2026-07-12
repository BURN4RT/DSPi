"""User filter module for compare_filter.py: DSPi Linkwitz Transform (LT).

Mirrors the firmware coefficient computation in dsp_pipeline.c
dsp_compute_coefficients() for the Linkwitz Transform, which reshapes a
sealed-box driver's natural rolloff (f0, Q0) into a chosen target alignment
(fp, Qp).

Wire-format mapping (how the harness signature carries the four LT params,
identical to the firmware wire format):

    fc      -> f0   (driver's measured rolloff corner, Hz)
    Q       -> Q0   (driver's measured rolloff Q)
    gain_db -> fp   (target alignment corner, Hz; NOT a dB gain here)
    Qp      -> module-level QP_DEFAULT (target alignment Q); the __main__
               property check overrides QP_DEFAULT directly per test case.

Firmware clamps: f0 and fp to [10, 0.15*Fs] (LT corner clamp; bounds Q28 coefficients); Q0 and Qp to [0.1, 20].

Biquad path (RP2040, and the RP2350 fallback when max(f0,fp) >= Fs/7.5):

    g0 = tan(pi*f0/Fs);  gp = tan(pi*fp/Fs)
    b  = [1 + g0/Q0 + g0^2,  2*(g0^2 - 1),  1 - g0/Q0 + g0^2]
    a  = [1 + gp/Qp + gp^2,  2*(gp^2 - 1),  1 - gp/Qp + gp^2]
    then normalized by a[0].

So H(z) places zeros at the driver's pole pair and poles at the target pole
pair: the section un-does the driver's rolloff and imposes the target one.
DC gain is (g0/gp)^2 ~ (f0/fp)^2; the response returns to unity well above
both corners.

Two type labels exercise both platforms against the same design:
  'linkwitz'    - RP2350 float path (coefficients used as computed)
  'linkwitzq28' - RP2040 fixed-point: each normalized coefficient quantized to
                  Q28 the way dsp_compute_coefficients() does, i.e.
                  (int32_t)(coef * 2^28).

Run directly (python user_linkwitz.py) for a reference-independent property
check: identity, DC/HF gain, analog-prototype match, biquad<->SVF equivalence,
and float<->Q28 agreement.
"""
import math

FILTER_SHIFT = 28  # Q28, matches firmware FILTER_SHIFT

# Target alignment Q (Qp). Carried out-of-band from the harness signature;
# the __main__ property check overrides this per test case.
QP_DEFAULT = 0.707


def _clamp(x, lo, hi):
    return lo if x < lo else (hi if x > hi else x)


def _lt_biquad(f0, Q0, fp, Qp, Fs):
    """Return (b, a) normalized by a[0], matching the firmware biquad path."""
    f0 = _clamp(f0, 10.0, 0.15 * Fs)
    fp = _clamp(fp, 10.0, 0.15 * Fs)
    Q0 = _clamp(Q0, 0.1, 20.0)
    Qp = _clamp(Qp, 0.1, 20.0)

    g0 = math.tan(math.pi * f0 / Fs)
    gp = math.tan(math.pi * fp / Fs)

    b0 = 1.0 + g0 / Q0 + g0 * g0
    b1 = 2.0 * (g0 * g0 - 1.0)
    b2 = 1.0 - g0 / Q0 + g0 * g0

    a0 = 1.0 + gp / Qp + gp * gp
    a1 = 2.0 * (gp * gp - 1.0)
    a2 = 1.0 - gp / Qp + gp * gp

    inv = 1.0 / a0
    b = [b0 * inv, b1 * inv, b2 * inv]
    a = [1.0, a1 * inv, a2 * inv]
    return b, a


def user_coefficients(filter_type, fc, Q, gain_db, Fs):
    # Wire-format mapping: fc=f0, Q=Q0, gain_db carries fp (Hz), Qp out-of-band.
    f0, Q0, fp, Qp = fc, Q, gain_db, QP_DEFAULT
    b, a = _lt_biquad(f0, Q0, fp, Qp, Fs)

    if filter_type == 'linkwitzq28':
        scale = float(1 << FILTER_SHIFT)
        # (int32_t) cast truncates toward zero; Python int() does too.
        def q(x):
            return float(int(x * scale)) / scale
        # a[0] is exactly 1.0 (normalization) and stays unquantized, exactly as
        # the firmware keeps a0 implicit; every other coefficient is quantized.
        b = [q(b[0]), q(b[1]), q(b[2])]
        a = [1.0, q(a[1]), q(a[2])]

    return {'kind': 'biquad', 'b': b, 'a': a}


if __name__ == '__main__':
    import numpy as np

    # ------------------------------------------------------------------ #
    # helpers
    # ------------------------------------------------------------------ #
    def coeffs(f0, Q0, fp, Qp, Fs, label='linkwitz'):
        global QP_DEFAULT
        saved = QP_DEFAULT
        QP_DEFAULT = Qp
        try:
            return user_coefficients(label, f0, Q0, fp, Fs)
        finally:
            QP_DEFAULT = saved

    def freqz_mag_db(c, f, Fs):
        """Biquad magnitude in dB at real frequencies f (Hz)."""
        b, a = c['b'], c['a']
        w = 2.0 * np.pi * np.asarray(f) / Fs
        z1 = np.exp(-1j * w)
        z2 = z1 * z1
        H = (b[0] + b[1] * z1 + b[2] * z2) / (a[0] + a[1] * z1 + a[2] * z2)
        return 20.0 * np.log10(np.abs(H)), H

    def analog_lt_mag_db(f0, Q0, fp, Qp, f):
        """Analog LT prototype magnitude in dB at real frequencies f (Hz)."""
        w = 2.0 * np.pi * np.asarray(f)
        w0 = 2.0 * np.pi * f0
        wp = 2.0 * np.pi * fp
        s = 1j * w
        num = s * s + s * w0 / Q0 + w0 * w0
        den = s * s + s * wp / Qp + wp * wp
        return 20.0 * np.log10(np.abs(num / den))

    def svf_impulse_mag_db(f0, Q0, fp, Qp, Fs, n=8192):
        """Run a unit impulse through the firmware Simper SVF LT loop, FFT it,
        and return (freqs_hz, magnitude_db) for the positive-frequency bins."""
        f0c = _clamp(f0, 10.0, 0.15 * Fs)
        fpc = _clamp(fp, 10.0, 0.15 * Fs)
        Q0c = _clamp(Q0, 0.1, 20.0)
        Qpc = _clamp(Qp, 0.1, 20.0)

        g = math.tan(math.pi * fpc / Fs)
        k = 1.0 / Qpc
        sva1 = 1.0 / (1.0 + g * (g + k))
        sva2 = g * sva1
        sva3 = g * sva2
        r = math.tan(math.pi * f0c / Fs) / g
        svm0 = 1.0
        svm1 = r / Q0c - k
        svm2 = r * r - 1.0

        out = np.empty(n, dtype=np.float64)
        ic1eq = 0.0
        ic2eq = 0.0
        for i in range(n):
            x = 1.0 if i == 0 else 0.0
            v3 = x - ic2eq
            v1 = sva1 * ic1eq + sva2 * v3
            v2 = ic2eq + sva2 * ic1eq + sva3 * v3
            ic1eq = 2.0 * v1 - ic1eq
            ic2eq = 2.0 * v2 - ic2eq
            out[i] = svm0 * x + svm1 * v1 + svm2 * v2

        H = np.fft.rfft(out)
        freqs = np.fft.rfftfreq(n, d=1.0 / Fs)
        return freqs, 20.0 * np.log10(np.abs(H))

    # ------------------------------------------------------------------ #
    # property checks
    # ------------------------------------------------------------------ #
    failures = []

    def check(name, cond, detail=''):
        status = 'PASS' if cond else 'FAIL'
        print(f"  [{status}] {name}{('  ' + detail) if detail else ''}")
        if not cond:
            failures.append(name + (('  ' + detail) if detail else ''))

    # (f0, Q0, fp, Qp)
    PARAM_SETS = [
        (55.0, 1.1, 20.0, 0.5),    # boost, big downward shift
        (80.0, 0.7, 30.0, 0.707),  # boost, moderate shift
        (40.0, 1.4, 45.0, 0.6),    # downward / attenuating (fp > f0)
    ]

    for Fs in (48000.0, 96000.0):
        print(f"\n=== Fs = {Fs:.0f} Hz ===")
        for (f0, Q0, fp, Qp) in PARAM_SETS:
            tag = f"f0={f0:g} Q0={Q0:g} fp={fp:g} Qp={Qp:g}"
            print(f"\n-- {tag} --")

            c = coeffs(f0, Q0, fp, Qp, Fs)
            fmax = 2.0 * max(f0, fp)

            # (a) Identity: f0==fp, Q0==Qp -> flat unity.
            cid = coeffs(f0, Q0, f0, Q0, Fs)
            fband = np.geomspace(10.0, 0.45 * Fs, 2000)
            mid, _ = freqz_mag_db(cid, fband, Fs)
            lin = 10.0 ** (mid / 20.0)
            id_err = float(np.max(np.abs(lin - 1.0)))
            check('(a) identity |H|==1 (<1e-5)', id_err < 1e-5,
                  f"max|H-1|={id_err:.2e}")

            # (b) DC gain == (g0/gp)^2 within 0.01 dB, ~ (f0/fp)^2.
            g0 = math.tan(math.pi * _clamp(f0, 10.0, 0.15 * Fs) / Fs)
            gp = math.tan(math.pi * _clamp(fp, 10.0, 0.15 * Fs) / Fs)
            dc_db, _ = freqz_mag_db(c, [1e-3], Fs)
            dc_db = float(dc_db[0])
            exp_dc_db = 20.0 * np.log10((g0 / gp) ** 2)
            check('(b) DC gain == (g0/gp)^2 (<0.01 dB)',
                  abs(dc_db - exp_dc_db) < 0.01,
                  f"dc={dc_db:.4f} exp={exp_dc_db:.4f} dB")
            approx_dc_db = 20.0 * np.log10((f0 / fp) ** 2)
            check('(b) DC gain ~ (f0/fp)^2 (<0.5 dB)',
                  abs(dc_db - approx_dc_db) < 0.5,
                  f"dc={dc_db:.4f} approx={approx_dc_db:.4f} dB")

            # (c) HF gain -> unity at 0.4*Fs within 0.05 dB.
            hf_db, _ = freqz_mag_db(c, [0.4 * Fs], Fs)
            hf_db = float(hf_db[0])
            check('(c) HF gain -> unity @0.4Fs (<0.05 dB)',
                  abs(hf_db) < 0.05, f"|H|={hf_db:.4f} dB")

            # (d) Analog match: exact at corners, close across band.
            for fc_corner in (f0, fp):
                dmag, _ = freqz_mag_db(c, [fc_corner], Fs)
                amag = analog_lt_mag_db(f0, Q0, fp, Qp, [fc_corner])
                err = abs(float(dmag[0]) - float(amag[0]))
                check(f'(d) corner match @{fc_corner:g}Hz (<0.1 dB)',
                      err < 0.1, f"err={err:.4f} dB")
            fsweep = np.geomspace(10.0, fmax, 1500)
            dsw, _ = freqz_mag_db(c, fsweep, Fs)
            asw = analog_lt_mag_db(f0, Q0, fp, Qp, fsweep)
            band_err = float(np.max(np.abs(dsw - asw)))
            check('(d) analog match 10..2*max (<0.5 dB)',
                  band_err < 0.5, f"maxerr={band_err:.4f} dB")

            # (e) SVF equivalence vs biquad, 10 Hz .. Fs/8, within 0.05 dB.
            sfreqs, smag = svf_impulse_mag_db(f0, Q0, fp, Qp, Fs)
            lo, hi = 10.0, Fs / 8.0
            sel = (sfreqs >= lo) & (sfreqs <= hi)
            bmag, _ = freqz_mag_db(c, sfreqs[sel], Fs)
            svf_err = float(np.max(np.abs(smag[sel] - bmag)))
            check('(e) SVF == biquad 10..Fs/8 (<0.05 dB)',
                  svf_err < 0.05, f"maxerr={svf_err:.4f} dB")

    # (f) Q28 vs float, low-frequency case at Fs=48000, above 20 Hz.
    print(f"\n=== Q28 vs float (Fs=48000) ===")
    Fs = 48000.0
    f0, Q0, fp, Qp = 55.0, 1.1, 20.0, 0.5
    cf = coeffs(f0, Q0, fp, Qp, Fs, label='linkwitz')
    cq = coeffs(f0, Q0, fp, Qp, Fs, label='linkwitzq28')
    fq = np.geomspace(20.0, 0.45 * Fs, 3000)
    mf, _ = freqz_mag_db(cf, fq, Fs)
    mq, _ = freqz_mag_db(cq, fq, Fs)
    q28_err = float(np.max(np.abs(mf - mq)))
    check('(f) Q28 == float >20 Hz (<0.05 dB)',
          q28_err < 0.05, f"maxerr={q28_err:.4f} dB")

    # optional plot (guarded)
    try:
        import matplotlib  # noqa: F401
        _have_mpl = True
    except Exception:
        _have_mpl = False

    print('\n' + '=' * 60)
    if failures:
        print(f"FAILED: {len(failures)} check(s)")
        for f in failures:
            print(f"  - {f}")
        import sys
        sys.exit(1)
    else:
        print("ALL CHECKS PASSED"
              + ("" if _have_mpl else "  (matplotlib not present; no plot)"))
