#!/usr/bin/env python3
"""
test_crossover.py — Verify firmware/DSPi/crossover.c's coefficient design
against scipy.signal as an independent reference.

Validates 32 filter types (LR2/4/6/8, BW1-8, Bes2/4/6/8 — each in LP and HP)
across multiple sample rates and cutoff frequencies. Compares both magnitude
and phase responses to catch:

  - Pole table typos (Bessel)
  - Butterworth pole angle errors
  - Bilinear-with-prewarping math errors
  - LP→HP transform mistakes (the Bessel HP path uses reciprocal poles —
    this test catches the difference between right and wrong implementations
    because Bessel pole radii are NOT 1)
  - LR cascade assembly errors (LR_2N = (BW_N)²)

The "user under test" is user_crossover.py, a 1:1 Python transliteration of
crossover.c. The "reference" is scipy.signal.butter / bessel (with the same
bilinear-with-prewarping approach) — independent implementations of the same
canonical math.

Usage (from inside the prepared venv):

    python tools/filter_tester/test_crossover.py
    python tools/filter_tester/test_crossover.py --threshold 0.05
    python tools/filter_tester/test_crossover.py --plot-failures
"""

import argparse
import sys

import numpy as np
import scipy.signal as sig

# Local user-module mirror of crossover.c
sys.path.insert(0, '/Users/weeblabs/DSPi/tools/filter_tester')
import user_crossover as uc


# ---------------------------------------------------------------------------
# scipy reference: digital biquad-cascade frequency response for any of our
# crossover filter types.  Uses scipy's design routines as the ground truth.
# ---------------------------------------------------------------------------

def reference_sos(ftype, fc, Fs):
    """Build a scipy second-order-section description of the reference filter.

    For Butterworth and Bessel we use scipy's design directly (digital,
    bilinear-prewarped).  For Linkwitz-Riley we exploit LR_{2N} = (BW_N)²
    and concatenate the BW_N sections with themselves — scipy doesn't have
    a native LR call, but the squared cascade is the textbook definition.

    Returns a (K, 6) ndarray of SOS rows (b0,b1,b2,a0,a1,a2 per row).
    """
    family, order, is_hp, _ = uc.filter_meta(ftype)
    btype = 'highpass' if is_hp else 'lowpass'

    if family == 'bw':
        return sig.butter(order, fc, btype=btype, output='sos', fs=Fs)
    if family == 'bes':
        # norm='mag' selects the -3 dB normalization convention (matches
        # crossover.c's Bessel pole table convention).
        return sig.bessel(order, fc, btype=btype, output='sos', fs=Fs,
                          norm='mag')
    if family == 'lr':
        # LR_{2N} = (BW_N)². Compute BW_N SOS at half the LR order, then
        # concatenate with itself for the squared cascade.
        bw_sos = sig.butter(order // 2, fc, btype=btype, output='sos', fs=Fs)
        return np.vstack([bw_sos, bw_sos])
    raise ValueError(f"Unknown family for {ftype}")


def reference_response(ftype, fc, Fs, w):
    """Evaluate the reference's H(e^jw) at the given normalized frequencies."""
    sos = reference_sos(ftype, fc, Fs)
    _, H = sig.sosfreqz(sos, worN=w)
    return H


# ---------------------------------------------------------------------------
# Sweep / comparison
# ---------------------------------------------------------------------------

# All 32 crossover filter types.
#   LR2, LR4, LR6, LR8           — 4 orders × 2 shapes =  8
#   BW1 .. BW8                   — 8 orders × 2 shapes = 16
#   Bes2, Bes4, Bes6, Bes8       — 4 orders × 2 shapes =  8
# Must stay in lock-step with xover_type_table in firmware/DSPi/crossover.c.
ALL_TYPES = (
    [f'lr{n}_{s}' for n in (2, 4, 6, 8) for s in ('lp', 'hp')]
    + [f'bw{n}_{s}' for n in range(1, 9) for s in ('lp', 'hp')]
    + [f'bes{n}_{s}' for n in (2, 4, 6, 8) for s in ('lp', 'hp')]
)
assert len(ALL_TYPES) == 32, "ALL_TYPES drifted from firmware crossover enum"


# Sweep points: cover sub, midbass, mid, treble, and near-Nyquist boundary.
# Two sample rates: 48 kHz (primary) and 96 kHz (high-Fs sanity).
TEST_CONFIGS = [
    # (Fs, fc) — fc chosen to land below, near, and above the SVF/TDF2
    # boundary at Fs/7.5, plus an extreme near-Nyquist test.
    (48000,   80.0),    # sub crossover band
    (48000,  500.0),    # well within SVF regime
    (48000, 2000.0),    # close to Fs/7.5 = 6400 Hz
    (48000, 8000.0),    # above SVF boundary
    (48000, 18000.0),   # near Nyquist
    (96000,  100.0),
    (96000, 1000.0),
    (96000, 10000.0),
    (96000, 40000.0),
]


def compare_one(ftype, fc, Fs, n_freqs=512):
    """Compare our implementation against scipy at one config.  Returns a
    dict with magnitude and phase error metrics + the raw response data
    (for optional plotting)."""
    f = np.logspace(np.log10(10.0), np.log10(Fs * 0.475), n_freqs)
    w = 2 * np.pi * f / Fs

    user = uc.user_response(ftype, fc, 0.0, 0.0, Fs, w)
    ref = reference_response(ftype, fc, Fs, w)

    # Magnitude (dB)
    mag_user_db = 20 * np.log10(np.abs(user) + 1e-30)
    mag_ref_db = 20 * np.log10(np.abs(ref) + 1e-30)
    mag_err_db = mag_user_db - mag_ref_db

    # Phase — only meaningful where both magnitudes are above noise floor.
    # Below ~-100 dB, the angle is dominated by numerical noise and the
    # comparison is meaningless. Mask it out.
    mask = (mag_user_db > -100) & (mag_ref_db > -100)
    phase_user = np.unwrap(np.angle(user))
    phase_ref = np.unwrap(np.angle(ref))
    phase_diff = np.degrees(phase_user - phase_ref)
    # Fold to [0, 180] so direction doesn't matter
    phase_diff = (phase_diff + 180) % 360 - 180
    phase_err_deg = np.abs(phase_diff)

    max_mag_err = float(np.max(np.abs(mag_err_db)))
    max_phase_err = float(np.max(phase_err_deg[mask])) if mask.any() else 0.0

    return {
        'type': ftype, 'fc': fc, 'Fs': Fs,
        'max_mag_err_db': max_mag_err,
        'max_phase_err_deg': max_phase_err,
        'f_hz': f, 'mag_user_db': mag_user_db, 'mag_ref_db': mag_ref_db,
        'phase_user_deg': np.degrees(phase_user),
        'phase_ref_deg': np.degrees(phase_ref),
        'mag_err_db': mag_err_db, 'phase_err_deg': phase_err_deg,
    }


def run_sweep(threshold_mag_db, threshold_phase_deg, types=ALL_TYPES,
              configs=TEST_CONFIGS, summary_only=False):
    """Run the sweep.  If `summary_only`, prints one aggregated row per type
    (worst-case across all configs) instead of the per-config detail rows."""
    failures = []
    total = 0
    per_type = {}   # ftype → {'max_mag': float, 'max_phase': float, 'fails': int, 'configs': int}

    if not summary_only:
        header = (f"{'type':<10} {'Fs Hz':>7} {'fc Hz':>9} "
                  f"{'max |mag err|':>15} {'max phase err':>15}  status")
        print(header)
        print('-' * len(header))

    for ftype in types:
        slot = per_type.setdefault(ftype, {'max_mag': 0.0, 'max_phase': 0.0,
                                           'fails': 0, 'configs': 0,
                                           'worst_config': None})
        for Fs, fc in configs:
            total += 1
            r = compare_one(ftype, fc, Fs)
            mag_ok = r['max_mag_err_db'] <= threshold_mag_db
            phase_ok = r['max_phase_err_deg'] <= threshold_phase_deg
            ok = mag_ok and phase_ok

            slot['configs'] += 1
            if r['max_mag_err_db'] > slot['max_mag']:
                slot['max_mag'] = r['max_mag_err_db']
                slot['worst_config'] = (Fs, fc)
            if r['max_phase_err_deg'] > slot['max_phase']:
                slot['max_phase'] = r['max_phase_err_deg']
            if not ok:
                slot['fails'] += 1
                failures.append(r)

            if not summary_only:
                status = 'OK  ' if ok else 'FAIL'
                print(f"{ftype:<10} {Fs:>7} {fc:>9.1f} "
                      f"{r['max_mag_err_db']:>11.4f} dB "
                      f"{r['max_phase_err_deg']:>11.3f}°    {status}")

    if summary_only:
        header = (f"{'type':<10} {'configs':>8} "
                  f"{'worst |mag err|':>17} {'worst phase err':>17} "
                  f"{'worst (Fs/fc)':>22}  status")
        print(header)
        print('-' * len(header))
        for ftype in types:
            slot = per_type[ftype]
            status = ('OK  ' if slot['fails'] == 0
                      else f'FAIL ({slot["fails"]})')
            Fs_w, fc_w = slot['worst_config'] if slot['worst_config'] else (0, 0)
            print(f"{ftype:<10} "
                  f"{slot['configs']:>4} ok? "
                  f"{slot['max_mag']:>13.4f} dB "
                  f"{slot['max_phase']:>13.3f}°  "
                  f"{Fs_w:>10}/{fc_w:>9.1f}   {status}")

    print()
    print(f"{total - len(failures)} / {total} configs pass "
          f"(mag ≤ {threshold_mag_db} dB, phase ≤ {threshold_phase_deg}°)")
    return failures


def plot_failure(r):
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available; skipping plot")
        return
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    axes[0].semilogx(r['f_hz'], r['mag_ref_db'], label='scipy ref', lw=1.8)
    axes[0].semilogx(r['f_hz'], r['mag_user_db'], label='user', lw=1.0, ls='--')
    axes[0].set_ylabel('Magnitude (dB)')
    axes[0].legend(); axes[0].grid(True, which='both', alpha=0.3)
    axes[1].semilogx(r['f_hz'], r['mag_err_db'], color='tab:red')
    axes[1].set_ylabel('Mag err (dB)')
    axes[1].grid(True, which='both', alpha=0.3)
    axes[2].semilogx(r['f_hz'], r['phase_ref_deg'], label='scipy ref', lw=1.8)
    axes[2].semilogx(r['f_hz'], r['phase_user_deg'], label='user', lw=1.0, ls='--')
    axes[2].set_ylabel('Phase (deg)')
    axes[2].set_xlabel('Frequency (Hz)')
    axes[2].legend(); axes[2].grid(True, which='both', alpha=0.3)
    fig.suptitle(f"{r['type']} fc={r['fc']:g} Hz Fs={r['Fs']:g} "
                 f"(max mag err {r['max_mag_err_db']:.3f} dB, "
                 f"max phase err {r['max_phase_err_deg']:.2f}°)")
    fig.tight_layout()
    plt.show()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0])
    ap.add_argument('--threshold', type=float, default=0.05,
                    help='Max acceptable magnitude error in dB (default 0.05)')
    ap.add_argument('--phase-threshold', type=float, default=1.0,
                    help='Max acceptable phase error in degrees (default 1.0)')
    ap.add_argument('--types', default=None,
                    help='Comma-separated subset (default: all 30)')
    ap.add_argument('--plot-failures', action='store_true')
    ap.add_argument('--summary', action='store_true',
                    help='Print one row per filter type (worst-case across '
                         'all configs) instead of the full per-config table')
    args = ap.parse_args()

    types = ALL_TYPES if args.types is None else [t.strip() for t in args.types.split(',')]
    failures = run_sweep(args.threshold, args.phase_threshold, types=types,
                         summary_only=args.summary)

    if failures and args.plot_failures:
        for r in failures:
            plot_failure(r)

    return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
