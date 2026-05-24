"""
user_crossover.py — Python mirror of firmware/DSPi/crossover.c's coefficient
design, exposed for the filter tester.

The intent is a 1:1 transliteration of the C implementation (same pole tables,
same bilinear-with-prewarping math, same LP→HP reciprocal-pole transform,
same cascade ordering). Pair this with test_crossover.py to compare against
scipy.signal as an INDEPENDENT reference — together they verify the C code
produces canonical Butterworth / Linkwitz-Riley / Bessel responses.

Filter type strings follow the firmware enum naming, lowercased:
    lr2_lp,  lr2_hp
    lr4_lp,  lr4_hp
    lr6_lp,  lr6_hp
    lr8_lp,  lr8_hp
    bw1_lp,  bw1_hp .. bw8_lp,  bw8_hp
    bes2_lp, bes2_hp, bes4_lp, bes4_hp, bes6_lp, bes6_hp, bes8_lp, bes8_hp
"""

import numpy as np


# ---------------------------------------------------------------------------
# Filter type metadata (mirrors xover_filter_meta() in crossover.c)
# ---------------------------------------------------------------------------

FAMILY_LR  = 'lr'
FAMILY_BW  = 'bw'
FAMILY_BES = 'bes'


def filter_meta(ftype):
    """Decode a type string into (family, order, is_highpass, num_sections).

    Section count derivation matches crossover.c's xover_type_table.
    """
    ft = ftype.lower()
    family, rest = ft.split('_', 1)   # 'bw3', 'hp'  →  'bw3', 'hp'
    is_hp = (rest == 'hp')

    if family.startswith('lr'):
        order = int(family[2:])
        # Section count matches firmware xover_type_table:
        #   LR2 → 1 (the (BW1)² double real pole collapses into one biquad)
        #   LR_{2N} for N>1 → 2·ceil(N/2) sections.  This covers:
        #     LR4 (N=2, BW2² = biquad × 2)                    → 2
        #     LR6 (N=3, BW3² = (1st + biquad) × 2)            → 4
        #     LR8 (N=4, BW4² = (biquad + biquad) × 2)         → 4
        if order == 2:
            sections = 1
        else:
            bw_order = order // 2
            sections = 2 * ((bw_order + 1) // 2)
        return FAMILY_LR, order, is_hp, sections
    if family.startswith('bw'):
        order = int(family[2:])
        # ceil(N/2) biquads (with a leading 1st-order section for odd N)
        sections = (order + 1) // 2
        return FAMILY_BW, order, is_hp, sections
    if family.startswith('bes'):
        order = int(family[3:])
        sections = order // 2   # only even-order Bessel supported
        return FAMILY_BES, order, is_hp, sections
    raise ValueError(f"Unknown crossover type: {ftype}")


# ---------------------------------------------------------------------------
# Analog prototype pole tables
# ---------------------------------------------------------------------------

# Butterworth: bw_pole_pair(order, pair_idx) → (σ_n, ω_n) for the pair_idx-th
# complex pole pair, where pair_idx counts only complex pairs (not the real
# pole on odd orders).
def bw_pole_pair(order, pair_idx):
    if order & 1:
        theta = np.pi * (pair_idx + 1) / order
    else:
        theta = np.pi * (2 * pair_idx + 1) / (2 * order)
    return np.cos(theta), np.sin(theta)


# Bessel (-3 dB normalized) pole tables — must match crossover.c exactly.
# Verified against scipy.signal.bessel(..., norm='mag') by test_crossover.py.
BESSEL_TABLES = {
    2: [(1.10160, 0.63601)],
    4: [(1.37007, 0.41025), (0.99521, 1.25711)],
    6: [(1.57149, 0.32090), (1.38186, 0.97147), (0.93066, 1.66186)],
    8: [(1.75741, 0.27287), (1.63694, 0.82280),
        (1.37384, 1.38836), (0.89287, 1.99833)],
}


# ---------------------------------------------------------------------------
# Section-level coefficient design (mirrors biquad_assign_*_rp2350 in TDF2
# mode — we don't model the SVF path because the SVF is an equivalent
# realization of the same transfer function; testing TDF2 covers the math).
# ---------------------------------------------------------------------------

def bilinear_2nd_order(sigma, omega, is_hp, Fs):
    """Build a digital biquad from a denormalised analog 2nd-order section.

    Mirrors biquad_assign_2nd_order_rp2350()'s TDF2 branch.  sigma/omega are
    already scaled by ω_a (the prewarped analog cutoff).  Returns
    (b0, b1, b2, a0=1, a1, a2) in the firmware's struct convention
    (a1, a2 are POSITIVE coefficients in y[n] = b0 x + ... − a1 y[n-1] − a2 y[n-2]).
    """
    K = 2.0 * Fs
    A = 2.0 * sigma
    B = sigma * sigma + omega * omega
    A0 = K * K + A * K + B
    A1 = 2.0 * (B - K * K)
    A2 = K * K - A * K + B
    inv = 1.0 / A0
    if is_hp:
        b0 = (K * K) * inv
        b1 = (-2.0 * K * K) * inv
        b2 = (K * K) * inv
    else:
        b0 = B * inv
        b1 = (2.0 * B) * inv
        b2 = B * inv
    a1 = A1 * inv
    a2 = A2 * inv
    return (b0, b1, b2, 1.0, a1, a2)


def bilinear_1st_order(sigma_real, is_hp, Fs):
    """Mirrors biquad_assign_1st_order_*().  sigma_real is already scaled
    by ω_a.  b2 = a2 = 0."""
    K = 2.0 * Fs
    A0 = K + sigma_real
    A1 = sigma_real - K
    inv = 1.0 / A0
    if is_hp:
        b0 = K * inv
        b1 = -K * inv
    else:
        b0 = sigma_real * inv
        b1 = sigma_real * inv
    a1 = A1 * inv
    return (b0, b1, 0.0, 1.0, a1, 0.0)


def emit_2nd_order(sigma_n, omega_n, omega_a, is_hp, Fs):
    """Mirrors section_emit_2nd_order().  Applies the LP→HP reciprocal pole
    transform when is_hp (no-op for r²=1 families, mandatory for Bessel)."""
    if is_hp:
        r2 = sigma_n * sigma_n + omega_n * omega_n
        if r2 > 0.0:
            sigma_n = sigma_n / r2
            omega_n = omega_n / r2
    sigma = sigma_n * omega_a
    omega = omega_n * omega_a
    return bilinear_2nd_order(sigma, omega, is_hp, Fs)


def emit_1st_order(sigma_n, omega_a, is_hp, Fs):
    """Mirrors section_emit_1st_order().  No-op reciprocal for BW1 (σ_n=1);
    defensive for any future family with a non-unit-radius real pole."""
    if is_hp and sigma_n > 0.0:
        sigma_n = 1.0 / sigma_n
    sigma = sigma_n * omega_a
    return bilinear_1st_order(sigma, is_hp, Fs)


# ---------------------------------------------------------------------------
# Family-level cascade assembly (mirrors design_butterworth / _linkwitz_riley
# / _bessel in crossover.c).  Each function returns a LIST of biquad tuples.
# ---------------------------------------------------------------------------

def design_butterworth(order, is_hp, omega_a, Fs):
    sections = []
    if order & 1:
        # Real pole at σ_n = 1
        sections.append(emit_1st_order(1.0, omega_a, is_hp, Fs))
    pairs = order // 2
    for p in range(pairs):
        sigma_n, omega_n = bw_pole_pair(order, p)
        sections.append(emit_2nd_order(sigma_n, omega_n, omega_a, is_hp, Fs))
    return sections


def design_linkwitz_riley(order_lr, is_hp, omega_a, Fs):
    if order_lr == 2:
        # (BW1)² → one 2nd-order section with σ_n=1, ω_n=0 (double real pole)
        return [emit_2nd_order(1.0, 0.0, omega_a, is_hp, Fs)]
    # LR_{2N} = (BW_N)² for N ∈ {2, 3, 4} — LR4/LR6/LR8. Build BW_N's
    # cascade (1st-order first for odd N, then biquads), then duplicate.
    # Resulting layouts mirror design_linkwitz_riley() in crossover.c:
    #   LR4 → [biq0, biq0_dup]
    #   LR6 → [1st, biq0, 1st_dup, biq0_dup]
    #   LR8 → [biq0, biq1, biq0_dup, biq1_dup]
    bw_order = order_lr // 2
    pairs    = bw_order // 2
    has_1st  = (bw_order & 1) != 0
    base = []
    if has_1st:
        # BW real pole at σ_n = 1.  emit_1st_order handles LP→HP reciprocal
        # (no-op for σ=1) and ω_a denormalisation just like the C side.
        base.append(emit_1st_order(1.0, omega_a, is_hp, Fs))
    for p in range(pairs):
        sigma_n, omega_n = bw_pole_pair(bw_order, p)
        base.append(emit_2nd_order(sigma_n, omega_n, omega_a, is_hp, Fs))
    # Duplicate every section (state is independent per-section so duplicates
    # behave as a true cascade-of-two).
    return base + base


def design_bessel(order, is_hp, omega_a, Fs):
    table = BESSEL_TABLES.get(order)
    if not table:
        raise ValueError(f"Bessel order {order} not supported")
    sections = []
    for sigma_n, omega_n in table:
        sections.append(emit_2nd_order(sigma_n, omega_n, omega_a, is_hp, Fs))
    return sections


# ---------------------------------------------------------------------------
# Top-level design + composite frequency response
# ---------------------------------------------------------------------------

def design_filter(ftype, fc, Fs):
    """Mirrors xover_design_filter().  Clamps fc and prewarps before
    handing off to the family-level cascade builder."""
    fc = max(10.0, min(fc, Fs * 0.45))
    omega_a = 2.0 * Fs * np.tan(np.pi * fc / Fs)
    family, order, is_hp, _ = filter_meta(ftype)
    if family == FAMILY_BW:
        return design_butterworth(order, is_hp, omega_a, Fs)
    if family == FAMILY_LR:
        return design_linkwitz_riley(order, is_hp, omega_a, Fs)
    if family == FAMILY_BES:
        return design_bessel(order, is_hp, omega_a, Fs)
    raise ValueError(f"Unsupported family for {ftype}")


def eval_biquad_section(coefs, w):
    """H(e^jw) for a single (b0, b1, b2, a0, a1, a2) section."""
    b0, b1, b2, a0, a1, a2 = coefs
    z1 = np.exp(-1j * w)
    z2 = z1 * z1
    num = b0 + b1 * z1 + b2 * z2
    den = a0 + a1 * z1 + a2 * z2
    return num / den


def eval_cascade(sections, w):
    """Composite H(e^jw) for a cascade — product of per-section responses."""
    H = np.ones_like(w, dtype=complex)
    for sec in sections:
        H *= eval_biquad_section(sec, w)
    return H


# ---------------------------------------------------------------------------
# Filter-tester interface
# ---------------------------------------------------------------------------

def user_response(filter_type, fc, Q, gain_db, Fs, w):
    """Returns H(e^jw) for a crossover filter type.  Q and gain_db are
    ignored (crossover types don't use them) but kept for interface
    compatibility with compare_filter.py."""
    _ = Q, gain_db
    sections = design_filter(filter_type, fc, Fs)
    return eval_cascade(sections, w)
