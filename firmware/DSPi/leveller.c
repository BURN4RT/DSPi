/*
 * leveller.c; Volume Leveller (Dynamic Range Compressor)
 *
 * Implements a feedforward, channel-linked, single-band RMS compressor over
 * the active input channels.  See leveller.h for the multichannel model
 * (detector_mask / apply_mask) and coefficient conventions.
 *
 * Signal flow per block:
 *   1. Per-sample: update RMS envelopes for detector-mask channels
 *   2. Per-block:  link = loudest envelope; compute gain via soft-knee curve
 *   3. Per-block:  smooth gain with asymmetric attack/release
 *   4. Per-sample: lookahead delay on ALL active channels (if enabled),
 *      interpolated gain + safety limiter on apply-mask channels
 *
 * Alignment invariant: with lookahead on, every active input channel passes
 * through its ring (same depth), so applied and non-applied channels stay
 * sample-aligned and mask changes never cause a time shift.
 */

#include <math.h>
#include <string.h>
#include "leveller.h"
#include "dsp_pipeline.h"

// ---------------------------------------------------------------------------
// Speed preset tables: {attack_sec, release_sec, rms_window_sec}
// ---------------------------------------------------------------------------

static const float speed_presets[LEVELLER_SPEED_COUNT][3] = {
    /* Slow   */ { 0.100f, 2.000f, 0.400f },   // Music, orchestral
    /* Medium */ { 0.050f, 1.000f, 0.200f },   // General purpose
    /* Fast   */ { 0.020f, 0.500f, 0.100f },   // Speech, dialogue
};

// ---------------------------------------------------------------------------
// Coefficient Computation
// ---------------------------------------------------------------------------

// Compute one-pole retention coefficient for a given time constant.
// Form A: env = alpha * env + (1-alpha) * x
// alpha near 1.0 = slow, alpha near 0.0 = fast.
// T is the 0%-to-90% step response time.
static float compute_alpha(float sample_rate, float time_sec) {
    if (time_sec <= 0.0f || sample_rate <= 0.0f) return 0.0f;
    return expf(-logf(10.0f) / (sample_rate * time_sec));
}

void leveller_compute_coefficients(LevellerCoeffs *out,
                                   const LevellerConfig *cfg,
                                   float sample_rate) {
    if (sample_rate < 1.0f) sample_rate = 48000.0f;

    // Clamp speed to valid range
    uint8_t spd = cfg->speed;
    if (spd >= LEVELLER_SPEED_COUNT) spd = LEVELLER_SPEED_MEDIUM;

    // Time constants from speed preset
    float attack_sec  = speed_presets[spd][0];
    float release_sec = speed_presets[spd][1];
    float rms_sec     = speed_presets[spd][2];

    // One-pole retention coefficients (Form A)
    out->alpha_rms     = compute_alpha(sample_rate, rms_sec);
    out->alpha_attack  = compute_alpha(sample_rate, attack_sec);
    out->alpha_release = compute_alpha(sample_rate, release_sec);

    // Fixed compression curve parameters
    out->threshold_db      = LEVELLER_THRESHOLD_DB;
    out->knee_width_db     = LEVELLER_KNEE_WIDTH_DB;
    // Gate threshold from config (user-configurable)
    float gate = cfg->gate_threshold_db;
    if (gate < LEVELLER_GATE_MIN) gate = LEVELLER_GATE_MIN;
    if (gate > LEVELLER_GATE_MAX) gate = LEVELLER_GATE_MAX;
    out->gate_threshold_db = gate;

    // Clamp amount
    float amount = cfg->amount;
    if (amount < LEVELLER_AMOUNT_MIN) amount = LEVELLER_AMOUNT_MIN;
    if (amount > LEVELLER_AMOUNT_MAX) amount = LEVELLER_AMOUNT_MAX;

    // Ratio: 1:1 at amount=0%, 20:1 at amount=100%
    float norm = amount / 100.0f;
    out->ratio = 1.0f + norm * 19.0f;

    // Max gain ceiling (clamp from config)
    float max_g = cfg->max_gain_db;
    if (max_g < LEVELLER_MAX_GAIN_MIN) max_g = LEVELLER_MAX_GAIN_MIN;
    if (max_g > LEVELLER_MAX_GAIN_MAX) max_g = LEVELLER_MAX_GAIN_MAX;
#if !PICO_RP2350
    // Q28 linear gain tops out just below 8.0 (18 dB); higher settings
    // would overflow the fixed-point conversion in the apply loop.
    if (max_g > 18.0f) max_g = 18.0f;
#endif
    out->max_gain_db = max_g;

    // No makeup gain; upward compression provides the boost directly.
    // Content below the threshold is boosted, content above is untouched.
    out->makeup_db = 0.0f;

}

// ---------------------------------------------------------------------------
// State Reset
// ---------------------------------------------------------------------------

void leveller_reset_state(LevellerState *state) {
    memset(state, 0, sizeof(LevellerState));
#if PICO_RP2350
    state->gain_linear = 1.0f;
    state->gain_prev_linear = 1.0f;
#else
    state->gain_q28 = (1 << FILTER_SHIFT);       // 1.0 in Q28
    state->gain_prev_q28 = (1 << FILTER_SHIFT);
#endif
    state->gain_smooth_db = 0.0f;  // 0 dB = unity
}

// ---------------------------------------------------------------------------
// Upward Compression Gain Computer
//
// Returns the gain boost in dB for a given input level.
// Boosts content BELOW the threshold, leaves content ABOVE untouched.
// Uses a quadratic soft knee around the threshold for smooth transition.
//
//   Above knee:  no boost (loud content untouched)
//   Within knee: quadratic blend from full boost to 0
//   Below knee:  full upward compression = (threshold - x) * (1 - 1/R)
//
// This is the inverse of a traditional downward compressor: instead of
// pushing loud content down, it lifts quiet content up. No makeup gain
// needed; the boost IS the compression. Loud content passes through
// at unity, so the limiter rarely engages.
// ---------------------------------------------------------------------------

static inline float gain_computer(float x_db, float threshold, float ratio,
                                  float knee_width) {
    float half_knee = knee_width * 0.5f;

    if (x_db > (threshold + half_knee)) {
        // Above knee: no boost (leave loud content alone)
        return 0.0f;
    } else if (x_db >= (threshold - half_knee)) {
        // Within soft knee: quadratic transition from boost to unity
        float d = threshold + half_knee - x_db;
        return (1.0f - 1.0f / ratio) * d * d / (2.0f * knee_width);
    } else {
        // Below knee: full upward compression
        return (threshold - x_db) * (1.0f - 1.0f / ratio);
    }
}

// ---------------------------------------------------------------------------
// RP2350 Float Block Processing
// ---------------------------------------------------------------------------

#if PICO_RP2350

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            float *const *bufs, uint32_t n_bufs,
                            uint32_t count) {
    if (count == 0 || n_bufs == 0) return;
    if (n_bufs > NUM_INPUT_CHANNELS) n_bufs = NUM_INPUT_CHANNELS;

    // Inputs that just became active carry stale ring/envelope content from
    // the last time this many channels streamed; clear before use.
    if (n_bufs != state->active_prev) {
        for (uint32_t k = state->active_prev; k < n_bufs; k++) {
            memset(state->lookahead_buf[k], 0, sizeof(state->lookahead_buf[k]));
            state->env_sq[k] = 0.0f;
        }
        state->active_prev = (uint8_t)n_bufs;
    }

    // Snapshot masks once per block, gated to the active channel set
    const uint32_t active_mask = (1u << n_bufs) - 1u;
    const uint32_t det_mask = (uint32_t)cfg->detector_mask & active_mask;
    const uint32_t app_mask = (uint32_t)cfg->apply_mask & active_mask;

    // ---- Per-sample: update RMS envelopes (detector channels only) ----
    const float a_rms = coeffs->alpha_rms;
    const float one_minus_a_rms = 1.0f - a_rms;
    float link_sq = 0.0f;   // Loudest detector envelope (linked level)

    for (uint32_t k = 0; k < n_bufs; k++) {
        if (!(det_mask & (1u << k))) {
            // Outside the detector set: drop the envelope so a later mask
            // re-enable starts fresh instead of pumping from a stale level.
            state->env_sq[k] = 0.0f;
            continue;
        }
        const float *s = bufs[k];
        float env = state->env_sq[k];
        for (uint32_t i = 0; i < count; i++) {
            float x = s[i];
            env = a_rms * env + one_minus_a_rms * (x * x);
        }
        // Prevent denormals in silent passages
        if (env < 1e-30f) env = 0.0f;
        state->env_sq[k] = env;
        if (env > link_sq) link_sq = env;
    }

    // ---- Per-block: compute target gain from the linked level ----
    float rms_db = 10.0f * log10f(link_sq + 1e-30f);

    float gc_db;
    if (rms_db < coeffs->gate_threshold_db) {
        // Below silence gate: unity gain (no boost, prevents noise pumping)
        gc_db = 0.0f;
    } else {
        // Soft-knee compression curve
        gc_db = gain_computer(rms_db, coeffs->threshold_db,
                              coeffs->ratio, coeffs->knee_width_db);
        gc_db += coeffs->makeup_db;

        // Clamp to max gain ceiling
        if (gc_db > coeffs->max_gain_db) gc_db = coeffs->max_gain_db;
    }

    // ---- Per-block: asymmetric gain smoothing ----
    // alpha_attack/release are per-SAMPLE coefficients. Since we apply the
    // smoother once per BLOCK, raise to the block size to get the correct
    // per-block alpha. Without this, time constants are block_size× too slow.
    float alpha_sample = (gc_db < state->gain_smooth_db) ? coeffs->alpha_attack
                                                          : coeffs->alpha_release;
    float alpha = powf(alpha_sample, (float)count);
    state->gain_smooth_db = alpha * state->gain_smooth_db
                          + (1.0f - alpha) * gc_db;

    // Save previous gain for interpolation, compute new linear gain
    state->gain_prev_linear = state->gain_linear;
    state->gain_linear = powf(10.0f, state->gain_smooth_db / 20.0f);

    // Snapshot the applied-channel buffers once (not per sample)
    float *ap[NUM_INPUT_CHANNELS];
    uint32_t na = 0;
    for (uint32_t k = 0; k < n_bufs; k++) {
        if (app_mask & (1u << k)) ap[na++] = bufs[k];
    }

    bool use_la = cfg->lookahead;
    if (!use_la && na == 0) return;   // Gain state updated; nothing to touch

    // ---- Per-sample: lookahead delay + interpolated gain + limiter ----
    // The limiter caps the GAIN (not the output level) so the leveller never
    // creates content above the ceiling, but content already above it passes
    // through untouched. Per-sample: gain = min(leveller_gain, ceil / |input|),
    // linked over the applied channels so the mix balance is preserved.
    float gain_prev = state->gain_prev_linear;
    float gain_cur  = state->gain_linear;
    float gain, gain_step;

    if (count == 1) {
        gain = gain_cur;
        gain_step = 0.0f;
    } else {
        gain_step = (gain_cur - gain_prev) / (float)(count - 1);
        gain = gain_prev;
    }

    const float ceil = LEVELLER_LIMITER_CEIL;
    uint32_t la_idx = state->la_write_idx;

    for (uint32_t i = 0; i < count; i++) {
        if (use_la) {
            // Every active channel goes through its ring so applied and
            // non-applied channels stay sample-aligned (see header).
            for (uint32_t k = 0; k < n_bufs; k++) {
                float *ring = state->lookahead_buf[k];
                float delayed = ring[la_idx];
                ring[la_idx] = bufs[k][i];
                bufs[k][i] = delayed;
            }
            la_idx++;
            if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
        }

        // Cap gain so the leveller never boosts a sample above the ceiling.
        // If a sample is already above the ceiling, gain is capped at 1.0
        // (pass-through); existing loud content is never attenuated.
        float g = gain;
        if (g > 1.0f && na > 0) {
            float peak = 0.0f;
            for (uint32_t j = 0; j < na; j++) {
                float a = fabsf(ap[j][i]);
                if (a > peak) peak = a;
            }
            if (peak > 0.0f) {
                float max_g = ceil / peak;
                if (max_g < g) g = (max_g > 1.0f) ? max_g : 1.0f;
            }
        }

        for (uint32_t j = 0; j < na; j++) {
            ap[j][i] *= g;
        }
        gain += gain_step;
    }

    if (use_la) state->la_write_idx = la_idx;
}

#else  // RP2040

// ---------------------------------------------------------------------------
// RP2040 Q28 Fixed-Point Block Processing
//
// Envelope update and gain application use Q28 arithmetic via fast_mul_q28().
// Gain computation (log/exp/soft knee) uses float; it runs once per block
// (~1ms), so the cost of the Pico SDK ROM float routines is acceptable.
// RP2040 has exactly 2 input channels, so the masked loops stay 2-wide.
// ---------------------------------------------------------------------------

DSP_TIME_CRITICAL
void leveller_process_block(LevellerState *state,
                            const LevellerCoeffs *coeffs,
                            const LevellerConfig *cfg,
                            int32_t *const *bufs, uint32_t n_bufs,
                            uint32_t count) {
    if (count == 0 || n_bufs == 0) return;
    if (n_bufs > NUM_INPUT_CHANNELS) n_bufs = NUM_INPUT_CHANNELS;

    int32_t *buf_l = bufs[0];
    int32_t *buf_r = (n_bufs > 1) ? bufs[1] : NULL;

    const uint32_t active_mask = (1u << n_bufs) - 1u;
    const uint32_t det_mask = (uint32_t)cfg->detector_mask & active_mask;
    const uint32_t app_mask = (uint32_t)cfg->apply_mask & active_mask;
    const bool det_l = (det_mask & 1u) != 0;
    const bool det_r = (det_mask & 2u) != 0 && buf_r;
    const bool app_l = (app_mask & 1u) != 0;
    const bool app_r = (app_mask & 2u) != 0 && buf_r;

    // ---- Per-sample: update RMS envelopes (Q28, detector channels only) ----

    // Pre-compute (1 - alpha_rms) in Q28 for the envelope update.
    // alpha_rms is a float in [0,1]; convert both coefficients to Q28.
    int32_t a_rms_q28 = (int32_t)(coeffs->alpha_rms * (float)(1 << FILTER_SHIFT));
    int32_t one_minus_a_q28 = (1 << FILTER_SHIFT) - a_rms_q28;

    int32_t env_l = state->env_sq[0];
    int32_t env_r = state->env_sq[1];

    // Channels outside the detector set drop their envelope so a later
    // mask re-enable starts fresh instead of pumping from a stale level.
    if (det_l) {
        for (uint32_t i = 0; i < count; i++) {
            int32_t sl = buf_l[i];
            int32_t sq = fast_mul_q28(sl, sl);
            env_l = fast_mul_q28(a_rms_q28, env_l) + fast_mul_q28(one_minus_a_q28, sq);
        }
        state->env_sq[0] = env_l;
    } else {
        env_l = 0;
        state->env_sq[0] = 0;
    }
    if (det_r) {
        for (uint32_t i = 0; i < count; i++) {
            int32_t sr = buf_r[i];
            int32_t sq = fast_mul_q28(sr, sr);
            env_r = fast_mul_q28(a_rms_q28, env_r) + fast_mul_q28(one_minus_a_q28, sq);
        }
        state->env_sq[1] = env_r;
    } else {
        env_r = 0;
        state->env_sq[1] = 0;
    }

    // ---- Per-block: compute target gain from the linked level (float) ----

    const float inv_q28 = 1.0f / (float)(1 << FILTER_SHIFT);
    float link_sq = 0.0f;
    if (det_l) {
        float e = (float)env_l * inv_q28;
        if (e > link_sq) link_sq = e;
    }
    if (det_r) {
        float e = (float)env_r * inv_q28;
        if (e > link_sq) link_sq = e;
    }
    float rms_db = 10.0f * log10f(link_sq + 1e-30f);

    float gc_db;
    if (rms_db < coeffs->gate_threshold_db) {
        gc_db = 0.0f;
    } else {
        gc_db = gain_computer(rms_db, coeffs->threshold_db,
                              coeffs->ratio, coeffs->knee_width_db);
        gc_db += coeffs->makeup_db;
        if (gc_db > coeffs->max_gain_db) gc_db = coeffs->max_gain_db;
    }

    // Asymmetric gain smoothing (float)
    // Raise per-sample alpha to block size for correct per-block time constant
    float alpha_sample = (gc_db < state->gain_smooth_db) ? coeffs->alpha_attack
                                                          : coeffs->alpha_release;
    float alpha = powf(alpha_sample, (float)count);
    state->gain_smooth_db = alpha * state->gain_smooth_db
                          + (1.0f - alpha) * gc_db;

    // Convert smoothed gain to Q28 linear; clamp below 8.0 so the
    // conversion cannot overflow (coeffs already cap max gain at 18 dB)
    float gain_linear = powf(10.0f, state->gain_smooth_db / 20.0f);
    if (gain_linear > 7.99f) gain_linear = 7.99f;
    state->gain_prev_q28 = state->gain_q28;
    state->gain_q28 = (int32_t)(gain_linear * (float)(1 << FILTER_SHIFT));

    bool use_la = cfg->lookahead;
    if (!use_la && !app_l && !app_r) return;   // Gain state updated; no apply

    // ---- Per-sample: lookahead delay + interpolated gain + limiter ----
    // Limiter caps the GAIN so the leveller never boosts a sample above the
    // ceiling; existing loud content passes through untouched.  Linked over
    // the applied channels only.
    int32_t g_prev = state->gain_prev_q28;
    int32_t g_cur  = state->gain_q28;
    const int32_t unity_q28 = (1 << FILTER_SHIFT);
    const float ceil = LEVELLER_LIMITER_CEIL;

    uint32_t la_idx = state->la_write_idx;

    for (uint32_t i = 0; i < count; i++) {
        int32_t gain;
        if (count == 1) {
            gain = g_cur;
        } else {
            gain = g_prev + (int32_t)(((int64_t)(g_cur - g_prev) * i) / (int32_t)(count - 1));
        }

        int32_t out_l, out_r = 0;

        if (use_la) {
            // Both channels always pass through their rings so applied and
            // non-applied channels stay sample-aligned.
            out_l = state->lookahead_buf[0][la_idx];
            state->lookahead_buf[0][la_idx] = buf_l[i];
            buf_l[i] = out_l;
            if (buf_r) {
                out_r = state->lookahead_buf[1][la_idx];
                state->lookahead_buf[1][la_idx] = buf_r[i];
                buf_r[i] = out_r;
            }
            la_idx++;
            if (la_idx >= LEVELLER_LOOKAHEAD_SAMPLES) la_idx = 0;
        } else {
            out_l = buf_l[i];
            if (buf_r) out_r = buf_r[i];
        }

        // Cap gain so leveller never boosts above ceiling; pass-through if already loud
        if (gain > unity_q28 && (app_l || app_r)) {
            float peak = 0.0f;
            if (app_l) peak = fabsf((float)out_l * inv_q28);
            if (app_r) {
                float pr = fabsf((float)out_r * inv_q28);
                if (pr > peak) peak = pr;
            }
            if (peak > 0.0f) {
                float max_g_f = ceil / peak;
                // Beyond Q28 range the cap can never bind on a <8.0 gain,
                // and converting it would overflow; skip instead.
                if (max_g_f < 8.0f) {
                    int32_t max_g_q28 = (int32_t)(max_g_f * (float)unity_q28);
                    if (max_g_q28 < gain) gain = (max_g_q28 > unity_q28) ? max_g_q28 : unity_q28;
                }
            }
        }

        if (app_l) buf_l[i] = fast_mul_q28(out_l, gain);
        if (app_r) buf_r[i] = fast_mul_q28(out_r, gain);
    }

    if (use_la) state->la_write_idx = la_idx;
}

#endif  // PICO_RP2350
