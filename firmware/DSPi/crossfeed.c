/*
 * BS2B Crossfeed Implementation
 *
 * Implements Bauer Stereophonic-to-Binaural (BS2B) crossfeed for headphone listening.
 * Reduces unnatural stereo separation by mixing a filtered portion of each channel
 * into the opposite channel, simulating speaker listening in a room.
 *
 * Uses complementary filter design with ITD:
 *   - Lowpass filter computes the crossfeed signal (ILD/head shadow)
 *   - First-order all-pass adds interaural time delay to the crossfeed path
 *   - Direct path is the complement: input - lowpass(input)
 *   - Output: out_L = (in_L - lp_L) + allpass(lp_R)
 *
 * Mono signals pass through at unity gain at DC (complementary property).
 * Hard-panned HF content is unchanged (lowpass → 0 at HF).
 *
 * Per-output-pair design: coefficients are shared (one user config drives
 * every selected pair); filter state is independent per output pair so the
 * pipeline can run crossfeed on each stereo output pair in isolation.
 */

#include <math.h>
#include <string.h>
#include "crossfeed.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"   // matrix_mixer: per-output enables gate pair processing
#include "siggen.h"      // siggen_raw_mask: RAW outputs bypass crossfeed

// Preset definitions: {cutoff_hz, feed_db}
// feed_db = level difference between direct and crossfeed at DC
static const float presets[][2] = {
    { 700.0f,  4.5f },  // Default - balanced, most popular
    { 700.0f,  6.0f },  // Chu Moy - stronger spatial effect
    { 650.0f,  9.5f },  // Jan Meier - subtle, natural
};

// Per-output-pair filter state, one entry per stereo output pair.
CrossfeedPairState crossfeed_pair_state[NUM_SPDIF_INSTANCES];

// Published coefficient set the pipeline snapshots each packet; NULL = disabled.
volatile const CrossfeedCoeffs *current_crossfeed_coeffs = NULL;

// Double buffer so crossfeed_apply_config() can compute into the inactive
// buffer and publish, never writing through the currently published pointer.
static CrossfeedCoeffs xf_coeff_bufs[2];
static uint8_t xf_coeff_idx = 0;

void crossfeed_compute_coefficients(CrossfeedCoeffs *coeffs, const CrossfeedConfig *config, float sample_rate) {
    if (!config->enabled || sample_rate < 1.0f) {
        memset(coeffs, 0, sizeof(CrossfeedCoeffs));
        return;
    }

    // Get cutoff and feed level from preset or custom
    float fc, feed_db;
    if (config->preset < 3) {
        fc = presets[config->preset][0];
        feed_db = presets[config->preset][1];
    } else {
        fc = config->custom_fc;
        feed_db = config->custom_feed_db;
        if (fc < CROSSFEED_FREQ_MIN) fc = CROSSFEED_FREQ_MIN;
        if (fc > CROSSFEED_FREQ_MAX) fc = CROSSFEED_FREQ_MAX;
        if (feed_db < CROSSFEED_FEED_MIN) feed_db = CROSSFEED_FEED_MIN;
        if (feed_db > CROSSFEED_FEED_MAX) feed_db = CROSSFEED_FEED_MAX;
    }

    // =========================================================================
    // Compute crossfeed gain G using complementary constraint
    //
    // feed_db is the level difference: 20*log10(direct_dc / cross_dc)
    // With complementary constraint: direct_dc + cross_dc = 1
    //
    //   direct_dc / cross_dc = 10^(feed_db/20) = level_ratio
    //   cross_dc = 1 / (1 + level_ratio) = G
    //   direct_dc = 1 - G
    //
    // Example (4.5 dB): level_ratio=1.679, G=0.373, direct=0.627
    // =========================================================================
    float level_ratio = powf(10.0f, feed_db / 20.0f);
    float G = 1.0f / (1.0f + level_ratio);

    // =========================================================================
    // Lowpass filter (crossfeed path) - single pole IIR
    // H(z) = G*(1-x) / (1 - x*z^-1)   where x = exp(-2π*Fc/Fs)
    // DC gain = G, HF gain → 0
    // =========================================================================
    float x = expf(-2.0f * 3.1415926535f * fc / sample_rate);
    float lp_a0_f = G * (1.0f - x);
    float lp_b1_f = x;

    // =========================================================================
    // All-pass filter for Interaural Time Delay (ITD)
    //
    // The lowpass filter already introduces phase delay at DC:
    //   τ_lp = x / ((1-x) * Fs)  seconds
    //
    // The remaining delay is provided by a first-order all-pass:
    //   H_ap(z) = (a + z^-1) / (1 + a*z^-1)
    //   Group delay at DC = (1-a) / (1+a) samples
    //
    // Solving for a:  a = (1 - D) / (1 + D)
    // where D = remaining delay in samples
    //
    // For 700Hz @ 48kHz: lp_delay ≈ 217µs, ITD = 220µs, remainder ≈ 3µs
    // For 2000Hz @ 48kHz: lp_delay ≈ 80µs, ITD = 220µs, remainder ≈ 140µs
    //
    // When ITD is disabled, a = 1.0 makes the all-pass a pure passthrough.
    // =========================================================================
    float ap_a_f;
    if (config->itd_enabled) {
        float lp_delay_sec = x / ((1.0f - x) * sample_rate);
        float remaining_sec = CROSSFEED_ITD_SEC - lp_delay_sec;
        if (remaining_sec > 0.0f) {
            float D = remaining_sec * sample_rate;  // remaining delay in samples
            ap_a_f = (1.0f - D) / (1.0f + D);
        } else {
            ap_a_f = 1.0f;  // No additional delay needed (lowpass already provides enough)
        }
    } else {
        ap_a_f = 1.0f;  // ITD disabled: all-pass is passthrough
    }

#if PICO_RP2350
    coeffs->lp_a0 = lp_a0_f;
    coeffs->lp_b1 = lp_b1_f;
    coeffs->ap_a = ap_a_f;
#else
    float scale = (float)(1LL << 28);
    coeffs->lp_a0 = (int32_t)(lp_a0_f * scale);
    coeffs->lp_b1 = (int32_t)(lp_b1_f * scale);
    coeffs->ap_a = (int32_t)(ap_a_f * scale);
#endif
}

void crossfeed_apply_config(const CrossfeedConfig *config, float sample_rate) {
    // Compute into the inactive buffer, then publish the pointer.
    // Publish-then-snapshot contract: the pipeline reads current_crossfeed_coeffs
    // once per packet, so a plain atomic pointer store is sufficient here; we
    // never mutate the buffer the published pointer refers to.
    CrossfeedCoeffs *next = &xf_coeff_bufs[xf_coeff_idx ^ 1];
    crossfeed_compute_coefficients(next, config, sample_rate);
    if (config->enabled) {
        xf_coeff_idx ^= 1;
        current_crossfeed_coeffs = next;
    } else {
        current_crossfeed_coeffs = NULL;
    }
}

#if PICO_RP2350
// RP2350 Float processing
DSP_TIME_CRITICAL
void crossfeed_process_pair_block(const CrossfeedCoeffs *coeffs, CrossfeedPairState *st,
                                  float *left, float *right, uint32_t n) {
    // Load shared coefficients and per-pair state into locals once.
    const float lp_a0 = coeffs->lp_a0, lp_b1 = coeffs->lp_b1, ap_a = coeffs->ap_a;
    float lp_state_L = st->lp_state_L, lp_state_R = st->lp_state_R;
    float ap_state_L = st->ap_state_L, ap_state_R = st->ap_state_R;

    for (uint32_t i = 0; i < n; i++) {
        float in_L = left[i];
        float in_R = right[i];

        // Lowpass filter both channels: cross = G × L(z) × input
        float lp_out_L = lp_a0 * in_L + lp_b1 * lp_state_L;
        float lp_out_R = lp_a0 * in_R + lp_b1 * lp_state_R;
        lp_state_L = lp_out_L;
        lp_state_R = lp_out_R;

        // All-pass filter on crossfeed signals for ITD
        // First-order all-pass, transposed direct form II:
        //   y[n] = a * x[n] + s[n]
        //   s[n+1] = x[n] - a * y[n]
        float ap_out_L = ap_a * lp_out_L + ap_state_L;
        ap_state_L = lp_out_L - ap_a * ap_out_L;
        float ap_out_R = ap_a * lp_out_R + ap_state_R;
        ap_state_R = lp_out_R - ap_a * ap_out_R;

        // Complementary mixing with ITD:
        //   direct = input - own_lowpass (undelayed complement)
        //   output = direct + allpass(opp_lowpass) (delayed crossfeed from opposite)
        left[i]  = (in_L - lp_out_L) + ap_out_R;
        right[i] = (in_R - lp_out_R) + ap_out_L;
    }

    // Store per-pair state back.
    st->lp_state_L = lp_state_L;
    st->lp_state_R = lp_state_R;
    st->ap_state_L = ap_state_L;
    st->ap_state_R = ap_state_R;
}

#else
// RP2040 Fixed-point processing (Q28)
DSP_TIME_CRITICAL
void crossfeed_process_pair_block(const CrossfeedCoeffs *coeffs, CrossfeedPairState *st,
                                  int32_t *left, int32_t *right, uint32_t n) {
    // Load shared coefficients and per-pair state into locals once.
    const int32_t lp_a0 = coeffs->lp_a0, lp_b1 = coeffs->lp_b1, ap_a = coeffs->ap_a;
    int32_t lp_state_L = st->lp_state_L, lp_state_R = st->lp_state_R;
    int32_t ap_state_L = st->ap_state_L, ap_state_R = st->ap_state_R;

    for (uint32_t i = 0; i < n; i++) {
        int32_t in_L = left[i];
        int32_t in_R = right[i];

        // Lowpass filter both channels
        int32_t lp_out_L = fast_mul_q28(lp_a0, in_L) + fast_mul_q28(lp_b1, lp_state_L);
        int32_t lp_out_R = fast_mul_q28(lp_a0, in_R) + fast_mul_q28(lp_b1, lp_state_R);
        lp_state_L = lp_out_L;
        lp_state_R = lp_out_R;

        // All-pass filter on crossfeed signals for ITD (Q28)
        int32_t ap_out_L = fast_mul_q28(ap_a, lp_out_L) + ap_state_L;
        ap_state_L = lp_out_L - fast_mul_q28(ap_a, ap_out_L);
        int32_t ap_out_R = fast_mul_q28(ap_a, lp_out_R) + ap_state_R;
        ap_state_R = lp_out_R - fast_mul_q28(ap_a, ap_out_R);

        // Complementary mixing with ITD
        left[i]  = (in_L - lp_out_L) + ap_out_R;
        right[i] = (in_R - lp_out_R) + ap_out_L;
    }

    // Store per-pair state back.
    st->lp_state_L = lp_state_L;
    st->lp_state_R = lp_state_R;
    st->ap_state_L = ap_state_L;
    st->ap_state_R = ap_state_R;
}
#endif

// Per-pair dispatch: run crossfeed on selected pairs, reset the state of
// skipped ones so a re-enabled pair starts clean.  A pair runs when coeffs
// are published, its mask bit is set, neither channel carries a RAW test
// signal, and BOTH channels are matrix-enabled.  Requiring both matters:
// a disabled output's buffer is zeroed at matrix time and must stay silent,
// but crossfeed writes both channels, so running on a half-enabled pair
// would leak the enabled channel's bleed into the disabled (never re-zeroed)
// buffer.  A half pair gets no crossfeed; disabled pairs also skip, saving
// the cycles.
DSP_TIME_CRITICAL
#if PICO_RP2350
void crossfeed_process_pairs(const CrossfeedCoeffs *coeffs, uint32_t pair_mask,
                             int first_pair, int last_pair,
                             float (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n) {
#else
void crossfeed_process_pairs(const CrossfeedCoeffs *coeffs, uint32_t pair_mask,
                             int first_pair, int last_pair,
                             int32_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n) {
#endif
    for (int p = first_pair; p <= last_pair; p++) {
        int l = 2 * p;
        bool run = coeffs && ((pair_mask >> p) & 1u)
                && !(siggen_raw_mask & (3u << l))
                && matrix_mixer.outputs[l].enabled
                && matrix_mixer.outputs[l + 1].enabled;
        if (run)
            crossfeed_process_pair_block(coeffs, &crossfeed_pair_state[p],
                                         buf_out[l], buf_out[l + 1], n);
        else
            crossfeed_reset_pair_state(&crossfeed_pair_state[p]);
    }
}
