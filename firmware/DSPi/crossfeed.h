#ifndef CROSSFEED_H
#define CROSSFEED_H

#include <string.h>
#include "config.h"

// BS2B Crossfeed Presets
#define CROSSFEED_PRESET_DEFAULT    0   // 700 Hz / 4.5 dB - Balanced, most popular
#define CROSSFEED_PRESET_CHUMOY     1   // 700 Hz / 6.0 dB - Stronger spatial effect
#define CROSSFEED_PRESET_MEIER      2   // 650 Hz / 9.5 dB - Subtle, natural
#define CROSSFEED_PRESET_CUSTOM     3   // User-defined

// Custom parameter limits
#define CROSSFEED_FREQ_MIN      500.0f
#define CROSSFEED_FREQ_MAX      2000.0f
#define CROSSFEED_FEED_MIN      0.0f
#define CROSSFEED_FEED_MAX      15.0f

// Interaural Time Delay for standard 60-degree stereo speaker placement
// Computed from head model: head_width=0.15m, distance=1.0m, speed=340m/s
// d_far  = sqrt(1 + 0.005625 + 0.075) = 1.0395m
// d_near = sqrt(1 + 0.005625 - 0.075) = 0.9647m
// ITD = (d_far - d_near) / 340 = 220us
#define CROSSFEED_ITD_SEC       0.000220f

// Configuration (persisted to flash)
typedef struct {
    bool enabled;
    bool itd_enabled;       // Interaural time delay on/off
    uint8_t preset;         // 0-3 (CROSSFEED_PRESET_*)
    float custom_fc;        // Custom cutoff frequency (500-2000 Hz)
    float custom_feed_db;   // Custom feed level (0-15 dB)
    uint8_t output_pair_mask; // bit p runs crossfeed on output pair p (outputs 2p/2p+1); mono PDM sub is not a pair and is excluded
} CrossfeedConfig;

// Filter coefficients and per-pair state (runtime only, not persisted)
//
// Signal flow per sample:
//   lp_out   = lowpass(input)             // crossfeed component (ILD)
//   ap_out   = allpass(lp_out)            // add ITD to crossfeed path
//   direct   = input - lp_out            // complementary direct path
//   output   = direct + ap_opposite      // mix
//
// The complementary subtraction guarantees mono unity at DC.
// The all-pass on the crossfeed path adds interaural time delay (~220us)
// to simulate sound traveling around the head.
//
// Coefficients are shared: one user config drives every selected pair.
// Filter state is independent per output pair so pairs run in isolation.
#if PICO_RP2350
typedef struct {
    float lp_a0, lp_b1;                // Lowpass coefficients
    float ap_a;                         // All-pass coefficient (ITD)
} CrossfeedCoeffs;
typedef struct {
    float lp_state_L, lp_state_R;      // Lowpass filter state
    float ap_state_L, ap_state_R;      // All-pass filter state
} CrossfeedPairState;
#else
typedef struct {
    int32_t lp_a0, lp_b1;
    int32_t ap_a;
} CrossfeedCoeffs;
typedef struct {
    int32_t lp_state_L, lp_state_R;
    int32_t ap_state_L, ap_state_R;
} CrossfeedPairState;
#endif

// API Functions
static inline void crossfeed_reset_pair_state(CrossfeedPairState *st) {
    memset(st, 0, sizeof(CrossfeedPairState));
}
void crossfeed_compute_coefficients(CrossfeedCoeffs *coeffs, const CrossfeedConfig *config, float sample_rate);

// Recompute shared coefficients from config and publish current_crossfeed_coeffs.
// Called from the main loop while audio runs; never touches per-pair state.
void crossfeed_apply_config(const CrossfeedConfig *config, float sample_rate);

// Time-critical block processing for one stereo pair - modifies left/right in place
#if PICO_RP2350
void crossfeed_process_pair_block(const CrossfeedCoeffs *coeffs, CrossfeedPairState *st, float *left, float *right, uint32_t n);
#else
void crossfeed_process_pair_block(const CrossfeedCoeffs *coeffs, CrossfeedPairState *st, int32_t *left, int32_t *right, uint32_t n);
#endif

// Runs crossfeed on output pairs first_pair..last_pair of buf_out: selected
// pairs are processed in place, skipped pairs get their state reset.  Called
// once per packet by whichever core owns those pairs, with a per-packet
// snapshot of coeffs/mask so both cores apply the same view.
#if PICO_RP2350
void crossfeed_process_pairs(const CrossfeedCoeffs *coeffs, uint32_t pair_mask,
                             int first_pair, int last_pair,
                             float (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n);
#else
void crossfeed_process_pairs(const CrossfeedCoeffs *coeffs, uint32_t pair_mask,
                             int first_pair, int last_pair,
                             int32_t (*buf_out)[AUDIO_BUFFER_SAMPLES], uint32_t n);
#endif

// Per-output-pair filter state, one entry per stereo output pair.
extern CrossfeedPairState crossfeed_pair_state[NUM_SPDIF_INSTANCES];

// Published coefficient set the pipeline snapshots each packet.
// NULL means crossfeed is disabled (replaces the old crossfeed_bypassed flag).
extern volatile const CrossfeedCoeffs *current_crossfeed_coeffs;

#endif // CROSSFEED_H
