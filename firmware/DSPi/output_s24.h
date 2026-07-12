#pragma once

// ----------------------------------------------------------------------------
// In-place float -> S24 finalization of output blocks (RP2350).
//
// Once per block, after the last float consumer (EQ/gain, loudness, delay,
// peak metering), each output channel row of buf_out is converted in place to
// a clamped 24-bit sample held in an int32.  The S/PDIF slot interleave and
// the ADAT encoder both read this integer form, so the clamp/scale runs
// exactly once per sample per channel instead of once per consumer.
//
// The rows are declared float; every integer access to them must go through
// out_s24_t (may_alias) so GCC preserves cross-type access ordering.
// ----------------------------------------------------------------------------

#include <stdint.h>
#include <math.h>

typedef int32_t out_s24_t __attribute__((may_alias));

static inline void output_block_to_s24_inplace(float *buf, uint32_t n) {
    out_s24_t *dst = (out_s24_t *)buf;
    for (uint32_t i = 0; i < n; i++) {
        float x = fmaxf(-1.0f, fminf(1.0f, buf[i]));
        dst[i] = (int32_t)(x * 8388607.0f);
    }
}
