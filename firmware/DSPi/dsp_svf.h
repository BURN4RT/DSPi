#ifndef DSP_SVF_H
#define DSP_SVF_H

#include "config.h"

#if PICO_RP2350
static inline void dsp_svf_first_order(Biquad * __restrict bq, float * __restrict samples, uint32_t count) {
    // Load SVF coefficients
    float a1 = bq->sva1, a2 = bq->sva2, a3 = bq->sva3;
    float m0 = bq->svm0, m1 = bq->svm1, m2 = bq->svm2;
    float ic1eq = bq->svic1eq, ic2eq = bq->svic2eq;
    float g = bq->g;
    float *sp = samples;
    float v0, v1;
    uint32_t blk_count = count >> 2; // unroll loops by 4
    // Per-type specialization: eliminates zero-multiplies in inner loop
    switch (bq->svf_type)
    {
        // One-pole TPT SVF: a1 = 1/(1+g), a2 = g/(1+g) (multiply-only).
        case FILTER_ALLPASS1:
            while(blk_count > 0) {

                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v1 + v1 - v0;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v1 + v1 - v0;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v1 + v1 - v0;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v1 + v1 - v0;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v1 + v1 - v0;
                blk_count--;
            }
        break;

        case FILTER_HIGHSHELF1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 + m2 * (v0 - v1);

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 + m2 * (v0 - v1);

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 + m2 * (v0 - v1);

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 + m2 * (v0 - v1);

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 + m2 * (v0 - v1);
                blk_count--;
            }
        break;

        case FILTER_LOWSHELF1:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 + m1 * v1;

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 + m1 * v1;

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 + m1 * v1;

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 + m1 * v1;

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 + m1 * v1;
                blk_count--;
            }
        break;

        default:
            while(blk_count > 0) {
                v0 = sp[0];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[0] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[1];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[1] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[2];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[2] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                v0 = sp[3];
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                sp[3] = v0 * m0 + m1 * v1 + m2 * (v0 - v1);

                sp += 4;
                blk_count--;
            }
            blk_count = count & 0x3;
            while(blk_count > 0) {
                float v0, v1;
                v0 = *sp;
                v1 = a2 * v0 + a1 * ic1eq;
                ic1eq = v1 + v1 - ic1eq;
                *sp++ = v0 * m0 + m1 * v1 + m2 * (v0 - v1);
                blk_count--;
            }
    }

    bq->svic1eq = ic1eq;
}

static inline void dsp_svf_second_order(Biquad * __restrict bq, float * __restrict samples, uint32_t count) {
    // Load SVF coefficients
    float a1 = bq->sva1, a2 = bq->sva2, a3 = bq->sva3;
    float m0 = bq->svm0, m1 = bq->svm1, m2 = bq->svm2;
    float ic1eq = bq->svic1eq, ic2eq = bq->svic2eq;
    float g = bq->g;
    float *sp = samples;
    float v0, v1, v2, v3;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    // Per-type specialization: eliminates zero-multiplies in inner loop
    switch (bq->svf_type) {
        case FILTER_LOWPASS:
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[0] = v2;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[1] = v2;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[2] = v2;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[3] = v2;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                *sp++ = v2;
                blk_count--;
            }
            break;
        case FILTER_HIGHPASS:
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[0] = v0 + m1 * v1 - v2;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[1] = v0 + m1 * v1 - v2;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[2] = v0 + m1 * v1 - v2;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[3] = v0 + m1 * v1 - v2;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                *sp++ = v0 + m1 * v1 - v2;
                blk_count--;
            }
            break;
        case FILTER_PEAKING:
        case FILTER_NOTCH:
        case FILTER_ALLPASS:
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[0] = v0 + m1 * v1;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[1] = v0 + m1 * v1;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[2] = v0 + m1 * v1;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[3] = v0 + m1 * v1;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                *sp++ = v0 + m1 * v1;
                blk_count--;
            }
            break;
        default: // FILTER_LOWSHELF, FILTER_HIGHSHELF, FILTER_LINKWITZ_TRANSFORM
            while(blk_count > 0) {
                v0 = sp[0];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[0] = v0 * m0 + m1 * v1 + m2 * v2;

                v0 = sp[1];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[1] = v0 * m0 + m1 * v1 + m2 * v2;

                v0 = sp[2];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[2] = v0 * m0 + m1 * v1 + m2 * v2;

                v0 = sp[3];
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                sp[3] = v0 * m0 + m1 * v1 + m2 * v2;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                v0 = *sp;
                v3 = v0 - ic2eq;
                v1 = a1 * ic1eq + a2 * v3;
                v2 = ic2eq + g * v1;
                ic1eq = v1 + v1 - ic1eq;
                ic2eq = v2 + v2 - ic2eq;
                *sp++ = v0 * m0 + m1 * v1 + m2 * v2;
                blk_count--;
            }
            break;
    }
    bq->svic1eq = ic1eq;
    bq->svic2eq = ic2eq;
}
#endif // PICO_RP2350

#endif // DSP_SVF_H