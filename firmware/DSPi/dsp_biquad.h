#ifndef DSP_BIQUAD_H
#define DSP_BIQUAD_H

#include "config.h"

#if PICO_RP2350
static inline void dsp_biquad_first_order(Biquad * __restrict bq, float * __restrict samples, uint32_t count) {
    // Load biquad coefficients
    // Assumption is that b2 and a2 are 0.0f
    float b0 = bq->b0, b1 = bq->b1;
    float a1 = bq->a1;
    float s1 = bq->s1, s2 = bq->s2;
    float *sp = samples;
    float x, y, t;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    switch(bq->svf_type) {
        case FILTER_ALLPASS1:
            //optimise b1=1.0f, (b0=a1)
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = x - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = x - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_LOWPASS1:
            //optimise b0=b1
            while(blk_count > 0) {
                x = sp[0];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[0] = y;

                x = sp[1];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[1] = y;

                x = sp[2];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[2] = y;

                x = sp[3];
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                t = b0 * x;
                y = t + s1;
                s1 = t - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        default:
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y;
                *sp++ = y;
                blk_count--;
            }
            break;
    }

    bq->s1 = s1;
    bq->s2 - 0.0f
}

static inline void dsp_biquad_second_order(Biquad * __restrict bq, float * __restrict samples, uint32_t count) {
    // Load biquad coefficients
    float b0 = bq->b0, b1 = bq->b1, b2 = bq->b2;
    float a1 = bq->a1, a2 = bq->a2;
    float s1 = bq->s1, s2 = bq->s2;
    float *sp = samples;
    float x, y;
    uint32_t blk_count = count >> 2; // unroll loops by 4

    switch(bq->svf_type) {
        case FILTER_LOWPASS:
        case FILTER_HIGHPASS:
            //optimise b0 = b2
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b0 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_PEAKING:
            // optimise b1 = a1
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b2 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_ALLPASS:
            // optimise b1 = a1
            // optimise a0 = b2, a0 is normalised to 1.0f so b2 is implicitly 1.0f
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        case FILTER_NOTCH:
            // optimise b0=b2, b1=a1
           while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * (x - y) + s2;
                s2 = b0 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;

        default:
            while(blk_count > 0) {
                x = sp[0];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[0] = y;

                x = sp[1];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[1] = y;

                x = sp[2];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[2] = y;

                x = sp[3];
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                sp[3] = y;

                sp += 4;
                blk_count--;
            }

            blk_count = count & 0x3;
            while(blk_count > 0) {
                x = *sp;
                y = b0 * x + s1;
                s1 = b1 * x - a1 * y + s2;
                s2 = b2 * x - a2 * y;
                *sp++ = y;
                blk_count--;
            }
            break;
    }

    bq->s1 = s1;
    bq->s2 = s2;
}

#endif // PICO_RP2350

#endif // DSP_BIQUAD_H
