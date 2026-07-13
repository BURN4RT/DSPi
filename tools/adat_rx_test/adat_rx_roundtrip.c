/*
 * adat_rx_roundtrip.c; host-side ADAT encode/decode round-trip test
 *
 * Uses the firmware's ADAT TX encoder (copied verbatim from adat_output.c)
 * as a bit-exact reference transmitter, models the PIO NRZI receiver as a
 * level XOR (decoded bit n = level n ^ level n-1, which is exactly what the
 * adat_rx PIO program emits), and runs the firmware's frame sync + decode
 * logic (copied from adat_input.c) over the decoded stream at every bit
 * offset 0..31. Any mismatch between decoded and transmitted samples fails.
 *
 * Build and run:  cc -O2 -o adat_rx_roundtrip adat_rx_roundtrip.c && ./adat_rx_roundtrip
 *
 * The encoder and decoder cores here are copies; if adat_output.c's
 * adat_encode_frame or adat_input.c's adat_rx_decode_frame changes, refresh
 * them before trusting a run.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>

#define ADAT_SYNC_HEADER 0x8010u

// ---------------------------------------------------------------------------
// Reference transmitter (adat_output.c)
// ---------------------------------------------------------------------------

static uint16_t adat_token_lut[256];
static uint16_t adat_hdr_nrzi[2];
static uint32_t adat_hdr_exit[2];

static void tx_init(void) {
    for (uint32_t b = 0; b < 256; b++) {
        uint32_t stuffed = 0x210u | ((b & 0xF0u) << 1) | (b & 0x0Fu);
        uint32_t t = 0, l = 0;
        for (int k = 9; k >= 0; k--) {
            l ^= (stuffed >> k) & 1u;
            t = (t << 1) | l;
        }
        adat_token_lut[b] = (uint16_t)t;
    }
    for (uint32_t lvl = 0; lvl < 2; lvl++) {
        uint32_t t = 0, l = lvl;
        for (int k = 15; k >= 0; k--) {
            l ^= (ADAT_SYNC_HEADER >> k) & 1u;
            t = (t << 1) | l;
        }
        adat_hdr_nrzi[lvl] = (uint16_t)t;
        adat_hdr_exit[lvl] = l;
    }
}

static uint32_t adat_encode_frame(uint32_t *w, const int32_t s24[8],
                                  uint32_t entry) {
    uint32_t hdr = adat_hdr_nrzi[entry];
    uint32_t m = (0u - adat_hdr_exit[entry]) & 0x3FFu;
    uint32_t c[8];
    for (int ch = 0; ch < 8; ch++) {
        uint32_t s = (uint32_t)s24[ch];
        uint32_t t2 = adat_token_lut[(s >> 16) & 0xFFu] ^ m;
        m = (0u - (t2 & 1u)) & 0x3FFu;
        uint32_t t1 = adat_token_lut[(s >> 8) & 0xFFu] ^ m;
        m = (0u - (t1 & 1u)) & 0x3FFu;
        uint32_t t0 = adat_token_lut[s & 0xFFu] ^ m;
        m = (0u - (t0 & 1u)) & 0x3FFu;
        c[ch] = (t2 << 20) | (t1 << 10) | t0;
    }
    w[0] = (hdr << 16)  | (c[0] >> 14);
    w[1] = (c[0] << 18) | (c[1] >> 12);
    w[2] = (c[1] << 20) | (c[2] >> 10);
    w[3] = (c[2] << 22) | (c[3] >> 8);
    w[4] = (c[3] << 24) | (c[4] >> 6);
    w[5] = (c[4] << 26) | (c[5] >> 4);
    w[6] = (c[5] << 28) | (c[6] >> 2);
    w[7] = (c[6] << 30) |  c[7];
    return m & 1u;
}

// ---------------------------------------------------------------------------
// Receiver core (adat_input.c), ring replaced by a flat word array
// ---------------------------------------------------------------------------

static const uint32_t *rx_words;
static uint32_t rx_nwords;

static bool rx_header_ok(uint32_t rd, uint32_t k) {
    uint32_t a = rx_words[rd];
    uint32_t b = rx_words[rd + 1];
    uint64_t v = ((uint64_t)a << 32) | b;
    return ((uint32_t)(v >> (52u - k)) & 0xFFFu) == 0x801u;
}

static bool rx_decode_frame(uint32_t rd, uint32_t k, int32_t *smp) {
    uint32_t w[8];
    if (k == 0) {
        for (uint32_t i = 0; i < 8u; i++) w[i] = rx_words[rd + i];
    } else {
        uint32_t prev = rx_words[rd];
        for (uint32_t i = 0; i < 8u; i++) {
            uint32_t next = rx_words[rd + i + 1u];
            w[i] = (prev << k) | (next >> (32u - k));
            prev = next;
        }
    }
    if ((w[0] & 0xFFF00000u) != 0x80100000u) return false;
    for (uint32_t ch = 0; ch < 8u; ch++) {
        uint32_t p = 16u + 30u * ch;
        uint32_t a = w[p >> 5];
        uint32_t b = ((p >> 5) < 7u) ? w[(p >> 5) + 1u] : 0u;
        uint32_t sh = p & 31u;
        uint32_t v = (uint32_t)(((((uint64_t)a << 32) | b) >> (34u - sh))) & 0x3FFFFFFFu;
        uint32_t s24 = (((v >> 25) & 0xFu) << 20) | (((v >> 20) & 0xFu) << 16) |
                       (((v >> 15) & 0xFu) << 12) | (((v >> 10) & 0xFu) << 8)  |
                       (((v >> 5)  & 0xFu) << 4)  |   (v         & 0xFu);
        smp[ch] = (int32_t)(s24 << 8);
    }
    return true;
}

// Sync scan (adat_rx_scan core): find [1][10x0][1] and return word/bit.
static bool rx_scan(uint32_t *out_word, uint32_t *out_bit) {
    for (uint32_t i = 0; i + 1 < rx_nwords; i++) {
        uint32_t prev = rx_words[i], curr = rx_words[i + 1];
        if (!(prev | curr)) continue;
        uint64_t b64 = ((uint64_t)prev << 32) | curr;
        for (uint32_t b = 0; b < 32u; b++) {
            if (((uint32_t)(b64 >> (52u - b)) & 0xFFFu) == 0x801u) {
                *out_word = i;
                *out_bit = b;
                return true;
            }
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Channel model: pre-NRZI'd line levels -> what the PIO pushes
// ---------------------------------------------------------------------------

// The TX words are line LEVELS (MSB first). The adat_rx PIO emits 1 when the
// level changed from the previous bit cell, 0 otherwise; chained across the
// whole stream. That is exactly level[n] ^ level[n-1].

typedef struct {
    uint32_t *words;
    uint32_t nbits;
    uint32_t prev_level;
} BitSink;

static void sink_push(BitSink *s, uint32_t level) {
    uint32_t decoded = level ^ s->prev_level;
    s->prev_level = level;
    if (decoded) s->words[s->nbits >> 5] |= 0x80000000u >> (s->nbits & 31u);
    s->nbits++;
}

// ---------------------------------------------------------------------------

#define N_FRAMES 64

int main(void) {
    tx_init();
    srand(12345);

    int failures = 0;

    for (uint32_t pad = 0; pad < 32; pad++) {
        // Reference samples (random s24, sign-extended into int32 low 24)
        static int32_t ref[N_FRAMES][8];
        for (int f = 0; f < N_FRAMES; f++)
            for (int ch = 0; ch < 8; ch++) {
                uint32_t r = ((uint32_t)rand() << 16) ^ (uint32_t)rand();
                ref[f][ch] = (int32_t)(r & 0xFFFFFFu);
            }
        // Force known corner values into frame 0
        ref[0][0] = 0x000000; ref[0][1] = 0x7FFFFF; ref[0][2] = 0x800000;
        ref[0][3] = 0xFFFFFF; ref[0][4] = 0x555555; ref[0][5] = 0xAAAAAA;

        // Encode with a chained line level, preceded by `pad` idle (constant
        // level) bits so the frame boundary lands at every bit offset.
        static uint32_t words[(32 + N_FRAMES * 256 + 63) / 32 + 2];
        memset(words, 0, sizeof(words));
        BitSink sink = { words, 0, 0 };

        for (uint32_t i = 0; i < pad; i++) sink_push(&sink, 0);   // dark line

        uint32_t entry = 0;
        for (int f = 0; f < N_FRAMES; f++) {
            uint32_t w[8];
            entry = adat_encode_frame(w, ref[f], entry);
            for (int i = 0; i < 8; i++)
                for (int b = 31; b >= 0; b--)
                    sink_push(&sink, (w[i] >> b) & 1u);
        }

        rx_words = words;
        rx_nwords = (sink.nbits + 31) / 32;

        // Sync
        uint32_t rd = 0, k = 0;
        if (!rx_scan(&rd, &k)) {
            printf("pad %2u: SYNC NOT FOUND\n", pad);
            failures++;
            continue;
        }
        uint32_t expect_bit = pad;   // frame 0 starts right after the pad
        if (rd * 32 + k != expect_bit) {
            printf("pad %2u: sync at bit %u, expected %u\n", pad, rd * 32 + k, expect_bit);
            failures++;
            continue;
        }

        // Decode every frame that fits (leave the spill word for k > 0)
        int bad = 0;
        for (int f = 0; f < N_FRAMES; f++) {
            if (rd + 9 > rx_nwords) break;   // conservative spill guard
            int32_t got[8];
            if (!rx_header_ok(rd, k)) {      // SYNCING verify path
                printf("pad %2u frame %d: header_ok false\n", pad, f);
                bad++;
                break;
            }
            if (!rx_decode_frame(rd, k, got)) {
                printf("pad %2u frame %d: header mismatch\n", pad, f);
                bad++;
                break;
            }
            for (int ch = 0; ch < 8; ch++) {
                int32_t want = (int32_t)((uint32_t)ref[f][ch] << 8);
                if (got[ch] != want) {
                    printf("pad %2u frame %d ch %d: got %08x want %08x\n",
                           pad, f, ch, (unsigned)got[ch], (unsigned)want);
                    bad++;
                }
            }
            rd += 8;
        }
        if (bad) failures++;
    }

    if (failures) {
        printf("FAIL: %d offset(s) failed\n", failures);
        return 1;
    }
    printf("PASS: all 32 bit offsets, %d frames each, samples bit-exact\n", N_FRAMES);
    return 0;
}
