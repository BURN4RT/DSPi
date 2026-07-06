// ----------------------------------------------------------------------------
// ADAT lightpipe bulk output (RP2350 only)
//
// One 256-bit ADAT frame per sample carries all 8 post-gain output channels:
//   [1][10x0][1][u3..u0]  then 8 x ( 6 x [1][nibble] ), nibbles MSB-first,
// NRZI on the wire (1 = level transition).  The PIO program transmits at
// 2 cycles/bit, so PIO clock = 512*Fs and the clkdiv is exactly half
// the S/PDIF TX divider at 44.1/48 kHz; the effective sample rate matches
// the output slots bit-for-bit, so ADAT can never drift against them.
//
// Buffering: a single 896-frame ring (28 KB BSS) drained by a free-running
// DMA data channel; a chained control channel rewrites the read address at
// the ring end (no IRQ, no power-of-2 size).  Frames are pushed after the
// slot gives, so the ring lead equals the slot-0 consumer fill plus a fixed
// ADAT_ALIGN_LEAD_FRAMES cushion.  The blocking give caps the fill at
// 16 x 48 samples, which bounds the lead below the ring size; overwrite is
// structurally impossible while the DMA runs.
//
// Underruns: silence frames are inserted slaved 1:1 to slot 0's DMA
// starvation counter, so ADAT shifts by exactly the amount the slots shift
// and the constant ADAT-to-slot offset survives host underruns.  While the
// host stream is stopped (starvation counting disabled) the cushion is
// simply kept topped up with silence.
// ----------------------------------------------------------------------------

#include "adat_output.h"

#if PICO_RP2350

#include <string.h>
#include <math.h>
#include "config.h"
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "usb_audio.h"
#include "notify.h"

#define ADAT_PIO                pio1   // PDM owns SM 0 on this block
#define ADAT_SM                 1
#define ADAT_DMA_DATA_CH        13     // 13/14 are unused in every config:
#define ADAT_DMA_CTRL_CH        14     // outputs 0-3, PDM 4, input 5-12
#define ADAT_FRAME_WORDS        8      // 256 bits
#define ADAT_RING_FRAMES        896    // 28 KB; > cushion + blocking-give cap (96 + 768)
#define ADAT_ALIGN_LEAD_FRAMES  96     // constant ADAT-vs-slot lead; underrun cushion
#define ADAT_SYNC_HEADER        0x8010u // [1][10x0][1][0000]; user bits always 0

// NRZI TX, 2 cycles/bit: stay in the low/high bank while bits are 0, cross
// banks on 1 (side-set drives the pin).  Same structure as the S/PDIF TX
// program; pio_add_program relocates the JMP targets.
//   0: out x,1  side 0      1: jmp !x,0  side 0
//   2: out x,1  side 1      3: jmp !x,2  side 1
static const uint16_t adat_pio_instr[] = { 0x6021, 0x0020, 0x7021, 0x1022 };
static const struct pio_program adat_pio_program = {
    .instructions = adat_pio_instr, .length = 4, .origin = -1,
};

static uint32_t adat_ring[ADAT_RING_FRAMES * ADAT_FRAME_WORDS];
// Read by the control DMA channel to re-point the data channel at wrap.
static uint32_t adat_ring_base_holder;

static uint32_t adat_stuffed_silence[ADAT_FRAME_WORDS];

static uint32_t adat_wr_frame;
static bool     adat_running;
static bool     adat_hw_init_done;
static uint8_t  adat_hw_pin = 0xFF;       // GPIO currently routed to the PIO
static uint     adat_pio_offset;
static uint32_t adat_cur_freq = 48000;
static bool     adat_rate_ok = true;
static bool     adat_stream_active_mode;  // starvation counting is live
static uint32_t adat_slot0_starv_seen;
static bool     adat_need_local_resync;

static volatile uint8_t adat_cfg_enabled;
static volatile uint8_t adat_cfg_pin = PICO_ADAT_PIN;
volatile bool adat_output_config_dirty;

static uint16_t adat_resync_count;
static uint16_t adat_slip_count;
static uint32_t adat_last_notified_state = 0xFFFFFFFF;

extern uint8_t output_types[];  // usb_audio.c; slot 0 type selects starvation source

// ---------------------------------------------------------------------------
// Frame construction
// ---------------------------------------------------------------------------

// Expand a 24-bit sample into its 30-bit channel chunk: a '1' before each of
// the six nibbles, MSB nibble first.  0x21084210 = the six '1' bits.
static inline uint32_t adat_stuff30(int32_t s24) {
    uint32_t s = (uint32_t)s24 & 0xFFFFFFu;
    return 0x21084210u
         | ((s & 0xF00000u) << 5) | ((s & 0x0F0000u) << 4) | ((s & 0x00F000u) << 3)
         | ((s & 0x000F00u) << 2) | ((s & 0x0000F0u) << 1) |  (s & 0x00000Fu);
}

// Pack header + eight 30-bit chunks into 8 words, transmit order = w[0] MSB
// first (the SM shifts left).  Bit budget: 16 + 8*30 = 256.
static inline void adat_pack_frame(uint32_t *w, const uint32_t c[8]) {
    w[0] = (ADAT_SYNC_HEADER << 16) | (c[0] >> 14);
    w[1] = (c[0] << 18) | (c[1] >> 12);
    w[2] = (c[1] << 20) | (c[2] >> 10);
    w[3] = (c[2] << 22) | (c[3] >> 8);
    w[4] = (c[3] << 24) | (c[4] >> 6);
    w[5] = (c[4] << 26) | (c[5] >> 4);
    w[6] = (c[5] << 28) | (c[6] >> 2);
    w[7] = (c[6] << 30) |  c[7];
}

// ---------------------------------------------------------------------------
// Ring bookkeeping
// ---------------------------------------------------------------------------

static inline uint32_t adat_rd_frame(void) {
    uint32_t off = (uint32_t)(dma_hw->ch[ADAT_DMA_DATA_CH].read_addr
                              - (uintptr_t)adat_ring);
    // At the wrap instant read_addr can equal ring end for one control-channel
    // transfer; the modulo folds it to 0.
    return (off >> 5) % ADAT_RING_FRAMES;   // 32 bytes per frame
}

static inline uint32_t adat_lead_frames(void) {
    return (adat_wr_frame + ADAT_RING_FRAMES - adat_rd_frame()) % ADAT_RING_FRAMES;
}

static void adat_write_silence(uint32_t frames) {
    uint32_t wf = adat_wr_frame;
    for (uint32_t i = 0; i < frames; i++) {
        memcpy(&adat_ring[wf * ADAT_FRAME_WORDS], adat_stuffed_silence,
               ADAT_FRAME_WORDS * sizeof(uint32_t));
        if (++wf == ADAT_RING_FRAMES) wf = 0;
    }
    adat_wr_frame = wf;
}

// ---------------------------------------------------------------------------
// Hardware bring-up / teardown
// ---------------------------------------------------------------------------

static void adat_update_divider(void) {
    // 16.8 clkdiv for PIO clock = 512*Fs, i.e. ceil(sys / (2*Fs)) in 1/256
    // steps; exactly half the S/PDIF TX divider at 44.1/48 kHz (6966 and 6400
    // are even), so both interfaces consume samples at the identical rate.
    uint32_t fs = adat_cur_freq;
    uint32_t div248 = (clock_get_hz(clk_sys) + 2 * fs - 1) / (2 * fs);
    pio_sm_set_clkdiv_int_frac(ADAT_PIO, ADAT_SM,
                               (uint16_t)(div248 >> 8), (uint8_t)(div248 & 0xFF));
}

static void adat_hw_init_once(void) {
    if (adat_hw_init_done) return;
    pio_sm_claim(ADAT_PIO, ADAT_SM);
    adat_pio_offset = pio_add_program(ADAT_PIO, &adat_pio_program);
    dma_channel_claim(ADAT_DMA_DATA_CH);
    dma_channel_claim(ADAT_DMA_CTRL_CH);
    adat_ring_base_holder = (uint32_t)(uintptr_t)adat_ring;
    adat_hw_init_done = true;
}

static void adat_sm_configure(void) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, adat_pio_offset, adat_pio_offset + 3);
    sm_config_set_sideset(&c, 1, false, false);
    sm_config_set_sideset_pins(&c, adat_hw_pin);
    sm_config_set_set_pins(&c, adat_hw_pin, 1);      // lets stop force the pin low
    sm_config_set_out_shift(&c, false, true, 32);    // shift left: MSB first
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    pio_sm_init(ADAT_PIO, ADAT_SM, adat_pio_offset, &c);
    adat_update_divider();                            // pio_sm_init resets the divider
}

static void adat_stop_hw(void) {
    if (!adat_hw_init_done) return;
    // Break the retrigger chain first: a data completion racing the aborts
    // would otherwise fire the control channel and re-arm the data channel.
    dma_channel_config dc = dma_get_channel_config(ADAT_DMA_DATA_CH);
    channel_config_set_chain_to(&dc, ADAT_DMA_DATA_CH);
    dma_channel_set_config(ADAT_DMA_DATA_CH, &dc, false);
    dma_channel_abort(ADAT_DMA_CTRL_CH);
    dma_channel_abort(ADAT_DMA_DATA_CH);
    pio_sm_set_enabled(ADAT_PIO, ADAT_SM, false);
    pio_sm_clear_fifos(ADAT_PIO, ADAT_SM);
    // Force only OUR pin low via the SM's SET mapping; pio_sm_set_pins()
    // would transiently drive every GPIO muxed to this PIO block (PDM).
    if (adat_hw_pin != 0xFF)
        pio_sm_exec(ADAT_PIO, ADAT_SM, pio_encode_set(pio_pins, 0));
    adat_running = false;
}

static void adat_release_pin(void) {
    if (adat_hw_pin == 0xFF) return;
    // Only release if still ours: another peripheral may have re-claimed the
    // pin between a deferred disable and this service (its mux is not PIO1).
    if (gpio_get_function(adat_hw_pin) == GPIO_FUNC_PIO1) {
        gpio_set_function(adat_hw_pin, GPIO_FUNC_NULL);
        gpio_set_dir(adat_hw_pin, GPIO_IN);
    }
    adat_hw_pin = 0xFF;
}

static void adat_arm_dma(void) {
    // Control channel: on data completion, rewrite the data channel's read
    // address (trigger alias) back to the ring base.  TRANS_COUNT reloads
    // from its shadow on every trigger, so this runs forever without an IRQ.
    dma_channel_config cc = dma_channel_get_default_config(ADAT_DMA_CTRL_CH);
    channel_config_set_transfer_data_size(&cc, DMA_SIZE_32);
    channel_config_set_read_increment(&cc, false);
    channel_config_set_write_increment(&cc, false);
    dma_channel_configure(ADAT_DMA_CTRL_CH, &cc,
                          &dma_hw->ch[ADAT_DMA_DATA_CH].al3_read_addr_trig,
                          &adat_ring_base_holder, 1, false);

    dma_channel_config dc = dma_channel_get_default_config(ADAT_DMA_DATA_CH);
    channel_config_set_transfer_data_size(&dc, DMA_SIZE_32);
    channel_config_set_read_increment(&dc, true);
    channel_config_set_write_increment(&dc, false);
    channel_config_set_dreq(&dc, pio_get_dreq(ADAT_PIO, ADAT_SM, true));
    channel_config_set_chain_to(&dc, ADAT_DMA_CTRL_CH);
    channel_config_set_high_priority(&dc, true);
    dma_channel_configure(ADAT_DMA_DATA_CH, &dc, &ADAT_PIO->txf[ADAT_SM],
                          adat_ring, ADAT_RING_FRAMES * ADAT_FRAME_WORDS, true);
}

static inline uint32_t adat_slot0_starvations(void) {
    return (output_types[0] == OUTPUT_TYPE_I2S)
        ? audio_i2s_get_dma_starvations_instance(0)
        : audio_spdif_get_dma_starvations_instance(0);
}

static void adat_notify_if_changed(void) {
    // Whole tuple, so enable/pin changes made while rate-suspended (active
    // unchanged) still produce an event.
    uint32_t state = ((uint32_t)adat_cfg_pin << 16)
                   | ((uint32_t)adat_cfg_enabled << 8)
                   | (adat_running ? 1u : 0u);
    if (state == adat_last_notified_state) return;
    adat_last_notified_state = state;
    notify_push_adat_state(adat_cfg_enabled, adat_running ? 1 : 0, adat_cfg_pin);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void adat_output_init(void) {
    uint32_t c[8];
    for (int i = 0; i < 8; i++) c[i] = adat_stuff30(0);
    adat_pack_frame(adat_stuffed_silence, c);
}

void adat_output_set_config(bool enabled, uint8_t pin) {
    if (adat_cfg_enabled == (uint8_t)enabled && adat_cfg_pin == pin) return;
    adat_cfg_enabled = (uint8_t)enabled;
    adat_cfg_pin = pin;
    adat_output_config_dirty = true;
}

bool    adat_output_config_enabled(void) { return adat_cfg_enabled != 0; }
uint8_t adat_output_pin(void)            { return adat_cfg_pin; }
bool    adat_output_is_active(void)      { return adat_running; }

void adat_output_get_status(AdatStatus *out) {
    out->enabled      = adat_cfg_enabled;
    out->active       = adat_running ? 1 : 0;
    out->pin          = adat_cfg_pin;
    out->rate_ok      = adat_rate_ok ? 1 : 0;
    out->resync_count = adat_resync_count;
    out->slip_count   = adat_slip_count;
}

void adat_output_on_rate_change(uint32_t freq) {
    adat_cur_freq = freq;
    adat_rate_ok = (freq <= 48000);
    // Caller (perform_rate_change) holds the mute; the following
    // complete_pipeline_reset() restarts the stream via resync if valid.
    if (!adat_rate_ok && adat_running) adat_stop_hw();
}

void adat_output_stream_stop(void) {
    adat_stop_hw();
}

void adat_output_resync(void) {
    adat_stop_hw();
    adat_output_config_dirty = false;
    adat_need_local_resync = false;

    // Re-derive from the live rate: a boot preset can put the system at
    // 44.1 kHz without perform_rate_change() ever running.
    adat_cur_freq = audio_state.freq;
    adat_rate_ok = (adat_cur_freq <= 48000);

    if (!adat_cfg_enabled || !adat_rate_ok) {
        adat_release_pin();
        adat_notify_if_changed();
        return;
    }

    adat_hw_init_once();
    if (adat_hw_pin != adat_cfg_pin) {
        adat_release_pin();
        adat_hw_pin = adat_cfg_pin;
        pio_gpio_init(ADAT_PIO, adat_hw_pin);
        pio_sm_set_consecutive_pindirs(ADAT_PIO, ADAT_SM, adat_hw_pin, 1, true);
    }
    adat_sm_configure();

    // Alignment cushion: the constant ADAT-vs-slot lead, re-established at
    // every synchronized output restart.
    adat_wr_frame = 0;
    adat_write_silence(ADAT_ALIGN_LEAD_FRAMES);

    adat_slot0_starv_seen = adat_slot0_starvations();
    adat_arm_dma();
    pio_sm_set_enabled(ADAT_PIO, ADAT_SM, true);
    adat_running = true;
    adat_resync_count++;
    adat_notify_if_changed();
}

void adat_output_set_stream_active(bool active) {
    if (active && !adat_stream_active_mode)
        adat_slot0_starv_seen = adat_slot0_starvations();  // re-baseline
    adat_stream_active_mode = active;
}

void adat_output_task(void) {
    if (adat_need_local_resync) {
        adat_slip_count++;
        adat_output_resync();
        return;
    }
    if (!adat_running) return;

    if (adat_stream_active_mode) {
        // Slaved insertion: mirror slot 0's silence fallbacks 1:1 so the
        // ADAT-to-slot offset survives underruns exactly.
        uint32_t starv = adat_slot0_starvations();
        uint32_t pending = starv - adat_slot0_starv_seen;
        if (pending > 256) {
            // Impossible as a real backlog: the counter was reset (stream
            // (re)start, stats reset).  Re-baseline without inserting.
            adat_slot0_starv_seen = starv;
        } else if (pending) {
            if (pending > 8) pending = 8;   // bound per pass; catch up next call
            adat_write_silence(pending * PICO_AUDIO_SPDIF_DMA_SAMPLE_COUNT);
            adat_slot0_starv_seen += pending;
        }
    } else {
        // Host stream stopped: no gives, no starvation counting.  Keep the
        // cushion topped up; alignment re-canonicalizes at the next resync.
        uint32_t lead = adat_lead_frames();
        if (lead < ADAT_ALIGN_LEAD_FRAMES)
            adat_write_silence(ADAT_ALIGN_LEAD_FRAMES - lead);
    }

    // The DMA passing the write pointer means the cushion is gone (stalled
    // main loop or dead DMA); restart cleanly instead of playing the ring.
    if (adat_lead_frames() == 0) adat_need_local_resync = true;
}

DSP_TIME_CRITICAL
void adat_output_push_block(const float (*bufs)[192], uint32_t sample_count) {
    if (!adat_running) return;

    // Structurally unreachable while the blocking give caps the slot fill;
    // reaching it means the DMA stalled.  Skip and restart rather than
    // overwrite unplayed frames.
    if (adat_lead_frames() + sample_count >= ADAT_RING_FRAMES) {
        adat_need_local_resync = true;
        return;
    }

    uint32_t wf = adat_wr_frame;
    for (uint32_t i = 0; i < sample_count; i++) {
        uint32_t c[8];
        for (int ch = 0; ch < 8; ch++) {
            float x = bufs[ch][i];
            x = fmaxf(-1.0f, fminf(1.0f, x));
            c[ch] = adat_stuff30((int32_t)(x * 8388607.0f));
        }
        adat_pack_frame(&adat_ring[wf * ADAT_FRAME_WORDS], c);
        if (++wf == ADAT_RING_FRAMES) wf = 0;
    }
    adat_wr_frame = wf;
}

#endif // PICO_RP2350
