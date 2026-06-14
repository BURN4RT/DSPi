/*
 * i2s_input.c - I2S receiver integration for DSPi
 *
 * Structure mirrors spdif_input.c with everything clock-related removed:
 * the input is synchronous to the device's own clock domain, so there is
 * no servo, no rate detection and no lock handling.
 *
 * Hardware:
 *   - PIO: same block/SM as SPDIF RX (PICO_SPDIF_RX_PIO; SM 2 on RP2040,
 *     SM 0 on RP2350), claimed only while running
 *   - DMA: the two SPDIF RX channels in an IRQ-less infinite ring:
 *     channel A moves PIO RX FIFO words into a power-of-2-aligned ring
 *     (write-address wrap) and chains to channel B, which rewrites A's
 *     write address with the ring base and retriggers it. Zero IRQs, so
 *     capture survives IRQ-disabled windows.
 *
 * L/R framing: the PIO programs guarantee the first word pushed after a
 * (re)start is a LEFT word, and the ring holds an even number of words,
 * so a word's position in the ring fixes its channel permanently. Even a
 * writer-laps-reader overrun garbles audio momentarily but can never
 * swap channels.
 */

#include "i2s_input.h"
#include "audio_input.h"
#include "audio_pipeline.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "usb_audio.h"

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "i2s_input.pio.h"

#include <stdio.h>
#include <string.h>

// ============================================================================
// RESOURCES (shared with SPDIF RX, mutually exclusive by input switching)
// ============================================================================

#define i2s_rx_pio __CONCAT(pio, PICO_SPDIF_RX_PIO)

#if PICO_RP2350
#define I2S_RX_SM           0
#else
#define I2S_RX_SM           2
#endif

#define I2S_RX_DMA_DATA     PICO_SPDIF_RX_DMA_CH0   // PIO RXF -> ring
#define I2S_RX_DMA_RELOAD   PICO_SPDIF_RX_DMA_CH1   // re-arms the data channel

// Ring sizing: must be a power of 2 (DMA address wrap) and even (L/R
// parity). At 96 kHz stereo (192k words/s) this is ~5 ms of headroom on
// RP2040 and ~10 ms on RP2350; main-loop stalls longer than that only
// happen around flash operations, which suspend the input anyway.
#if PICO_RP2350
#define I2S_RX_RING_WORDS   2048u
#define I2S_RX_RING_BITS    13u                     // log2(ring bytes)
#else
#define I2S_RX_RING_WORDS   1024u
#define I2S_RX_RING_BITS    12u
#endif
#define I2S_RX_RING_BYTES   (I2S_RX_RING_WORDS * 4u)

// Minimum stereo frames to accumulate before feeding the pipeline.
//
// The DMA ring's write address advances per word, so the main loop (which
// polls far faster than samples arrive) would otherwise call
// process_input_block() with only a handful of frames each time. The CPU
// meter is budget-based (busy_us / (frames / Fs)), so the fixed per-block
// cost (Core 1 EQ-worker handshake, pipeline setup) divided by a tiny frame
// count reads as a large inflation. Batching to 48 frames matches the USB
// packet / consumer-buffer granularity, bringing I2S CPU in line with USB.
// At 48 kHz this adds ~1 ms of input latency; the consumer pool (50% prefill)
// absorbs the resulting sub-buffer fill ripple.
#define I2S_INPUT_MIN_BLOCK 48u

static uint32_t __attribute__((aligned(I2S_RX_RING_BYTES)))
    i2s_rx_ring[I2S_RX_RING_WORDS];

// Source word for the reload channel (holds the ring base address)
static uintptr_t i2s_rx_ring_base_addr;

// ============================================================================
// STATE
// ============================================================================

static volatile I2sInputState i2s_state = I2S_INPUT_INACTIVE;
static bool i2s_role_master = false;
static int  i2s_prog_offset = -1;

// Pins captured at start time.  stop() must release what was actually
// configured, NOT the live globals: the hot-swap handlers update
// i2s_rx_pin / i2s_bck_pin before the deferred stop runs, so releasing
// the globals would strand the old pins on the input PIO function.
static uint8_t i2s_active_rx_pin;
static uint8_t i2s_active_bck_pin;

// Software read index into the ring, in words (0 .. I2S_RX_RING_WORDS-1).
// Always advanced in whole stereo pairs so its parity is preserved.
static uint32_t i2s_rd_word;

// RAM copy of the slave program with the BCK/LRCLK GPIO numbers patched
// into the wait instructions (i2s_bck_pin is runtime-configurable).
static uint16_t i2s_slave_prog_ram[7];
static struct pio_program i2s_slave_prog = {
    .instructions = i2s_slave_prog_ram,
    .length = 7,
    .origin = -1,
};

// ============================================================================
// HELPERS
// ============================================================================

// 24.8 fixed-point divider for the clock-master role; identical ceiling
// math to the I2S TX library so input BCK matches output BCK exactly.
static uint32_t rx_master_divider_24_8(uint32_t sample_freq) {
    uint64_t num = (uint64_t)clock_get_hz(clk_sys) * 2u;
    return (uint32_t)((num + sample_freq - 1) / sample_freq);
}

// Patch the 5-bit GPIO index field of a `wait gpio` instruction
static inline uint16_t patch_wait_gpio(uint16_t instr, uint8_t pin) {
    return (uint16_t)((instr & ~0x1Fu) | (pin & 0x1Fu));
}

static inline uint32_t ring_write_word(void) {
    return (uint32_t)((dma_hw->ch[I2S_RX_DMA_DATA].write_addr -
                       (uint32_t)i2s_rx_ring_base_addr) / 4u) %
           I2S_RX_RING_WORDS;
}

// Wait (bounded) for DMA to drain the PIO RX FIFO. Used before re-anchoring
// the read pointer so no in-flight words land after the anchor; avoids
// clearing the FIFO while the DMA holds unserviced DREQ credits.
static void drain_rx_fifo(void) {
    for (uint32_t spin = 0; spin < 10000; spin++) {
        if (pio_sm_get_rx_fifo_level(i2s_rx_pio, I2S_RX_SM) == 0) break;
        tight_loop_contents();
    }
}

static void start_dma_ring(void) {
    // Reload channel: one word, no increments, no chain (chain-to-self).
    // Each completion of the data channel chains here; this writes the
    // ring base into the data channel's write address trigger alias,
    // which also reloads its transfer count. Runs forever, zero IRQs.
    dma_channel_config cb = dma_channel_get_default_config(I2S_RX_DMA_RELOAD);
    channel_config_set_transfer_data_size(&cb, DMA_SIZE_32);
    channel_config_set_read_increment(&cb, false);
    channel_config_set_write_increment(&cb, false);
    channel_config_set_chain_to(&cb, I2S_RX_DMA_RELOAD);
    dma_channel_configure(I2S_RX_DMA_RELOAD, &cb,
                          &dma_hw->ch[I2S_RX_DMA_DATA].al2_write_addr_trig,
                          &i2s_rx_ring_base_addr, 1, false);

    // Data channel: PIO RX FIFO -> ring with write-address wrap
    dma_channel_config ca = dma_channel_get_default_config(I2S_RX_DMA_DATA);
    channel_config_set_transfer_data_size(&ca, DMA_SIZE_32);
    channel_config_set_read_increment(&ca, false);
    channel_config_set_write_increment(&ca, true);
    channel_config_set_ring(&ca, true, I2S_RX_RING_BITS);
    channel_config_set_dreq(&ca, pio_get_dreq(i2s_rx_pio, I2S_RX_SM, false));
    channel_config_set_chain_to(&ca, I2S_RX_DMA_RELOAD);
    dma_channel_configure(I2S_RX_DMA_DATA, &ca,
                          i2s_rx_ring, &i2s_rx_pio->rxf[I2S_RX_SM],
                          I2S_RX_RING_WORDS, true);
}

// Race-free teardown of the self-retriggering ring.
//
// The data channel chains to the reload channel, and the reload channel
// re-triggers the data channel by writing its al2_write_addr_trig. Aborting
// the two naively (two sequential dma_channel_abort calls) lets one channel
// re-arm the other in the gap between the aborts, or lets aborting the data
// channel trigger its chain target. Either way a channel can be left BUSY,
// so dma_channel_abort's `while (BUSY)` spin never returns (watchdog reset)
// or a channel is left live after unclaim (corrupted on the next start).
//
// Break the loop deterministically: first disarm the data channel's chain
// (the reload channel only ever runs because the data channel chains to it,
// so once disarmed it cannot fire again), then abort BOTH channels in a
// single write so neither can re-arm the other. The bounded guard ensures a
// hardware quirk degrades to a clean stop rather than an unbounded spin.
static void stop_dma_ring(void) {
    // Disarm data -> reload chaining via the non-triggering CTRL alias
    // (al1_ctrl); point the data channel's chain_to at itself.
    hw_write_masked(&dma_hw->ch[I2S_RX_DMA_DATA].al1_ctrl,
                    (uint32_t)I2S_RX_DMA_DATA << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB,
                    DMA_CH0_CTRL_TRIG_CHAIN_TO_BITS);

    // Abort both channels atomically (one register write, both bits set).
    dma_hw->abort = (1u << I2S_RX_DMA_DATA) | (1u << I2S_RX_DMA_RELOAD);

    uint32_t guard = 1000000u;
    while (((dma_hw->ch[I2S_RX_DMA_DATA].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS) ||
            (dma_hw->ch[I2S_RX_DMA_RELOAD].ctrl_trig & DMA_CH0_CTRL_TRIG_BUSY_BITS)) &&
           --guard) {
        tight_loop_contents();
    }
}

// ============================================================================
// PUBLIC API
// ============================================================================

void i2s_input_init(void) {
    i2s_state = I2S_INPUT_INACTIVE;
    i2s_rx_ring_base_addr = (uintptr_t)i2s_rx_ring;
}

void i2s_input_start(bool clock_master) {
    // Guard against double-start (would panic on resource re-claim)
    if (i2s_state != I2S_INPUT_INACTIVE) return;

    pio_sm_claim(i2s_rx_pio, I2S_RX_SM);
    dma_channel_claim(I2S_RX_DMA_DATA);
    dma_channel_claim(I2S_RX_DMA_RELOAD);

    // Defensive: SPDIF RX shares these channels on DMA_IRQ_1; make sure no
    // stale per-channel IRQ enables survive from a previous SPDIF session,
    // since this ring runs without any IRQs.
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, I2S_RX_DMA_DATA, false);
    dma_irqn_set_channel_enabled(PICO_SPDIF_RX_DMA_IRQ, I2S_RX_DMA_RELOAD, false);
    dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, I2S_RX_DMA_DATA);
    dma_irqn_acknowledge_channel(PICO_SPDIF_RX_DMA_IRQ, I2S_RX_DMA_RELOAD);

    i2s_role_master = clock_master;
    i2s_active_rx_pin = i2s_rx_pin;
    i2s_active_bck_pin = i2s_bck_pin;

    // Data pin: input path only. pio_gpio_init sets pad IE and clears the
    // RP2350 pad isolation latch; the SM never drives it (pindir input).
    pio_gpio_init(i2s_rx_pio, i2s_rx_pin);

    if (clock_master) {
        // We own BCK/LRCLK: route them to this PIO block
        pio_gpio_init(i2s_rx_pio, i2s_bck_pin);
        pio_gpio_init(i2s_rx_pio, i2s_bck_pin + 1);

        i2s_prog_offset = pio_add_program(i2s_rx_pio,
                                          &audio_i2s_rx_clkmaster_program);
        audio_i2s_rx_clkmaster_program_init(i2s_rx_pio, I2S_RX_SM,
                                            (uint)i2s_prog_offset,
                                            i2s_rx_pin, i2s_bck_pin);

        uint32_t div = rx_master_divider_24_8(audio_state.freq);
        pio_sm_set_clkdiv_int_frac(i2s_rx_pio, I2S_RX_SM,
                                   (uint16_t)(div >> 8u), (uint8_t)(div & 0xFFu));
    } else {
        // Clocks come from the I2S TX master (another PIO block). Do NOT
        // touch their function; just make sure the input buffers are on
        // (gpio_set_function set IE already, this is belt and braces).
        gpio_set_input_enabled(i2s_bck_pin, true);
        gpio_set_input_enabled(i2s_bck_pin + 1, true);

        // Patch the BCK/LRCLK wait instructions with the runtime pins.
        // Patch map per i2s_input.pio: 0,1 = LRCLK; 2..5 = BCK.
        memcpy(i2s_slave_prog_ram, audio_i2s_rx_slave_program_instructions,
               sizeof(i2s_slave_prog_ram));
        i2s_slave_prog_ram[0] = patch_wait_gpio(i2s_slave_prog_ram[0], i2s_bck_pin + 1);
        i2s_slave_prog_ram[1] = patch_wait_gpio(i2s_slave_prog_ram[1], i2s_bck_pin + 1);
        for (int i = 2; i <= 5; i++) {
            i2s_slave_prog_ram[i] = patch_wait_gpio(i2s_slave_prog_ram[i], i2s_bck_pin);
        }

        i2s_prog_offset = pio_add_program(i2s_rx_pio, &i2s_slave_prog);
        audio_i2s_rx_slave_program_init(i2s_rx_pio, I2S_RX_SM,
                                        (uint)i2s_prog_offset, i2s_rx_pin);
        pio_sm_set_clkdiv_int_frac(i2s_rx_pio, I2S_RX_SM, 1, 0);
    }

    // DMA first (SM is disabled with clean FIFOs after pio_sm_init), then
    // the SM, so the first pushed word lands at the ring base = LEFT word.
    i2s_rd_word = 0;
    start_dma_ring();

    if (clock_master) {
        // Preload the bit counter and enter at the start of a left frame
        pio_sm_exec(i2s_rx_pio, I2S_RX_SM, pio_encode_set(pio_x, 29));
        pio_sm_exec(i2s_rx_pio, I2S_RX_SM,
                    pio_encode_jmp((uint)i2s_prog_offset +
                                   audio_i2s_rx_clkmaster_wrap_target));
    }
    // Slave role: pio_sm_init left the PC at the program entry point

    pio_sm_set_enabled(i2s_rx_pio, I2S_RX_SM, true);

    i2s_state = I2S_INPUT_RUNNING;
    printf("I2S RX: started on GPIO %u (%s)\n", i2s_rx_pin,
           clock_master ? "clock master" : "slave");
}

void i2s_input_stop(void) {
    if (i2s_state == I2S_INPUT_INACTIVE) return;

    pio_sm_set_enabled(i2s_rx_pio, I2S_RX_SM, false);

    // Tear down the chained DMA ring race-free (see stop_dma_ring). A naive
    // pair of dma_channel_abort() calls here intermittently hung (watchdog
    // reset) because the two channels re-trigger each other.
    stop_dma_ring();

    if (i2s_role_master) {
        if (i2s_prog_offset >= 0) {
            pio_remove_program(i2s_rx_pio, &audio_i2s_rx_clkmaster_program,
                               (uint)i2s_prog_offset);
        }
        // Release BCK/LRCLK to high-Z; if an output master is taking over
        // it re-initializes them on its own PIO block immediately after.
        gpio_set_function(i2s_active_bck_pin, GPIO_FUNC_NULL);
        gpio_set_dir(i2s_active_bck_pin, GPIO_IN);
        gpio_set_function(i2s_active_bck_pin + 1, GPIO_FUNC_NULL);
        gpio_set_dir(i2s_active_bck_pin + 1, GPIO_IN);
    } else if (i2s_prog_offset >= 0) {
        pio_remove_program(i2s_rx_pio, &i2s_slave_prog, (uint)i2s_prog_offset);
    }
    i2s_prog_offset = -1;

    // Release the data pin to high-Z
    gpio_set_function(i2s_active_rx_pin, GPIO_FUNC_NULL);
    gpio_set_dir(i2s_active_rx_pin, GPIO_IN);

    pio_sm_unclaim(i2s_rx_pio, I2S_RX_SM);
    dma_channel_unclaim(I2S_RX_DMA_DATA);
    dma_channel_unclaim(I2S_RX_DMA_RELOAD);

    i2s_state = I2S_INPUT_INACTIVE;
    printf("I2S RX: stopped\n");
}

void i2s_input_resync(void) {
    // Only a running SLAVE needs re-phasing: the TX clock master restarts
    // from its PIO entry point during synchronized output starts, which
    // resets LRCLK phase under our bit counter. The master role generates
    // its own clocks and is unaffected.
    if (i2s_state != I2S_INPUT_RUNNING || i2s_role_master) return;

    pio_sm_set_enabled(i2s_rx_pio, I2S_RX_SM, false);

    // Let DMA finish moving whatever the SM already pushed, then anchor
    // the read pointer at the current write position: the next word the
    // re-entered program pushes is a LEFT word.
    drain_rx_fifo();
    pio_sm_restart(i2s_rx_pio, I2S_RX_SM);   // clears ISR shift counter
    i2s_rd_word = ring_write_word();

    pio_sm_exec(i2s_rx_pio, I2S_RX_SM,
                pio_encode_jmp((uint)i2s_prog_offset +
                               audio_i2s_rx_slave_offset_entry_point));
    pio_sm_set_enabled(i2s_rx_pio, I2S_RX_SM, true);
}

// ============================================================================
// MAIN-LOOP POLL
// ============================================================================

DSP_TIME_CRITICAL
uint32_t i2s_input_poll(void) {
    if (i2s_state != I2S_INPUT_RUNNING) return 0;

    uint32_t wr = ring_write_word();
    uint32_t avail = (wr - i2s_rd_word) % I2S_RX_RING_WORDS;
    uint32_t frames = avail / 2u;
    // Batch: wait for a pipeline-sized block so the fixed per-block cost is
    // amortized like the USB path (see I2S_INPUT_MIN_BLOCK). The input is
    // continuous, so avail always climbs to the threshold; this never starves.
    if (frames < I2S_INPUT_MIN_BLOCK) return 0;
    if (frames > 192u) frames = 192u;   // buf_l/buf_r capacity

#if PICO_RP2350
    float preamp_l = global_preamp_linear[0];
    float preamp_r = global_preamp_linear[1];
    const float inv_2147483648 = 1.0f / 2147483648.0f;
#else
    int32_t preamp_l = global_preamp_mul[0];
    int32_t preamp_r = global_preamp_mul[1];
#endif

    uint32_t idx = i2s_rd_word;
    for (uint32_t i = 0; i < frames; i++) {
        // 24-bit audio in bits [31:8]; mask the don't-care low byte
        int32_t raw_l = (int32_t)(i2s_rx_ring[idx] & 0xFFFFFF00u);
        idx = (idx + 1u) % I2S_RX_RING_WORDS;
        int32_t raw_r = (int32_t)(i2s_rx_ring[idx] & 0xFFFFFF00u);
        idx = (idx + 1u) % I2S_RX_RING_WORDS;

#if PICO_RP2350
        buf_l[i] = (float)raw_l * inv_2147483648 * preamp_l;
        buf_r[i] = (float)raw_r * inv_2147483648 * preamp_r;
#else
        // Q28: int32 full-scale >> 2 -> Q28, then preamp; matches the
        // SPDIF RX and USB 24-bit paths so output unity gain holds
        buf_l[i] = fast_mul_q28(raw_l >> 2, preamp_l);
        buf_r[i] = fast_mul_q28(raw_r >> 2, preamp_r);
#endif
    }
    i2s_rd_word = idx;

    process_input_block(frames);
    return frames;
}

// Push one silent block through the DSP pipeline to prefill the output consumer
// pools when the input cannot supply samples itself.
//
// This is only used during a SLAVE-role prefill. In slave mode the input is
// clocked by the I2S output clock master, so draining the outputs to prefill
// (the SPDIF-style handshake) also stops the input's BCK/LRCLK and no input
// samples arrive; the pools could never reach the 50% target and outputs would
// never re-enable. Synthesizing silence fills the pools deterministically;
// real audio resumes after enable_outputs_in_sync() restarts the clock master
// and re-phases the input ring. Master-role prefill uses real input audio via
// i2s_input_poll() and never calls this.
void i2s_input_prefill_silence(uint32_t frames) {
    if (frames == 0) return;
    if (frames > 192u) frames = 192u;   // buf_l/buf_r capacity
    memset(buf_l, 0, frames * sizeof(buf_l[0]));
    memset(buf_r, 0, frames * sizeof(buf_r[0]));
    process_input_block(frames);
}

// ============================================================================
// STATUS
// ============================================================================

I2sInputState i2s_input_get_state(void) {
    return i2s_state;
}

bool i2s_input_is_clock_master(void) {
    return (i2s_state == I2S_INPUT_RUNNING) && i2s_role_master;
}
