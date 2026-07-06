/*
 * vendor_commands.c — Vendor USB control request handlers for DSPi
 *
 * Phase 2 (TinyUSB): the legacy pico-extras setup/packet handlers have been
 * replaced by TinyUSB's stage-based tud_control_xfer flow.  All 30+ command
 * handlers inside the SET and GET switch statements are unchanged — they
 * continue to consume `vendor_rx_buf` / produce responses via
 * `vendor_send_response()`, which now trampolines into `tud_control_xfer()`.
 *
 * All state DEFINITIONS remain in usb_audio.c; this file accesses them
 * via extern declarations in usb_audio.h.
 */

#include "vendor_commands.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "spdif_input.h"
#include "lg_sound_sync.h"
#include "dac_hw_mute.h"
#include "audio_pipeline.h"
#include "config.h"
#include "dsp_pipeline.h"
#include "crossover.h"
#include "flash_storage.h"
#include "loudness.h"
#include "crossfeed.h"
#include "leveller.h"
#include "bulk_params.h"
#include "notify.h"
#include "uart_control.h"
#include "i2c_control.h"
#include "control_surfaces.h"
#include "pdm_generator.h"
#include "siggen.h"
#include "adat_output.h"
#include "usb_descriptors.h"
#include "tusb.h"
#include "pico/audio_spdif.h"
#include "pico/audio_i2s_multi.h"
#include "hardware/adc.h"
#include "hardware/vreg.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"

// ----------------------------------------------------------------------------
// VENDOR-INTERNAL STATIC STATE
// ----------------------------------------------------------------------------

// SET payload buffer (wLength <= 64 for regular SET; bulk SET uses bulk_param_buf).
static uint8_t vendor_rx_buf[64];
static uint8_t vendor_last_request = 0;
static uint16_t vendor_last_wValue = 0;
static uint16_t vendor_last_wLength = 0;

// Captured during SETUP so vendor_send_response() can issue tud_control_xfer()
// without every GET case having to plumb rhport + request through.
static uint8_t _vendor_rhport;
static tusb_control_request_t const *_vendor_current_req;

// ----------------------------------------------------------------------------
// ORCHESTRATOR STATE (transport-neutral dispatch)
// ----------------------------------------------------------------------------

// Which transport the current dispatch runs for.  USB responses go straight
// to tud_control_xfer(); external ones are captured for the transport to
// stream at its own pace.  Only ever changed from main-loop context.
static CtrlSource _active_source = CTRL_SOURCE_USB;
static const uint8_t *_ext_resp_data;
static uint16_t _ext_resp_len;

// notify source tag for the running dispatch; entry points set it before
// invoking the SET/GET handlers (HOST_SET for USB, UART/I2C for external).
static ParamSource _dispatch_src = PARAM_SRC_HOST_SET;

// A USB control SET is between SETUP and DATA/ACK: the USB ISR may still be
// writing vendor_rx_buf / bulk_param_buf, and vendor_last_* is live.
// External dispatch must wait it out (BUSY).  Stale after 100 ms in case the
// host aborts the transfer mid-flight.
static volatile bool usb_set_in_flight = false;
static uint32_t usb_set_setup_time;

static bool usb_set_busy(void) {
    if (!usb_set_in_flight) return false;
    if ((uint32_t)(time_us_32() - usb_set_setup_time) > 100000u) {
        usb_set_in_flight = false;   // aborted transfer; reclaim
        return false;
    }
    return true;
}

// bulk_param_buf ownership.  The buffer is shared by all transports for both
// 0xA0 collect-and-stream and 0xA1 receive-then-apply; exactly one owner at
// a time.  bulk_params_pending additionally guards the main-loop apply
// window after a SET is handed over.  Owner is stored as (CtrlSource + 1)
// so any future transport works without a mapping table; 0 = unowned.
#define BULK_OWNER_NONE  0u
#define BULK_OWNER_USB   ((uint8_t)(CTRL_SOURCE_USB + 1))
static volatile uint8_t bulk_buf_owner = BULK_OWNER_NONE;
static uint32_t bulk_owner_since;

static inline uint8_t bulk_owner_for(CtrlSource src) {
    return (uint8_t)(src + 1);
}

// IRQ-safe: the I2C target ISR acquires the lock when a bulk SET header
// arrives mid-transaction, racing the main loop's USB/UART acquires.
bool vendor_bulk_try_acquire(CtrlSource src) {
    uint32_t save = save_and_disable_interrupts();
    // Owners release deterministically (external transports from their poll
    // loops, USB at the control-transfer ACK or the next SETUP); the stale
    // windows only matter if an owner dies mid-stream.  External streamers
    // refresh via vendor_bulk_touch(); USB gets a longer window because EP0
    // has no refresh hook, and a healthy EP0 bulk completes in well under
    // a second (a cable pull mid-transfer must not wedge the other
    // transports' bulk access forever).
    if (bulk_buf_owner != BULK_OWNER_NONE) {
        uint32_t limit = (bulk_buf_owner == BULK_OWNER_USB) ? 3000000u : 500000u;
        if ((uint32_t)(time_us_32() - bulk_owner_since) > limit) {
            bulk_buf_owner = BULK_OWNER_NONE;
        }
    }
    bool ok = (bulk_buf_owner == BULK_OWNER_NONE && !bulk_params_pending);
    if (ok) {
        bulk_buf_owner = bulk_owner_for(src);
        bulk_owner_since = time_us_32();
    }
    restore_interrupts(save);
    return ok;
}

void vendor_bulk_release(CtrlSource src) {
    uint32_t save = save_and_disable_interrupts();
    if (bulk_buf_owner == bulk_owner_for(src)) bulk_buf_owner = BULK_OWNER_NONE;
    restore_interrupts(save);
}

// Refresh the stale timer while a slow transport actively streams a bulk
// payload (a full bulk GET takes ~0.5 s at 115200 baud).
void vendor_bulk_touch(CtrlSource src) {
    if (bulk_buf_owner == bulk_owner_for(src)) bulk_owner_since = time_us_32();
}

// Single definition of "this frame is a bulk SET"; the USB SETUP gate and
// both external transports must agree or they will disagree on where the
// payload lands.  bulk_params_apply() re-checks version + length.
bool vendor_is_bulk_set(uint8_t bRequest, uint16_t wLength) {
    return bRequest == REQ_SET_ALL_PARAMS &&
           wLength >= WIRE_BULK_PARAMS_MIN_SIZE &&
           wLength <= sizeof(WireBulkParams);
}

// --- Chunked bulk-params sessions (USB only; see 0xA2/0xA3 in config.h) ---
// A session spans multiple EP0 control transfers, so the bulk lock must
// survive intervening chunk SETUPs; the SETUP-stage USB-owner cleanup
// exempts chunk requests while a session is open.  Abandoned sessions are
// torn down by the first non-chunk vendor request or the 3 s stale reap.
static bool     usb_chunk_get_open = false;   // snapshot parked in bulk_param_buf
static bool     usb_chunk_get_done = false;   // final chunk queued; release at ACK
static bool     usb_chunk_set_open = false;
static uint16_t usb_chunk_set_received = 0;   // next expected byte offset

static void usb_chunk_sessions_close(void) {
    usb_chunk_get_open = false;
    usb_chunk_get_done = false;
    usb_chunk_set_open = false;
    usb_chunk_set_received = 0;
}

// True while `req` continues an open chunk session (so the SETUP cleanup
// must not drop the USB-held bulk lock underneath it).
static bool usb_chunk_session_continues(uint8_t bRequest) {
    return (bRequest == REQ_GET_ALL_PARAMS_CHUNK && usb_chunk_get_open) ||
           (bRequest == REQ_SET_ALL_PARAMS_CHUNK && usb_chunk_set_open);
}

// Shared CtrlDispatchResult to CTRL_STATUS_* wire mapping for the external
// transports (one definition so UART and I2C can never disagree).
uint8_t ctrl_status_from_dispatch(CtrlDispatchResult r) {
    switch (r) {
        case CTRL_DISPATCH_OK:          return CTRL_STATUS_OK;
        case CTRL_DISPATCH_BLOCKED:     return CTRL_STATUS_BLOCKED;
        case CTRL_DISPATCH_BULK_LOCKED: return CTRL_STATUS_BULK_LOCKED;
        case CTRL_DISPATCH_BUSY:        return CTRL_STATUS_BUSY;
        default:                        return CTRL_STATUS_ERROR;
    }
}

// Internal helpers exported via vendor_send_response() replacement.
static void vendor_send_response(const void *data, uint16_t len);

// Forward declarations: SET data-stage and GET dispatch.
static bool vendor_handle_set_data(tusb_control_request_t const *req);
static bool vendor_handle_get(tusb_control_request_t const *req);

// Shim type preserved from the pico-extras era so the SET handler case
// bodies (which still read `buffer->data_len` and `buffer->data`) compile
// unmodified against TinyUSB's already-buffered control data.
typedef struct {
    uint8_t *data;
    uint16_t data_len;
} vendor_buffer_t;

// ----------------------------------------------------------------------------
// SYSTEM STATISTICS HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Convert vreg voltage enum to millivolts
uint16_t vreg_voltage_to_mv(enum vreg_voltage voltage) {
    // Voltage enum values map to specific voltages
    // See hardware/vreg.h for the full table
    static const uint16_t voltage_table[] = {
#if !PICO_RP2040
        550,  // VREG_VOLTAGE_0_55 = 0b00000
        600,  // VREG_VOLTAGE_0_60 = 0b00001
        650,  // VREG_VOLTAGE_0_65 = 0b00010
        700,  // VREG_VOLTAGE_0_70 = 0b00011
        750,  // VREG_VOLTAGE_0_75 = 0b00100
        800,  // VREG_VOLTAGE_0_80 = 0b00101
#endif
        850,  // VREG_VOLTAGE_0_85 = 0b00110
        900,  // VREG_VOLTAGE_0_90 = 0b00111
        950,  // VREG_VOLTAGE_0_95 = 0b01000
        1000, // VREG_VOLTAGE_1_00 = 0b01001
        1050, // VREG_VOLTAGE_1_05 = 0b01010
        1100, // VREG_VOLTAGE_1_10 = 0b01011
        1150, // VREG_VOLTAGE_1_15 = 0b01100
        1200, // VREG_VOLTAGE_1_20 = 0b01101
        1250, // VREG_VOLTAGE_1_25 = 0b01110
        1300, // VREG_VOLTAGE_1_30 = 0b01111
#if !PICO_RP2040
        1350, // VREG_VOLTAGE_1_35 = 0b10000
        1400, // VREG_VOLTAGE_1_40 = 0b10001
        1500, // VREG_VOLTAGE_1_50 = 0b10010
        1600, // VREG_VOLTAGE_1_60 = 0b10011
        1650, // VREG_VOLTAGE_1_65 = 0b10100
        1700, // VREG_VOLTAGE_1_70 = 0b10101
        1800, // VREG_VOLTAGE_1_80 = 0b10110
        1900, // VREG_VOLTAGE_1_90 = 0b10111
        2000, // VREG_VOLTAGE_2_00 = 0b11000
        2350, // VREG_VOLTAGE_2_35 = 0b11001
        2500, // VREG_VOLTAGE_2_50 = 0b11010
        2650, // VREG_VOLTAGE_2_65 = 0b11011
        2800, // VREG_VOLTAGE_2_80 = 0b11100
        3000, // VREG_VOLTAGE_3_00 = 0b11101
        3150, // VREG_VOLTAGE_3_15 = 0b11110
        3300, // VREG_VOLTAGE_3_30 = 0b11111
#endif
    };

#if PICO_RP2040
    // RP2040: offset by 6 entries (0.55-0.80V not available)
    uint8_t index = (voltage >= 6) ? (voltage - 6) : 0;
#else
    uint8_t index = voltage;
#endif

    if (index < sizeof(voltage_table) / sizeof(voltage_table[0])) {
        return voltage_table[index];
    }
    return 1100; // Default fallback
}

// Read ADC temperature sensor and return temperature in centi-degrees C
// Formula from SDK docs (same for RP2040/RP2350):
// T = 27 - (ADC_Voltage - 0.706) / 0.001721
int16_t read_temperature_cdeg(void) {
    const float conversion_factor = 3.3f / 4095.0f;

    // Temperature sensor channel: auto-detects based on chip variant
    // RP2040, RP2350A (QFN-60): channel 4
    // RP2350B (QFN-80): channel 8
    adc_select_input(NUM_ADC_CHANNELS - 1);
    uint16_t adc_raw = adc_read();
    float voltage = adc_raw * conversion_factor;
    float temp_c = 27.0f - (voltage - 0.706f) / 0.001721f;

    return (int16_t)(temp_c * 100.0f); // Convert to centi-degrees
}

// ----------------------------------------------------------------------------
// CORE 1 MODE DERIVATION (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// derive_core1_mode lives in usb_audio.c (moved during Phase 1 migration).

// ----------------------------------------------------------------------------
// MCK HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Encode/decode for wire and flash persistence: 0 = 128x, 1 = 256x
uint8_t  mck_encode(uint16_t val) { return (val == 256) ? 1 : 0; }
uint16_t mck_decode(uint8_t raw)  { return (raw == 1) ? 256 : 128; }

// Note: the previous PIO-toggle MCK had a 96 kHz × 256× clamp here because
// the fractional 6.25 divider made that combo unreliable.  CLK_GPOUTn gives
// us 12.5 at 96 kHz × 256× — still fractional, but stable enough on real
// hardware that we no longer pre-emptively clamp.  All the other 256×
// combinations we used to reject (48 kHz × 256×, 96 kHz × 128×) are now
// integer dividers with GPOUT.  See audio_i2s_multi.c MCK section for the
// reference table.

// ----------------------------------------------------------------------------
// PIN VALIDATION HELPERS (moved from usb_audio.c)
// ----------------------------------------------------------------------------

bool is_valid_gpio_pin(uint8_t pin) {
    // GPIO 16/17 are general-purpose again since the debug UART was removed;
    // when the UART control interface claims them, is_pin_in_use() covers it.
    if (pin >= 23 && pin <= 25) return false;   // Power/LED
#if PICO_RP2350
    return pin <= 29;
#else
    return pin <= 28;
#endif
}

// Pin claimed by a "fixed" peripheral: a pin output, the I2S MCK, the SPDIF RX
// input, or the DAC hardware-mute.  Deliberately EXCLUDES the I2S BCK/LRCLK
// clocks and the I2S RX data pins — those have use-dependent rules the callers
// below apply (the clocks are reserved only while running for general queries,
// but treated as always-claimed when validating an I2S RX data pin).
static bool pin_used_by_fixed_peripheral(uint8_t pin, uint8_t exclude_output) {
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        if (i == exclude_output) continue;
        if (output_pins[i] == pin) return true;
    }
    if (i2s_mck_enabled && pin == i2s_mck_pin) return true;   // MCK (if enabled)
#if PICO_RP2350
    // ADAT data pin: claimed only while ADAT is config-enabled (mirrors MCK).
    if (adat_output_config_enabled() && pin == adat_output_pin()) return true;
#endif
    if (pin == spdif_rx_pin) return true;                     // SPDIF RX input
    if (dac_hw_mute_owns_pin(pin)) return true;               // DAC hardware-mute
    if (uart_ctrl_owns_pin(pin)) return true;                 // UART control (if live)
    if (i2c_ctrl_owns_pin(pin)) return true;                  // I2C control (if live)
    if (control_surfaces_owns_pin(pin)) return true;         // Control Surfaces (live bindings)
    return false;
}

bool is_pin_in_use(uint8_t pin, uint8_t exclude) {
    if (pin_used_by_fixed_peripheral(pin, exclude)) return true;
    // I2S BCK/LRCLK: reserved only while the clocks actually run (any I2S output,
    // or I2S is the active input — both its master and slave roles drive/read the
    // same clock pair).  A general caller (e.g. assigning a pin output) may
    // legitimately use these GPIOs when no I2S path is live.
    bool i2s_clocks_in_use = (active_input_source == INPUT_SOURCE_I2S);
    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
        if (output_types[i] == OUTPUT_TYPE_I2S) {
            i2s_clocks_in_use = true;
            break;  // All I2S slots share the same BCK/LRCLK
        }
    }
    if (i2s_clocks_in_use &&
        (pin == i2s_bck_pin || pin == (uint8_t)(i2s_bck_pin + 1))) return true;
    // I2S RX data pins for every active (by count) stereo pair.
    for (int p = 0; p < i2s_input_channels / 2 && p < I2S_RX_MAX_PAIRS; p++)
        if (pin == i2s_rx_pin[p]) return true;
    return false;
}

// True if `pin` is the data pin of an I2S RX stereo pair other than
// `exclude_pair`, active OR inactive.  is_pin_in_use() only reserves the
// currently-active pairs (so the inactive placeholder pins never block other
// functions in stereo mode); this is the complementary guard that keeps all
// configured pair pins mutually distinct, so raising i2s_input_channels can
// never bring two state machines up on the same GPIO.
static bool i2s_rx_pair_pin_taken(uint8_t pin, uint8_t exclude_pair) {
    for (int q = 0; q < I2S_RX_MAX_PAIRS; q++)
        if (q != exclude_pair && i2s_rx_pin[q] == pin)
            return true;
    return false;
}

// PIN_CONFIG_* status for assigning `pin` to I2S RX stereo `pair`.  Unlike a bare
// is_pin_in_use() check, the I2S BCK/LRCLK clocks are treated as ALWAYS claimed:
// an I2S RX data pin coexists with the clocks whenever the input runs, so it must
// avoid them even while I2S is currently inactive (otherwise the clash would only
// surface on the switch INTO I2S).  Also rejects a pin already on another pair.
static uint8_t check_i2s_rx_pin(uint8_t pin, uint8_t pair) {
    if (!is_valid_gpio_pin(pin))                              return PIN_CONFIG_INVALID_PIN;
    if (pin == i2s_bck_pin || pin == (uint8_t)(i2s_bck_pin + 1))
                                                              return PIN_CONFIG_PIN_IN_USE;
    if (pin_used_by_fixed_peripheral(pin, 0xFF))             return PIN_CONFIG_PIN_IN_USE;
    if (i2s_rx_pair_pin_taken(pin, pair))                    return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}

// Validate a proposed I2S RX data-pin SET (the first `n_pairs` entries of
// `pins[]` are the pairs that will be active) against the given effective
// `bck_pin`: each active pin must be a valid GPIO, not a clock pin (BCK/LRCLK),
// not used by a fixed peripheral, and mutually distinct.  The bulk/preset
// restore paths use this to reject an inconsistent pushed/stored I2S config as a
// unit before it can reach i2s_input_start().  `bck_pin` is passed explicitly
// because a restore may set BCK in the same transaction, so the live value is
// not yet updated when this runs.
bool i2s_rx_pin_set_acceptable(const uint8_t *pins, uint8_t n_pairs, uint8_t bck_pin) {
    if (n_pairs > I2S_RX_MAX_PAIRS) return false;
    for (uint8_t p = 0; p < n_pairs; p++) {
        uint8_t pin = pins[p];
        if (!is_valid_gpio_pin(pin)) return false;
        if (pin == bck_pin || pin == (uint8_t)(bck_pin + 1)) return false;
        if (pin_used_by_fixed_peripheral(pin, 0xFF)) return false;
        for (uint8_t q = 0; q < p; q++)
            if (pins[q] == pin) return false;   // mutual distinctness
    }
    return true;
}

// True if `bck_pin` is acceptable as the I2S clock base (LRCLK = bck_pin + 1):
// both GPIOs valid, and neither collides with a fixed peripheral (a pin output,
// MCK, SPDIF RX, or DAC mute).  BCK/LRCLK are push-pull clock OUTPUTS, so a
// collision means two drivers contending on one pad (worse than the input-only
// RX pins) and an invalid GPIO can fault pio_gpio_init() — the bulk/preset
// restore paths use this to reject a pushed/stored BCK they would otherwise
// install raw.  The BCK-vs-I2S-RX-data-pin direction is intentionally NOT
// re-checked here: it is covered by i2s_rx_pin_set_acceptable() (the RX set is
// validated against the installed BCK), and checking it here against the
// not-yet-restored RX pins would spuriously reject a valid stored config.  No
// "reject while an I2S output is active" guard (cf. REQ_SET_I2S_BCK_PIN): a
// restore installs the output config and the clock pair together as one set.
bool i2s_bck_pin_acceptable(uint8_t bck_pin) {
    uint8_t lrck = (uint8_t)(bck_pin + 1);
    if (!is_valid_gpio_pin(bck_pin) || !is_valid_gpio_pin(lrck)) return false;
    if (pin_used_by_fixed_peripheral(bck_pin, 0xFF)) return false;
    if (pin_used_by_fixed_peripheral(lrck, 0xFF)) return false;
    return true;
}

#if PICO_RP2350
// True if `pin` is acceptable as the ADAT data GPIO for the bulk/preset
// restore paths.  ADAT is a push-pull output driver, so a collision with any
// owned pin is driver contention; ADAT's own current claim never blocks it.
bool adat_pin_acceptable(uint8_t pin) {
    if (!is_valid_gpio_pin(pin)) return false;
    if (adat_output_config_enabled() && pin == adat_output_pin()) return true;
    return !is_pin_in_use(pin, 0xFF);
}
#endif

// PIN_CONFIG_* status for claiming `pin` as a UART/I2C control-interface
// GPIO.  Like check_i2s_rx_pin, the I2S BCK/LRCLK pair is treated as ALWAYS
// claimed: control pins are long-lived, so they must avoid the clock pair
// even while no I2S path is currently running (otherwise the clash surfaces
// only when an output switches to I2S or the input switches to I2S and the
// clocks stomp the live control link).
uint8_t ctrl_iface_check_pin(uint8_t pin) {
    if (!is_valid_gpio_pin(pin))                              return PIN_CONFIG_INVALID_PIN;
    if (pin == i2s_bck_pin || pin == (uint8_t)(i2s_bck_pin + 1))
                                                              return PIN_CONFIG_PIN_IN_USE;
    if (is_pin_in_use(pin, 0xFF))                             return PIN_CONFIG_PIN_IN_USE;
    return PIN_CONFIG_SUCCESS;
}

// ----------------------------------------------------------------------------
// VENDOR SET HANDLER (moved from usb_audio.c)
// ----------------------------------------------------------------------------

// Returns false for a bRequest with no SET case so external transports can
// report ERROR instead of a false OK.  The USB path ignores the result (an
// unknown SET is ACKed, preserving long-standing host-visible behavior).
static bool vendor_handle_set_data(tusb_control_request_t const *req) {
    (void)req;
    bool handled = true;
    // Shim for legacy handler bodies below.  The real payload was delivered
    // into vendor_rx_buf by tud_control_xfer() during the DATA stage; we
    // synthesize a local struct with the same shape as the pico-extras
    // `buffer` the handlers used to read, so the case bodies stay unchanged.
    vendor_buffer_t _buf = { vendor_rx_buf, vendor_last_wLength };
    vendor_buffer_t *buffer = &_buf;
    (void)buffer;

    // Tag every setter call that runs inside this dispatch with the origin
    // the entry point selected (HOST_SET for USB, UART/I2C for external
    // transports).  Cleared at the single exit point below.
    notify_set_source(_dispatch_src);

    // Process command based on saved request info
    switch (vendor_last_request) {
        case REQ_SET_EQ_PARAM:
            if (buffer->data_len >= sizeof(EqParamPacket)) {
                memcpy((void*)&pending_packet, vendor_rx_buf, sizeof(EqParamPacket));
                // Normalize bypass byte at the boundary so legacy hosts
                // sending 0xFF padding cannot accidentally bypass the band.
                pending_packet.bypass = (pending_packet.bypass == 1) ? 1 : 0;

                uint8_t ch = pending_packet.channel;
                uint8_t b  = pending_packet.band;
                if (ch < NUM_CHANNELS) {
                    // Two band-index ranges are accepted:
                    //   0..channel_band_counts[ch]-1 → PEQ (existing)
                    //   XOVER_BAND_BASE..XOVER_BAND_BASE+MAX_XOVER_BANDS-1 → crossover
                    // Bands in [channel_band_counts, XOVER_BAND_BASE) are the
                    // reserved gap for future PEQ-count expansion; reject
                    // silently.  Crossover on master channels is rejected
                    // (feature is output-channel-only).  See
                    // Documentation/Features/crossover_filters_spec.md.
                    bool is_peq = (b < channel_band_counts[ch]);
                    bool is_xover = (b >= XOVER_BAND_BASE && b < XOVER_BAND_BASE + MAX_XOVER_BANDS);
                    if (is_peq || (is_xover && ch >= CH_OUT_1)) {
                        eq_update_pending = true;
                    }
                }
            }
            break;

        case REQ_SET_BAND_BYPASS: {
            // wValue = (channel << 8) | band; payload = 1 byte (1 = bypass, anything else = active).
            // Accepts both PEQ and crossover band indices — see
            // crossover_filters_spec.md for the unified band-index map.
            uint8_t channel = (vendor_last_wValue >> 8) & 0xFF;
            uint8_t band = vendor_last_wValue & 0xFF;
            if (channel < NUM_CHANNELS && buffer->data_len >= 1) {
                EqParamPacket p;
                bool valid = false;
                if (band < channel_band_counts[channel]) {
                    p = filter_recipes[channel][band];
                    valid = true;
                } else if (band >= XOVER_BAND_BASE && band < (XOVER_BAND_BASE + MAX_XOVER_BANDS)
                           && channel >= CH_OUT_1) {
                    p = xover_recipes[channel][band - XOVER_BAND_BASE];
                    valid = true;
                }
                if (valid) {
                    // Normalize the band field at the wire boundary.  Without
                    // this, a stale local-band-index inadvertently stored in
                    // either recipe array would propagate via pending_packet
                    // and main.c::eq_update_pending would misroute the write
                    // to the wrong storage.  See "Band-field normalization"
                    // in crossover_filters_spec.md.
                    p.channel = channel;
                    p.band    = band;
                    p.bypass  = (vendor_rx_buf[0] == 1) ? 1 : 0;
                    memcpy((void*)&pending_packet, &p, sizeof(EqParamPacket));
                    eq_update_pending = true;
                }
            }
            break;
        }

        case REQ_SET_PREAMP:
            // Legacy: sets ALL input channels to the same preamp value.
            // Payload: 4 bytes (float dB).
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                for (int ch = 0; ch < NUM_INPUT_CHANNELS; ch++)
                    update_preamp(ch, db);
            }
            break;

        case REQ_SET_PREAMP_CH: {
            // Per-channel preamp.  wValue = input channel index (0=L, 1=R).
            // Payload: 4 bytes (float dB).
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_INPUT_CHANNELS && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                update_preamp(ch, db);
            }
            break;
        }

        case REQ_SET_MASTER_VOLUME:
            // Set device-side master volume ceiling.
            // Payload: 4 bytes (float dB).  -128 = mute, -127..0 = attenuation range.
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                // Bracket the call so NOTIFY_SUPPRESS_HOST_ECHO (in usb_audio.c)
                // can filter out host-originated echoes if enabled.
                notify_master_vol_host_initiated = true;
                update_master_volume(db);
                notify_master_vol_host_initiated = false;
            }
            break;

        case REQ_SET_USER_VOLUME:
            // Vendor-channel user-perceived volume.  Same field as the UAC1
            // host slider (audio_state.volume); update_user_volume() applies
            // it to vol_mul + the loudness coefficient pointer regardless of
            // input source so equal-loudness compensation tracks the change
            // even during SPDIF playback.  Payload: 4 bytes (float dB).
            if (buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                update_user_volume(db);
            }
            break;

        case REQ_SET_USER_MUTE:
            // Vendor-channel mute.  Distinct from audio_state.mute (UAC1) —
            // the pipeline ORs them, but UAC1 mute is USB-gated while this
            // flag is always honored.  Symmetric with REQ_SET_USER_VOLUME's
            // always-apply contract.  Payload: 1 byte (0/1).
            if (buffer->data_len >= 1) {
                user_mute = (vendor_rx_buf[0] != 0);
                uint8_t v = user_mute ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                                   sizeof(uint8_t), &v);
            }
            break;

        case REQ_SET_DELAY: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                channel_delays_ms[ch] = ms;
                dsp_update_delay_samples((float)audio_state.freq);
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams, delays.delay_ms) + ch * sizeof(float)),
                    sizeof(float), &ms);
            }
            break;
        }

        case REQ_SET_BYPASS:
            if (buffer->data_len >= 1) {
                bypass_master_eq = (vendor_rx_buf[0] != 0);
                uint8_t v = bypass_master_eq ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, global.bypass), 1, &v);
            }
            break;

        case REQ_SET_CHANNEL_GAIN: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                channel_gain_db[ch] = db;
                float linear = powf(10.0f, db / 20.0f);
                channel_gain_mul[ch] = (int32_t)(linear * 32768.0f);
                channel_gain_linear[ch] = linear;
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams, legacy.gain_db) + ch * sizeof(float)),
                    sizeof(float), &db);
            }
            break;
        }

        case REQ_SET_CHANNEL_MUTE: {
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < 3 && buffer->data_len >= 1) {
                channel_mute[ch] = (vendor_rx_buf[0] != 0);
                uint8_t v = channel_mute[ch] ? 1 : 0;
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams, legacy.mute) + ch),
                    1, &v);
            }
            break;
        }

        case REQ_SET_LOUDNESS:
            if (buffer->data_len >= 1) {
                loudness_enabled = (vendor_rx_buf[0] != 0);
                if (loudness_enabled && loudness_active_table) {
                    // Re-select coefficients for the *active* volume.  Reading
                    // effective_vol_index (set in lock-step with vol_mul by
                    // apply_vol_index_to_audio) ensures the coefficient table
                    // matches whatever currently drives vol_mul — the USB host
                    // slider, LG Sound Sync, or any future volume owner —
                    // rather than the USB-cached audio_state.volume which is
                    // frozen during SPDIF playback.
                    uint8_t idx = effective_vol_index;
                    if (idx > CENTER_VOLUME_INDEX) idx = CENTER_VOLUME_INDEX;
                    current_loudness_coeffs = loudness_active_table[idx];
                } else {
                    current_loudness_coeffs = NULL;
                }
                uint8_t v = loudness_enabled ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, global.loudness_enabled), 1, &v);
            }
            break;

        case REQ_SET_LOUDNESS_REF:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 40.0f) val = 40.0f;
                if (val > 100.0f) val = 100.0f;
                loudness_ref_spl = val;
                loudness_recompute_pending = true;
                notify_param_write(offsetof(WireBulkParams, global.loudness_ref_spl),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_LOUDNESS_INTENSITY:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < 0.0f) val = 0.0f;
                if (val > 200.0f) val = 200.0f;
                loudness_intensity_pct = val;
                loudness_recompute_pending = true;
                notify_param_write(offsetof(WireBulkParams, global.loudness_intensity_pct),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_CROSSFEED:
            if (buffer->data_len >= 1) {
                crossfeed_config.enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
                uint8_t v = crossfeed_config.enabled ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, crossfeed.enabled), 1, &v);
            }
            break;

        case REQ_SET_CROSSFEED_PRESET:
            if (buffer->data_len >= 1) {
                uint8_t preset = vendor_rx_buf[0];
                if (preset <= CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_config.preset = preset;
                    crossfeed_update_pending = true;
                    notify_param_write(offsetof(WireBulkParams, crossfeed.preset), 1, &preset);
                }
            }
            break;

        case REQ_SET_CROSSFEED_FREQ:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FREQ_MIN) val = CROSSFEED_FREQ_MIN;
                if (val > CROSSFEED_FREQ_MAX) val = CROSSFEED_FREQ_MAX;
                crossfeed_config.custom_fc = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
                notify_param_write(offsetof(WireBulkParams, crossfeed.custom_fc),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_CROSSFEED_FEED:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < CROSSFEED_FEED_MIN) val = CROSSFEED_FEED_MIN;
                if (val > CROSSFEED_FEED_MAX) val = CROSSFEED_FEED_MAX;
                crossfeed_config.custom_feed_db = val;
                if (crossfeed_config.preset == CROSSFEED_PRESET_CUSTOM) {
                    crossfeed_update_pending = true;
                }
                notify_param_write(offsetof(WireBulkParams, crossfeed.custom_feed_db),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_CROSSFEED_ITD:
            if (buffer->data_len >= 1) {
                crossfeed_config.itd_enabled = (vendor_rx_buf[0] != 0);
                crossfeed_update_pending = true;
                uint8_t v = crossfeed_config.itd_enabled ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, crossfeed.itd_enabled), 1, &v);
            }
            break;

        // Volume Leveller Commands
        case REQ_SET_LEVELLER_ENABLE:
            if (buffer->data_len >= 1) {
                leveller_config.enabled = (vendor_rx_buf[0] != 0);
                leveller_update_pending = true;
                leveller_reset_pending = true;  // Reset state when toggling
                uint8_t v = leveller_config.enabled ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, leveller.enabled), 1, &v);
            }
            break;

        case REQ_SET_LEVELLER_AMOUNT:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_AMOUNT_MIN) val = LEVELLER_AMOUNT_MIN;
                if (val > LEVELLER_AMOUNT_MAX) val = LEVELLER_AMOUNT_MAX;
                leveller_config.amount = val;
                leveller_update_pending = true;
                notify_param_write(offsetof(WireBulkParams, leveller.amount),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_LEVELLER_SPEED:
            if (buffer->data_len >= 1) {
                uint8_t spd = vendor_rx_buf[0];
                if (spd < LEVELLER_SPEED_COUNT) {
                    leveller_config.speed = spd;
                    leveller_update_pending = true;
                    notify_param_write(offsetof(WireBulkParams, leveller.speed), 1, &spd);
                }
            }
            break;

        case REQ_SET_LEVELLER_MAX_GAIN:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_MAX_GAIN_MIN) val = LEVELLER_MAX_GAIN_MIN;
                if (val > LEVELLER_MAX_GAIN_MAX) val = LEVELLER_MAX_GAIN_MAX;
                leveller_config.max_gain_db = val;
                leveller_update_pending = true;
                notify_param_write(offsetof(WireBulkParams, leveller.max_gain_db),
                                   sizeof(float), &val);
            }
            break;

        case REQ_SET_LEVELLER_LOOKAHEAD:
            if (buffer->data_len >= 1) {
                leveller_config.lookahead = (vendor_rx_buf[0] != 0);
                leveller_update_pending = true;
                leveller_reset_pending = true;  // Clear delay buffer on toggle
                uint8_t v = leveller_config.lookahead ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, leveller.lookahead), 1, &v);
            }
            break;

        case REQ_SET_LEVELLER_GATE:
            if (buffer->data_len >= 4) {
                float val;
                memcpy(&val, vendor_rx_buf, 4);
                if (val < LEVELLER_GATE_MIN) val = LEVELLER_GATE_MIN;
                if (val > LEVELLER_GATE_MAX) val = LEVELLER_GATE_MAX;
                leveller_config.gate_threshold_db = val;
                leveller_update_pending = true;
                notify_param_write(offsetof(WireBulkParams, leveller.gate_threshold_db),
                                   sizeof(float), &val);
            }
            break;

        // Matrix Mixer Commands
        case REQ_SET_MATRIX_ROUTE:
            if (buffer->data_len >= sizeof(MatrixRoutePacket)) {
                MatrixRoutePacket pkt;
                memcpy(&pkt, vendor_rx_buf, sizeof(pkt));
                if (pkt.input < NUM_INPUT_CHANNELS && pkt.output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[pkt.input][pkt.output];
                    xp->enabled = pkt.enabled;
                    xp->phase_invert = pkt.phase_invert;
                    xp->gain_db = pkt.gain_db;
                    // Compute linear gain
                    xp->gain_linear = powf(10.0f, pkt.gain_db / 20.0f);

                    WireCrosspoint wxp;
                    memset(&wxp, 0, sizeof(wxp));
                    wxp.enabled = pkt.enabled ? 1 : 0;
                    wxp.phase_invert = pkt.phase_invert ? 1 : 0;
                    wxp.gain_db = pkt.gain_db;
                    // crosspoints[input][output], row-major (direct, 8 inputs).
                    uint16_t off = (uint16_t)(offsetof(WireBulkParams, crosspoints)
                        + ((uint16_t)pkt.input * WIRE_MAX_OUTPUT_CHANNELS + pkt.output)
                          * sizeof(WireCrosspoint));
                    notify_param_write(off, sizeof(WireCrosspoint), &wxp);
                }
            }
            break;

        case REQ_SET_OUTPUT_ENABLE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                bool want_enable = (vendor_rx_buf[0] != 0);

                // Mutual exclusion interlock: PDM vs EQ worker outputs
                // Core 1 can only do one: PDM or EQ worker (outputs 2+ on both platforms)
                if (want_enable) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);

                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) goto skip_enable;
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) goto skip_enable;
                    }
                }

                matrix_mixer.outputs[out].enabled = want_enable ? 1 : 0;

                // Determine new Core 1 mode and transition
                Core1Mode new_mode = derive_core1_mode();
                if (new_mode != core1_mode) {
                    core1_mode = new_mode;
#if ENABLE_SUB
                    pdm_set_enabled(new_mode == CORE1_MODE_PDM);
#endif
                    __sev();  // Wake Core 1 to pick up mode change
                }

                uint8_t v = matrix_mixer.outputs[out].enabled ? 1 : 0;
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, outputs)
                    + (uint16_t)out * sizeof(WireOutputChannel)
                    + offsetof(WireOutputChannel, enabled));
                notify_param_write(off, 1, &v);
            }
            skip_enable:
            break;
        }

        case REQ_SET_OUTPUT_GAIN: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float db;
                memcpy(&db, vendor_rx_buf, 4);
                matrix_mixer.outputs[out].gain_db = db;
                matrix_mixer.outputs[out].gain_linear = powf(10.0f, db / 20.0f);
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, outputs)
                    + (uint16_t)out * sizeof(WireOutputChannel)
                    + offsetof(WireOutputChannel, gain_db));
                notify_param_write(off, sizeof(float), &db);
            }
            break;
        }

        case REQ_SET_OUTPUT_MUTE: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 1) {
                matrix_mixer.outputs[out].mute = vendor_rx_buf[0];
                uint8_t v = vendor_rx_buf[0];
                uint16_t off = (uint16_t)(offsetof(WireBulkParams, outputs)
                    + (uint16_t)out * sizeof(WireOutputChannel)
                    + offsetof(WireOutputChannel, mute));
                notify_param_write(off, 1, &v);
            }
            break;
        }

        case REQ_SET_OUTPUT_DELAY: {
            uint8_t out = vendor_last_wValue & 0xFF;
            if (out < NUM_OUTPUT_CHANNELS && buffer->data_len >= 4) {
                float ms;
                memcpy(&ms, vendor_rx_buf, 4);
                if (ms < 0) ms = 0;
                matrix_mixer.outputs[out].delay_ms = ms;
                // Update the channel delay used by DSP pipeline
                channel_delays_ms[CH_OUT_1 + out] = ms;
                dsp_update_delay_samples((float)audio_state.freq);

                uint16_t off = (uint16_t)(offsetof(WireBulkParams, outputs)
                    + (uint16_t)out * sizeof(WireOutputChannel)
                    + offsetof(WireOutputChannel, delay_ms));
                notify_param_write(off, sizeof(float), &ms);
                // Also update the per-channel delay shadow for CH_OUT_1+out.
                notify_param_write(
                    (uint16_t)(offsetof(WireBulkParams, delays.delay_ms)
                               + (CH_OUT_1 + out) * sizeof(float)),
                    sizeof(float), &ms);
            }
            break;
        }

        // --- Preset SET commands ---

        case REQ_PRESET_SET_NAME: {
            // Deferred to main loop — flash write in dir_flush() is too
            // slow for USB IRQ context.  Copy payload to pending buffer.
            uint8_t slot = vendor_last_wValue & 0xFF;
            if (buffer->data_len > 0) {
                memset(flash_set_name_buf, 0, PRESET_NAME_LEN);
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(flash_set_name_buf, vendor_rx_buf, copy_len);
                flash_set_name_slot = slot;
                __dmb();
                flash_set_name_pending = true;
            }
            break;
        }

        case REQ_PRESET_SET_STARTUP: {
            // Deferred to main loop — flash write in dir_flush().
            if (buffer->data_len >= 2) {
                flash_set_startup_mode = vendor_rx_buf[0];
                flash_set_startup_slot = vendor_rx_buf[1];
                __dmb();
                flash_set_startup_pending = true;
            }
            break;
        }

        case REQ_SET_OUTPUT_CONFIG_MODE: {
            // Set physical IO/output-config persistence mode (0 = independent,
            // 1 = with-preset).  Deferred to main loop — flash write in dir_flush().
            if (buffer->data_len >= 1) {
                uint8_t m = vendor_rx_buf[0];
                if (m > OUTPUT_CONFIG_MODE_WITH_PRESET) m = OUTPUT_CONFIG_MODE_INDEPENDENT;
                flash_set_output_config_mode_val = m;
                __dmb();
                flash_set_output_config_mode_pending = true;
            }
            break;
        }

        case REQ_SET_MASTER_VOLUME_MODE: {
            // Set master-volume persistence mode (0 = independent, 1 = per-preset).
            // Deferred to main loop — flash write in dir_flush().
            if (buffer->data_len >= 1) {
                uint8_t m = vendor_rx_buf[0];
                if (m > MASTER_VOLUME_MODE_WITH_PRESET) m = MASTER_VOLUME_MODE_INDEPENDENT;
                flash_set_master_volume_mode_val = m;
                __dmb();
                flash_set_master_volume_mode_pending = true;
            }
            break;
        }

        case REQ_SET_DAC_HW_MUTE_CONFIG: {
            // Payload: 16-byte DacHwMuteConfig.  Deferred to main loop —
            // validation, pin claim, flash write, and notify all happen
            // in dac_hw_mute_set_config() which can run for tens of ms
            // (flash erase + program).  Validation failures are silently
            // swallowed by the deferred handler — the immediate vendor
            // response cannot wait for it.  Hosts that need a definitive
            // success/failure should follow up with REQ_GET_DAC_HW_MUTE_CONFIG
            // and compare against what they sent.
            if (buffer->data_len >= sizeof(DacHwMuteConfig)) {
                extern volatile bool flash_set_dac_hw_mute_pending;
                extern DacHwMuteConfig flash_set_dac_hw_mute_val;
                memcpy((void *)&flash_set_dac_hw_mute_val,
                       vendor_rx_buf, sizeof(DacHwMuteConfig));
                __dmb();
                flash_set_dac_hw_mute_pending = true;
            }
            break;
        }

        case REQ_SET_UART_CONFIG: {
            // USB-only (vendor_dispatch_set refuses it on UART/I2C).
            // Deferred to main loop: apply does GPIO/IRQ work and the
            // persist is a ~45 ms flash write.  Result lands in
            // ctrl_uart_last_status, readable via REQ_GET_CTRL_IFACE_STATUS.
            if (buffer->data_len >= sizeof(UartCtrlConfig)) {
                memcpy((void *)&ctrl_set_uart_val, vendor_rx_buf,
                       sizeof(UartCtrlConfig));
                __dmb();
                ctrl_set_uart_pending = true;
            }
            break;
        }

        case REQ_SET_I2C_CONFIG: {
            // USB-only; same deferred apply+persist shape as UART above.
            if (buffer->data_len >= sizeof(I2cCtrlConfig)) {
                memcpy((void *)&ctrl_set_i2c_val, vendor_rx_buf,
                       sizeof(I2cCtrlConfig));
                __dmb();
                ctrl_set_i2c_pending = true;
            }
            break;
        }

        case REQ_SET_CS_BINDING: {
            // Deferred to main loop: apply does GPIO/ADC work and the
            // persist is a directory flash write.  Result lands in
            // cs_last_status, readable via REQ_GET_CS_STATUS.
            uint8_t slot = vendor_last_wValue & 0xFF;
            if (slot >= CS_MAX_BINDINGS) {
                cs_last_status = CS_STATUS_INVALID_SLOT;
                cs_last_slot = slot;
            } else if (buffer->data_len >= sizeof(CsBinding)) {
                memcpy((void *)&cs_set_binding_val, vendor_rx_buf, sizeof(CsBinding));
                cs_set_binding_slot = slot;
                // "Accepted, not yet applied"; the main-loop apply
                // overwrites this with the real result.
                cs_last_status = CS_STATUS_PENDING;
                cs_last_slot = slot;
                __dmb();
                cs_set_binding_pending = true;
            }
            break;
        }

        case REQ_SET_CHANNEL_NAME: {
            // wValue = channel index, payload = 1-32 bytes of name
            uint8_t ch = vendor_last_wValue & 0xFF;
            if (ch < NUM_CHANNELS && buffer->data_len > 0) {
                memset(channel_names[ch], 0, PRESET_NAME_LEN);
                size_t copy_len = buffer->data_len < (PRESET_NAME_LEN - 1)
                                ? buffer->data_len : (PRESET_NAME_LEN - 1);
                memcpy(channel_names[ch], vendor_rx_buf, copy_len);

                uint16_t off = (uint16_t)(offsetof(WireBulkParams, channel_names.names)
                                          + (uint16_t)ch * WIRE_NAME_LEN);
                notify_param_write(off, WIRE_NAME_LEN, channel_names[ch]);
            }
            break;
        }

        case REQ_SET_INPUT_SOURCE: {
            // Payload: 1 byte = InputSource enum value.
            // Deferred to main loop — actual source switch requires
            // pipeline reset and (Phase 2) hardware start/stop.
            // Notification is emitted at apply time (main.c), not here —
            // the shadow tracks *active* state, not requested state.
            if (buffer->data_len >= 1) {
                uint8_t src = vendor_rx_buf[0];
                if (input_source_valid(src) && src != active_input_source) {
                    pending_input_source = src;
                    __dmb();
                    input_source_change_pending = true;
                }
            }
            break;
        }

        case REQ_SET_INPUT_RATE: {
            // Payload: uint32_t Hz (44100 / 48000 / 96000). The device is
            // the rate authority in I2S input mode, so the selection is
            // stored always and applied immediately (deferred rate change,
            // same mechanism as spdif_input_check_rate_change) when I2S is
            // the active input.
            if (buffer->data_len >= 4) {
                uint32_t rate;
                memcpy(&rate, vendor_rx_buf, 4);
                if (rate == 44100 || rate == 48000 || rate == 96000) {
                    i2s_input_rate = rate;
                    uint8_t enc = i2s_rate_encode(rate);
                    notify_param_write(offsetof(WireBulkParams,
                                                input_config.i2s_input_rate),
                                       1, &enc);
                    if (active_input_source == INPUT_SOURCE_I2S &&
                        rate != audio_state.freq) {
                        pending_rate = rate;
                        __dmb();
                        rate_change_pending = true;
                    }
                }
            }
            break;
        }

        case REQ_SET_LG_SOUND_SYNC_ENABLE: {
            // Payload: 1 byte (0 = off, anything else = on).
            // The setter handles its own PARAM_CHANGED emit and the
            // demote/restore side-effects on enabled→disabled.  No
            // flash write here — flash persistence happens on
            // REQ_SAVE_PRESET (matches the loudness/crossfeed/leveller
            // toggle pattern).
            if (buffer->data_len >= 1) {
                lg_sound_sync_set_enabled(vendor_rx_buf[0] != 0);
            }
            break;
        }

        case REQ_SIGGEN_SET_CONFIG:
            // siggen_stage_config validates type, mask and params; STALL on
            // reject so external transports report ERROR, not a false OK.
            if (!siggen_stage_config(vendor_rx_buf, buffer->data_len)) {
                handled = false;
            }
            break;

        default:
            // No SET case for this bRequest (unknown command, or a
            // wValue-only SET that is dispatched through the GET path).
            handled = false;
            break;
    }

    // Clear the source tag so any writes that happen outside a dispatch
    // bracket (e.g. Core 1 update paths, timer callbacks) aren't falsely
    // attributed to the host.
    notify_set_source(PARAM_SRC_UNKNOWN);

    // TinyUSB auto-sends the status-stage ZLP after control_xfer_cb returns
    // true from CONTROL_STAGE_DATA; no explicit empty-IN transfer needed.
    return handled;
}

// ----------------------------------------------------------------------------
// VENDOR RESPONSE HELPER
// ----------------------------------------------------------------------------

// Send a control IN response from the current vendor GET handler.  For USB
// this completes the EP0 transfer.  For an external transport the bytes are
// consumed by the caller only AFTER the handler returns, so stack-local
// response buffers would be dangling by then; non-bulk responses are
// therefore copied into a static sink here, while the handler frame is
// still alive.  Only bulk_param_buf (serialized by the bulk owner lock)
// stays zero-copy.  USB tolerates stack buffers up to 64 bytes because
// tud_control_xfer copies one EP0 transaction's worth synchronously;
// anything larger must be static (today only the bulk buffer is).
static uint8_t _ext_resp_copy[64];
static void vendor_send_response(const void *data, uint16_t len) {
    if (_active_source == CTRL_SOURCE_USB) {
        tud_control_xfer(_vendor_rhport, _vendor_current_req, (void *)data, len);
    } else if (data == bulk_param_buf) {
        _ext_resp_data = bulk_param_buf;
        _ext_resp_len  = len;
    } else {
        if (len > sizeof(_ext_resp_copy)) len = sizeof(_ext_resp_copy);
        memcpy(_ext_resp_copy, data, len);
        _ext_resp_data = _ext_resp_copy;
        _ext_resp_len  = len;
    }
}

// Legacy compatibility shim for the pico-extras helper that sent a small
// integer value as a control IN response in one call.  Several handlers use
// it for scalar responses (status/enum/flag values).  Buffer must outlive
// the EP0 transfer — static storage is fine since control xfers are
// serialized by TinyUSB.
static uint32_t _vendor_scalar_resp;
static inline void usb_start_tiny_control_in_transfer(uint32_t val, uint16_t len) {
    _vendor_scalar_resp = val;
    vendor_send_response(&_vendor_scalar_resp, len);
}

// ----------------------------------------------------------------------------
// VENDOR GET DISPATCH
// ----------------------------------------------------------------------------

static bool vendor_handle_get(tusb_control_request_t const *req) {
    // Shim to let legacy case bodies reference `setup->...`.
    tusb_control_request_t const *setup = req;
    (void)setup;

    // Some "SET" commands are dispatched through vendor_handle_get because
    // they carry all parameters in wValue and have no DATA stage (e.g.
    // REQ_SET_OUTPUT_TYPE, REQ_SET_MCK_*).  Tag them with the entry point's
    // origin so their param_write calls get the correct source.
    notify_set_source(_dispatch_src);

    {
        // Device -> Host (GET requests)
        static uint8_t resp_buf[64];

        switch (setup->bRequest) {
            case REQ_GET_PREAMP: {
                // Legacy: returns channel 0's preamp value (backward compat)
                float current_db = global_preamp_db[0];
                memcpy(resp_buf, &current_db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_PREAMP_CH: {
                // Per-channel preamp GET.  wValue = input channel index.
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < NUM_INPUT_CHANNELS) {
                    float current_db = global_preamp_db[ch];
                    memcpy(resp_buf, &current_db, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_MASTER_VOLUME: {
                // Returns device master volume in dB (-128 = mute, -127..0 range)
                float db = master_volume_db;
                memcpy(resp_buf, &db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_USER_VOLUME: {
                // Returns the user-perceived volume (same field UAC1 reports
                // via GET_CUR) as a float dB.  audio_state.volume is in 8.8
                // fixed-point dB; divide by 256 to recover the real value.
                // Range: [-CENTER_VOLUME_INDEX, 0] dB.
                float db = (float)audio_state.volume / 256.0f;
                memcpy(resp_buf, &db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_USER_MUTE: {
                // Returns the vendor-channel user_mute flag (NOT
                // audio_state.mute — that's UAC1's, queryable via UAC1
                // GET_CUR).  The two have different gating semantics; surface
                // each via its own native interface.
                resp_buf[0] = user_mute ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_DELAY: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < NUM_CHANNELS) {
                    memcpy(resp_buf, (void*)&channel_delays_ms[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_BYPASS: {
                resp_buf[0] = bypass_master_eq ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_GAIN: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    memcpy(resp_buf, (void*)&channel_gain_db[ch], 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CHANNEL_MUTE: {
                uint8_t ch = (uint8_t)setup->wValue;
                if (ch < 3) {
                    resp_buf[0] = channel_mute[ch] ? 1 : 0;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_LOUDNESS: {
                resp_buf[0] = loudness_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LOUDNESS_REF: {
                float val = loudness_ref_spl;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LOUDNESS_INTENSITY: {
                float val = loudness_intensity_pct;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED: {
                resp_buf[0] = crossfeed_config.enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_PRESET: {
                resp_buf[0] = crossfeed_config.preset;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CROSSFEED_FREQ: {
                float val = crossfeed_config.custom_fc;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_FEED: {
                float val = crossfeed_config.custom_feed_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_CROSSFEED_ITD: {
                resp_buf[0] = crossfeed_config.itd_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // Volume Leveller GET commands
            case REQ_GET_LEVELLER_ENABLE: {
                resp_buf[0] = leveller_config.enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_AMOUNT: {
                float val = leveller_config.amount;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LEVELLER_SPEED: {
                resp_buf[0] = leveller_config.speed;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_MAX_GAIN: {
                float val = leveller_config.max_gain_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_LEVELLER_LOOKAHEAD: {
                resp_buf[0] = leveller_config.lookahead ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LEVELLER_GATE: {
                float val = leveller_config.gate_threshold_db;
                memcpy(resp_buf, &val, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_GET_STATUS: {
                if (setup->wValue == 9) {
                    // Combined status: all peaks + CPU load + clip flags (32-bit)
                    // + live active-input-channel count.  Layout:
                    //   peaks[NUM_CHANNELS]×2, cpu0, cpu1, clip_flags(4),
                    //   active_input_channels(1).
                    // RP2350: 17×2 + 2 + 4 + 1 = 41 B.  RP2040: 7×2 + 7 = 21 B.
                    for (int i = 0; i < NUM_CHANNELS; i++) {
                        resp_buf[i*2]     = global_status.peaks[i] & 0xFF;
                        resp_buf[i*2 + 1] = global_status.peaks[i] >> 8;
                    }
                    int off = NUM_CHANNELS * 2;
                    resp_buf[off + 0] = global_status.cpu0_load;
                    resp_buf[off + 1] = global_status.cpu1_load;
                    resp_buf[off + 2] = (uint8_t)(global_status.clip_flags & 0xFF);
                    resp_buf[off + 3] = (uint8_t)((global_status.clip_flags >> 8) & 0xFF);
                    resp_buf[off + 4] = (uint8_t)((global_status.clip_flags >> 16) & 0xFF);
                    resp_buf[off + 5] = (uint8_t)((global_status.clip_flags >> 24) & 0xFF);
                    resp_buf[off + 6] = active_input_channel_count();   // live active input count (source-aware: USB/I2S/SPDIF)
                    vendor_send_response(resp_buf, off + 7);
                    return true;
                }

                uint32_t resp = 0;
                switch (setup->wValue) {
                    // Legacy packed-peak registers: raw channel-indexed peaks[0..4].
                    // Under the unified channel model these low indices are INPUT
                    // channels (0/1 = stereo bus; 2..4 = extra inputs on RP2350),
                    // not outputs.  Use wValue==9 (combined status) for the full,
                    // unambiguous per-channel peak array — that is the canonical view.
                    case 0: resp = (uint32_t)global_status.peaks[0] | ((uint32_t)global_status.peaks[1] << 16); break;
                    case 1: resp = (uint32_t)global_status.peaks[2] | ((uint32_t)global_status.peaks[3] << 16); break;
                    case 2: resp = (uint32_t)global_status.peaks[4] | ((uint32_t)global_status.cpu0_load << 16) | ((uint32_t)global_status.cpu1_load << 24); break;
                    case 3: resp = pdm_ring_overruns; break;
                    case 4: resp = pdm_ring_underruns; break;
                    case 5: resp = pdm_dma_overruns; break;
                    case 6: resp = pdm_dma_underruns; break;
                    case 7: resp = spdif_overruns; break;
                    case 8: resp = spdif_underruns; break;
                    case 10: resp = usb_audio_packets; break;
                    case 11: resp = usb_audio_alt_set; break;
                    case 12: resp = usb_audio_mounted; break;
                    case 13: resp = clock_get_hz(clk_sys); break;  // System clock frequency in Hz
                    case 14: resp = vreg_voltage_to_mv(vreg_get_voltage()); break;  // Core voltage in mV
                    case 15: resp = audio_state.freq; break;  // Sample rate in Hz
                    case 16: resp = (uint32_t)read_temperature_cdeg(); break;  // Temperature in centi-degrees C
                    case 17: resp = audio_spdif_get_dma_starvations(); break;  // Total SPDIF DMA starvations
                    case 18: resp = audio_spdif_get_dma_starvations_instance(0); break;  // SPDIF instance 0
                    case 19: resp = audio_spdif_get_dma_starvations_instance(1); break;  // SPDIF instance 1
                    case 20: resp = audio_spdif_get_dma_starvations_instance(2); break;  // SPDIF instance 2
                    case 21: resp = audio_spdif_get_dma_starvations_instance(3); break;  // SPDIF instance 3
                    case 22: resp = usb_audio_ring_overrun_count(); break;  // USB audio ring overruns
                    case 23: resp = active_input_channel_count(); break;  // live active input count (source-aware: USB/I2S/SPDIF)
                }
                usb_start_tiny_control_in_transfer(resp, 4);
                return true;
            }

            case REQ_SAVE_PARAMS: {
                // Legacy command retained for compatibility, but deferred to
                // main loop to avoid flash write blackout in IRQ context.
                save_params_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(FLASH_OK, 1);  // Accepted
                return true;
            }

            case REQ_SAVE_OUTPUT_CONFIG: {
                // Action-style command (repurposed dead 0x52): persist the live
                // physical IO config into the directory's device-global block —
                // the explicit "save output config" for INDEPENDENT mode.
                // Mirrors REQ_SAVE_MASTER_VOLUME: 1-byte status response, flash
                // write deferred to the main loop.
                flash_save_output_config_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(PRESET_OK, 1);
                return true;
            }

            case REQ_FACTORY_RESET: {
                // Deferred to main loop — same pipeline protection as preset
                // load/save/delete: mute, Core 1 sync, delay line zero, and
                // output type switch if the live config had I2S outputs.
                factory_reset_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(FLASH_OK, 1);
                return true;
            }

            case REQ_SAVE_MASTER_VOLUME: {
                // Action-style command: persist current live master volume into
                // the directory's independent field.  Handled in the IN switch
                // (matching REQ_FACTORY_RESET) so the host can issue a zero- or
                // 1-byte-IN control transfer; responds with a 1-byte status so
                // the transfer is shaped like other action commands.  The flash
                // write itself is deferred to the main loop.
                flash_save_master_volume_pending = true;
                __dmb();
                usb_start_tiny_control_in_transfer(PRESET_OK, 1);
                return true;
            }

            case REQ_GET_BAND_BYPASS: {
                // wValue = (channel << 8) | band; returns 1 byte (0 or 1).
                // PEQ bands 0..channel_band_counts-1 and crossover bands
                // XOVER_BAND_BASE..XOVER_BAND_BASE+MAX_XOVER_BANDS-1 both supported.
                uint8_t channel = (setup->wValue >> 8) & 0xFF;
                uint8_t band = setup->wValue & 0xFF;
                if (channel < NUM_CHANNELS) {
                    uint8_t v;
                    bool ok = false;
                    if (band < channel_band_counts[channel]) {
                        v = (filter_recipes[channel][band].bypass == 1) ? 1 : 0;
                        ok = true;
                    } else if (band >= XOVER_BAND_BASE && band < (XOVER_BAND_BASE + MAX_XOVER_BANDS)
                               && channel >= CH_OUT_1) {
                        v = (xover_recipes[channel][band - XOVER_BAND_BASE].bypass == 1) ? 1 : 0;
                        ok = true;
                    }
                    if (ok) {
                        usb_start_tiny_control_in_transfer(v, 1);
                        return true;
                    }
                }
                return false;
            }

            case REQ_GET_EQ_PARAM: {
                // wValue: bits[15:8]=channel, bits[7:3]=band (5 bits, 0..31),
                //         bits[2:0]=param (0=type, 1=freq, 2=Q, 3=gain_db,
                //         4=bypass).  Returns 4 bytes regardless of which
                //         scalar; the unused bytes are zeroed.
                //
                // The band field is 5 bits (not the original 4) so crossover
                // bands at XOVER_BAND_BASE..+3 (= 20..23) remain addressable
                // after the reserved gap was widened.  Hosts must build
                // wValue as (channel<<8) | (band<<3) | param.
                uint8_t channel = (setup->wValue >> 8) & 0xFF;
                uint8_t band = (setup->wValue >> 3) & 0x1F;
                uint8_t param = setup->wValue & 0x07;
                if (channel < NUM_CHANNELS) {
                    EqParamPacket *p = NULL;
                    if (band < channel_band_counts[channel]) {
                        p = &filter_recipes[channel][band];
                    } else if (band >= XOVER_BAND_BASE && band < (XOVER_BAND_BASE + MAX_XOVER_BANDS)
                               && channel >= CH_OUT_1) {
                        p = &xover_recipes[channel][band - XOVER_BAND_BASE];
                    }
                    if (p) {
                        uint32_t val_to_send = 0;
                        switch (param) {
                            case 0: val_to_send = (uint32_t)p->type; break;
                            case 1: memcpy(&val_to_send, &p->freq, 4); break;
                            case 2: memcpy(&val_to_send, &p->Q, 4); break;
                            case 3: memcpy(&val_to_send, &p->gain_db, 4); break;
                            case 4: val_to_send = (p->bypass == 1) ? 1u : 0u; break;
                        }
                        usb_start_tiny_control_in_transfer(val_to_send, 4);
                        return true;
                    }
                }
                return false;
            }

            // Matrix Mixer GET commands
            case REQ_GET_MATRIX_ROUTE: {
                // wValue = (input << 8) | output
                uint8_t input = (setup->wValue >> 8) & 0xFF;
                uint8_t output = setup->wValue & 0xFF;
                if (input < NUM_INPUT_CHANNELS && output < NUM_OUTPUT_CHANNELS) {
                    MatrixCrosspoint *xp = &matrix_mixer.crosspoints[input][output];
                    MatrixRoutePacket pkt = {
                        .input = input,
                        .output = output,
                        .enabled = xp->enabled,
                        .phase_invert = xp->phase_invert,
                        .gain_db = xp->gain_db
                    };
                    memcpy(resp_buf, &pkt, sizeof(pkt));
                    vendor_send_response(resp_buf, sizeof(pkt));
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_ENABLE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].enabled;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_GAIN: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].gain_db, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_MUTE: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    resp_buf[0] = matrix_mixer.outputs[out].mute;
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_OUTPUT_DELAY: {
                uint8_t out = (uint8_t)setup->wValue;
                if (out < NUM_OUTPUT_CHANNELS) {
                    memcpy(resp_buf, &matrix_mixer.outputs[out].delay_ms, 4);
                    vendor_send_response(resp_buf, 4);
                    return true;
                }
                return false;
            }

            case REQ_GET_CORE1_MODE: {
                resp_buf[0] = (uint8_t)core1_mode;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CORE1_CONFLICT: {
                // wValue = proposed output index to enable
                // Returns 1 if enabling it would conflict, 0 if OK
                uint8_t out = (uint8_t)setup->wValue;
                uint8_t conflict = 0;
                if (out < NUM_OUTPUT_CHANNELS) {
                    bool is_pdm = (out == NUM_OUTPUT_CHANNELS - 1);
                    bool is_core1_eq = (out >= CORE1_EQ_FIRST_OUTPUT && out <= CORE1_EQ_LAST_OUTPUT);
                    if (is_pdm) {
                        for (int i = CORE1_EQ_FIRST_OUTPUT; i <= CORE1_EQ_LAST_OUTPUT; i++) {
                            if (matrix_mixer.outputs[i].enabled) { conflict = 1; break; }
                        }
                    } else if (is_core1_eq) {
                        if (matrix_mixer.outputs[NUM_OUTPUT_CHANNELS - 1].enabled) conflict = 1;
                    }
                }
                resp_buf[0] = conflict;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_OUTPUT_PIN: {
                // wValue = (new_pin << 8) | output_index
                uint8_t out_idx = setup->wValue & 0xFF;
                uint8_t new_pin = (setup->wValue >> 8) & 0xFF;
                uint8_t status;

                if (out_idx >= NUM_PIN_OUTPUTS) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (is_pin_in_use(new_pin, out_idx)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else if (new_pin == output_pins[out_idx]) {
                    // No-op: pin unchanged
                    status = PIN_CONFIG_SUCCESS;
                } else if (out_idx < NUM_SPDIF_INSTANCES) {
                    // SPDIF/I2S slot: record the target pin (RAM-only, like
                    // spdif_rx_pin) and flag the slot.  process_pin_changes() in
                    // the main loop applies it through a muted, synchronized
                    // pipeline reset, so the moved slot restarts in phase with
                    // the others — a live in-ISR restart would not.
                    extern volatile uint8_t output_pin_change_mask;
                    output_pins[out_idx] = new_pin;
                    output_pin_change_mask |= (1u << out_idx);
                    status = PIN_CONFIG_SUCCESS;
                } else {
                    // PDM output (out_idx == 4): must be disabled first
                    if (pdm_enabled || core1_mode == CORE1_MODE_PDM) {
                        status = PIN_CONFIG_OUTPUT_ACTIVE;
                    } else {
                        pdm_change_pin(new_pin);
                        output_pins[out_idx] = new_pin;
                        status = PIN_CONFIG_SUCCESS;
                    }
                }

                if (status == PIN_CONFIG_SUCCESS && out_idx < NUM_PIN_OUTPUTS) {
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, pins.pins) + out_idx),
                        1, &output_pins[out_idx]);
                }

                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_OUTPUT_PIN: {
                uint8_t out_idx = (uint8_t)setup->wValue;
                if (out_idx < NUM_PIN_OUTPUTS) {
                    resp_buf[0] = output_pins[out_idx];
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_GET_SERIAL: {
                memcpy(resp_buf, usb_descriptor_str_serial, 16);
                vendor_send_response(resp_buf, 16);
                return true;
            }

            case REQ_GET_PLATFORM: {
                resp_buf[0] = PLATFORM_RP2040;
#if PICO_RP2350
                resp_buf[0] = PLATFORM_RP2350;
#endif
                resp_buf[1] = (uint8_t)(FW_VERSION_BCD >> 8);    // major
                resp_buf[2] = (uint8_t)(FW_VERSION_BCD & 0xFF);  // minor.patch BCD
                resp_buf[3] = NUM_OUTPUT_CHANNELS;
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_CLEAR_CLIPS: {
                // Read-then-clear: return the 32-bit clip flags that were set, then reset
                uint32_t flags = global_status.clip_flags;
                global_status.clip_flags = 0;
                usb_start_tiny_control_in_transfer(flags, 4);
                return true;
            }

            // --- Preset Commands ---

            case REQ_PRESET_SAVE: {
                // Deferred to main loop: flash writes must not run in IRQ
                // context (45ms interrupt blackout per sector).  The main loop
                // brackets save with prepare/complete_pipeline_reset() so audio
                // resumes from a fully resynced output pipeline after blackout.
                // Response is fire-and-forget: "accepted".
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    pending_preset_save_slot = slot;
                    preset_save_pending = true;
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_LOAD: {
                // Deferred to main loop so that:
                //  - Flash reads and dir_flush don't run in IRQ context
                //  - The operation is bracketed with prepare/complete_pipeline_reset
                //    to drain stale consumer buffers and resync outputs
                //  - Delay lines are zeroed to prevent old audio bleed-through
                // Response is fire-and-forget: "accepted".  The host can poll
                // REQ_PRESET_GET_ACTIVE to confirm the load completed.
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    pending_preset_load_slot = slot;
                    preset_load_pending = true;
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_DELETE: {
                // Deferred to main loop: flash erase runs with interrupts
                // disabled for ~45ms.  If the deleted slot is the active slot,
                // factory defaults are applied — which needs pipeline reset
                // to flush stale buffers processed with the old parameters.
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot >= PRESET_SLOTS) {
                    resp_buf[0] = PRESET_ERR_INVALID_SLOT;
                } else {
                    preset_delete_mask |= (1u << slot);
                    __dmb();
                    resp_buf[0] = PRESET_OK;
                }
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_PRESET_GET_NAME: {
                // wValue = slot index.  Returns 32-byte NUL-terminated name.
                uint8_t slot = (uint8_t)setup->wValue;
                char name[PRESET_NAME_LEN];
                uint8_t result = preset_get_name(slot, name);
                if (result == PRESET_OK) {
                    memcpy(resp_buf, name, PRESET_NAME_LEN);
                    vendor_send_response(resp_buf, PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_PRESET_GET_DIR: {
                // Returns 7-byte directory summary:
                //   [0-1] slot_occupied bitmask (little-endian u16)
                //   [2]   startup_mode
                //   [3]   default_slot
                //   [4]   last_active_slot
                //   [5]   output_config_mode (0 = independent, 1 = with-preset)
                //   [6]   master_volume_mode (0 = independent, 1 = per-preset)
                uint16_t occupied;
                uint8_t startup, def_slot, last_active, oc_mode, mv_mode;
                preset_get_directory(&occupied, &startup, &def_slot,
                                     &last_active, &oc_mode, &mv_mode);
                resp_buf[0] = occupied & 0xFF;
                resp_buf[1] = occupied >> 8;
                resp_buf[2] = startup;
                resp_buf[3] = def_slot;
                resp_buf[4] = last_active;
                resp_buf[5] = oc_mode;
                resp_buf[6] = mv_mode;
                vendor_send_response(resp_buf, 7);
                return true;
            }

            case REQ_PRESET_GET_STARTUP: {
                // Returns 3 bytes: startup_mode, default_slot, last_active
                uint16_t occupied;
                uint8_t startup, def_slot, last_active, inc_pins, mv_mode;
                preset_get_directory(&occupied, &startup, &def_slot,
                                     &last_active, &inc_pins, &mv_mode);
                resp_buf[0] = startup;
                resp_buf[1] = def_slot;
                resp_buf[2] = last_active;
                vendor_send_response(resp_buf, 3);
                return true;
            }

            case REQ_GET_OUTPUT_CONFIG_MODE: {
                uint16_t occupied;
                uint8_t startup, def_slot, last_active, oc_mode, mv_mode;
                preset_get_directory(&occupied, &startup, &def_slot,
                                     &last_active, &oc_mode, &mv_mode);
                resp_buf[0] = oc_mode;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MASTER_VOLUME_MODE: {
                // Returns master-volume persistence mode (0 or 1).
                uint16_t occupied;
                uint8_t startup, def_slot, last_active, inc_pins, mv_mode;
                preset_get_directory(&occupied, &startup, &def_slot,
                                     &last_active, &inc_pins, &mv_mode);
                resp_buf[0] = mv_mode;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_SAVED_MASTER_VOLUME: {
                // Returns the directory's independent master volume (mode 0 source).
                float db = preset_get_saved_master_volume();
                memcpy(resp_buf, &db, 4);
                vendor_send_response(resp_buf, 4);
                return true;
            }

            case REQ_PRESET_GET_ACTIVE: {
                resp_buf[0] = preset_get_active();
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_CHANNEL_NAME: {
                uint8_t ch = setup->wValue & 0xFF;
                if (ch < NUM_CHANNELS) {
                    vendor_send_response(channel_names[ch], PRESET_NAME_LEN);
                    return true;
                }
                return false;
            }

            case REQ_GET_UART_CONFIG: {
                // Returns the persisted directory config (source of truth);
                // live state and last apply status come from 0xF9.
                UartCtrlConfig cfg;
                preset_get_ctrl_iface(&cfg, NULL);
                memcpy(resp_buf, &cfg, sizeof(cfg));
                vendor_send_response(resp_buf, sizeof(cfg));
                return true;
            }

            case REQ_GET_I2C_CONFIG: {
                I2cCtrlConfig cfg;
                preset_get_ctrl_iface(NULL, &cfg);
                memcpy(resp_buf, &cfg, sizeof(cfg));
                vendor_send_response(resp_buf, sizeof(cfg));
                return true;
            }

            case REQ_GET_CTRL_IFACE_STATUS: {
                CtrlIfaceStatus st = {
                    .uart_last_status = ctrl_uart_last_status,
                    .uart_live        = uart_ctrl_is_live() ? 1 : 0,
                    .i2c_last_status  = ctrl_i2c_last_status,
                    .i2c_live         = i2c_ctrl_is_live() ? 1 : 0,
                    .proto_version    = CTRL_IFACE_PROTO_VERSION,
                    .reserved         = {0},
                };
                memcpy(resp_buf, &st, sizeof(st));
                vendor_send_response(resp_buf, sizeof(st));
                return true;
            }

            case REQ_GET_CS_BINDING: {
                // wValue = slot; returns the live 16-byte binding.
                if (setup->wValue >= CS_MAX_BINDINGS) return false;
                uint8_t slot = (uint8_t)setup->wValue;
                const CsBinding *b = control_surfaces_get_binding(slot);
                if (b == NULL) return false;
                vendor_send_response(b, sizeof(CsBinding));
                return true;
            }

            case REQ_GET_CS_CAPS: {
                // wValue = 0xFFFF: capability header + type table; else the
                // per-noun descriptor.  Accessor pointers are static/const.
                if (setup->wValue == 0xFFFF) {
                    const CsCapsHeader *h = control_surfaces_caps_header();
                    vendor_send_response(h, sizeof(CsCapsHeader));
                    return true;
                }
                if (setup->wValue >= CS_NOUN_COUNT) return false;
                const CsNounDesc *d = control_surfaces_noun_desc((uint8_t)setup->wValue);
                if (d == NULL) return false;
                vendor_send_response(d, sizeof(CsNounDesc));
                return true;
            }

            case REQ_GET_CS_STATUS: {
                // 12-byte snapshot: last SET result + per-slot apply status.
                CsStatusPacket pkt;
                control_surfaces_get_status(&pkt);
                vendor_send_response(&pkt, sizeof(pkt));
                return true;
            }

            case REQ_GET_ALL_PARAMS_CHUNK: {
                // USB-only workaround for the WinUSB 4 KB control cap:
                // wValue = byte offset, wLength = chunk size.  Offset 0
                // acquires the bulk lock and snapshots the full struct so
                // every chunk reads one coherent image; the lock releases
                // at the ACK of the final chunk (or via cleanup/reap if
                // the host abandons the session).
                uint32_t off = setup->wValue;
                uint32_t n   = setup->wLength;
                if (_active_source != CTRL_SOURCE_USB) return false;
                if (n == 0 || off >= sizeof(WireBulkParams)) return false;

                if (off == 0) {
                    if (bulk_buf_owner != BULK_OWNER_USB &&
                        !vendor_bulk_try_acquire(CTRL_SOURCE_USB)) {
                        return false;   // another transport owns the buffer
                    }
                    usb_chunk_set_open = false;   // GET supersedes a stale SET
                    usb_chunk_set_received = 0;
                    bulk_params_collect((WireBulkParams *)bulk_param_buf);
                    usb_chunk_get_open = true;
                    usb_chunk_get_done = false;
                } else if (!usb_chunk_get_open ||
                           bulk_buf_owner != BULK_OWNER_USB) {
                    return false;   // no snapshot (session lost); restart at 0
                }

                if (off + n > sizeof(WireBulkParams))
                    n = sizeof(WireBulkParams) - off;
                usb_chunk_get_done = (off + n >= sizeof(WireBulkParams));
                vendor_bulk_touch(CTRL_SOURCE_USB);
                return tud_control_xfer(_vendor_rhport, _vendor_current_req,
                                        bulk_param_buf + off, (uint16_t)n);
            }

            case REQ_GET_ALL_PARAMS: {
                // Caller (USB SETUP path or vendor_dispatch_get) must hold
                // the bulk lock before this runs; see the entry points.
                bulk_params_collect((WireBulkParams *)bulk_param_buf);
                uint16_t len = sizeof(WireBulkParams);
                if (setup->wLength < len) len = setup->wLength;
                if (_active_source == CTRL_SOURCE_USB) {
                    // Propagate the EP0 queue result so a failed xfer
                    // STALLs and the SETUP path releases the bulk lock.
                    // tud_control_xfer handles EP0 chunking (including the
                    // trailing ZLP on exact-multiple-of-64 transfers).
                    return tud_control_xfer(_vendor_rhport,
                                            _vendor_current_req,
                                            bulk_param_buf, len);
                }
                vendor_send_response(bulk_param_buf, len);
                return true;
            }

            case REQ_GET_BUFFER_STATS: {
                BufferStatsPacket pkt;
                memset(&pkt, 0, sizeof(pkt));
                pkt.num_spdif = NUM_SPDIF_INSTANCES;
                pkt.flags = (pdm_enabled ? 0x01 : 0) | (sync_started ? 0x02 : 0);
                pkt.sequence = buffer_stats_sequence++;

                uint consumer_capacity = SPDIF_CONSUMER_BUFFER_COUNT;

                for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                    uint cons_free, cons_prepared, playing;
                    get_slot_consumer_stats(i, &cons_free, &cons_prepared, &playing);
                    pkt.spdif[i].consumer_free = (uint8_t)cons_free;
                    pkt.spdif[i].consumer_prepared = (uint8_t)cons_prepared;
                    pkt.spdif[i].consumer_playing = (uint8_t)playing;
                    // Fill % is based on total occupied buffers (capacity - free), so it
                    // includes hidden staging buffers (e.g., I2S partial-assembly buffer).
                    uint cons_fill = (cons_free > consumer_capacity) ? 0 : (consumer_capacity - cons_free);
                    pkt.spdif[i].consumer_fill_pct = (uint8_t)(cons_fill * 100 / consumer_capacity);
                    pkt.spdif[i].consumer_min_fill_pct = spdif_consumer_min_fill_pct[i];
                    pkt.spdif[i].consumer_max_fill_pct = spdif_consumer_max_fill_pct[i];
                }

                if (pdm_enabled) {
                    pkt.pdm.dma_fill_pct = pdm_get_dma_fill_pct();
                    pkt.pdm.dma_min_fill_pct = pdm_dma_min_fill_pct;
                    pkt.pdm.dma_max_fill_pct = pdm_dma_max_fill_pct;
                    pkt.pdm.ring_fill_pct = pdm_get_ring_fill_pct();
                    pkt.pdm.ring_min_fill_pct = pdm_ring_min_fill_pct;
                    pkt.pdm.ring_max_fill_pct = pdm_ring_max_fill_pct;
                }

                memcpy(resp_buf, &pkt, sizeof(pkt));
                vendor_send_response(resp_buf, sizeof(pkt));
                return true;
            }

            case REQ_RESET_BUFFER_STATS: {
                uint16_t flags = setup->wValue;
                if (flags & 0x01) {
                    reset_buffer_watermarks();
                }
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_USB_ERROR_STATS: {
                // TinyUSB does not expose per-category USB error counters.
                // Return all zeros so the host app keeps working; re-plumb if
                // a future TinyUSB version surfaces these via dcd_event or
                // equivalent.
                typedef struct __attribute__((packed)) {
                    uint32_t total;
                    uint32_t crc;
                    uint32_t bitstuff;
                    uint32_t rx_overflow;
                    uint32_t rx_timeout;
                    uint32_t data_seq;
                } UsbErrorStatsPacket;

                UsbErrorStatsPacket pkt = {0};
                memcpy(resp_buf, &pkt, sizeof(pkt));
                vendor_send_response(resp_buf, sizeof(pkt));
                return true;
            }

            case REQ_RESET_USB_ERROR_STATS: {
                // No-op under TinyUSB (see REQ_GET_USB_ERROR_STATS).
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // ----------------------------------------------------------------
            // System Commands (0xF0+)
            // ----------------------------------------------------------------

            case REQ_ENTER_BOOTLOADER: {
                // Send response before rebooting so the host sees success
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                // Brief delay to let the USB response complete
                busy_wait_ms(100);
                reset_usb_boot(0, 0);
                // Never returns
            }

            // ----------------------------------------------------------------
            // I2S / MCK Configuration Commands (0xC0-0xC9)
            // ----------------------------------------------------------------

            case REQ_SET_OUTPUT_TYPE: {
                // wValue = (new_type << 8) | slot_index
                //
                // Type switching is DEFERRED to the main loop because it involves
                // heap allocation (consumer pool creation) which cannot safely run
                // in USB ISR context (malloc uses a spin lock that can deadlock if
                // the main loop is also in malloc).
                //
                uint8_t slot = setup->wValue & 0xFF;
                uint8_t new_type = (setup->wValue >> 8) & 0xFF;
                uint8_t status;

                if (slot >= NUM_SPDIF_INSTANCES) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else if (new_type > 1) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_type == output_types[slot]) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else {
                    // Defer to main loop — per-slot bitmask supports
                    // back-to-back requests without dropping any
                    extern volatile uint8_t output_type_change_mask;
                    extern volatile uint8_t pending_output_types[];
                    pending_output_types[slot] = new_type;
                    __dmb();
                    output_type_change_mask |= (1u << slot);
                    status = PIN_CONFIG_SUCCESS;
                }

                // Notify at request time against the eventual type.  Main
                // loop applies the deferred switch; shadow matches host
                // expectation either way.
                if (status == PIN_CONFIG_SUCCESS && slot < NUM_SPDIF_INSTANCES) {
                    notify_param_write(
                        (uint16_t)(offsetof(WireBulkParams, i2s_config.output_types) + slot),
                        1, &new_type);
                }

                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_OUTPUT_TYPE: {
                uint8_t slot = (uint8_t)setup->wValue;
                if (slot < NUM_SPDIF_INSTANCES) {
                    resp_buf[0] = output_types[slot];
                    vendor_send_response(resp_buf, 1);
                    return true;
                }
                return false;
            }

            case REQ_SET_I2S_BCK_PIN: {
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;

                if (!is_valid_gpio_pin(new_pin) || !is_valid_gpio_pin(new_pin + 1)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_pin == i2s_bck_pin) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else {
                    // Reject if any slot is currently I2S
                    bool any_i2s = false;
                    for (int i = 0; i < NUM_SPDIF_INSTANCES; i++) {
                        if (output_types[i] == OUTPUT_TYPE_I2S) { any_i2s = true; break; }
                    }
                    if (any_i2s) {
                        status = PIN_CONFIG_OUTPUT_ACTIVE;
                    } else if (is_pin_in_use(new_pin, 0xFF) || is_pin_in_use(new_pin + 1, 0xFF)) {
                        status = PIN_CONFIG_PIN_IN_USE;
                    } else {
                        i2s_bck_pin = new_pin;
                        if (active_input_source == INPUT_SOURCE_I2S) {
                            // Input SM is the clock master here (any-I2S-output
                            // was rejected above); restart it on the new pins.
                            // Deferred: PIO teardown is too heavy for ISR context.
                            i2s_input_restart_pending = true;
                        }
                        status = PIN_CONFIG_SUCCESS;
                        notify_param_write(offsetof(WireBulkParams, i2s_config.bck_pin),
                                           1, &i2s_bck_pin);
                    }
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_I2S_BCK_PIN: {
                resp_buf[0] = i2s_bck_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_ENABLE: {
                bool enable = (setup->wValue != 0);
                if (enable && !i2s_mck_enabled) {
                    // Order matters: divider must be loaded *before* the
                    // GPOUTn block is enabled so the DAC sees a valid clock
                    // on the first cycle.  Reversing the order would briefly
                    // run MCK at whatever DIV value the GPOUTn block had
                    // before, causing a transient PLL-relock chirp on
                    // connected DACs.
                    audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                    audio_i2s_mck_set_enabled(true);
                    i2s_mck_enabled = true;
                } else if (!enable && i2s_mck_enabled) {
                    audio_i2s_mck_set_enabled(false);
                    i2s_mck_enabled = false;
                }
                uint8_t v = i2s_mck_enabled ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, i2s_config.mck_enabled), 1, &v);
                resp_buf[0] = PIN_CONFIG_SUCCESS;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_ENABLE: {
                resp_buf[0] = i2s_mck_enabled ? 1 : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_PIN: {
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;
                if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                }
                // GPOUT-capability check: MCK is now driven by hardware
                // CLK_GPOUTn, so the GPIO must map to one of clk_gpout0..3.
                // GPIO_TO_GPOUT_CLOCK_HANDLE() is the SDK macro that returns
                // the matching clk_gpoutN enum value for valid pins or the
                // supplied default for non-GPOUT pins.  We pass clk_sys as
                // the sentinel: a result of clk_sys means "no GPOUTn for
                // this pin on this platform" — reject it.
                //
                // RP2040 GPOUT pins: 21 (only one not blocked by
                //   is_valid_gpio_pin); 23-25 are board-reserved.
                // RP2350 GPOUT pins: 13, 15, 21 (all unblocked).
                else if (GPIO_TO_GPOUT_CLOCK_HANDLE(new_pin, clk_sys) == clk_sys) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (i2s_mck_enabled) {
                    status = PIN_CONFIG_OUTPUT_ACTIVE;
                } else if (is_pin_in_use(new_pin, 0xFF)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else if (new_pin == i2s_mck_pin) {
                    status = PIN_CONFIG_SUCCESS;
                } else {
                    audio_i2s_mck_change_pin(new_pin);
                    i2s_mck_pin = new_pin;
                    status = PIN_CONFIG_SUCCESS;
                    notify_param_write(offsetof(WireBulkParams, i2s_config.mck_pin),
                                       1, &i2s_mck_pin);
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_PIN: {
                resp_buf[0] = i2s_mck_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_MCK_MULTIPLIER: {
                // Wire encoding: 0 = 128x, 1 = 256x
                uint16_t raw = setup->wValue;
                if (raw > 1) { resp_buf[0] = PIN_CONFIG_INVALID_PIN; vendor_send_response(resp_buf, 1); return true; }
                uint16_t mult = (raw == 1) ? 256 : 128;

                // No per-rate multiplier rejection here anymore — CLK_GPOUTn
                // gives integer dividers at 48 kHz × {128×, 256×} and at
                // 96 kHz × 128×, and a stable 12.5 fractional divider at
                // 96 kHz × 256×.  See audio_i2s_multi.c MCK section for the
                // full divider table.
                i2s_mck_multiplier = mult;
                if (i2s_mck_enabled) {
                    audio_i2s_mck_update_frequency(audio_state.freq, i2s_mck_multiplier);
                }
                // Wire encoding: 0 = 128x, 1 = 256x
                uint8_t wire_mult = (mult == 256) ? 1 : 0;
                notify_param_write(offsetof(WireBulkParams, i2s_config.mck_multiplier),
                                   1, &wire_mult);
                resp_buf[0] = PIN_CONFIG_SUCCESS;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_MCK_MULTIPLIER: {
                resp_buf[0] = mck_encode(i2s_mck_multiplier);
                vendor_send_response(resp_buf, 1);
                return true;
            }

            // ---- ADAT Bulk Output Commands (RP2350 only) ----

            case REQ_SET_ADAT_ENABLE: {
#if !PICO_RP2350
                resp_buf[0] = PIN_CONFIG_INVALID_OUTPUT;
#else
                uint8_t value = setup->wValue & 0xFF;
                uint8_t status;
                if (value > 1) {
                    status = PIN_CONFIG_INVALID_PARAM;
                } else if (value == (adat_output_config_enabled() ? 1 : 0)) {
                    status = PIN_CONFIG_SUCCESS;  // No-op: already in this state
                } else if (value != 0) {
                    // Enabling from disabled: validate the configured ADAT pin.
                    // ADAT is not yet config-enabled, so it never blocks itself.
                    uint8_t pin = adat_output_pin();
                    if (!is_valid_gpio_pin(pin)) {
                        status = PIN_CONFIG_INVALID_PIN;
                    } else if (is_pin_in_use(pin, 0xFF)) {
                        status = PIN_CONFIG_PIN_IN_USE;
                    } else {
                        adat_output_set_config(true, pin);
                        status = PIN_CONFIG_SUCCESS;
                    }
                } else {
                    // Disabling: always allowed.
                    adat_output_set_config(false, adat_output_pin());
                    status = PIN_CONFIG_SUCCESS;
                }
                if (status == PIN_CONFIG_SUCCESS) {
                    uint8_t v = adat_output_config_enabled() ? 1 : 0;
                    notify_param_write(offsetof(WireBulkParams, adat_config.enabled), 1, &v);
                }
                resp_buf[0] = status;
#endif
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_ADAT_ENABLE: {
#if PICO_RP2350
                resp_buf[0] = adat_output_config_enabled() ? 1 : 0;
#else
                resp_buf[0] = 0;
#endif
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_ADAT_PIN: {
#if !PICO_RP2350
                resp_buf[0] = PIN_CONFIG_INVALID_OUTPUT;
#else
                uint8_t pin = setup->wValue & 0xFF;
                uint8_t status;
                if (pin == 0) pin = PICO_ADAT_PIN;  // 0 = reset to default (matches flash/bulk)
                if (!is_valid_gpio_pin(pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (pin == adat_output_pin()) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else if (is_pin_in_use(pin, 0xFF)) {
                    // New pin differs from the current ADAT pin, so the ADAT
                    // pin itself can never self-block this change.
                    status = PIN_CONFIG_PIN_IN_USE;
                } else {
                    // Re-route allowed even while enabled; the deferred apply
                    // moves the stream under mute.
                    adat_output_set_config(adat_output_config_enabled(), pin);
                    status = PIN_CONFIG_SUCCESS;
                }
                if (status == PIN_CONFIG_SUCCESS) {
                    uint8_t p = adat_output_pin();
                    notify_param_write(offsetof(WireBulkParams, adat_config.pin), 1, &p);
                }
                resp_buf[0] = status;
#endif
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_ADAT_PIN: {
#if PICO_RP2350
                resp_buf[0] = adat_output_pin();
#else
                resp_buf[0] = 0;
#endif
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_ADAT_STATUS: {
                AdatStatus s;
#if PICO_RP2350
                adat_output_get_status(&s);
#else
                memset(&s, 0, sizeof(s));  // ADAT unavailable: 8 zero bytes
#endif
                vendor_send_response(&s, sizeof(s));
                return true;
            }

            // ---- Audio Input Source Commands ----

            case REQ_GET_INPUT_SOURCE: {
                resp_buf[0] = active_input_source;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_SPDIF_RX_PIN: {
                // Hot-swappable: when SPDIF input is active, the main-loop
                // handler stops the RX library, persists the new pin to the
                // directory, and restarts on the new pin. While stopped,
                // outputs play silence; on lock-acquisition the polling
                // block's prefill handshake re-engages playback. Mirrors
                // the preset_load_pending pattern at main.c:1242+ which
                // also brackets a flash blackout with stop/start.
                //
                // wValue = new GPIO pin number.
                uint8_t new_pin = (uint8_t)setup->wValue;
                uint8_t status;
                if (!is_valid_gpio_pin(new_pin)) {
                    status = PIN_CONFIG_INVALID_PIN;
                } else if (new_pin == spdif_rx_pin) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else if (is_pin_in_use(new_pin, 0xFF)) {
                    status = PIN_CONFIG_PIN_IN_USE;
                } else {
                    // RAM-only update.  Persistence is now slot-scoped:
                    // the user must REQ_PRESET_SAVE to capture the new
                    // pin in a preset slot, exactly like REQ_SET_OUTPUT_PIN.
                    spdif_rx_pin = new_pin;
                    if (active_input_source == INPUT_SOURCE_SPDIF) {
                        // Hot-swap: defer the stop/start to main loop
                        // because spdif_rx library teardown is too heavy
                        // for USB ISR context.
                        extern volatile bool spdif_rx_pin_change_pending;
                        spdif_rx_pin_change_pending = true;
                    }
                    status = PIN_CONFIG_SUCCESS;
                    notify_param_write(offsetof(WireBulkParams, input_config.spdif_rx_pin),
                                       1, &spdif_rx_pin);
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_SPDIF_RX_PIN: {
                resp_buf[0] = spdif_rx_pin;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_I2S_RX_PIN: {
                // Mirrors REQ_SET_SPDIF_RX_PIN: RAM-only update, slot-scoped
                // persistence via REQ_PRESET_SAVE, applied to the live input
                // when I2S is the active source.
                //
                // wValue = (pair << 8) | GPIO.  The high byte selects the stereo
                // pair (0..I2S_RX_MAX_PAIRS-1); old hosts that send just the pin
                // address pair 0 (high byte 0).
                uint8_t new_pin = (uint8_t)(setup->wValue & 0xFF);
                uint8_t pair    = (uint8_t)((setup->wValue >> 8) & 0xFF);
                uint8_t status;
                if (pair >= I2S_RX_MAX_PAIRS) {
                    status = PIN_CONFIG_INVALID_OUTPUT;   // no such stereo pair
                } else if (new_pin == i2s_rx_pin[pair]) {
                    status = PIN_CONFIG_SUCCESS;  // No-op
                } else if ((status = check_i2s_rx_pin(new_pin, pair)) != PIN_CONFIG_SUCCESS) {
                    // check_i2s_rx_pin set status (invalid GPIO, clock pin,
                    // peripheral clash, or another pair's pin) — reject.
                } else {
                    i2s_rx_pin[pair] = new_pin;
                    if (active_input_source == INPUT_SOURCE_I2S) {
                        // Pair 0 alone (stereo) can hot-swap its data pin; any
                        // higher pair, or a multichannel config, must restart so
                        // every pair re-syncs (re-initing one SM in isolation
                        // would break the inter-channel alignment guarantee).
                        if (pair == 0 && i2s_input_channels <= 2)
                            i2s_rx_pin_change_pending = true;
                        else
                            i2s_input_restart_pending = true;
                    }
                    status = PIN_CONFIG_SUCCESS;
                    // Pair 0 maps to input_config.i2s_rx_pin; pairs 1..3 to
                    // input_config.i2s_rx_pin_ext[pair-1].
                    uint16_t off = (pair == 0)
                        ? (uint16_t)offsetof(WireBulkParams, input_config.i2s_rx_pin)
                        : (uint16_t)(offsetof(WireBulkParams, input_config.i2s_rx_pin_ext) +
                                     (pair - 1));
                    notify_param_write(off, 1, &i2s_rx_pin[pair]);
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_I2S_RX_PIN: {
                // wValue = pair (0..I2S_RX_MAX_PAIRS-1); old hosts send 0.
                uint8_t pair = (uint8_t)(setup->wValue & 0xFF);
                resp_buf[0] = (pair < I2S_RX_MAX_PAIRS) ? i2s_rx_pin[pair] : 0;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SET_I2S_INPUT_CHANNELS: {
                // wValue = channel count (2/4/6/8).  Restarts the input when I2S
                // is the active source so the new pair set is (de)allocated and
                // every pair re-syncs.
                uint8_t count = (uint8_t)(setup->wValue & 0xFF);
                uint8_t status;
                if (count != 2 && count != 4 && count != 6 && count != 8) {
                    status = PIN_CONFIG_INVALID_PIN;        // not a valid count
                } else if (count / 2 > I2S_RX_MAX_PAIRS) {
                    status = PIN_CONFIG_INVALID_OUTPUT;     // more pairs than this part supports
                } else {
                    // When RAISING the count, every newly-activated pair's data
                    // pin must pass the full I2S RX check (valid GPIO; not a
                    // clock pin even though I2S may be inactive right now; no
                    // peripheral or cross-pair clash).  Inactive pairs don't
                    // reserve their pins, so a pin a pair was parked on may have
                    // been taken since; reject rather than bring two SMs up on
                    // one GPIO.  Lowering the count skips this (loop is empty).
                    status = PIN_CONFIG_SUCCESS;
                    uint8_t old_pairs = (uint8_t)(i2s_input_channels / 2);
                    uint8_t new_pairs = (uint8_t)(count / 2);
                    for (uint8_t p = old_pairs; p < new_pairs && status == PIN_CONFIG_SUCCESS; p++)
                        status = check_i2s_rx_pin(i2s_rx_pin[p], p);
                    if (status == PIN_CONFIG_SUCCESS && count != i2s_input_channels) {
                        i2s_input_channels = count;
                        if (active_input_source == INPUT_SOURCE_I2S) {
                            i2s_input_restart_pending = true;
                            // Active count changed live; push the input-format
                            // event so the host relayouts (matches USB alts).
                            notify_push_input_format(active_input_channel_count());
                        }
                        notify_param_write(
                            offsetof(WireBulkParams, input_config.i2s_input_channels),
                            1, &i2s_input_channels);
                    }
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_I2S_INPUT_CHANNELS: {
                resp_buf[0] = i2s_input_channels;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_INPUT_RATE: {
                // {current pipeline Hz, selected I2S input Hz}
                uint32_t vals[2] = { audio_state.freq, i2s_input_rate };
                memcpy(resp_buf, vals, sizeof(vals));
                vendor_send_response(resp_buf, sizeof(vals));
                return true;
            }

            case REQ_GET_SPDIF_RX_STATUS: {
                SpdifRxStatusPacket status;
                spdif_input_get_status(&status);
                vendor_send_response(&status, sizeof(status));
                return true;
            }

            case REQ_GET_SPDIF_RX_CH_STATUS: {
                uint8_t ch_status[24];
                spdif_input_get_channel_status(ch_status);
                vendor_send_response(ch_status, 24);
                return true;
            }

            case REQ_GET_LG_SOUND_SYNC_ENABLE: {
                resp_buf[0] = lg_sound_sync_get_enabled() ? 1u : 0u;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_GET_LG_SOUND_SYNC_STATUS: {
                /* Snapshot via the module's status getter so the host
                 * sees a coherent struct (the getter brackets the read
                 * with a brief IRQ disable to avoid torn fields).
                 * Static so it outlives the EP0 transfer — control
                 * transfers are serialised by TinyUSB. */
                static LgSoundSyncStatus status;
                lg_sound_sync_get_status(&status);
                vendor_send_response(&status, sizeof(status));
                return true;
            }

            case REQ_GET_DAC_HW_MUTE_CONFIG: {
                /* Return the live DAC hardware-mute config.  The module's
                 * live mirror always matches the flash directory's
                 * persisted value (set together inside dac_hw_mute_set_config).
                 * Static so the buffer outlives the EP0 DATA stage. */
                static DacHwMuteConfig hw;
                dac_hw_mute_get_config(&hw);
                vendor_send_response(&hw, sizeof(hw));
                return true;
            }

            case REQ_TEST_DAC_HW_MUTE: {
                /* Pulse the mute pin for ~1 s so the installer can
                 * confirm pin number + polarity by ear (audio cuts out
                 * then returns).  The actual pulse runs from the main
                 * loop because it busy-waits 1 s — the USB ISR cannot
                 * block that long.  We respond immediately with the
                 * status that would be returned synchronously
                 * (PIN_CONFIG_SUCCESS if enabled, PIN_CONFIG_INVALID_OUTPUT
                 * otherwise).  The actual audible pulse follows when the
                 * main loop drains dac_hw_mute_test_pending. */
                static DacHwMuteConfig hw;
                dac_hw_mute_get_config(&hw);
                uint8_t status;
                if (hw.enabled == 0) {
                    status = PIN_CONFIG_INVALID_OUTPUT;
                } else {
                    extern volatile bool dac_hw_mute_test_pending;
                    dac_hw_mute_test_pending = true;
                    __dmb();
                    status = PIN_CONFIG_SUCCESS;
                }
                resp_buf[0] = status;
                vendor_send_response(resp_buf, 1);
                return true;
            }

            case REQ_SIGGEN_GET_CONFIG: {
                SiggenConfig tmp;
                siggen_get_config(&tmp);
                memcpy(resp_buf, &tmp, sizeof(SiggenConfig));
                vendor_send_response(resp_buf, sizeof(SiggenConfig));
                return true;
            }

            case REQ_SIGGEN_GET_STATUS: {
                SiggenStatus tmp;
                siggen_get_status(&tmp);
                memcpy(resp_buf, &tmp, sizeof(SiggenStatus));
                vendor_send_response(resp_buf, sizeof(SiggenStatus));
                return true;
            }

            case REQ_SIGGEN_GET_CAPS: {
                // wValue = 0xFFFF -> capabilities header; else a type index.
                if (setup->wValue == 0xFFFF) {
                    memcpy(resp_buf, siggen_caps_header(), sizeof(SiggenCapsHeader));
                    vendor_send_response(resp_buf, sizeof(SiggenCapsHeader));
                    return true;
                }
                // Reject a set high byte instead of aliasing it to type 0.
                if (setup->wValue > 0xFF) {
                    return false;
                }
                const SiggenTypeDesc *d = siggen_caps_type((uint8_t)setup->wValue);
                if (d == NULL) {
                    return false;
                }
                memcpy(resp_buf, d, sizeof(SiggenTypeDesc));
                vendor_send_response(resp_buf, sizeof(SiggenTypeDesc));
                return true;
            }

            case REQ_SIGGEN_CONTROL: {
                // Parameterless action carried in wValue (SIGGEN_CTL_*);
                // acknowledged with a single status byte like OUTPUT_TYPE.
                if (!siggen_control((uint8_t)(setup->wValue & 0xFF))) {
                    return false;
                }
                resp_buf[0] = 1;
                vendor_send_response(resp_buf, 1);
                return true;
            }
        }

        return false;
    }
}

// ----------------------------------------------------------------------------
// PUBLIC ENTRY POINT — overrides TinyUSB's weak tud_vendor_control_xfer_cb
// (usbd.c:82).  TinyUSB calls this directly for every vendor-type request
// (usbd.c:727-730), bypassing class drivers.  Dispatches SETUP/DATA/ACK.
// ----------------------------------------------------------------------------

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                tusb_control_request_t const *req) {
    // Stash rhport + request so vendor_send_response() can complete the xfer
    // without every case body having to plumb them through.
    _vendor_rhport       = rhport;
    _vendor_current_req  = req;
    _dispatch_src        = PARAM_SRC_HOST_SET;

    if (stage == CONTROL_STAGE_SETUP) {
        // EP0 transfers are strictly serialized: a new SETUP means any prior
        // USB control transfer is over, so drop a USB-held bulk lock (covers
        // host-aborted 0xA0/0xA1 transfers whose ACK never fired).  Chunked
        // sessions (0xA2/0xA3) intentionally span SETUPs and keep the lock;
        // any other vendor request tears an open session down.
        if (bulk_buf_owner == BULK_OWNER_USB &&
            !usb_chunk_session_continues(req->bRequest)) {
            bulk_buf_owner = BULK_OWNER_NONE;
            usb_chunk_sessions_close();
        }
        usb_set_in_flight = false;

        // ---- Microsoft OS 2.0 platform-capability vendor requests ----
        // Windows 8.1+ sends these after reading our BOS descriptor:
        //   bmRequestType=0xC0, bRequest=MS_VENDOR_CODE, wIndex=7  → return
        //                       the descriptor set (advertises WinUSB +
        //                       DeviceInterfaceGUID).
        //   bmRequestType=0xC0, bRequest=MS_VENDOR_CODE, wIndex=8  → SET
        //                       alt-enumeration; not supported, STALL.
        //
        // MS_VENDOR_CODE is reserved for Microsoft on the vendor-type
        // request channel; STALL any OUT-direction or non-7 wIndex hit so
        // the value can never accidentally route into vendor_handle_set_data
        // or be silently ACK'd by the generic SET path below. Note this
        // value (0x01) numerically aliases UAC1_REQ_SET_CUR but cannot
        // collide because UAC1 is TUSB_REQ_TYPE_CLASS and reaches the audio
        // class handler — vendor-type requests come here.
        if (req->bRequest == MS_VENDOR_CODE) {
            if (req->bmRequestType_bit.direction == TUSB_DIR_IN &&
                req->wIndex == 7) {
                return tud_control_xfer(rhport, req,
                                        (void *)(uintptr_t)desc_ms_os_20,
                                        (uint16_t)desc_ms_os_20_len);
            }
            // OUT-direction, wIndex=8 (SET_ALT_ENUMERATION), or anything
            // else: STALL. Must run BEFORE vendor_handle_get / SET path so
            // bRequest=0x01 can never hit the application dispatcher.
            return false;
        }

        if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
            // Bulk GET streams from bulk_param_buf across many EP0 packets
            // after the handler returns; hold the lock until ACK so an
            // external transport can't re-collect into it mid-stream.
            if (req->bRequest == REQ_GET_ALL_PARAMS &&
                !vendor_bulk_try_acquire(CTRL_SOURCE_USB)) {
                return false;
            }
            // GET path — dispatches into the legacy switch.
            bool ok = vendor_handle_get(req);
            if (!ok && req->bRequest == REQ_GET_ALL_PARAMS) {
                vendor_bulk_release(CTRL_SOURCE_USB);
            }
            if (!ok && req->bRequest == REQ_GET_ALL_PARAMS_CHUNK) {
                // Failed or invalid chunk: drop the session so the host's
                // restart from offset 0 takes a fresh snapshot.
                vendor_bulk_release(CTRL_SOURCE_USB);
                usb_chunk_sessions_close();
            }
            // Clear any source tag that the GET dispatcher set for embedded
            // SETs — bleeding HOST_SET would misattribute unrelated writes
            // that happen outside a dispatch bracket (e.g. timer callbacks).
            notify_set_source(PARAM_SRC_UNKNOWN);
            return ok;
        }

        // SET path — schedule DATA stage receive.
        vendor_last_request = req->bRequest;
        vendor_last_wValue  = req->wValue;
        vendor_last_wLength = req->wLength;

        if (vendor_is_bulk_set(req->bRequest, req->wLength)) {
            // Large bulk SET — tud_control_xfer handles EP0 chunking.  At V16
            // compatibility is broken, so WIRE_BULK_PARAMS_MIN_SIZE ==
            // sizeof(WireBulkParams) and the gate is effectively strict
            // equality: only a full current-version payload is accepted here.
            if (!vendor_bulk_try_acquire(CTRL_SOURCE_USB)) return false;
            usb_set_in_flight  = true;
            usb_set_setup_time = time_us_32();
            return tud_control_xfer(rhport, req, bulk_param_buf, req->wLength);
        }

        if (req->bRequest == REQ_SET_ALL_PARAMS_CHUNK) {
            // Chunked bulk SET (USB-only WinUSB workaround): wValue = byte
            // offset, payload lands directly at bulk_param_buf + offset.
            // Chunks must arrive sequentially from offset 0; completion is
            // detected at the ACK stage, which hands the buffer to the
            // main-loop apply exactly like a single-shot 0xA1.
            uint32_t off = req->wValue;
            uint32_t n   = req->wLength;
            if (n == 0 || off + n > sizeof(WireBulkParams)) return false;
            if (off == 0) {
                if (bulk_buf_owner != BULK_OWNER_USB &&
                    !vendor_bulk_try_acquire(CTRL_SOURCE_USB)) {
                    return false;
                }
                usb_chunk_get_open = false;   // SET supersedes a stale GET
                usb_chunk_get_done = false;
                usb_chunk_set_open = true;
                usb_chunk_set_received = 0;
            } else if (!usb_chunk_set_open ||
                       bulk_buf_owner != BULK_OWNER_USB ||
                       off != usb_chunk_set_received) {
                return false;   // out of order or session lost; restart at 0
            }
            vendor_bulk_touch(CTRL_SOURCE_USB);
            usb_set_in_flight  = true;
            usb_set_setup_time = time_us_32();
            return tud_control_xfer(rhport, req, bulk_param_buf + off,
                                    (uint16_t)n);
        }

        if (req->wLength == 0) {
            // Zero-length SET — no DATA stage; ACK immediately.
            return tud_control_status(rhport, req);
        }

        if (req->wLength <= sizeof(vendor_rx_buf)) {
            // The USB ISR fills vendor_rx_buf between now and the DATA
            // callback; external dispatch is held off until then.
            usb_set_in_flight  = true;
            usb_set_setup_time = time_us_32();
            return tud_control_xfer(rhport, req, vendor_rx_buf, req->wLength);
        }

        // Oversized SET we can't handle — STALL.
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        // SET data received — run the legacy command dispatcher.
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            vendor_handle_set_data(req);
            usb_set_in_flight = false;
        }
        return true;
    }

    if (stage == CONTROL_STAGE_ACK) {
        // Status-stage completed.  Signal the main loop to apply bulk SET;
        // the pending flag takes over guarding bulk_param_buf from the lock.
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT &&
            req->bRequest == REQ_SET_ALL_PARAMS) {
            bulk_params_pending = true;
            vendor_bulk_release(CTRL_SOURCE_USB);
        }
        if (req->bmRequestType_bit.direction == TUSB_DIR_IN &&
            req->bRequest == REQ_GET_ALL_PARAMS) {
            vendor_bulk_release(CTRL_SOURCE_USB);   // bulk GET fully streamed
        }
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT &&
            req->bRequest == REQ_SET_ALL_PARAMS_CHUNK && usb_chunk_set_open) {
            // Chunk landed in bulk_param_buf; on the final one, hand the
            // buffer to the main-loop apply exactly like single-shot 0xA1.
            usb_chunk_set_received = (uint16_t)(req->wValue + req->wLength);
            if (usb_chunk_set_received >= sizeof(WireBulkParams)) {
                usb_chunk_sessions_close();
                bulk_params_pending = true;
                vendor_bulk_release(CTRL_SOURCE_USB);
            }
        }
        if (req->bmRequestType_bit.direction == TUSB_DIR_IN &&
            req->bRequest == REQ_GET_ALL_PARAMS_CHUNK && usb_chunk_get_done) {
            usb_chunk_sessions_close();             // final chunk fully streamed
            vendor_bulk_release(CTRL_SOURCE_USB);
        }
        if (req->bmRequestType_bit.direction == TUSB_DIR_OUT) {
            usb_set_in_flight = false;
        }
    }
    return true;
}

// ----------------------------------------------------------------------------
// EXTERNAL TRANSPORT ENTRY POINTS (UART / I2C target)
//
// Main-loop context only.  These wrap the same SET/GET switches the USB path
// uses; the synthesized tusb_control_request_t is a plain struct here, no
// TinyUSB machinery is involved for external transports.
// ----------------------------------------------------------------------------

// Commands refused on the external transports.  Interface self-configuration
// is USB-only so an external controller can never reconfigure (and lock
// itself out of) its own transport; the chunked bulk commands exist solely
// for the Windows/WinUSB 4 KB control cap, and UART/I2C (which have no size
// cap) must use plain 0xA0/0xA1 instead of holding chunk sessions open.
static bool ctrl_cmd_usb_only(uint8_t bRequest) {
    return bRequest == REQ_SET_UART_CONFIG ||
           bRequest == REQ_SET_I2C_CONFIG ||
           bRequest == REQ_GET_ALL_PARAMS_CHUNK ||
           bRequest == REQ_SET_ALL_PARAMS_CHUNK;
}

CtrlDispatchResult vendor_dispatch_get(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex, uint16_t wLength,
                                       const uint8_t **resp_data, uint16_t *resp_len) {
    if (src == CTRL_SOURCE_USB) return CTRL_DISPATCH_ERROR;  // USB uses the TinyUSB path
    if (usb_set_busy())         return CTRL_DISPATCH_BUSY;
    if (ctrl_cmd_usb_only(bRequest)) return CTRL_DISPATCH_BLOCKED;

    bool bulk = (bRequest == REQ_GET_ALL_PARAMS);
    if (bulk && !vendor_bulk_try_acquire(src)) return CTRL_DISPATCH_BULK_LOCKED;

    tusb_control_request_t req = {
        .bmRequestType = 0xC0,   // vendor | device-to-host (informational only)
        .bRequest = bRequest,
        .wValue   = wValue,
        .wIndex   = wIndex,
        .wLength  = wLength ? wLength : 0xFFFF,
    };
    _active_source = src;
    _ext_resp_data = NULL;
    _ext_resp_len  = 0;
    _dispatch_src  = (src == CTRL_SOURCE_UART) ? PARAM_SRC_UART
                   : (src == CTRL_SOURCE_I2C)  ? PARAM_SRC_I2C
                   : PARAM_SRC_GPIO;   // CTRL_SOURCE_GPIO (Control Surfaces)
    bool ok = vendor_handle_get(&req);
    notify_set_source(PARAM_SRC_UNKNOWN);
    _active_source = CTRL_SOURCE_USB;
    _dispatch_src  = PARAM_SRC_HOST_SET;

    if (!ok || _ext_resp_data == NULL) {
        if (bulk) vendor_bulk_release(src);
        return CTRL_DISPATCH_ERROR;
    }
    *resp_data = _ext_resp_data;
    *resp_len  = _ext_resp_len;
    // Honor the caller's response-size cap for every GET, exactly like the
    // USB path (tud_control_xfer clamps EP0 IN transfers to wLength).
    if (wLength && *resp_len > wLength) *resp_len = wLength;
    return CTRL_DISPATCH_OK;
}

CtrlDispatchResult vendor_dispatch_set(CtrlSource src, uint8_t bRequest,
                                       uint16_t wValue, uint16_t wIndex,
                                       const uint8_t *payload, uint16_t wLength) {
    (void)wIndex;   // SET handlers carry sub-params in wValue only (matches USB)
    if (src == CTRL_SOURCE_USB) return CTRL_DISPATCH_ERROR;
    if (usb_set_busy())         return CTRL_DISPATCH_BUSY;
    if (ctrl_cmd_usb_only(bRequest)) return CTRL_DISPATCH_BLOCKED;

    if (bRequest == REQ_SET_ALL_PARAMS) {
        if (!vendor_is_bulk_set(bRequest, wLength)) return CTRL_DISPATCH_ERROR;
        // Caller streamed the payload into bulk_param_buf under its own
        // lock.  Owner check + hand-off to the main-loop apply must be one
        // atomic step: the I2C ISR's stale-reclaim path could otherwise
        // steal the lock between the check and the pending-flag set.
        uint32_t save = save_and_disable_interrupts();
        bool owned = (bulk_buf_owner == bulk_owner_for(src));
        if (owned) {
            bulk_params_pending = true;   // pending flag guards the buffer now
            bulk_buf_owner = BULK_OWNER_NONE;
        }
        restore_interrupts(save);
        return owned ? CTRL_DISPATCH_OK : CTRL_DISPATCH_ERROR;
    }

    if (wLength > sizeof(vendor_rx_buf)) return CTRL_DISPATCH_ERROR;
    vendor_last_request = bRequest;
    vendor_last_wValue  = wValue;
    vendor_last_wLength = wLength;
    if (wLength) memcpy(vendor_rx_buf, payload, wLength);

    tusb_control_request_t req = {
        .bmRequestType = 0x40,   // vendor | host-to-device (informational only)
        .bRequest = bRequest,
        .wValue   = wValue,
        .wIndex   = wIndex,
        .wLength  = wLength,
    };
    _active_source = src;
    _dispatch_src  = (src == CTRL_SOURCE_UART) ? PARAM_SRC_UART
                   : (src == CTRL_SOURCE_I2C)  ? PARAM_SRC_I2C
                   : PARAM_SRC_GPIO;   // CTRL_SOURCE_GPIO (Control Surfaces)
    bool handled = vendor_handle_set_data(&req);
    _active_source = CTRL_SOURCE_USB;
    _dispatch_src  = PARAM_SRC_HOST_SET;
    // Unknown commands and wValue-only SETs (which live on the GET path and
    // must be sent as GET-type frames) report ERROR instead of a false OK.
    return handled ? CTRL_DISPATCH_OK : CTRL_DISPATCH_ERROR;
}
