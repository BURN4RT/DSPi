/*
 * bulk_params.c — Bulk parameter collect/apply for DSPi
 *
 * Snapshots the entire live DSP state into the wire format (GET), or
 * applies a received wire-format payload back to live state (SET).
 *
 * bulk_params_apply() writes to the same globals as
 * apply_slot_to_live() in flash_storage.c but does NOT recalculate
 * filters or delays — the caller must do that.
 */

#include "bulk_params.h"
#include "config.h"
#include "audio_input.h"
#include "dsp_pipeline.h"
#include "crossover.h"
#include "usb_audio.h"
#include "crossfeed.h"
#include "leveller.h"
#include "lg_sound_sync.h"
#include "dac_hw_mute.h"
#include "adat_output.h"
#include "notify.h"
#include "uart_control.h"
#include "i2c_control.h"

#include <string.h>
#include <stdio.h>   // printf() for rejected-config diagnostics
#include <math.h>    // powf() for master volume (db_to_linear() clamps at -60 dB, insufficient)
#include <assert.h>  // _Static_assert

#include "hardware/sync.h"    // __dmb()
#include "hardware/clocks.h"  // GPIO_TO_GPOUT_CLOCK_HANDLE() — MCK pin migration

// LgSoundSyncStatus (lg_sound_sync.h, used by REQ_GET_LG_SOUND_SYNC_STATUS)
// and WireLgSoundSync (this file's WireBulkParams section) must have
// identical wire layout — bulk_params_collect() field-copies between them
// and the host SDK consumes both via the same parser.  If they ever drift
// (someone adds a field to one but not the other), this static assert
// fails at compile time before silent struct mismatches reach runtime.
_Static_assert(sizeof(WireInputConfig) == 16,
               "V12 claimed WireInputConfig reserved bytes; the section size "
               "must not change (V12 and V11 payloads are byte-identical)");
_Static_assert(sizeof(WireLgSoundSync) == sizeof(LgSoundSyncStatus),
               "WireLgSoundSync and LgSoundSyncStatus must have identical layout");
_Static_assert(sizeof(WireBulkParams) <= WIRE_BULK_BUF_SIZE,
               "WireBulkParams must fit in the bulk transfer buffer");

// External variables (defined in usb_audio.c)
extern volatile float global_preamp_db[NUM_INPUT_CHANNELS];
extern volatile int32_t global_preamp_mul[NUM_INPUT_CHANNELS];
extern volatile float global_preamp_linear[NUM_INPUT_CHANNELS];
extern volatile float master_volume_db;
extern volatile float master_volume_linear;
extern volatile int32_t master_volume_q15;
extern volatile bool user_mute;
extern volatile float channel_gain_db[3];
extern volatile int32_t channel_gain_mul[3];
extern volatile float channel_gain_linear[3];
extern volatile bool channel_mute[3];
extern volatile bool loudness_enabled;
extern volatile float loudness_ref_spl;
extern volatile float loudness_intensity_pct;
extern volatile bool loudness_recompute_pending;
extern volatile CrossfeedConfig crossfeed_config;
extern volatile bool crossfeed_update_pending;
extern volatile LevellerConfig leveller_config;
extern volatile bool leveller_update_pending;
extern volatile bool leveller_reset_pending;
extern MatrixMixer matrix_mixer;
extern uint8_t output_pins[NUM_PIN_OUTPUTS];

// ============================================================================
// dB-TO-LINEAR CONVERSION (duplicated from flash_storage.c — it's static there)
// ============================================================================

static float db_to_linear(float db) {
    if (db == 0.0f) return 1.0f;
    if (db < -60.0f) db = -60.0f;
    if (db > 20.0f) db = 20.0f;
    float x = db * 0.1151292546f;  // ln(10)/20
    float linear = 1.0f + x + x*x*0.5f + x*x*x*0.1666667f + x*x*x*x*0.0416667f;
    return (linear < 0.0f) ? 0.0f : linear;
}

// ============================================================================
// COLLECT: Live state → wire format
// ============================================================================

void bulk_params_collect(WireBulkParams *out) {
    memset(out, 0, sizeof(*out));

    // Header
    out->header.format_version = WIRE_FORMAT_VERSION;
#if PICO_RP2350
    out->header.platform_id = WIRE_PLATFORM_RP2350;
#else
    out->header.platform_id = WIRE_PLATFORM_RP2040;
#endif
    out->header.num_channels = NUM_CHANNELS;
    out->header.num_output_channels = NUM_OUTPUT_CHANNELS;
    out->header.num_input_channels = NUM_INPUT_CHANNELS;
    out->header.max_bands = MAX_BANDS;
    out->header.payload_length = sizeof(WireBulkParams);
    out->header.fw_version_major = FW_VERSION_MAJOR;
    out->header.fw_version_minor = FW_VERSION_MINOR;

    // Global params
    out->global.preamp_gain_db = global_preamp_db[0];  // Legacy field: channel 0
    out->global.bypass = bypass_master_eq ? 1 : 0;
    out->global.loudness_enabled = loudness_enabled ? 1 : 0;
    out->global.loudness_ref_spl = loudness_ref_spl;
    out->global.loudness_intensity_pct = loudness_intensity_pct;

    // Crossfeed
    out->crossfeed.enabled = crossfeed_config.enabled ? 1 : 0;
    out->crossfeed.preset = crossfeed_config.preset;
    out->crossfeed.itd_enabled = crossfeed_config.itd_enabled ? 1 : 0;
    out->crossfeed.custom_fc = crossfeed_config.custom_fc;
    out->crossfeed.custom_feed_db = crossfeed_config.custom_feed_db;

    // Legacy channels
    for (int i = 0; i < 3; i++) {
        out->legacy.gain_db[i] = channel_gain_db[i];
        out->legacy.mute[i] = channel_mute[i] ? 1 : 0;
    }

    // Delays
    for (int i = 0; i < NUM_CHANNELS; i++) {
        out->delays.delay_ms[i] = channel_delays_ms[i];
    }

    // Matrix crosspoints — inputs 0..N_in-1, direct (row-major).
    for (int in = 0; in < NUM_INPUT_CHANNELS; in++) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            out->crosspoints[in][o].enabled = matrix_mixer.crosspoints[in][o].enabled;
            out->crosspoints[in][o].phase_invert = matrix_mixer.crosspoints[in][o].phase_invert;
            out->crosspoints[in][o].gain_db = matrix_mixer.crosspoints[in][o].gain_db;
        }
    }

    // Matrix outputs
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        out->outputs[o].enabled = matrix_mixer.outputs[o].enabled;
        out->outputs[o].mute = matrix_mixer.outputs[o].mute;
        out->outputs[o].gain_db = matrix_mixer.outputs[o].gain_db;
        out->outputs[o].delay_ms = matrix_mixer.outputs[o].delay_ms;
    }

    // Pin config (always included in GET regardless of include_pins setting)
    out->pins.num_pin_outputs = NUM_PIN_OUTPUTS;
    for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
        out->pins.pins[i] = output_pins[i];
    }

    // EQ bands
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
            out->eq[ch][b].type = filter_recipes[ch][b].type;
            out->eq[ch][b].bypass = (filter_recipes[ch][b].bypass == 1) ? 1 : 0;
            out->eq[ch][b].reserved[0] = 0;
            out->eq[ch][b].reserved[1] = 0;
            out->eq[ch][b].freq = filter_recipes[ch][b].freq;
            out->eq[ch][b].q = filter_recipes[ch][b].Q;
            out->eq[ch][b].gain_db = filter_recipes[ch][b].gain_db;
        }
    }

    // Channel names
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        memcpy(out->channel_names.names[ch], channel_names[ch], PRESET_NAME_LEN);
    }

    // I2S configuration (V3)
    {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint16_t i2s_mck_multiplier;
        memset(&out->i2s_config, 0, sizeof(out->i2s_config));
        memcpy(out->i2s_config.output_types, output_types, NUM_SPDIF_INSTANCES);
        out->i2s_config.bck_pin = i2s_bck_pin;
        out->i2s_config.mck_pin = i2s_mck_pin;
        out->i2s_config.mck_enabled = i2s_mck_enabled ? 1 : 0;
        out->i2s_config.mck_multiplier = (i2s_mck_multiplier == 256) ? 1 : 0;  // 0=128x, 1=256x
    }

    // Volume Leveller (V4+)
    out->leveller.enabled = leveller_config.enabled ? 1 : 0;
    out->leveller.speed = leveller_config.speed;
    out->leveller.lookahead = leveller_config.lookahead ? 1 : 0;
    out->leveller.amount = leveller_config.amount;
    out->leveller.max_gain_db = leveller_config.max_gain_db;
    out->leveller.gate_threshold_db = leveller_config.gate_threshold_db;

    // Per-channel preamp (V6+)
    for (int i = 0; i < NUM_INPUT_CHANNELS && i < WIRE_MAX_INPUT_CHANNELS; i++)
        out->preamp.preamp_db[i] = global_preamp_db[i];

    // Master volume (V6+)
    out->master_volume.master_volume_db = master_volume_db;

    // Input source configuration (V7+; I2S fields V12+)
    out->input_config.input_source = active_input_source;
    out->input_config.spdif_rx_pin = spdif_rx_pin;
    out->input_config.i2s_rx_pin = i2s_rx_pin[0];
    out->input_config.i2s_input_rate = i2s_rate_encode(i2s_input_rate);
    out->input_config.i2s_input_channels = i2s_input_channels;
    for (int p = 0; p < 3; p++)
        out->input_config.i2s_rx_pin_ext[p] =
            (p + 1 < I2S_RX_MAX_PAIRS) ? i2s_rx_pin[p + 1] : 0;

    // LG Sound Sync (V8+).  All four fields are filled here so a single
    // GET round-trips both the user toggle and the runtime observation.
    // bulk_params_apply() honors only `enabled`; the other three are
    // produced by the detection state machine and have no meaningful
    // host-pushable value.
    {
        LgSoundSyncStatus s;
        lg_sound_sync_get_status(&s);
        out->lg_sound_sync.enabled = s.enabled;
        out->lg_sound_sync.present = s.present;
        out->lg_sound_sync.volume  = s.volume;
        out->lg_sound_sync.muted   = s.muted;
    }

    // User volume / mute (V9+).  audio_state.volume is in 8.8 fixed-point
    // dB; convert to float so the wire field matches the vendor command's
    // float dB convention and every other dB field in this packet.
    out->user_volume.user_volume_db = (float)audio_state.volume / 256.0f;
    out->user_volume.user_mute      = user_mute ? 1 : 0;

    // DAC hardware mute (V10+).  Pulled from the module's live config
    // (which mirrors the flash directory's stored value).  The struct
    // layout matches WireDacHwMute byte-for-byte so we can copy directly.
    {
        DacHwMuteConfig hw;
        dac_hw_mute_get_config(&hw);
        memcpy(&out->dac_hw_mute, &hw, sizeof(out->dac_hw_mute));
    }

    // Crossover bands (V11+).  Mirror PEQ's per-band copy but with the
    // crossover-side recipe array.  Master rows (channel < CH_OUT_1) are
    // zeroed: crossovers are an output-channel-only feature, and emitting
    // their live state on master rows would mislead the host into thinking
    // crossover bands exist for inputs.
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        bool is_master = (ch < CH_OUT_1);
        for (int b = 0; b < MAX_XOVER_BANDS; b++) {
            WireBandParams *wb = &out->crossovers.bands[ch][b];
            if (is_master) {
                memset(wb, 0, sizeof(*wb));
                continue;
            }
            wb->type    = xover_recipes[ch][b].type;
            wb->bypass  = (xover_recipes[ch][b].bypass == 1) ? 1 : 0;
            wb->reserved[0] = 0;
            wb->reserved[1] = 0;
            wb->freq    = xover_recipes[ch][b].freq;
            wb->q       = xover_recipes[ch][b].Q;
            wb->gain_db = xover_recipes[ch][b].gain_db;
        }
    }

    // ADAT output configuration (V17+).  RP2350 only; the whole section (including
    // reserved) stays zeroed on RP2040 from the memset above.
#if PICO_RP2350
    out->adat_config.enabled = adat_output_config_enabled() ? 1 : 0;
    out->adat_config.pin     = adat_output_pin();
#endif
}

// ============================================================================
// APPLY: Wire format → live state
// ============================================================================

int bulk_params_apply(const WireBulkParams *in, bool apply_pins) {
#if PICO_RP2350
    if (in->header.platform_id != WIRE_PLATFORM_RP2350)
        return -2;
#else
    if (in->header.platform_id != WIRE_PLATFORM_RP2040)
        return -2;
#endif

    if (in->header.num_channels != NUM_CHANNELS)
        return -3;
    if (in->header.num_input_channels != NUM_INPUT_CHANNELS)
        return -3;
    if (in->header.num_output_channels != NUM_OUTPUT_CHANNELS)
        return -3;
    // Backward compatibility is intentionally broken at V16: accept ONLY the
    // current full-size layout.  Every section is then guaranteed present, so
    // the apply path below is unconditional (no per-version gates).
    if (in->header.format_version != WIRE_FORMAT_VERSION ||
        in->header.payload_length != sizeof(WireBulkParams))
        return -4;

    // Bracket the wholesale state rewrite.  Per-field writes are suppressed
    // and one BULK_INVALIDATED(source=BULK_SET) is emitted at notify_end_bulk().
    notify_begin_bulk(PARAM_SRC_BULK_SET);

    // Global params — preamp from the legacy single field (a baseline applied to
    // all inputs); overridden by the per-channel preamp section below.
    {
        float db = in->global.preamp_gain_db;
        float linear = db_to_linear(db);
        for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
            global_preamp_db[i]      = db;
            global_preamp_mul[i]     = (int32_t)(linear * (float)(1 << 28));
            global_preamp_linear[i]  = linear;
        }
    }

    bypass_master_eq = (in->global.bypass != 0);

    loudness_enabled = (in->global.loudness_enabled != 0);
    loudness_ref_spl = in->global.loudness_ref_spl;
    loudness_intensity_pct = in->global.loudness_intensity_pct;
    loudness_recompute_pending = true;

    // Crossfeed
    crossfeed_config.enabled = (in->crossfeed.enabled != 0);
    crossfeed_config.preset = in->crossfeed.preset;
    crossfeed_config.itd_enabled = (in->crossfeed.itd_enabled != 0);
    crossfeed_config.custom_fc = in->crossfeed.custom_fc;
    crossfeed_config.custom_feed_db = in->crossfeed.custom_feed_db;
    crossfeed_update_pending = true;

    // Legacy channels
    for (int i = 0; i < 3; i++) {
        channel_gain_db[i] = in->legacy.gain_db[i];
        float g = db_to_linear(in->legacy.gain_db[i]);
        channel_gain_mul[i] = (int32_t)(g * 32768.0f);
        channel_gain_linear[i] = g;
        channel_mute[i] = (in->legacy.mute[i] != 0);
    }

    // Delays
    for (int i = 0; i < NUM_CHANNELS; i++) {
        channel_delays_ms[i] = in->delays.delay_ms[i];
    }

    // Matrix crosspoints — inputs 0..N_in-1, direct.
    for (int inp = 0; inp < NUM_INPUT_CHANNELS; inp++) {
        for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
            matrix_mixer.crosspoints[inp][o].enabled = in->crosspoints[inp][o].enabled;
            matrix_mixer.crosspoints[inp][o].phase_invert = in->crosspoints[inp][o].phase_invert;
            matrix_mixer.crosspoints[inp][o].gain_db = in->crosspoints[inp][o].gain_db;
            matrix_mixer.crosspoints[inp][o].gain_linear = db_to_linear(in->crosspoints[inp][o].gain_db);
        }
    }

    // Matrix outputs
    for (int o = 0; o < NUM_OUTPUT_CHANNELS; o++) {
        matrix_mixer.outputs[o].enabled = in->outputs[o].enabled;
        matrix_mixer.outputs[o].mute = in->outputs[o].mute;
        matrix_mixer.outputs[o].gain_db = in->outputs[o].gain_db;
        matrix_mixer.outputs[o].gain_linear = db_to_linear(in->outputs[o].gain_db);
        matrix_mixer.outputs[o].delay_ms = in->outputs[o].delay_ms;
        channel_delays_ms[CH_OUT_1 + o] = in->outputs[o].delay_ms;
    }

    // Pin config
    if (apply_pins) {
#if PICO_RP2350
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2,
            PICO_SPDIF_PIN_3, PICO_SPDIF_PIN_4, PICO_PDM_PIN
        };
#else
        static const uint8_t default_pins[NUM_PIN_OUTPUTS] = {
            PICO_AUDIO_SPDIF_PIN, PICO_SPDIF_PIN_2, PICO_PDM_PIN
        };
#endif
        for (int i = 0; i < NUM_PIN_OUTPUTS; i++) {
            uint8_t pin = in->pins.pins[i];
            bool valid = (pin <= 29) && !(pin >= 23 && pin <= 25);
#if !PICO_RP2350
            if (pin > 28) valid = false;
#endif
            // Never let a pushed config steal a live control interface's
            // GPIOs: over UART/I2C that would sever the link doing the push
            // (the self-lockout the USB-only config rule exists to prevent).
            if (uart_ctrl_owns_pin(pin) || i2c_ctrl_owns_pin(pin)) valid = false;
            output_pins[i] = valid ? pin : default_pins[i];
        }

        // SPDIF RX pin: if valid AND changed, fire the hot-swap when SPDIF
        // input is currently active so the running RX library picks up the
        // new GPIO without a vendor-command round trip.
        {
            uint8_t pin = in->input_config.spdif_rx_pin;
            bool valid = (pin > 0) && (pin <= 29) &&
                         !(pin >= 23 && pin <= 25);
#if !PICO_RP2350
            if (pin > 28) valid = false;
#endif
            if (uart_ctrl_owns_pin(pin) || i2c_ctrl_owns_pin(pin)) valid = false;
            if (valid && pin != spdif_rx_pin) {
                spdif_rx_pin = pin;
                if (active_input_source == INPUT_SOURCE_SPDIF) {
                    extern volatile bool spdif_rx_pin_change_pending;
                    spdif_rx_pin_change_pending = true;
                }
            }
        }

        // I2S RX data pins (pair 0 + multichannel extras) and channel count,
        // validated as a SET so a pushed config can't bring two state machines
        // up on one GPIO or on a clock pin.  0 = keep-live per field; an invalid
        // count keeps the live count.  The proposed active set is checked against
        // the BCK pin THIS transfer installs (in->i2s_config.bck_pin, applied
        // unconditionally below), and applied only if acceptable — otherwise the
        // whole I2S RX section is ignored (live config retained) and logged.  A
        // change restarts the input so every pair re-syncs.
        {
            uint8_t proposed[I2S_RX_MAX_PAIRS];
            proposed[0] = (in->input_config.i2s_rx_pin != 0)
                              ? in->input_config.i2s_rx_pin : i2s_rx_pin[0];
#if I2S_RX_MAX_PAIRS > 1
            for (int p = 1; p < I2S_RX_MAX_PAIRS; p++) {
                uint8_t pin = in->input_config.i2s_rx_pin_ext[p - 1];
                proposed[p] = (pin != 0) ? pin : i2s_rx_pin[p];
            }
#endif
            uint8_t ch = in->input_config.i2s_input_channels;
            uint8_t count = (ch == 2 || ch == 4 || ch == 6 || ch == 8)
                                ? ch : i2s_input_channels;
            if (count / 2 > I2S_RX_MAX_PAIRS) count = i2s_input_channels;

            // Validate the RX set against the BCK that will ACTUALLY be installed
            // (the pushed BCK if it passes its own check below, else the kept-live
            // pin), so a rejected/invalid BCK doesn't leave the RX validated
            // against a value the device never adopts.  Output pins are already
            // applied, so this matches the install-time check.
            uint8_t eff_bck = i2s_bck_pin_acceptable(in->i2s_config.bck_pin)
                                  ? in->i2s_config.bck_pin : i2s_bck_pin;
            if (i2s_rx_pin_set_acceptable(proposed, count / 2, eff_bck)) {
                bool changed = false;
                for (int p = 0; p < I2S_RX_MAX_PAIRS; p++)
                    if (proposed[p] != i2s_rx_pin[p]) {
                        i2s_rx_pin[p] = proposed[p];
                        changed = true;
                    }
                if (count != i2s_input_channels) {
                    i2s_input_channels = count;
                    changed = true;
                }
                if (changed && active_input_source == INPUT_SOURCE_I2S)
                    i2s_input_restart_pending = true;
            } else {
                printf("Bulk apply: I2S RX pin/count config rejected (conflict); kept live\n");
            }
        }
    }

    // EQ bands
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_BANDS; b++) {
            filter_recipes[ch][b].channel = ch;
            filter_recipes[ch][b].band = b;
            filter_recipes[ch][b].type = in->eq[ch][b].type;
            // Normalize at the boundary: 0xFF padding from legacy hosts
            // must not accidentally bypass the band.
            filter_recipes[ch][b].bypass = (in->eq[ch][b].bypass == 1) ? 1 : 0;
            filter_recipes[ch][b].freq = in->eq[ch][b].freq;
            filter_recipes[ch][b].Q = in->eq[ch][b].q;
            filter_recipes[ch][b].gain_db = in->eq[ch][b].gain_db;
        }
    }

    // Channel names
    for (int ch = 0; ch < NUM_CHANNELS; ch++) {
        memcpy(channel_names[ch], in->channel_names.names[ch], PRESET_NAME_LEN);
        channel_names[ch][PRESET_NAME_LEN - 1] = '\0';  // Enforce NUL termination
    }

    // I2S configuration.
    {
        extern uint8_t output_types[];
        extern uint8_t i2s_bck_pin;
        extern uint8_t i2s_mck_pin;
        extern bool    i2s_mck_enabled;
        extern uint16_t i2s_mck_multiplier;
        memcpy(output_types, in->i2s_config.output_types, NUM_SPDIF_INSTANCES);
        // Validate the pushed BCK before installing it raw: BCK/LRCLK are clock
        // OUTPUTS, so an invalid GPIO can fault pio_gpio_init() and a collision
        // with an output pin is driver contention.  Reject (keep the live, known-
        // valid pin) on failure.  Output pins are already applied above, so the
        // conflict check sees the final config; the RX set is validated against
        // the BCK separately.
        if (i2s_bck_pin_acceptable(in->i2s_config.bck_pin)) {
            i2s_bck_pin = in->i2s_config.bck_pin;
        } else {
            printf("Bulk apply: I2S BCK pin %u rejected (invalid/conflict); kept %u\n",
                   (unsigned)in->i2s_config.bck_pin, (unsigned)i2s_bck_pin);
        }

        // MCK pin migration mirrors flash_storage.c apply_slot_to_live():
        // CLK_GPOUTn requires the pin to map to clk_gpout0..3 on this
        // platform.  An RP2040 receiving a bulk payload from an RP2350
        // host (or vice versa) can have an mck_pin that is invalid on
        // this side — fall back to the platform default and disable
        // MCK so the user-visible failure mode is "MCK off" rather than
        // "MCK on a dead pin".
        if (GPIO_TO_GPOUT_CLOCK_HANDLE(in->i2s_config.mck_pin, clk_sys) == clk_sys) {
            i2s_mck_pin = PICO_I2S_MCK_PIN;
            i2s_mck_enabled = false;
        } else {
            i2s_mck_pin = in->i2s_config.mck_pin;
            i2s_mck_enabled = (in->i2s_config.mck_enabled != 0);
        }

        // mck_multiplier encoding: 0 = 128×, 1 = 256×.
        i2s_mck_multiplier = (in->i2s_config.mck_multiplier == 1) ? 256 : 128;
    }

    // Volume Leveller
    {
        leveller_config.enabled = (in->leveller.enabled != 0);
        leveller_config.speed = in->leveller.speed;
        leveller_config.lookahead = (in->leveller.lookahead != 0);
        leveller_config.amount = in->leveller.amount;
        leveller_config.max_gain_db = in->leveller.max_gain_db;
        leveller_config.gate_threshold_db = in->leveller.gate_threshold_db;
    }
    leveller_update_pending = true;
    leveller_reset_pending = true;

    // Per-channel preamp — overrides the legacy single value set above.
    for (int i = 0; i < NUM_INPUT_CHANNELS; i++) {
        float db = in->preamp.preamp_db[i];
        float linear = db_to_linear(db);
        global_preamp_db[i]      = db;
        global_preamp_mul[i]     = (int32_t)(linear * (float)(1 << 28));
        global_preamp_linear[i]  = linear;
    }

    // Master volume (bulk params always applies, ignoring the directory flag).
    // Delegated to update_master_volume() for consistent clamping and to emit
    // a device→host notification via interrupt EP 0x83.
    {
        float db = in->master_volume.master_volume_db;
        if (!isfinite(db)) db = MASTER_VOL_MAX_DB;
        update_master_volume(db);
    }

    // Input source
    {
        uint8_t src = in->input_config.input_source;
        if (input_source_valid(src) && src != active_input_source) {
            pending_input_source = src;
            __dmb();
            input_source_change_pending = true;
        }
    }

    // I2S input rate.  Store only; the bulk_params_pending handler in main.c
    // owns triggering a deferred rate change after it restarts the input, and
    // an input-source switch picks the rate up on its own.
    i2s_input_rate = i2s_rate_decode(in->input_config.i2s_input_rate);

    // LG Sound Sync (V8+ payloads).  Only `enabled` is honored — the
    // other three fields are runtime observations the host cannot push.
    // Using the public setter ensures any side-effects (demote on
    // disable, streak reset on enable) fire correctly; the PARAM_CHANGED
    // notification it emits is suppressed by the bulk bracket and
    // replaced by a single BULK_INVALIDATED at end.
    lg_sound_sync_set_enabled(in->lg_sound_sync.enabled != 0);

    // User volume + mute (V9+ payloads).  Routed through update_user_volume()
    // so the same clamp/encode/apply funnel runs as on REQ_SET_USER_VOLUME
    // (vol_mul + loudness coefficient pointer move together, LG cache is
    // invalidated).  user_mute is a plain bool so the apply is just a
    // store + notify_param_write, suppressed by the bulk bracket.
    {
        update_user_volume(in->user_volume.user_volume_db);
        user_mute = (in->user_volume.user_mute != 0);
        uint8_t mv = user_mute ? 1 : 0;
        notify_param_write(offsetof(WireBulkParams, user_volume.user_mute),
                           sizeof(uint8_t), &mv);
    }

    // DAC hardware mute (V10+).  Funnel through dac_hw_mute_set_config()
    // so validation, persistence, and pin re-claim all run consistently
    // with the vendor-command path.  An invalid config in the payload
    // (out-of-range pin, collision, bad timing) is silently ignored —
    // the bulk SET dispatcher has no per-section error channel, so this
    // matches every other section that "best-effort applies" what it
    // can.  Bulk SETs that include a dac_hw_mute section that fails
    // validation leave the previous config in place.
    {
        DacHwMuteConfig hw;
        _Static_assert(sizeof(hw) == sizeof(in->dac_hw_mute),
                       "WireDacHwMute and DacHwMuteConfig must match");
        memcpy(&hw, &in->dac_hw_mute, sizeof(hw));
        (void)dac_hw_mute_set_config(&hw);
    }

    // Crossover bands.  Output rows applied; input rows skipped (crossover is
    // output-channel-only).  Band-field is always overwritten with the wire
    // index `XOVER_BAND_BASE + i` so a stale local index in the payload cannot
    // trigger the live-edit misrouting bug described in crossover_filters_spec.md.
    for (int ch = CH_OUT_1; ch < NUM_CHANNELS; ch++) {
        for (int b = 0; b < MAX_XOVER_BANDS; b++) {
            const WireBandParams *wb = &in->crossovers.bands[ch][b];
            xover_recipes[ch][b].channel = (uint8_t)ch;
            xover_recipes[ch][b].band    = (uint8_t)(XOVER_BAND_BASE + b);
            xover_recipes[ch][b].type    = wb->type;
            xover_recipes[ch][b].bypass  = (wb->bypass == 1) ? 1 : 0;
            xover_recipes[ch][b].freq    = wb->freq;
            xover_recipes[ch][b].Q       = wb->q;
            xover_recipes[ch][b].gain_db = wb->gain_db;
        }
    }
    // Caller invokes dsp_recalculate_all_filters() after this returns.

    // ADAT output configuration (V17+).  RP2350 only; RP2040 ignores the section.
    // ADAT is a push-pull output driver, so the pin gets the full ownership
    // check (adat_pin_acceptable) and a bad pin is rejected-and-kept-live.
    // pin == 0 means the platform default.
#if PICO_RP2350
    {
        uint8_t enabled = (in->adat_config.enabled != 0) ? 1 : 0;
        uint8_t pin = (in->adat_config.pin != 0) ? in->adat_config.pin : PICO_ADAT_PIN;
        if (!adat_pin_acceptable(pin)) {
            printf("Bulk apply: ADAT pin %u rejected (invalid/conflict); kept %u\n",
                   (unsigned)pin, (unsigned)adat_output_pin());
            pin = adat_output_pin();
        }
        adat_output_set_config(enabled != 0, pin);
    }
#endif

    // Close the bulk bracket — emits BULK_INVALIDATED(source=BULK_SET).
    notify_end_bulk();
    return 0;
}
