/*
 * control_surfaces.c; runtime engine for user-wired controls and indicators.
 *
 * Main-loop only.  A self-throttled 1 kHz tick polls every active binding:
 * buttons/switches are debounced, encoders are decoded with a quadrature
 * transition table, pots are read via the ADC (one per tick, round-robin)
 * with EMA smoothing and half-dB quantization, LEDs are re-evaluated against
 * their noun's live state.  Every resulting change is applied by dispatching
 * the corresponding vendor command through vendor_dispatch_set/get with
 * CTRL_SOURCE_GPIO, so validation, deferred-apply safety, and PARAM_SRC_GPIO
 * host notifications all come from the existing command surface.
 *
 * No GPIO IRQs and no PIO resources are used; polling at 1 kHz comfortably
 * tracks hand-operated detented encoders (a fast spin is ~250 quarter-steps
 * per second, 4 samples per transition).
 */

#include "control_surfaces.h"
#include "config.h"
#include "vendor_commands.h"
#include "usb_audio.h"
#include "audio_input.h"
#include "flash_storage.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <math.h>
#include <string.h>

// Poll cadence and input conditioning
#define CS_TICK_INTERVAL_US   1000u
#define CS_DEBOUNCE_TICKS     10      // 10 ms stable before a level is accepted
#define CS_POT_SEED_TICKS     50      // EMA settle time before the boot sync
#define CS_POT_DEADBAND       12      // raw ADC counts (~0.3%) before remapping
#define CS_LEVEL_UNKNOWN      0xFF

// ADC-capable pot pins: GPIO 26..28 = ADC channels 0..2 on both platforms.
// GPIO 29 is the board's VSYS/3 monitor on Pico and Pico 2, so it is excluded.
#define CS_ADC_PIN_FIRST      26
#define CS_ADC_PIN_LAST       28

// Deferred SET handoff (vendor_commands.c writes, main.c consumes)
volatile bool    cs_set_binding_pending = false;
uint8_t          cs_set_binding_slot = 0;
CsBinding        cs_set_binding_val;
volatile uint8_t cs_last_status = PIN_CONFIG_SUCCESS;
volatile uint8_t cs_last_slot = 0;

// ---------------------------------------------------------------------------
// Capability tables; the single source of truth for the validity model.
// REQ_GET_CS_CAPS serves these verbatim, so host UIs and the firmware can
// never disagree about which type/noun/action combinations are legal.
// ---------------------------------------------------------------------------

static const CsCapsHeader s_caps = {
    .caps_version = 1,
    .max_bindings = CS_MAX_BINDINGS,
    .type_count   = CS_TYPE_COUNT,
    .noun_count   = CS_NOUN_COUNT,
    .types = {
        [CS_TYPE_NONE]    = { 0, 0, CS_PINCLASS_ANY },
        [CS_TYPE_BUTTON]  = { CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) |
                              CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) |
                              CS_ACT_BIT(CS_ACT_TRIGGER), 1, CS_PINCLASS_ANY },
        [CS_TYPE_SWITCH]  = { CS_ACT_BIT(CS_ACT_FOLLOW), 1, CS_PINCLASS_ANY },
        [CS_TYPE_POT]     = { CS_ACT_BIT(CS_ACT_ADJUST), 1, CS_PINCLASS_ADC },
        [CS_TYPE_ENCODER] = { CS_ACT_BIT(CS_ACT_STEP), 2, CS_PINCLASS_ANY },
        [CS_TYPE_LED]     = { CS_ACT_BIT(CS_ACT_IND_EQUALS), 1, CS_PINCLASS_ANY },
    },
};

#define CS_CONTROL_ACTS  (CS_ACT_BIT(CS_ACT_ADJUST) | CS_ACT_BIT(CS_ACT_STEP) | \
                          CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) | \
                          CS_ACT_BIT(CS_ACT_SET))
#define CS_BOOL_ACTS     (CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) | \
                          CS_ACT_BIT(CS_ACT_FOLLOW) | CS_ACT_BIT(CS_ACT_IND_EQUALS))
#define CS_ENUM_ACTS     (CS_ACT_BIT(CS_ACT_STEP) | CS_ACT_BIT(CS_ACT_INC) | \
                          CS_ACT_BIT(CS_ACT_DEC) | CS_ACT_BIT(CS_ACT_SET) | \
                          CS_ACT_BIT(CS_ACT_IND_EQUALS))

static const CsNounDesc s_noun_desc[CS_NOUN_COUNT] = {
    [CS_NOUN_USER_VOLUME]   = { CS_KIND_CONTINUOUS, 0, CS_CONTROL_ACTS,
                                (int16_t)(-CENTER_VOLUME_INDEX * 256), 0 },
    [CS_NOUN_MASTER_VOLUME] = { CS_KIND_CONTINUOUS, 0, CS_CONTROL_ACTS,
                                (int16_t)(-127 * 256), 0 },
    [CS_NOUN_USER_MUTE]     = { CS_KIND_BOOL, 0, CS_BOOL_ACTS, 0, 0 },
    [CS_NOUN_LOUDNESS]      = { CS_KIND_BOOL, 0, CS_BOOL_ACTS, 0, 0 },
    [CS_NOUN_CROSSFEED]     = { CS_KIND_BOOL, 0, CS_BOOL_ACTS, 0, 0 },
    [CS_NOUN_LEVELLER]      = { CS_KIND_BOOL, 0, CS_BOOL_ACTS, 0, 0 },
    [CS_NOUN_PRESET]        = { CS_KIND_ENUM, PRESET_SLOTS, CS_ENUM_ACTS, 0, 0 },
    [CS_NOUN_INPUT_SOURCE]  = { CS_KIND_ENUM, INPUT_SOURCE_MAX + 1, CS_ENUM_ACTS, 0, 0 },
    [CS_NOUN_CLIP]          = { CS_KIND_BOOL, 0,
                                CS_ACT_BIT(CS_ACT_TRIGGER) | CS_ACT_BIT(CS_ACT_IND_EQUALS),
                                0, 0 },
};

// ---------------------------------------------------------------------------
// Live state
// ---------------------------------------------------------------------------

// Pending dispatch kinds.  Actions resolve to an absolute target at event
// time, so a BUSY retry (USB control SET in flight) can never re-toggle.
enum { CS_OP_SET_FLOAT, CS_OP_SET_U8, CS_OP_PRESET_LOAD, CS_OP_CLEAR_CLIPS };

typedef struct {
    bool     active;
    // buttons / switches
    uint8_t  stable;        // debounced logical level; CS_LEVEL_UNKNOWN at claim
    uint8_t  candidate;     // level being debounced
    uint8_t  debounce;
    // encoder
    uint8_t  enc_prev;      // previous AB state (2 bits)
    int8_t   enc_accum;     // quarter-steps toward a detent
    // pot
    uint16_t pot_filt;      // EMA-filtered 12-bit ADC value
    uint16_t pot_settle;    // seed ticks remaining before the boot sync
    uint16_t pot_sent_raw;  // filtered value at last dispatch
    int16_t  pot_sent_half; // half-dB units at last dispatch
    // LED
    uint8_t  led_lit;       // CS_LEVEL_UNKNOWN forces the first write
    // deferred dispatch (retried while the command surface reports BUSY)
    bool     op_pending;
    float    op_value;
    // enum nouns apply deferred (preset load, input source): step from the
    // last dispatched target while the live value catches up, so rapid
    // detents are not coalesced against a stale current value
    int16_t  target_shadow; // -1 = none
    uint16_t shadow_age;    // ticks since dispatch (timeout guard)
} CsRuntime;

#define CS_SHADOW_TIMEOUT_TICKS  500   // drop a never-confirmed target after 500 ms

static CsFlashConfig s_cfg;                      // live (and persisted) config
static CsRuntime     s_rt[CS_MAX_BINDINGS];
static uint8_t       s_slot_status[CS_MAX_BINDINGS];
static bool          s_any_active = false;
static uint8_t       s_pot_rr = 0;               // round-robin pot cursor
static uint64_t      s_last_tick_us = 0;

// Standard quadrature transition table, indexed by (prev << 2) | curr.
// Invalid two-bit jumps decode as 0 (skipped sample, no movement credited).
static const int8_t s_enc_table[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0,
};

// ---------------------------------------------------------------------------
// Noun access
// ---------------------------------------------------------------------------

// Live value of a noun as a float (bool 0/1, enum index, continuous dB).
static float cs_noun_get(uint8_t noun) {
    switch (noun) {
        case CS_NOUN_USER_VOLUME:   return (float)audio_state.volume * (1.0f / 256.0f);
        case CS_NOUN_MASTER_VOLUME: {
            float db = master_volume_db;   // -128 mute sentinel steps up from the floor
            return (db < MASTER_VOL_MIN_DB) ? MASTER_VOL_MIN_DB : db;
        }
        case CS_NOUN_USER_MUTE:     return user_mute ? 1.0f : 0.0f;
        case CS_NOUN_LOUDNESS:      return loudness_enabled ? 1.0f : 0.0f;
        case CS_NOUN_CROSSFEED:     return crossfeed_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_LEVELLER:      return leveller_config.enabled ? 1.0f : 0.0f;
        case CS_NOUN_PRESET:        return (float)preset_get_active();
        case CS_NOUN_INPUT_SOURCE:  return (float)active_input_source;
        case CS_NOUN_CLIP:          return global_status.clip_flags ? 1.0f : 0.0f;
        default:                    return 0.0f;
    }
}

// Dispatch a resolved target through the shared vendor-command surface.
// Returns false only on BUSY (a USB control SET is mid-flight); the caller
// latches the op and retries next tick.  OK and ERROR both count as done.
static bool cs_try_dispatch(uint8_t noun, float value) {
    const uint8_t *rd; uint16_t rl;
    CtrlDispatchResult r;
    switch (noun) {
        case CS_NOUN_USER_VOLUME:
        case CS_NOUN_MASTER_VOLUME: {
            float f = value;
            uint8_t req = (noun == CS_NOUN_USER_VOLUME) ? REQ_SET_USER_VOLUME
                                                        : REQ_SET_MASTER_VOLUME;
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, 0, 0,
                                    (const uint8_t *)&f, sizeof(f));
            break;
        }
        case CS_NOUN_PRESET:
            // Same deferred, pipeline-safe path a host load takes.
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_PRESET_LOAD,
                                    (uint16_t)value, 0, 1, &rd, &rl);
            break;
        case CS_NOUN_CLIP:
            r = vendor_dispatch_get(CTRL_SOURCE_GPIO, REQ_CLEAR_CLIPS,
                                    0, 0, 4, &rd, &rl);
            break;
        default: {
            uint8_t v = (uint8_t)value;
            uint8_t req;
            switch (noun) {
                case CS_NOUN_USER_MUTE:    req = REQ_SET_USER_MUTE;       break;
                case CS_NOUN_LOUDNESS:     req = REQ_SET_LOUDNESS;        break;
                case CS_NOUN_CROSSFEED:    req = REQ_SET_CROSSFEED;       break;
                case CS_NOUN_LEVELLER:     req = REQ_SET_LEVELLER_ENABLE; break;
                case CS_NOUN_INPUT_SOURCE: req = REQ_SET_INPUT_SOURCE;    break;
                default:                   return true;
            }
            r = vendor_dispatch_set(CTRL_SOURCE_GPIO, req, 0, 0, &v, 1);
            break;
        }
    }
    return r != CTRL_DISPATCH_BUSY;
}

static void cs_queue_op(uint8_t slot, float value) {
    CsRuntime *rt = &s_rt[slot];
    const CsBinding *b = &s_cfg.bindings[slot];
    if (s_noun_desc[b->noun].kind == CS_KIND_ENUM) {
        rt->target_shadow = (int16_t)value;
        rt->shadow_age = 0;
    }
    if (cs_try_dispatch(b->noun, value)) {
        rt->op_pending = false;
    } else {
        rt->op_pending = true;
        rt->op_value = value;
    }
}

// Base value for relative actions: the latched (not-yet-dispatched) target
// when one is pending, else the target shadow for deferred-apply enum nouns,
// else the live value.  Keeps rapid detents accumulating correctly across
// BUSY retries and across a preset load / input switch still in flight.
static float cs_base_value(uint8_t slot) {
    const CsRuntime *rt = &s_rt[slot];
    const CsBinding *b = &s_cfg.bindings[slot];
    if (rt->op_pending) return rt->op_value;
    if (s_noun_desc[b->noun].kind == CS_KIND_ENUM && rt->target_shadow >= 0)
        return (float)rt->target_shadow;
    return cs_noun_get(b->noun);
}

// ---------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------

static float cs_step_db(const CsBinding *b) {
    return (b->step > 0 ? (float)b->step : 256.0f) * (1.0f / 256.0f);
}

static float cs_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Step an enum noun by dir (+1/-1).  Presets step across OCCUPIED slots
// only; an empty device is a no-op.  Returns the target index or -1 for
// no movement.
static int cs_enum_step(uint8_t slot, int dir) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &s_noun_desc[b->noun];
    int count = nd->enum_count;
    int cur = (int)cs_base_value(slot);
    bool wrap = (b->flags & CS_FLAG_WRAP) != 0;

    if (b->noun == CS_NOUN_PRESET) {
        uint16_t occ; uint8_t sm, ds, la, ocm, mvm;
        preset_get_directory(&occ, &sm, &ds, &la, &ocm, &mvm);
        if (!occ) return -1;
        for (int i = 1; i <= count; i++) {
            int cand = cur + dir * i;
            if (wrap) cand = ((cand % count) + count) % count;
            else if (cand < 0 || cand >= count) return -1;
            if (occ & (1u << cand)) return (cand == cur) ? -1 : cand;
        }
        return -1;
    }

    int t = cur + dir;
    if (wrap) t = ((t % count) + count) % count;
    else t = t < 0 ? 0 : (t >= count ? count - 1 : t);
    return (t == cur) ? -1 : t;
}

// Apply one relative step (encoder detent or INC/DEC press).
static void cs_apply_step(uint8_t slot, int dir) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &s_noun_desc[b->noun];
    if (nd->kind == CS_KIND_CONTINUOUS) {
        float lo = (float)nd->min_q8 * (1.0f / 256.0f);
        float hi = (float)nd->max_q8 * (1.0f / 256.0f);
        cs_queue_op(slot, cs_clampf(cs_base_value(slot) + dir * cs_step_db(b), lo, hi));
    } else {
        int t = cs_enum_step(slot, dir);
        if (t >= 0) cs_queue_op(slot, (float)t);
    }
}

static void cs_button_press(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    switch (b->action) {
        case CS_ACT_INC:     cs_apply_step(slot, +1); break;
        case CS_ACT_DEC:     cs_apply_step(slot, -1); break;
        case CS_ACT_TOGGLE:  cs_queue_op(slot, cs_base_value(slot) >= 0.5f ? 0.0f : 1.0f); break;
        case CS_ACT_TRIGGER: cs_queue_op(slot, 0.0f); break;
        case CS_ACT_SET: {
            const CsNounDesc *nd = &s_noun_desc[b->noun];
            float v = (nd->kind == CS_KIND_CONTINUOUS)
                    ? (float)b->value * (1.0f / 256.0f) : (float)b->value;
            cs_queue_op(slot, v);
            break;
        }
        default: break;
    }
}

// ---------------------------------------------------------------------------
// Per-type tick handlers
// ---------------------------------------------------------------------------

// Logical level: 1 = pressed / on.  Default wiring is component-to-GND with
// the internal pull-up; CS_FLAG_INVERT selects active-high with pull-down.
static uint8_t cs_read_level(const CsBinding *b, uint8_t pin) {
    uint8_t raw = gpio_get(pin) ? 1 : 0;
    return (b->flags & CS_FLAG_INVERT) ? raw : (uint8_t)!raw;
}

static void cs_tick_button_switch(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    uint8_t level = cs_read_level(b, b->gpio[0]);

    if (level != rt->candidate) {
        rt->candidate = level;
        rt->debounce = 0;
        return;
    }
    if (rt->candidate == rt->stable) { rt->debounce = 0; return; }
    if (++rt->debounce < CS_DEBOUNCE_TICKS) return;
    rt->debounce = 0;

    bool first = (rt->stable == CS_LEVEL_UNKNOWN);
    rt->stable = rt->candidate;
    if (b->type == CS_TYPE_SWITCH) {
        // A switch is authoritative, including at claim/boot (matches the
        // pot's immediate-takeover semantics).
        cs_queue_op(slot, (float)rt->stable);
    } else if (!first && rt->stable == 1) {
        cs_button_press(slot);
    }
}

static void cs_tick_encoder(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    uint8_t curr = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                             (gpio_get(b->gpio[1]) ? 1 : 0));
    int8_t d = s_enc_table[(rt->enc_prev << 2) | curr];
    rt->enc_prev = curr;
    if (!d) return;
    rt->enc_accum += (b->flags & CS_FLAG_REVERSE) ? -d : d;
    // One detent = 4 quarter-steps (both channels through a full cycle).
    if (rt->enc_accum >= 4)       { rt->enc_accum -= 4; cs_apply_step(slot, +1); }
    else if (rt->enc_accum <= -4) { rt->enc_accum += 4; cs_apply_step(slot, -1); }
}

// Map a filtered ADC reading onto the binding's dB span in half-dB units.
static int16_t cs_pot_map_half_db(const CsBinding *b, uint16_t filt) {
    const CsNounDesc *nd = &s_noun_desc[b->noun];
    float lo = (float)nd->min_q8 * (1.0f / 256.0f);
    float hi = (float)nd->max_q8 * (1.0f / 256.0f);
    if (b->range_min != 0 || b->range_max != 0) {
        lo = (float)b->range_min * (1.0f / 256.0f);
        hi = (float)b->range_max * (1.0f / 256.0f);
    }
    float pos = (float)filt * (1.0f / 4095.0f);
    if (b->flags & CS_FLAG_REVERSE) pos = 1.0f - pos;
    return (int16_t)lroundf((lo + (hi - lo) * pos) * 2.0f);
}

static void cs_tick_pot(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    adc_select_input(b->gpio[0] - CS_ADC_PIN_FIRST);
    uint16_t raw = adc_read();
    // Truncate-toward-zero divide keeps the EMA directionally symmetric (an
    // arithmetic shift would creep on falling input but stall on rising).
    rt->pot_filt = (uint16_t)((int32_t)rt->pot_filt +
                              (((int32_t)raw - rt->pot_filt) / 8));

    if (rt->pot_settle) {
        // Immediate-takeover boot sync: once the EMA settles, the knob's
        // physical position becomes the value.
        if (--rt->pot_settle == 0) {
            rt->pot_sent_half = cs_pot_map_half_db(b, rt->pot_filt);
            rt->pot_sent_raw = rt->pot_filt;
            cs_queue_op(slot, (float)rt->pot_sent_half * 0.5f);
        }
        return;
    }

    int32_t moved = (int32_t)rt->pot_filt - rt->pot_sent_raw;
    if (moved < 0) moved = -moved;
    if (moved <= CS_POT_DEADBAND) return;
    int16_t half = cs_pot_map_half_db(b, rt->pot_filt);
    if (half != rt->pot_sent_half) {
        rt->pot_sent_half = half;
        rt->pot_sent_raw = rt->pot_filt;
        cs_queue_op(slot, (float)half * 0.5f);
    }
}

static void cs_tick_led(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    uint8_t lit = ((int)cs_noun_get(b->noun) == (int)b->value) ? 1 : 0;
    if (lit == rt->led_lit) return;
    rt->led_lit = lit;
    gpio_put(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? !lit : lit);
}

// ---------------------------------------------------------------------------
// Pin claim / release and validation
// ---------------------------------------------------------------------------

static uint8_t cs_pin_count(uint8_t type) {
    return (type < CS_TYPE_COUNT) ? s_caps.types[type].pin_count : 0;
}

static void cs_claim_pins(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    switch (b->type) {
        case CS_TYPE_POT:
            adc_gpio_init(b->gpio[0]);
            break;
        case CS_TYPE_LED:
            gpio_init(b->gpio[0]);
            gpio_put(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? 1 : 0);  // off
            gpio_set_dir(b->gpio[0], GPIO_OUT);
            break;
        default:  // button / switch / encoder inputs
            for (int i = 0; i < cs_pin_count(b->type); i++) {
                gpio_init(b->gpio[i]);
                gpio_set_dir(b->gpio[i], GPIO_IN);
                if (b->flags & CS_FLAG_INVERT) gpio_pull_down(b->gpio[i]);
                else                           gpio_pull_up(b->gpio[i]);
            }
            break;
    }
}

static void cs_release_pins(const CsBinding *b) {
    for (int i = 0; i < cs_pin_count(b->type); i++) {
        gpio_deinit(b->gpio[i]);
        gpio_disable_pulls(b->gpio[i]);
    }
}

static void cs_seed_runtime(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    memset(rt, 0, sizeof(*rt));
    rt->target_shadow = -1;
    switch (b->type) {
        case CS_TYPE_BUTTON:
        case CS_TYPE_SWITCH:
            rt->stable = CS_LEVEL_UNKNOWN;
            rt->candidate = CS_LEVEL_UNKNOWN;
            break;
        case CS_TYPE_ENCODER:
            rt->enc_prev = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                                     (gpio_get(b->gpio[1]) ? 1 : 0));
            break;
        case CS_TYPE_POT:
            adc_select_input(b->gpio[0] - CS_ADC_PIN_FIRST);
            rt->pot_filt = adc_read();
            rt->pot_settle = CS_POT_SEED_TICKS;
            break;
        case CS_TYPE_LED:
            rt->led_lit = CS_LEVEL_UNKNOWN;
            break;
        default:
            break;
    }
    rt->active = true;
}

static void cs_recount_active(void) {
    s_any_active = false;
    for (int i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_rt[i].active) { s_any_active = true; break; }
}

// Full validity check for a proposed binding.  Pin checks run against the
// live device with this slot's own pins already released by the caller.
static uint8_t cs_validate(const CsBinding *b) {
    if (b->type >= CS_TYPE_COUNT) return CS_STATUS_INVALID_TYPE;
    if (b->noun >= CS_NOUN_COUNT) return CS_STATUS_INVALID_NOUN;
    if (b->action >= CS_ACT_COUNT) return CS_STATUS_INVALID_ACTION;

    if (b->flags & (uint8_t)~(CS_FLAG_INVERT | CS_FLAG_REVERSE | CS_FLAG_WRAP))
        return CS_STATUS_INVALID_VALUE;   // unknown flag bits

    const CsTypeDesc *td = &s_caps.types[b->type];
    const CsNounDesc *nd = &s_noun_desc[b->noun];
    uint16_t bit = CS_ACT_BIT(b->action);
    if (!(td->actions & bit) || !(nd->actions & bit)) return CS_STATUS_INVALID_ACTION;

    // Value / step / range bounds per kind
    switch (nd->kind) {
        case CS_KIND_CONTINUOUS:
            if (b->action == CS_ACT_SET &&
                (b->value < nd->min_q8 || b->value > nd->max_q8))
                return CS_STATUS_INVALID_VALUE;
            if (b->step < 0) return CS_STATUS_INVALID_VALUE;
            if (b->action == CS_ACT_ADJUST &&
                (b->range_min != 0 || b->range_max != 0)) {
                if (b->range_min >= b->range_max ||
                    b->range_min < nd->min_q8 || b->range_max > nd->max_q8)
                    return CS_STATUS_INVALID_VALUE;
            }
            break;
        case CS_KIND_BOOL:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_EQUALS) &&
                (b->value < 0 || b->value > 1))
                return CS_STATUS_INVALID_VALUE;
            break;
        case CS_KIND_ENUM:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_EQUALS) &&
                (b->value < 0 || b->value >= nd->enum_count))
                return CS_STATUS_INVALID_VALUE;
            break;
        default:
            break;
    }

    // Pins: distinct, valid, unclaimed anywhere (ctrl_iface_check_pin treats
    // the I2S clock pair as always reserved; CS pins are long-lived too).
    uint8_t n = td->pin_count;
    if (n == 2 && b->gpio[0] == b->gpio[1]) return PIN_CONFIG_INVALID_PIN;
    for (int i = 0; i < n; i++) {
        uint8_t pin = b->gpio[i];
        if (td->pin_class == CS_PINCLASS_ADC &&
            (pin < CS_ADC_PIN_FIRST || pin > CS_ADC_PIN_LAST))
            return CS_STATUS_PIN_NOT_ADC;
        uint8_t st = ctrl_iface_check_pin(pin);
        if (st != PIN_CONFIG_SUCCESS) return st;
    }
    return PIN_CONFIG_SUCCESS;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool control_surfaces_owns_pin(uint8_t pin) {
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == pin) return true;
    }
    return false;
}

uint8_t control_surfaces_apply_binding(uint8_t slot, const CsBinding *nb) {
    if (slot >= CS_MAX_BINDINGS || !nb) return CS_STATUS_INVALID_SLOT;

    // Release this slot first so the new pins can reuse them; restore the
    // old binding untouched if the new one fails validation.
    CsBinding old = s_cfg.bindings[slot];
    bool was_active = s_rt[slot].active;
    if (was_active) {
        cs_release_pins(&old);
        s_rt[slot].active = false;
    }

    if (nb->type != CS_TYPE_NONE) {
        uint8_t st = cs_validate(nb);
        if (st != PIN_CONFIG_SUCCESS) {
            if (was_active) {
                s_cfg.bindings[slot] = old;
                cs_claim_pins(slot);
                cs_seed_runtime(slot);
            }
            cs_recount_active();
            return st;
        }
    }

    s_cfg.bindings[slot] = *nb;
    memset(&s_rt[slot], 0, sizeof(s_rt[slot]));
    if (nb->type != CS_TYPE_NONE) {
        cs_claim_pins(slot);
        cs_seed_runtime(slot);
    }
    s_slot_status[slot] = PIN_CONFIG_SUCCESS;
    cs_recount_active();
    return PIN_CONFIG_SUCCESS;
}

void control_surfaces_init(void) {
    CsFlashConfig stored;
    preset_get_cs_config(&stored);
    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(s_rt, 0, sizeof(s_rt));
    memset(s_slot_status, 0, sizeof(s_slot_status));
    s_cfg.version = CS_CONFIG_VERSION;
    if (stored.version > CS_CONFIG_VERSION) return;   // future format; stay idle

    for (uint8_t slot = 0; slot < CS_MAX_BINDINGS; slot++) {
        const CsBinding *b = &stored.bindings[slot];
        if (b->type == CS_TYPE_NONE) continue;
        uint8_t st = control_surfaces_apply_binding(slot, b);
        if (st != PIN_CONFIG_SUCCESS) {
            // Keep the stored binding visible (inactive) so the config
            // survives round-trips; the failure is reported in slot_status.
            s_cfg.bindings[slot] = *b;
            s_slot_status[slot] = st;
        }
    }
}

void control_surfaces_tick(void) {
    if (!s_any_active) return;
    uint64_t now = time_us_64();
    if (now - s_last_tick_us < CS_TICK_INTERVAL_US) return;
    s_last_tick_us = now;

    // Retry ops parked on a BUSY dispatch, and retire enum target shadows
    // once the live value confirms them (or on timeout), before sampling
    // new events.
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        CsRuntime *rt = &s_rt[s];
        if (!rt->active) continue;
        if (rt->op_pending &&
            cs_try_dispatch(s_cfg.bindings[s].noun, rt->op_value))
            rt->op_pending = false;
        if (rt->target_shadow >= 0 &&
            ((int)cs_noun_get(s_cfg.bindings[s].noun) == rt->target_shadow ||
             ++rt->shadow_age >= CS_SHADOW_TIMEOUT_TICKS))
            rt->target_shadow = -1;
    }

    bool pot_done = false;   // one ADC conversion per tick, round-robin
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++) {
        uint8_t s = (uint8_t)((s_pot_rr + 1 + i) % CS_MAX_BINDINGS);
        if (!s_rt[s].active) continue;
        switch (s_cfg.bindings[s].type) {
            case CS_TYPE_BUTTON:
            case CS_TYPE_SWITCH:  cs_tick_button_switch(s); break;
            case CS_TYPE_ENCODER: cs_tick_encoder(s);       break;
            case CS_TYPE_LED:     cs_tick_led(s);           break;
            case CS_TYPE_POT:
                if (!pot_done) { cs_tick_pot(s); s_pot_rr = s; pot_done = true; }
                break;
            default: break;
        }
    }
}

const CsFlashConfig *control_surfaces_config(void) { return &s_cfg; }

const CsBinding *control_surfaces_get_binding(uint8_t slot) {
    return (slot < CS_MAX_BINDINGS) ? &s_cfg.bindings[slot] : NULL;
}

void control_surfaces_get_status(CsStatusPacket *out) {
    if (!out) return;
    out->last_status = cs_last_status;
    out->last_slot = cs_last_slot;
    out->max_bindings = CS_MAX_BINDINGS;
    out->active_mask = 0;
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s_rt[s].active) out->active_mask |= (uint8_t)(1u << s);
        out->slot_status[s] = s_slot_status[s];
    }
}

const CsCapsHeader *control_surfaces_caps_header(void) { return &s_caps; }

const CsNounDesc *control_surfaces_noun_desc(uint8_t noun) {
    return (noun < CS_NOUN_COUNT) ? &s_noun_desc[noun] : NULL;
}
