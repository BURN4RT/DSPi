/*
 * control_surfaces.c; runtime engine for user-wired controls and indicators.
 *
 * Main-loop only.  A self-throttled 1 kHz tick polls every active binding:
 * button GPIOs are debounced and decoded per-pin into press / long-press /
 * double-press / hold gestures (several bindings may share one button GPIO
 * with distinct events), switches level-follow, encoders are decoded with a
 * quadrature transition table (optionally rate-accelerated), pots are read
 * via the ADC (one per tick, round-robin) with EMA smoothing and unit-aware
 * quantization, and LEDs (on/off or PWM-dimmed) are re-evaluated against
 * their noun's live state on a decimated schedule.  Every resulting change
 * is applied by dispatching the corresponding vendor command through
 * vendor_dispatch_set/get with CTRL_SOURCE_GPIO, so validation,
 * deferred-apply safety, and PARAM_SRC_GPIO host notifications all come
 * from the existing command surface.
 *
 * The engine is unit- and noun-agnostic: value semantics (dB vs Hz vs Q,
 * linear vs logarithmic stepping, live reads, dispatch mapping, target
 * validation) live in control_surfaces_nouns.c behind cs_noun_get /
 * cs_noun_dispatch / cs_noun_validate_target and the cs_noun_table.
 *
 * No GPIO IRQs and no PIO resources are used; polling at 1 kHz comfortably
 * tracks hand-operated detented encoders (a fast spin is ~250 quarter-steps
 * per second, 4 samples per transition).  PWM LEDs use the otherwise-unused
 * hardware PWM slices.
 *
 * Serialization assumption (load-bearing): this tick, the host command
 * handlers (tud_task), and the binding-apply handler all run on the core0
 * main loop.  The unguarded ADC muxing and the shared EQ pending packet
 * rely on that; nothing here may move to ISR or timer context.
 */

#include "control_surfaces.h"
#include "config.h"
#include "vendor_commands.h"
#include "flash_storage.h"

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"

#include <math.h>
#include <string.h>

// Poll cadence and input conditioning
#define CS_TICK_INTERVAL_US   1000u
#define CS_DEBOUNCE_TICKS     10      // 10 ms stable before a level is accepted
#define CS_POT_SEED_TICKS     50      // EMA settle time before the boot sync
#define CS_POT_DEADBAND       12      // raw ADC counts (~0.3%) before remapping
#define CS_LEVEL_UNKNOWN      0xFF

// Gesture timing (ticks = ms)
#define CS_LONG_TICKS         500     // hold this long = long press
#define CS_DOUBLE_TICKS       350     // second tap within this = double press
#define CS_REPEAT_DELAY_TICKS 400     // hold before auto-repeat starts
#define CS_REPEAT_RATE_TICKS  80      // one repeat per this many ticks (12.5/s)

// Encoder acceleration (CS_FLAG_ACCEL): step multiplier by inter-detent gap.
// Only continuous nouns accelerate; enums always step one position.
#define CS_ACCEL_GAP_X2       128     // slower than this = no acceleration
#define CS_ACCEL_GAP_X4       64
#define CS_ACCEL_GAP_X8       32

#define CS_SHADOW_TIMEOUT_TICKS  500  // drop a never-confirmed target after 500 ms

// Indicator refresh decimation: LEDs re-evaluate every 8 ticks (125 Hz),
// staggered by slot, bounding the per-tick cost of the heavier live reads
// (meter dB conversion, status-struct copies) while staying visually instant.
#define CS_IND_DECIM          8

// PWM LEDs: wrap 4095 at sysclk/16 (~2 kHz carrier); flicker-free and slow
// enough that the slice config is identical on both platforms.
#define CS_PWM_WRAP           4095
#define CS_PWM_CLKDIV         16.0f

// Log-unit quantization: 1/24 octave; linear units quantize to half a unit
// (half-dB, half-percent), matching the v1 pot behavior.
#define CS_LOG_QUANT_STEPS    24.0f
#define CS_LIN_QUANT          0.5f

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
// Type capability table; the per-noun half lives in control_surfaces_nouns.c.
// REQ_GET_CS_CAPS serves these verbatim, so host UIs and the firmware can
// never disagree about which type/noun/action combinations are legal.
// ---------------------------------------------------------------------------

static const CsCapsHeader s_caps = {
    .caps_version = 2,
    .max_bindings = CS_MAX_BINDINGS,
    .type_count   = CS_TYPE_COUNT,
    .noun_count   = CS_NOUN_COUNT,
    .types = {
        [CS_TYPE_NONE]    = { 0, 0, CS_PINCLASS_ANY },
        [CS_TYPE_BUTTON]  = { CS_ACT_BIT(CS_ACT_INC) | CS_ACT_BIT(CS_ACT_DEC) |
                              CS_ACT_BIT(CS_ACT_TOGGLE) | CS_ACT_BIT(CS_ACT_SET) |
                              CS_ACT_BIT(CS_ACT_TRIGGER) | CS_ACT_BIT(CS_ACT_MOMENTARY),
                              1, CS_PINCLASS_ANY },
        [CS_TYPE_SWITCH]  = { CS_ACT_BIT(CS_ACT_FOLLOW), 1, CS_PINCLASS_ANY },
        [CS_TYPE_POT]     = { CS_ACT_BIT(CS_ACT_ADJUST), 1, CS_PINCLASS_ADC },
        [CS_TYPE_ENCODER] = { CS_ACT_BIT(CS_ACT_STEP), 2, CS_PINCLASS_ANY },
        [CS_TYPE_LED]     = { CS_ACT_BIT(CS_ACT_IND_EQUALS) | CS_ACT_BIT(CS_ACT_IND_ABOVE),
                              1, CS_PINCLASS_ANY },
        [CS_TYPE_LED_PWM] = { CS_ACT_BIT(CS_ACT_IND_EQUALS) | CS_ACT_BIT(CS_ACT_IND_ABOVE) |
                              CS_ACT_BIT(CS_ACT_IND_LEVEL), 1, CS_PINCLASS_ANY },
    },
};

// ---------------------------------------------------------------------------
// Live state
// ---------------------------------------------------------------------------

typedef struct {
    bool     active;
    // switch (level-follow; buttons debounce in the per-pin gesture group)
    uint8_t  sw_stable;
    uint8_t  sw_candidate;
    uint8_t  sw_debounce;
    // encoder
    uint8_t  enc_prev;      // previous AB state (2 bits)
    int8_t   enc_accum;     // quarter-steps toward a detent
    uint16_t enc_gap;       // ticks since the last detent (saturating; accel)
    // pot
    uint16_t pot_filt;      // EMA-filtered 12-bit ADC value
    uint16_t pot_settle;    // seed ticks remaining before the boot sync
    uint16_t pot_sent_raw;  // filtered value at last dispatch
    int32_t  pot_sent_q;    // quantized units at last dispatch
    // LEDs
    uint8_t  led_lit;       // CS_LEVEL_UNKNOWN forces the first write
    uint16_t pwm_level;     // last written PWM compare level
    // deferred dispatch (retried while the command surface reports BUSY)
    bool     op_pending;
    float    op_value;
    // deferred-apply nouns (CS_NDF_DEFERRED): step from the last dispatched
    // target while the live value catches up, so rapid detents are not
    // coalesced against a stale current value
    bool     shadow_active;
    float    shadow;
    int32_t  shadow_q;      // quantized target, cached so the confirm check
                            // costs one cs_quantize, not two (RP2040 softfloat)
    uint16_t shadow_age;    // ticks since dispatch (timeout guard)
    // momentary
    bool     mom_engaged;
    float    mom_restore;   // value captured at press, restored at release
} CsRuntime;

// Per-pin gesture state for buttons.  All button bindings sharing a GPIO
// share one debouncer and one gesture decoder; events fan out to bindings
// by their `event` field.
typedef struct {
    uint8_t  pin;           // CS_GPIO_UNUSED = entry free
    uint8_t  invert;        // shared CS_FLAG_INVERT (validated identical)
    uint8_t  stable;        // debounced logical level; CS_LEVEL_UNKNOWN at claim
    uint8_t  candidate;
    uint8_t  debounce;
    bool     has_long;      // any CS_EVT_LONG binding on this pin
    bool     has_double;    // any CS_EVT_DOUBLE binding on this pin
    bool     consumed;      // long/double already fired for this hold
    uint16_t hold_ticks;    // while held
    bool     wait_double;   // released after a short tap, double window open
    uint16_t wait_ticks;
    bool     repeat_on;     // auto-repeat running for this hold
    uint16_t repeat_ticks;
} CsBtnGroup;

static CsFlashConfig s_cfg;                      // live (and persisted) config
static CsRuntime     s_rt[CS_MAX_BINDINGS];
static CsBtnGroup    s_btn[CS_MAX_BINDINGS];
static uint8_t       s_slot_status[CS_MAX_BINDINGS];
static bool          s_any_active = false;
static uint8_t       s_pot_rr = 0;               // round-robin pot cursor
static uint32_t      s_tick_ct = 0;              // decimation phase
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
// Unit math.  Natural units are float (dB, Hz, Q, percent); wire fields are
// int16 (8.8 for dB/Q/percent, plain integer for Hz).  HZ and Q step and
// quantize logarithmically (in octaves), the rest linearly.
// ---------------------------------------------------------------------------

static bool cs_unit_is_log(uint8_t unit) {
    return unit == CS_UNIT_HZ || unit == CS_UNIT_Q;
}

static float cs_decode(uint8_t unit, int16_t raw) {
    return (unit == CS_UNIT_HZ) ? (float)raw : (float)raw * (1.0f / 256.0f);
}

// Step size in natural units (linear) or octaves (log units).  step == 0
// selects the default: 1 dB, 1 percent, or 1/12 octave.
static float cs_step_size(const CsBinding *b, uint8_t unit) {
    if (b->step > 0) return (float)b->step * (1.0f / 256.0f);
    return cs_unit_is_log(unit) ? (1.0f / 12.0f) : 1.0f;
}

static float cs_clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// The binding's effective span: the noun's full range, or the custom
// [range_min, range_max] when either field is non-zero (pot / IND_LEVEL).
static void cs_span(const CsBinding *b, const CsNounDesc *nd, float *lo, float *hi) {
    if (b->range_min != 0 || b->range_max != 0) {
        *lo = cs_decode(nd->unit, b->range_min);
        *hi = cs_decode(nd->unit, b->range_max);
    } else {
        *lo = cs_decode(nd->unit, nd->min_q);
        *hi = cs_decode(nd->unit, nd->max_q);
    }
}

// Quantize a natural value: half-units for linear nouns, 1/24 octave above
// the noun minimum for log nouns.  Dispatches reconstruct from the quantized
// value so a pot's output is stable and jitter-free.  Live reads can surface
// a zeroed filter recipe (freq 0) or garbage; guard the log/round domain.
static int32_t cs_quantize(const CsNounDesc *nd, float v) {
    if (!isfinite(v)) return 0;
    if (cs_unit_is_log(nd->unit)) {
        if (v <= 0.0f) return 0;
        float lo = cs_decode(nd->unit, nd->min_q);
        return (int32_t)lroundf(log2f(v / lo) * CS_LOG_QUANT_STEPS);
    }
    return (int32_t)lroundf(v / CS_LIN_QUANT);
}

static float cs_unquantize(const CsNounDesc *nd, int32_t q) {
    if (cs_unit_is_log(nd->unit)) {
        float lo = cs_decode(nd->unit, nd->min_q);
        return lo * exp2f((float)q / CS_LOG_QUANT_STEPS);
    }
    return (float)q * CS_LIN_QUANT;
}

// True once the live value has caught up with a dispatched deferred target
// (within half a quantization step), so the shadow can retire.  Compares
// against the quantized target cached at dispatch time (rt->shadow_q).
static bool cs_shadow_confirmed(const CsNounDesc *nd, float live,
                                const CsRuntime *rt) {
    if (nd->kind != CS_KIND_CONTINUOUS)
        return (int)live == (int)rt->shadow;
    return cs_quantize(nd, live) == rt->shadow_q;
}

// ---------------------------------------------------------------------------
// Dispatch plumbing
// ---------------------------------------------------------------------------

static void cs_queue_op(uint8_t slot, float value) {
    CsRuntime *rt = &s_rt[slot];
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (nd->dflags & CS_NDF_DEFERRED) {
        rt->shadow_active = true;
        rt->shadow = value;
        rt->shadow_q = cs_quantize(nd, value);
        rt->shadow_age = 0;
    }
    if (cs_noun_dispatch(b->noun, b->target, b->index, value)) {
        rt->op_pending = false;
    } else {
        rt->op_pending = true;
        rt->op_value = value;
    }
}

// Base value for relative actions: the latched (not-yet-dispatched) target
// when one is pending, else the target shadow for deferred-apply nouns,
// else the live value.  Keeps rapid detents accumulating correctly across
// BUSY retries and across a preset load / EQ apply still in flight.
static float cs_base_value(uint8_t slot) {
    const CsRuntime *rt = &s_rt[slot];
    const CsBinding *b = &s_cfg.bindings[slot];
    if (rt->op_pending) return rt->op_value;
    if (rt->shadow_active) return rt->shadow;
    return cs_noun_get(b->noun, b->target, b->index);
}

// ---------------------------------------------------------------------------
// Actions
// ---------------------------------------------------------------------------

// Step an enum noun by dir (+1/-1).  Presets step across OCCUPIED slots
// only; an empty device is a no-op.  Returns the target index or -1 for
// no movement.
static int cs_enum_step(uint8_t slot, int dir) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
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

// Apply `steps` relative steps (encoder detents or INC/DEC presses).
// `steps` carries the acceleration multiplier for continuous nouns; enums
// always move one position per event.
static void cs_apply_step(uint8_t slot, int dir, int steps) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    if (nd->kind == CS_KIND_CONTINUOUS) {
        float lo = cs_decode(nd->unit, nd->min_q);
        float hi = cs_decode(nd->unit, nd->max_q);
        float d = (float)(dir * steps) * cs_step_size(b, nd->unit);
        float base = cs_base_value(slot);
        float v = cs_unit_is_log(nd->unit) ? base * exp2f(d) : base + d;
        cs_queue_op(slot, cs_clampf(v, lo, hi));
    } else {
        int t = cs_enum_step(slot, dir);
        if (t >= 0) cs_queue_op(slot, (float)t);
    }
}

static void cs_button_press(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    switch (b->action) {
        case CS_ACT_INC:     cs_apply_step(slot, +1, 1); break;
        case CS_ACT_DEC:     cs_apply_step(slot, -1, 1); break;
        case CS_ACT_TOGGLE:  cs_queue_op(slot, cs_base_value(slot) >= 0.5f ? 0.0f : 1.0f); break;
        case CS_ACT_TRIGGER: cs_queue_op(slot, 0.0f); break;
        case CS_ACT_SET: {
            const CsNounDesc *nd = &cs_noun_table[b->noun];
            float v = (nd->kind == CS_KIND_CONTINUOUS)
                    ? cs_decode(nd->unit, b->value) : (float)b->value;
            cs_queue_op(slot, v);
            break;
        }
        default: break;
    }
}

static void cs_momentary_engage(uint8_t slot) {
    CsRuntime *rt = &s_rt[slot];
    const CsBinding *b = &s_cfg.bindings[slot];
    rt->mom_restore = cs_base_value(slot);
    rt->mom_engaged = true;
    cs_queue_op(slot, (float)b->value);
}

static void cs_momentary_release(uint8_t slot) {
    CsRuntime *rt = &s_rt[slot];
    if (!rt->mom_engaged) return;
    rt->mom_engaged = false;
    cs_queue_op(slot, rt->mom_restore);
}

// ---------------------------------------------------------------------------
// Button gesture groups
// ---------------------------------------------------------------------------

static CsBtnGroup *cs_group_for_pin(uint8_t pin) {
    for (int i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_btn[i].pin == pin) return &s_btn[i];
    return NULL;
}

// Rebuild the per-pin gesture groups from the active button bindings.
// Debounce/gesture state of pins that persist across the rebuild is kept
// so editing an unrelated slot cannot glitch a held button.
static void cs_rebuild_groups(void) {
    CsBtnGroup old[CS_MAX_BINDINGS];
    memcpy(old, s_btn, sizeof(old));
    for (int i = 0; i < CS_MAX_BINDINGS; i++) {
        s_btn[i].pin = CS_GPIO_UNUSED;
    }
    int n = 0;
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active || s_cfg.bindings[s].type != CS_TYPE_BUTTON) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        CsBtnGroup *g = NULL;
        for (int i = 0; i < n; i++)
            if (s_btn[i].pin == b->gpio[0]) { g = &s_btn[i]; break; }
        if (!g) {
            g = &s_btn[n++];
            // Carry over live state if this pin already had a group.
            CsBtnGroup *prev = NULL;
            for (int i = 0; i < CS_MAX_BINDINGS; i++)
                if (old[i].pin == b->gpio[0]) { prev = &old[i]; break; }
            if (prev) {
                *g = *prev;
            } else {
                memset(g, 0, sizeof(*g));
                g->pin = b->gpio[0];
                g->stable = CS_LEVEL_UNKNOWN;
                g->candidate = CS_LEVEL_UNKNOWN;
            }
            g->invert = (b->flags & CS_FLAG_INVERT) ? 1 : 0;
            g->has_long = false;
            g->has_double = false;
        }
        if (b->event == CS_EVT_LONG)   g->has_long = true;
        if (b->event == CS_EVT_DOUBLE) g->has_double = true;
    }
    // If a LONG/DOUBLE binding was just added to a pin whose button is
    // physically held right now, consume the in-progress hold; otherwise the
    // new LONG could fire mid-hold, breaking the "sync, not a press" rule.
    for (int i = 0; i < CS_MAX_BINDINGS; i++) {
        CsBtnGroup *g = &s_btn[i];
        if (g->pin == CS_GPIO_UNUSED || g->stable != 1) continue;
        for (int j = 0; j < CS_MAX_BINDINGS; j++) {
            if (old[j].pin != g->pin) continue;
            if ((g->has_long && !old[j].has_long) ||
                (g->has_double && !old[j].has_double))
                g->consumed = true;
            break;
        }
    }
}

// Fire every non-momentary button binding on `pin` bound to `event`.
// Repeat deliveries only reach CS_FLAG_REPEAT bindings.
static void cs_group_fire(const CsBtnGroup *g, uint8_t event, bool is_repeat) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        if (b->type != CS_TYPE_BUTTON || b->gpio[0] != g->pin) continue;
        if (b->action == CS_ACT_MOMENTARY) continue;
        if (b->event != event) continue;
        if (is_repeat && !(b->flags & CS_FLAG_REPEAT)) continue;
        cs_button_press(s);
    }
}

static void cs_group_momentary(const CsBtnGroup *g, bool engage) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (!s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        if (b->type != CS_TYPE_BUTTON || b->gpio[0] != g->pin) continue;
        if (b->action != CS_ACT_MOMENTARY) continue;
        if (engage) cs_momentary_engage(s);
        else        cs_momentary_release(s);
    }
}

// Gesture decoding, one pin per call.  Semantics:
// - No LONG/DOUBLE bindings on the pin: PRESS fires on the press edge
//   (immediate, v1 behavior); REPEAT bindings auto-repeat while held.
// - LONG present: PRESS fires on release before the long threshold; LONG
//   fires once when the threshold is reached while held.
// - DOUBLE present: a second press inside the window fires DOUBLE at that
//   press edge; a lone tap fires PRESS when the window expires.
// - MOMENTARY bindings engage on the press edge and restore on release,
//   independent of the gesture outcome.
// The first debounced level after a claim is a sync, not a press.
static void cs_tick_button_group(CsBtnGroup *g) {
    uint8_t raw = gpio_get(g->pin) ? 1 : 0;
    uint8_t level = g->invert ? raw : (uint8_t)!raw;

    if (level != g->candidate) {
        g->candidate = level;
        g->debounce = 0;
    } else if (g->candidate != g->stable) {
        if (++g->debounce >= CS_DEBOUNCE_TICKS) {
            g->debounce = 0;
            bool first = (g->stable == CS_LEVEL_UNKNOWN);
            g->stable = g->candidate;
            if (first && g->stable == 1) {
                // Held at claim/boot: a sync, not a press.  Consume the
                // whole hold so LONG cannot fire from a pre-boot press.
                g->consumed = true;
            }
            if (!first && g->stable == 1) {
                // Press edge
                g->hold_ticks = 0;
                g->repeat_on = false;
                if (g->wait_double) {
                    g->wait_double = false;
                    g->consumed = true;      // second tap: whole hold consumed
                    cs_group_fire(g, CS_EVT_DOUBLE, false);
                } else {
                    g->consumed = false;
                    if (!g->has_long && !g->has_double) {
                        cs_group_fire(g, CS_EVT_PRESS, false);
                        g->repeat_on = true;
                        g->repeat_ticks = 0;
                    }
                }
                cs_group_momentary(g, true);
            } else if (!first && g->stable == 0) {
                // Release edge
                cs_group_momentary(g, false);
                g->repeat_on = false;
                if (!g->consumed && (g->has_long || g->has_double)) {
                    if (g->has_double) {
                        g->wait_double = true;   // tap; single fires on expiry
                        g->wait_ticks = 0;
                    } else {
                        cs_group_fire(g, CS_EVT_PRESS, false);
                    }
                }
                g->consumed = false;
            }
        }
    } else {
        g->debounce = 0;
    }

    if (g->stable == 1) {
        if (g->hold_ticks < 0xFFFF) g->hold_ticks++;
        if (g->has_long && !g->consumed && g->hold_ticks >= CS_LONG_TICKS) {
            g->consumed = true;
            cs_group_fire(g, CS_EVT_LONG, false);
        }
        if (g->repeat_on && g->hold_ticks >= CS_REPEAT_DELAY_TICKS) {
            if (++g->repeat_ticks >= CS_REPEAT_RATE_TICKS) {
                g->repeat_ticks = 0;
                cs_group_fire(g, CS_EVT_PRESS, true);
            }
        }
    } else if (g->wait_double) {
        if (++g->wait_ticks >= CS_DOUBLE_TICKS) {
            g->wait_double = false;
            cs_group_fire(g, CS_EVT_PRESS, false);
        }
    }
}

// ---------------------------------------------------------------------------
// Per-type tick handlers (switch / encoder / pot / LEDs)
// ---------------------------------------------------------------------------

// Logical level: 1 = pressed / on.  Default wiring is component-to-GND with
// the internal pull-up; CS_FLAG_INVERT selects active-high with pull-down.
static uint8_t cs_read_level(const CsBinding *b, uint8_t pin) {
    uint8_t raw = gpio_get(pin) ? 1 : 0;
    return (b->flags & CS_FLAG_INVERT) ? raw : (uint8_t)!raw;
}

static void cs_tick_switch(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    uint8_t level = cs_read_level(b, b->gpio[0]);

    if (level != rt->sw_candidate) {
        rt->sw_candidate = level;
        rt->sw_debounce = 0;
        return;
    }
    if (rt->sw_candidate == rt->sw_stable) { rt->sw_debounce = 0; return; }
    if (++rt->sw_debounce < CS_DEBOUNCE_TICKS) return;
    rt->sw_debounce = 0;
    rt->sw_stable = rt->sw_candidate;
    // A switch is authoritative, including at claim/boot (matches the
    // pot's immediate-takeover semantics).
    cs_queue_op(slot, (float)rt->sw_stable);
}

static void cs_tick_encoder(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    if (rt->enc_gap < 0xFFFF) rt->enc_gap++;
    uint8_t curr = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                             (gpio_get(b->gpio[1]) ? 1 : 0));
    int8_t d = s_enc_table[(rt->enc_prev << 2) | curr];
    rt->enc_prev = curr;
    if (!d) return;
    rt->enc_accum += (b->flags & CS_FLAG_REVERSE) ? -d : d;
    // One detent = 4 quarter-steps (both channels through a full cycle).
    int dir = 0;
    if (rt->enc_accum >= 4)       { rt->enc_accum -= 4; dir = +1; }
    else if (rt->enc_accum <= -4) { rt->enc_accum += 4; dir = -1; }
    if (!dir) return;

    int steps = 1;
    if (b->flags & CS_FLAG_ACCEL) {
        uint16_t gap = rt->enc_gap;
        if      (gap < CS_ACCEL_GAP_X8) steps = 8;
        else if (gap < CS_ACCEL_GAP_X4) steps = 4;
        else if (gap < CS_ACCEL_GAP_X2) steps = 2;
    }
    rt->enc_gap = 0;
    cs_apply_step(slot, dir, steps);
}

// Map a filtered ADC reading onto the binding's span, quantized per unit.
static int32_t cs_pot_map(const CsBinding *b, const CsNounDesc *nd, uint16_t filt) {
    float lo, hi;
    cs_span(b, nd, &lo, &hi);
    float pos = (float)filt * (1.0f / 4095.0f);
    if (b->flags & CS_FLAG_REVERSE) pos = 1.0f - pos;
    float v = cs_unit_is_log(nd->unit) ? lo * exp2f(pos * log2f(hi / lo))
                                       : lo + (hi - lo) * pos;
    return cs_quantize(nd, v);
}

static void cs_tick_pot(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    // The ADC input mux is unguarded; safe only because every ADC user (this
    // tick, the temperature read in vendor_commands.c) runs serialized in the
    // core0 main loop.  Do not move either to ISR/timer context.
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
            rt->pot_sent_q = cs_pot_map(b, nd, rt->pot_filt);
            rt->pot_sent_raw = rt->pot_filt;
            cs_queue_op(slot, cs_unquantize(nd, rt->pot_sent_q));
        }
        return;
    }

    int32_t moved = (int32_t)rt->pot_filt - rt->pot_sent_raw;
    if (moved < 0) moved = -moved;
    if (moved <= CS_POT_DEADBAND) return;
    int32_t q = cs_pot_map(b, nd, rt->pot_filt);
    if (q != rt->pot_sent_q) {
        rt->pot_sent_q = q;
        rt->pot_sent_raw = rt->pot_filt;
        cs_queue_op(slot, cs_unquantize(nd, q));
    }
}

static void cs_tick_led(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    float v = cs_noun_get(b->noun, b->target, b->index);
    uint8_t lit;
    if (b->action == CS_ACT_IND_ABOVE)
        lit = (v >= cs_decode(nd->unit, b->value)) ? 1 : 0;
    else
        lit = ((int)v == (int)b->value) ? 1 : 0;
    if (lit == rt->led_lit) return;
    rt->led_lit = lit;
    gpio_put(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? !lit : lit);
}

static void cs_tick_led_pwm(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    CsRuntime *rt = &s_rt[slot];
    float v = cs_noun_get(b->noun, b->target, b->index);
    uint16_t level;
    if (b->action == CS_ACT_IND_LEVEL) {
        float lo, hi;
        cs_span(b, nd, &lo, &hi);
        float norm = cs_unit_is_log(nd->unit)
                   ? log2f(v / lo) / log2f(hi / lo)
                   : (v - lo) / (hi - lo);
        // A zeroed recipe (freq 0) or garbage read yields -inf/NaN here;
        // NaN passes a min/max clamp, so gate explicitly.
        if (!(norm > 0.0f)) norm = 0.0f;
        else if (norm > 1.0f) norm = 1.0f;
        // Square the normalized position; a perceptually even sweep.
        level = (uint16_t)(norm * norm * (float)CS_PWM_WRAP);
    } else {
        uint8_t lit = (b->action == CS_ACT_IND_ABOVE)
                    ? (v >= cs_decode(nd->unit, b->value) ? 1 : 0)
                    : ((int)v == (int)b->value ? 1 : 0);
        level = lit ? CS_PWM_WRAP : 0;
    }
    if (b->flags & CS_FLAG_INVERT) level = CS_PWM_WRAP - level;
    if (level == rt->pwm_level) return;
    rt->pwm_level = level;
    pwm_set_gpio_level(b->gpio[0], level);
}

// ---------------------------------------------------------------------------
// Pin claim / release and validation
// ---------------------------------------------------------------------------

static uint8_t cs_pin_count(uint8_t type) {
    return (type < CS_TYPE_COUNT) ? s_caps.types[type].pin_count : 0;
}

// True if another ACTIVE binding (not `except_slot`) uses `pin`.
static bool cs_pin_used_by_other(uint8_t pin, uint8_t except_slot) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == except_slot || !s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == pin) return true;
    }
    return false;
}

// Sharing probe for button pins: every other active binding on `pin` must be
// a button with the same INVERT sense and a different event (momentary
// bindings count; two bindings may not claim the same pin+event).
static uint8_t cs_check_button_share(const CsBinding *nb, uint8_t except_slot) {
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s == except_slot || !s_rt[s].active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        bool uses = false;
        for (int i = 0; i < cs_pin_count(b->type); i++)
            if (b->gpio[i] == nb->gpio[0]) uses = true;
        if (!uses) continue;
        if (b->type != CS_TYPE_BUTTON) return PIN_CONFIG_PIN_IN_USE;
        if ((b->flags ^ nb->flags) & CS_FLAG_INVERT) return CS_STATUS_INVALID_VALUE;
        if (b->event == nb->event) return CS_STATUS_EVENT_IN_USE;
    }
    return PIN_CONFIG_SUCCESS;
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
        case CS_TYPE_LED_PWM: {
            uint slice = pwm_gpio_to_slice_num(b->gpio[0]);
            // Wrap/div writes are idempotent (all CS slices use the same
            // config), so sharing a slice between two CS LEDs is safe.
            pwm_set_wrap(slice, CS_PWM_WRAP);
            pwm_set_clkdiv(slice, CS_PWM_CLKDIV);
            pwm_set_gpio_level(b->gpio[0], (b->flags & CS_FLAG_INVERT) ? CS_PWM_WRAP : 0);
            gpio_set_function(b->gpio[0], GPIO_FUNC_PWM);
            pwm_set_enabled(slice, true);
            break;
        }
        default:  // button / switch / encoder inputs (idempotent re-claims OK)
            for (int i = 0; i < cs_pin_count(b->type); i++) {
                gpio_init(b->gpio[i]);
                gpio_set_dir(b->gpio[i], GPIO_IN);
                if (b->flags & CS_FLAG_INVERT) gpio_pull_down(b->gpio[i]);
                else                           gpio_pull_up(b->gpio[i]);
            }
            break;
    }
}

// Release a slot's pins.  Shared button pins stay claimed while another
// active binding still uses them; a PWM slice is stopped only when no other
// CS PWM LED remains on it.  Caller has already marked the slot inactive.
static void cs_release_pins(uint8_t slot, const CsBinding *b) {
    for (int i = 0; i < cs_pin_count(b->type); i++) {
        uint8_t pin = b->gpio[i];
        if (cs_pin_used_by_other(pin, slot)) continue;
        if (b->type == CS_TYPE_LED_PWM) {
            uint slice = pwm_gpio_to_slice_num(pin);
            bool slice_busy = false;
            for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
                if (s == slot || !s_rt[s].active) continue;
                const CsBinding *o = &s_cfg.bindings[s];
                if (o->type == CS_TYPE_LED_PWM &&
                    pwm_gpio_to_slice_num(o->gpio[0]) == slice)
                    slice_busy = true;
            }
            if (!slice_busy) pwm_set_enabled(slice, false);
        }
        gpio_deinit(pin);
        gpio_disable_pulls(pin);
    }
}

static void cs_seed_runtime(uint8_t slot) {
    const CsBinding *b = &s_cfg.bindings[slot];
    CsRuntime *rt = &s_rt[slot];
    memset(rt, 0, sizeof(*rt));
    switch (b->type) {
        case CS_TYPE_SWITCH:
            rt->sw_stable = CS_LEVEL_UNKNOWN;
            rt->sw_candidate = CS_LEVEL_UNKNOWN;
            break;
        case CS_TYPE_ENCODER:
            rt->enc_prev = (uint8_t)((gpio_get(b->gpio[0]) ? 2 : 0) |
                                     (gpio_get(b->gpio[1]) ? 1 : 0));
            rt->enc_gap = 0xFFFF;
            break;
        case CS_TYPE_POT:
            adc_select_input(b->gpio[0] - CS_ADC_PIN_FIRST);
            rt->pot_filt = adc_read();
            rt->pot_settle = CS_POT_SEED_TICKS;
            break;
        case CS_TYPE_LED:
            rt->led_lit = CS_LEVEL_UNKNOWN;
            break;
        case CS_TYPE_LED_PWM:
            rt->pwm_level = 0xFFFF;   // force the first write
            break;
        default:   // buttons keep their state in the pin group
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
static uint8_t cs_validate(const CsBinding *b, uint8_t slot) {
    if (b->type >= CS_TYPE_COUNT) return CS_STATUS_INVALID_TYPE;
    if (b->noun >= CS_NOUN_COUNT) return CS_STATUS_INVALID_NOUN;
    if (b->action >= CS_ACT_COUNT) return CS_STATUS_INVALID_ACTION;

    if (b->flags & (uint8_t)~CS_FLAG_ALL) return CS_STATUS_INVALID_VALUE;
    if (b->reserved != 0) return CS_STATUS_INVALID_VALUE;
    for (int i = 0; i < (int)sizeof(b->reserved2); i++)
        if (b->reserved2[i] != 0) return CS_STATUS_INVALID_VALUE;

    const CsTypeDesc *td = &s_caps.types[b->type];
    const CsNounDesc *nd = &cs_noun_table[b->noun];
    uint16_t bit = CS_ACT_BIT(b->action);
    if (!(td->actions & bit) || !(nd->actions & bit)) return CS_STATUS_INVALID_ACTION;

    // Events are a button concept; everything else must carry 0.
    if (b->type == CS_TYPE_BUTTON) {
        if (b->event >= CS_EVT_COUNT) return CS_STATUS_INVALID_EVENT;
        // Hold-to-repeat and hold-to-engage both own the hold; they only
        // make sense on the short-press event.
        if ((b->action == CS_ACT_MOMENTARY || (b->flags & CS_FLAG_REPEAT)) &&
            b->event != CS_EVT_PRESS)
            return CS_STATUS_INVALID_EVENT;
    } else if (b->event != 0) {
        return CS_STATUS_INVALID_EVENT;
    }
    if ((b->flags & CS_FLAG_REPEAT) &&
        (b->type != CS_TYPE_BUTTON ||
         (b->action != CS_ACT_INC && b->action != CS_ACT_DEC)))
        return CS_STATUS_INVALID_VALUE;
    if ((b->flags & CS_FLAG_ACCEL) && b->type != CS_TYPE_ENCODER)
        return CS_STATUS_INVALID_VALUE;

    uint8_t tst = cs_noun_validate_target(b);
    if (tst != PIN_CONFIG_SUCCESS) return tst;

    // Value / step / range bounds per kind
    switch (nd->kind) {
        case CS_KIND_CONTINUOUS:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_ABOVE) &&
                (b->value < nd->min_q || b->value > nd->max_q))
                return CS_STATUS_INVALID_VALUE;
            if (b->step < 0) return CS_STATUS_INVALID_VALUE;
            if ((b->action == CS_ACT_ADJUST || b->action == CS_ACT_IND_LEVEL) &&
                (b->range_min != 0 || b->range_max != 0)) {
                if (b->range_min >= b->range_max ||
                    b->range_min < nd->min_q || b->range_max > nd->max_q)
                    return CS_STATUS_INVALID_VALUE;
            }
            break;
        case CS_KIND_BOOL:
            if ((b->action == CS_ACT_SET || b->action == CS_ACT_IND_EQUALS ||
                 b->action == CS_ACT_MOMENTARY) &&
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

    // Pins: distinct, valid, unclaimed anywhere; the one sanctioned overlap
    // is several button bindings sharing a GPIO with distinct events.
    uint8_t n = td->pin_count;
    if (n == 2 && b->gpio[0] == b->gpio[1]) return PIN_CONFIG_INVALID_PIN;
    for (int i = 0; i < n; i++) {
        uint8_t pin = b->gpio[i];
        if (td->pin_class == CS_PINCLASS_ADC &&
            (pin < CS_ADC_PIN_FIRST || pin > CS_ADC_PIN_LAST))
            return CS_STATUS_PIN_NOT_ADC;
        if (control_surfaces_owns_pin(pin)) {
            if (b->type != CS_TYPE_BUTTON) return PIN_CONFIG_PIN_IN_USE;
            uint8_t st = cs_check_button_share(b, slot);
            if (st != PIN_CONFIG_SUCCESS) return st;
        } else {
            uint8_t st = ctrl_iface_check_pin(pin);
            if (st != PIN_CONFIG_SUCCESS) return st;
        }
    }

    // PWM LEDs on different GPIOs can still land on the same PWM slice
    // output (e.g. GPIO 0 and 16 are both slice 0 channel A on RP2040).
    if (b->type == CS_TYPE_LED_PWM) {
        uint slice = pwm_gpio_to_slice_num(b->gpio[0]);
        uint chan  = pwm_gpio_to_channel(b->gpio[0]);
        for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
            if (s == slot || !s_rt[s].active) continue;
            const CsBinding *o = &s_cfg.bindings[s];
            if (o->type == CS_TYPE_LED_PWM &&
                pwm_gpio_to_slice_num(o->gpio[0]) == slice &&
                pwm_gpio_to_channel(o->gpio[0]) == chan)
                return CS_STATUS_PWM_CONFLICT;
        }
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
        s_rt[slot].active = false;
        cs_release_pins(slot, &old);
    }

    if (nb->type != CS_TYPE_NONE) {
        uint8_t st = cs_validate(nb, slot);
        if (st != PIN_CONFIG_SUCCESS) {
            if (was_active) {
                s_cfg.bindings[slot] = old;
                cs_claim_pins(slot);
                cs_seed_runtime(slot);
            }
            cs_recount_active();
            cs_rebuild_groups();
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
    cs_rebuild_groups();
    return PIN_CONFIG_SUCCESS;
}

void control_surfaces_init(void) {
    CsFlashConfig stored;
    preset_get_cs_config(&stored);
    memset(&s_cfg, 0, sizeof(s_cfg));
    memset(s_rt, 0, sizeof(s_rt));
    memset(s_slot_status, 0, sizeof(s_slot_status));
    for (int i = 0; i < CS_MAX_BINDINGS; i++) s_btn[i].pin = CS_GPIO_UNUSED;
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
    s_tick_ct++;

    // Retry ops parked on a BUSY dispatch, and retire deferred-apply target
    // shadows once the live value confirms them (or on timeout), before
    // sampling new events.
    for (uint8_t s = 0; s < CS_MAX_BINDINGS; s++) {
        CsRuntime *rt = &s_rt[s];
        if (!rt->active) continue;
        const CsBinding *b = &s_cfg.bindings[s];
        if (rt->op_pending &&
            cs_noun_dispatch(b->noun, b->target, b->index, rt->op_value))
            rt->op_pending = false;
        // Confirm checks are slot-staggered to every 4th tick: the live read
        // plus quantize is the engine's costliest recurring float work on the
        // RP2040, and retiring a shadow a few ms late is harmless.
        if (rt->shadow_active) {
            if (++rt->shadow_age >= CS_SHADOW_TIMEOUT_TICKS)
                rt->shadow_active = false;
            else if (((s_tick_ct + s) & 3u) == 0 &&
                     cs_shadow_confirmed(&cs_noun_table[b->noun],
                                         cs_noun_get(b->noun, b->target, b->index),
                                         rt))
                rt->shadow_active = false;
        }
    }

    // Buttons decode once per pin group, not per binding.
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++)
        if (s_btn[i].pin != CS_GPIO_UNUSED) cs_tick_button_group(&s_btn[i]);

    bool pot_done = false;   // one ADC conversion per tick, round-robin
    for (uint8_t i = 0; i < CS_MAX_BINDINGS; i++) {
        uint8_t s = (uint8_t)((s_pot_rr + 1 + i) % CS_MAX_BINDINGS);
        if (!s_rt[s].active) continue;
        switch (s_cfg.bindings[s].type) {
            case CS_TYPE_SWITCH:  cs_tick_switch(s);  break;
            case CS_TYPE_ENCODER: cs_tick_encoder(s); break;
            case CS_TYPE_LED:
                if (((s_tick_ct + s) & (CS_IND_DECIM - 1)) == 0) cs_tick_led(s);
                break;
            case CS_TYPE_LED_PWM:
                if (((s_tick_ct + s) & (CS_IND_DECIM - 1)) == 0) cs_tick_led_pwm(s);
                break;
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
    out->reserved = 0;
    out->active_mask = 0;
    for (int s = 0; s < CS_MAX_BINDINGS; s++) {
        if (s_rt[s].active) out->active_mask |= (uint16_t)(1u << s);
        out->slot_status[s] = s_slot_status[s];
    }
}

const CsCapsHeader *control_surfaces_caps_header(void) { return &s_caps; }

const CsNounDesc *control_surfaces_noun_desc(uint8_t noun) {
    return (noun < CS_NOUN_COUNT) ? &cs_noun_table[noun] : NULL;
}
