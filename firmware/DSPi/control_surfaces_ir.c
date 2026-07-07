/*
 * control_surfaces_ir.c; IR remote capture and decode.
 *
 * Capture: both GPIO edges on the receiver pin are timestamped by a tiny
 * RAM-resident IO_IRQ_BANK0 handler into a lock-free single-producer ring
 * of {duration, level} entries.  No PIO, no timers; the handler is the
 * firmware's only GPIO interrupt user and runs at the lowest priority
 * (timing lives in the timestamps, not in latency).
 *
 * Decode (main loop, from the CS tick): a space longer than IR_FRAME_GAP_US
 * terminates a frame of alternating mark/space durations.  Frames are tried
 * against NEC (dedicated repeat frame drives hold-to-repeat), RC5 and RC6
 * mode 0 (Manchester; the toggle bit is masked so a learned button matches
 * every press), then fall back to an FNV-1a hash over the timing signature,
 * which matches any remote that repeats by re-transmission.
 *
 * Hold model: one button at a time.  A frame carrying the held code (or a
 * NEC repeat frame) within IR_HOLD_GAP_US extends the hold as a REPEAT;
 * silence past the gap emits RELEASE.  A different code releases the old
 * button and presses the new one.
 */

#include "control_surfaces.h"     // CS_IR_PROTO_* / CS_IR_LEARN_*
#include "control_surfaces_ir.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "pico/platform.h"

// Capture ring: bit 15 = the elapsed period's physical level, bits 14:0 =
// its duration in µs (saturating).  128 entries outlast the longest frame
// (Kaseikyo, 99 periods) with headroom.
#define IR_RING_SIZE       128
#define IR_RING_MASK       (IR_RING_SIZE - 1)
#define IR_DUR_MAX         0x7FFF

#define IR_FRAME_GAP_US    10000u    // space this long = end of frame
#define IR_MAX_DURS        112       // frame buffer (mark/space periods)
#define IR_HOLD_GAP_US     250000u   // no frame for this long = released
#define IR_LEARN_WINDOW_US 10000000u // learn listens for 10 s

// NEC timings (µs)
#define NEC_HDR_MARK    9000
#define NEC_HDR_SPACE   4500
#define NEC_RPT_SPACE   2250
#define NEC_BIT_MARK    560
#define NEC_ZERO_SPACE  560
#define NEC_ONE_SPACE   1690

// RC5 / RC6 Manchester units (µs)
#define RC5_UNIT        889
#define RC6_UNIT        444

static volatile uint16_t s_ring[IR_RING_SIZE];
static volatile uint8_t  s_head;             // ISR-owned
static volatile uint8_t  s_tail;             // poll-owned (ISR reads for full check)
static volatile uint32_t s_last_edge_us;

static bool    s_attached;
static uint8_t s_pin;
static uint8_t s_active_level;               // physical level of a mark

// Frame assembly
static uint16_t s_frame[IR_MAX_DURS];
static uint8_t  s_frame_n;
static bool     s_in_frame;
static bool     s_frame_overflow;

// Hold tracking
static bool     s_held;
static uint8_t  s_held_proto;
static uint32_t s_held_code;
static uint32_t s_last_frame_us;

// Event queue, drained every tick.  Sized so it cannot overflow in practice:
// a frame yields at most 2 events (RELEASE + PRESS) and frames are >= 20 ms
// apart, so even a poll delayed by a full flash blackout sees at most two
// frames' worth plus a synthetic learn-arm release.
static CsIrEvent s_evq[6];
static uint8_t   s_evq_n;

// Learn
static uint8_t  s_learn_state = CS_IR_LEARN_IDLE;
static bool     s_learn_changed;
static uint32_t s_learn_armed_us;
static uint8_t  s_learn_proto;
static uint32_t s_learn_code;

// ---------------------------------------------------------------------------
// Edge capture ISR.  RAM-resident: it may fire while other code stalls XIP,
// and flash-resident handlers during a flash erase are fatal.  Touches only
// SIO / IO_BANK0 / TIMER registers and this module's ring.
// ---------------------------------------------------------------------------
static void __not_in_flash_func(cs_ir_edge_irq)(void) {
    uint32_t now = time_us_32();
    uint8_t pin = s_pin;
    gpio_acknowledge_irq(pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);  // inline
    uint32_t dur = now - s_last_edge_us;
    s_last_edge_us = now;
    // The period that just ended ran at the opposite of the current level.
    uint16_t entry = (uint16_t)(dur > IR_DUR_MAX ? IR_DUR_MAX : dur);
    if (!gpio_get(pin)) entry |= 0x8000;
    uint8_t head = s_head;
    uint8_t next = (uint8_t)((head + 1) & IR_RING_MASK);
    if (next != s_tail) {          // full ring drops the newest edge
        s_ring[head] = entry;
        s_head = next;
    }
}

// ---------------------------------------------------------------------------
// Attach / detach
// ---------------------------------------------------------------------------

void cs_ir_attach(uint8_t pin, bool invert) {
    if (s_attached) cs_ir_detach();
    s_pin = pin;
    s_active_level = invert ? 1 : 0;   // idle-high receiver marks low
    s_head = 0; s_tail = 0;
    s_frame_n = 0; s_in_frame = false; s_frame_overflow = false;
    s_held = false; s_evq_n = 0;
    s_last_edge_us = time_us_32();
    irq_set_exclusive_handler(IO_IRQ_BANK0, cs_ir_edge_irq);
    // Lowest priority: durations come from timestamps, so latency only has
    // to stay under the shortest period (~440 µs), never under the audio path.
    irq_set_priority(IO_IRQ_BANK0, 0xC0);
    gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
    s_attached = true;
}

void cs_ir_detach(void) {
    if (!s_attached) return;
    irq_set_enabled(IO_IRQ_BANK0, false);
    gpio_set_irq_enabled(s_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, false);
    gpio_acknowledge_irq(s_pin, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE);
    irq_remove_handler(IO_IRQ_BANK0, cs_ir_edge_irq);
    s_attached = false;
    s_held = false;
    s_evq_n = 0;
    // A learn left armed can never complete without a receiver; surface it
    // as a timeout so a waiting host is not left hanging.
    if (s_learn_state == CS_IR_LEARN_ARMED) {
        s_learn_state = CS_IR_LEARN_TIMEOUT;
        s_learn_changed = true;
    }
}

bool cs_ir_attached(void) { return s_attached; }

// ---------------------------------------------------------------------------
// Decoders.  All operate on s_frame: alternating durations starting with a
// mark (trailing spaces merge into the inter-frame gap, so a frame always
// ends on a mark).
// ---------------------------------------------------------------------------

// ±25% plus a constant slack for receiver mark stretching.
static bool ir_match(uint16_t d, uint16_t t) {
    uint16_t slack = (uint16_t)(t / 4 + 150);
    return d + slack >= t && d <= t + slack;
}

// NEC / extended NEC.  Returns 1 = data frame (code out), 2 = repeat frame,
// 0 = no match.  Code is bit-0-first as transmitted: byte 0 = address,
// byte 3 = ~command for classic NEC.
static int ir_decode_nec(uint32_t *code) {
    const uint16_t *d = s_frame;
    if (s_frame_n == 3 &&
        ir_match(d[0], NEC_HDR_MARK) && ir_match(d[1], NEC_RPT_SPACE) &&
        ir_match(d[2], NEC_BIT_MARK))
        return 2;
    if (s_frame_n < 67 ||
        !ir_match(d[0], NEC_HDR_MARK) || !ir_match(d[1], NEC_HDR_SPACE))
        return 0;
    uint32_t c = 0;
    for (int i = 0; i < 32; i++) {
        if (!ir_match(d[2 + 2 * i], NEC_BIT_MARK)) return 0;
        uint16_t sp = d[3 + 2 * i];
        if (ir_match(sp, NEC_ONE_SPACE))       c |= 1u << i;
        else if (!ir_match(sp, NEC_ZERO_SPACE)) return 0;
    }
    if (!ir_match(d[66], NEC_BIT_MARK)) return 0;
    *code = c;
    return 1;
}

// Expand frame durations from `first` onward into a stream of equal-width
// half/unit levels (1 = mark).  Each duration must quantize to 1..max_units
// of `unit`; returns the total count, or 0 on a bad width / overflow of `cap`.
static int ir_expand_units(uint8_t first, uint16_t unit, int max_units,
                           uint8_t *out, int cap) {
    int n = 0;
    for (int i = first; i < s_frame_n; i++) {
        uint16_t d = s_frame[i];
        int units = (d + unit / 2) / unit;
        if (units < 1 || units > max_units) return 0;
        uint16_t ideal = (uint16_t)(units * unit);
        uint16_t err = d > ideal ? (uint16_t)(d - ideal) : (uint16_t)(ideal - d);
        if (err > unit / 3 + 100) return 0;
        uint8_t level = (i & 1) ? 0 : 1;    // even index = mark
        if (n + units > cap) return 0;
        while (units--) out[n++] = level;
    }
    return n;
}

// RC5 (14 bits, 889 µs half-bits, bit = second-half level).  The toggle bit
// (wire bit 11) is masked.  RC5X's inverted S2 rides along in bit 12.
static bool ir_decode_rc5(uint32_t *code) {
    if (s_frame_n < 11 || s_frame_n > 28) return false;
    uint8_t h[27];
    int n = ir_expand_units(0, RC5_UNIT, 2, h, 27);
    if (n < 25) return false;                 // must reach bit 12's second half
    while (n < 27) h[n++] = 0;                // trailing spaces are uncaptured
    if (h[0] != 1) return false;              // S1 = 1: frame opens with a mark
    uint32_t c = 1u << 13;
    for (int k = 1; k < 14; k++) {
        uint8_t first = h[2 * k - 1], second = h[2 * k];
        if (first == second) return false;    // Manchester violation
        if (second) c |= 1u << (13 - k);
    }
    *code = c & ~(1u << 11);                  // mask toggle
    return true;
}

// RC6 mode 0 (leader + start bit + 3 mode bits + double-width toggle +
// 16 data bits, 444 µs units, bit = FIRST-half level).  Toggle is checked
// for shape and dropped from the code: code = (mode+1)<<16 | control<<8 |
// info.  mode+1 keeps address 0 / command 0 away from 0 (the unlearned
// sentinel in IrCommand.code).
static bool ir_decode_rc6(uint32_t *code) {
    if (s_frame_n < 12 || s_frame_n > 44) return false;
    if (!ir_match(s_frame[0], 6 * RC6_UNIT) || !ir_match(s_frame[1], 2 * RC6_UNIT))
        return false;
    // Expand everything after the leader pair (index 2 is a mark, so the
    // even-index-equals-mark parity in ir_expand_units holds).
    uint8_t u[44];
    int n = ir_expand_units(2, RC6_UNIT, 3, u, 44);
    if (n < 43) return false;                 // data bit 15's mark must exist
    while (n < 44) u[n++] = 0;
    // Start bit = 1 (mark, space)
    if (u[0] != 1 || u[1] != 0) return false;
    uint32_t mode = 0;
    for (int k = 0; k < 3; k++) {
        uint8_t first = u[2 + 2 * k], second = u[3 + 2 * k];
        if (first == second) return false;
        mode = (mode << 1) | first;
    }
    // Toggle: two double-width halves
    if (u[8] != u[9] || u[10] != u[11] || u[8] == u[10]) return false;
    uint32_t data = 0;
    for (int k = 0; k < 16; k++) {
        uint8_t first = u[12 + 2 * k], second = u[13 + 2 * k];
        if (first == second) return false;
        data = (data << 1) | first;
    }
    *code = ((mode + 1) << 16) | data;
    return true;
}

// Protocol-agnostic fallback: FNV-1a over the ternary shape of consecutive
// durations (shorter / equal / longer within 20%).  Stable per button and
// per remote; toggle-free protocols repeat by re-transmission, so REPEAT
// still works.  Never returns 0 (0 marks an empty sub-slot).
static uint32_t ir_hash_frame(void) {
    uint32_t hsh = 2166136261u;
    for (int i = 1; i < s_frame_n; i++) {
        uint16_t a = s_frame[i - 1], b = s_frame[i];
        uint8_t sym = 0;
        if (b + b / 5 < a)      sym = 1;
        else if (a + a / 5 < b) sym = 2;
        hsh = (hsh ^ sym) * 16777619u;
    }
    hsh = (hsh ^ s_frame_n) * 16777619u;
    return hsh ? hsh : 0x5A5A5A5Au;
}

// ---------------------------------------------------------------------------
// Frame -> events
// ---------------------------------------------------------------------------

static void ir_emit(uint8_t kind, uint8_t proto, uint32_t code) {
    if (s_evq_n < (uint8_t)(sizeof(s_evq) / sizeof(s_evq[0]))) {
        s_evq[s_evq_n].kind = kind;
        s_evq[s_evq_n].protocol = proto;
        s_evq[s_evq_n].code = code;
        s_evq_n++;
    }
}

static void ir_frame_complete(uint32_t now) {
    uint8_t proto;
    uint32_t code;

    if (s_frame_overflow || s_frame_n < 3) return;

    int nec = ir_decode_nec(&code);
    if (nec == 2) {
        // NEC repeat frame: an unambiguous "still held" marker.  Extend any
        // held button, not just protocol NEC; NEC-variant remotes that fall
        // back to the hash decoder send this exact repeat frame too.
        if (s_learn_state == CS_IR_LEARN_ARMED) return;
        if (s_held) {
            s_last_frame_us = now;
            ir_emit(CS_IR_EVT_REPEAT, s_held_proto, s_held_code);
        }
        return;
    } else if (nec == 1) {
        proto = CS_IR_PROTO_NEC;
    } else if (ir_decode_rc5(&code)) {
        proto = CS_IR_PROTO_RC5;
    } else if (ir_decode_rc6(&code)) {
        proto = CS_IR_PROTO_RC6;
    } else if (s_frame_n >= 8) {
        proto = CS_IR_PROTO_HASH;
        code = ir_hash_frame();
    } else {
        return;                    // too short to hash: ambient IR noise
    }

    if (s_learn_state == CS_IR_LEARN_ARMED) {
        s_learn_proto = proto;
        s_learn_code = code;
        s_learn_state = CS_IR_LEARN_DONE;
        s_learn_changed = true;
        return;                    // learning consumes the press
    }

    if (s_held && s_held_proto == proto && s_held_code == code &&
        (uint32_t)(now - s_last_frame_us) < IR_HOLD_GAP_US) {
        s_last_frame_us = now;
        ir_emit(CS_IR_EVT_REPEAT, proto, code);
        return;
    }
    if (s_held)
        ir_emit(CS_IR_EVT_RELEASE, s_held_proto, s_held_code);
    s_held = true;
    s_held_proto = proto;
    s_held_code = code;
    s_last_frame_us = now;
    ir_emit(CS_IR_EVT_PRESS, proto, code);
}

// ---------------------------------------------------------------------------
// Poll
// ---------------------------------------------------------------------------

bool cs_ir_poll(CsIrEvent *ev) {
    uint32_t now = time_us_32();

    if (s_learn_state == CS_IR_LEARN_ARMED &&
        (uint32_t)(now - s_learn_armed_us) > IR_LEARN_WINDOW_US) {
        s_learn_state = CS_IR_LEARN_TIMEOUT;
        s_learn_changed = true;
    }

    if (s_attached) {
        // Drain the capture ring into the frame buffer.
        while (s_tail != s_head) {
            uint16_t entry = s_ring[s_tail];
            s_tail = (uint8_t)((s_tail + 1) & IR_RING_MASK);
            uint16_t dur = entry & IR_DUR_MAX;
            uint8_t level = (entry & 0x8000) ? 1 : 0;
            bool is_mark = (level == s_active_level);
            if (!is_mark && dur >= IR_FRAME_GAP_US) {
                // Long space: closes the current frame, opens the next.
                if (s_in_frame) ir_frame_complete(now);
                s_frame_n = 0;
                s_frame_overflow = false;
                s_in_frame = true;
                continue;
            }
            if (!s_in_frame) continue;         // mid-frame attach; wait for a gap
            if (s_frame_n >= IR_MAX_DURS) s_frame_overflow = true;
            else                          s_frame[s_frame_n++] = dur;
        }
        // Gap timeout with no further edges (the usual end of a frame: the
        // closing edge never comes because the line just idles).
        if (s_in_frame && s_frame_n > 0 &&
            (uint32_t)(now - s_last_edge_us) > IR_FRAME_GAP_US) {
            ir_frame_complete(now);
            s_frame_n = 0;
            s_frame_overflow = false;
        }
        // Hold release.  Emit before clearing so the queued RELEASE is the
        // record of the transition, not an afterthought that could be lost.
        if (s_held && (uint32_t)(now - s_last_frame_us) > IR_HOLD_GAP_US) {
            ir_emit(CS_IR_EVT_RELEASE, s_held_proto, s_held_code);
            s_held = false;
        }
    }

    if (s_evq_n == 0) return false;
    *ev = s_evq[0];
    s_evq_n--;
    for (uint8_t i = 0; i < s_evq_n; i++) s_evq[i] = s_evq[i + 1];
    return true;
}

// ---------------------------------------------------------------------------
// Learn
// ---------------------------------------------------------------------------

void cs_ir_learn_arm(void) {
    // A held button must not stay logically held into learn mode.
    if (s_held) {
        ir_emit(CS_IR_EVT_RELEASE, s_held_proto, s_held_code);
        s_held = false;
    }
    s_learn_state = CS_IR_LEARN_ARMED;
    s_learn_changed = false;
    s_learn_armed_us = time_us_32();
}

void cs_ir_learn_cancel(void) {
    s_learn_state = CS_IR_LEARN_IDLE;
    s_learn_changed = false;
}

uint8_t cs_ir_learn_state(void) { return s_learn_state; }

bool cs_ir_learn_take_change(void) {
    if (!s_learn_changed) return false;
    s_learn_changed = false;
    return true;
}

bool cs_ir_learn_result(uint8_t *protocol, uint32_t *code) {
    if (s_learn_state != CS_IR_LEARN_DONE) return false;
    *protocol = s_learn_proto;
    *code = s_learn_code;
    return true;
}
