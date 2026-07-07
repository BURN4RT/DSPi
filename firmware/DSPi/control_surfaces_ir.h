/*
 * control_surfaces_ir.h; IR remote receiver capture and decode for the
 * Control Surfaces CS_TYPE_IR component.  Engine-internal interface
 * (control_surfaces.c is the only client); the host-facing structures
 * live in control_surfaces.h.
 *
 * A GPIO edge interrupt timestamps mark/space durations into a small ring;
 * cs_ir_poll() (main loop, called from the CS tick) assembles frames and
 * decodes them: NEC/NECext (including repeat frames), RC5 and RC6 mode 0
 * (toggle bit masked), and a timing-signature hash fallback for every other
 * protocol.  Decoded remote buttons surface as press / repeat / release
 * events keyed by {protocol, code}, mirroring a physical button's press,
 * auto-repeat and release.
 */

#ifndef CONTROL_SURFACES_IR_H
#define CONTROL_SURFACES_IR_H

#include <stdint.h>
#include <stdbool.h>

// Event kinds surfaced to the engine
#define CS_IR_EVT_PRESS    0   // new button-down (a fresh code)
#define CS_IR_EVT_REPEAT   1   // button still held (NEC repeat frame, or the
                               // same code re-transmitted within the hold gap)
#define CS_IR_EVT_RELEASE  2   // hold gap elapsed with no further frames

typedef struct {
    uint8_t  kind;       // CS_IR_EVT_*
    uint8_t  protocol;   // CS_IR_PROTO_*
    uint32_t code;
} CsIrEvent;

// Claim / release the receiver pin.  attach() installs the IO_IRQ_BANK0
// edge handler (nothing else in the firmware uses GPIO interrupts); the
// caller has already configured the pin as an input with the right pull.
// invert = idle-low receiver (default assumes idle-high, e.g. TSOP38xx).
void cs_ir_attach(uint8_t pin, bool invert);
void cs_ir_detach(void);
bool cs_ir_attached(void);

// Drain one decoded event; call repeatedly until it returns false.  Runs
// frame assembly, decode, hold tracking and the learn state machine.
// While learn is armed, button events are suppressed (a synthetic RELEASE
// for a held button is emitted at arm time).
bool cs_ir_poll(CsIrEvent *ev);

// Learn: arm captures the next decoded press (10 s window).  State values
// are CS_IR_LEARN_* (control_surfaces.h).  take_change() returns true once
// per ARMED -> DONE/TIMEOUT transition so the engine can push one notify.
void cs_ir_learn_arm(void);
void cs_ir_learn_cancel(void);
uint8_t cs_ir_learn_state(void);
bool cs_ir_learn_take_change(void);
bool cs_ir_learn_result(uint8_t *protocol, uint32_t *code);  // valid when DONE

#endif // CONTROL_SURFACES_IR_H
