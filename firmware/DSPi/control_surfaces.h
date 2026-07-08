/*
 * control_surfaces.h; user-wired physical controls and indicators on spare GPIOs.
 *
 * A Control Surface binding attaches one physical component (push button,
 * toggle switch, potentiometer, rotary encoder, LED, PWM-dimmed LED) to one
 * firmware parameter (a "noun") through one operation (an "action").
 * Bindings are configured via vendor commands 0x84-0x87, persisted
 * device-global in the preset directory (V9+), and processed by a 1 kHz
 * main-loop tick.
 *
 * Every control action is applied by dispatching the SAME vendor command a
 * host would send, through vendor_dispatch_set/get with CTRL_SOURCE_GPIO.
 * That reuses all existing validation, deferred-apply safety, and host
 * notifications (tagged PARAM_SRC_GPIO); nothing in the apply path is
 * duplicated here.
 *
 * Validity is table-driven, not hand-enumerated: each component type
 * declares the actions it can drive (plus pin count and pin class) and each
 * noun declares the actions it accepts, its value unit, and whether it is
 * addressed by a channel/band target.  A binding is valid iff its action is
 * in both masks, its target/index address an existing channel/band, and its
 * pins pass the shared conflict checks.  Hosts read the same tables via
 * REQ_GET_CS_CAPS, so new types/nouns/actions appear in the UI without
 * host-side hardcoding.
 *
 * Format v2 (config version 2, caps version 2): bindings are 24 bytes and
 * gain explicit event / target / index fields; 16 slots.  Buttons support
 * press / long-press / double-press events (several bindings may share one
 * GPIO with distinct events), hold-to-repeat, and momentary (hold-to-engage)
 * actions; encoders support acceleration; nouns carry a unit so frequency
 * and Q step logarithmically.
 *
 * Caps v3 adds the IR remote component: one CS_TYPE_IR binding holds the
 * receiver GPIO and up to CS_MAX_IR_COMMANDS learned remote buttons live in
 * a separate command table (commands 0x8D-0x8F), each dispatching through
 * the same noun/action machinery as a physical button.  Binding, IR command
 * and slot-name SETs are apply-live-only previews; REQ_CS_SAVE persists the
 * whole live config and REQ_CS_REVERT reloads the stored one.
 *
 * See Documentation/Features/control_surfaces_spec.md.
 */

#ifndef CONTROL_SURFACES_H
#define CONTROL_SURFACES_H

#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// Component types.  Values are wire/flash-persistent; never renumber.
// ---------------------------------------------------------------------------
typedef enum {
    CS_TYPE_NONE    = 0,   // slot disabled
    CS_TYPE_BUTTON  = 1,   // momentary push button (1 GPIO, edge/gesture)
    CS_TYPE_SWITCH  = 2,   // latching toggle switch (1 GPIO, level-follow)
    CS_TYPE_POT     = 3,   // potentiometer / fader (1 ADC-capable GPIO)
    CS_TYPE_ENCODER = 4,   // quadrature rotary encoder (2 GPIOs)
    CS_TYPE_LED     = 5,   // indicator LED (1 GPIO, output)
    CS_TYPE_LED_PWM = 6,   // PWM-dimmed LED (1 GPIO, hardware PWM slice)
    CS_TYPE_IR      = 7,   // IR remote receiver (1 GPIO); a container slot
                           // whose commands live in the IrCommand table
    CS_TYPE_COUNT
} CsType;

// ---------------------------------------------------------------------------
// Nouns; the firmware parameters a surface can control or indicate.
// Values are wire/flash-persistent; never renumber, append only.
// ---------------------------------------------------------------------------
typedef enum {
    CS_NOUN_USER_VOLUME    = 0,  // continuous dB, shared with the host OS slider
    CS_NOUN_MASTER_VOLUME  = 1,  // continuous dB, device output ceiling
    CS_NOUN_USER_MUTE      = 2,  // bool
    CS_NOUN_LOUDNESS       = 3,  // bool (loudness compensation enable)
    CS_NOUN_CROSSFEED      = 4,  // bool (crossfeed enable)
    CS_NOUN_LEVELLER       = 5,  // bool (volume leveller enable)
    CS_NOUN_PRESET         = 6,  // enum 0..9 (active preset slot)
    CS_NOUN_INPUT_SOURCE   = 7,  // enum 0..2 (USB / SPDIF / I2S)
    CS_NOUN_CLIP           = 8,  // bool latch (any-channel clip); trigger clears
    // --- v2 additions ---
    CS_NOUN_EQ_BYPASS      = 9,  // bool (1 = master EQ bypassed)
    CS_NOUN_LG_SYNC        = 10, // bool (LG Sound Sync enable)
    CS_NOUN_CROSSFEED_PRESET = 11, // enum 0..3 (crossfeed voicing preset)
    CS_NOUN_CROSSFEED_ITD  = 12, // bool (crossfeed ITD enable)
    CS_NOUN_LEVELLER_AMOUNT = 13, // continuous percent 0..100
    CS_NOUN_LEVELLER_SPEED = 14, // enum 0..2 (slow / medium / fast)
    CS_NOUN_LEVELLER_LOOKAHEAD = 15, // bool
    CS_NOUN_PREAMP         = 16, // continuous dB; target = input channel
    CS_NOUN_OUTPUT_GAIN    = 17, // continuous dB; target = output channel
    CS_NOUN_OUTPUT_MUTE    = 18, // bool; target = output channel
    CS_NOUN_OUTPUT_ENABLE  = 19, // bool; target = output channel
    CS_NOUN_FILTER_FREQ    = 20, // continuous Hz; target = channel, index = band
    CS_NOUN_FILTER_GAIN    = 21, // continuous dB; target = channel, index = PEQ band
    CS_NOUN_FILTER_Q       = 22, // continuous Q; target = channel, index = PEQ band
    CS_NOUN_FILTER_TYPE    = 23, // enum (PEQ FilterType); target = channel, index = PEQ band
    CS_NOUN_FILTER_BYPASS  = 24, // bool; target = channel, index = band
    CS_NOUN_SIGGEN         = 25, // bool (test signal generator running)
    CS_NOUN_DAC_MUTE_TEST  = 26, // trigger (pulse the DAC hardware mute ~1 s)
    CS_NOUN_CLIP_CH        = 27, // bool, read-only; target = channel clip latch
    CS_NOUN_LEVEL          = 28, // continuous dB, read-only; target = channel peak meter
    CS_NOUN_SPDIF_LOCK     = 29, // bool, read-only (SPDIF RX locked)
    CS_NOUN_SAMPLE_RATE    = 30, // enum, read-only (0=44.1k 1=48k 2=96k)
    CS_NOUN_USB_STREAMING  = 31, // bool, read-only (USB input actively streaming)
    CS_NOUN_ADAT_ACTIVE    = 32, // bool, read-only (ADAT out streaming, rate ok; RP2350)
    CS_NOUN_LG_PRESENT     = 33, // bool, read-only (LG Sound Sync source detected)
    CS_NOUN_LG_MUTED       = 34, // bool, read-only (LG source present and muted)
    CS_NOUN_COUNT
} CsNoun;

// Value kinds (CsNounDesc.kind)
#define CS_KIND_CONTINUOUS  0
#define CS_KIND_BOOL        1
#define CS_KIND_ENUM        2

// Value units (CsNounDesc.unit).  Encoding of value/range fields and the
// stepping law follow the unit; see the spec, section 2.1.
#define CS_UNIT_NONE     0   // bool/enum: plain integers
#define CS_UNIT_DB       1   // 8.8 signed fixed point dB; linear stepping
#define CS_UNIT_HZ       2   // plain integer Hz; log stepping (step = 8.8 octaves)
#define CS_UNIT_Q        3   // 8.8 fixed point Q; log stepping (step = 8.8 octaves)
#define CS_UNIT_PERCENT  4   // 8.8 fixed point percent; linear stepping

// Target kinds (CsNounDesc.target_kind); what CsBinding.target addresses.
#define CS_TARGET_NONE      0   // target/index ignored
#define CS_TARGET_INPUT_CH  1   // target = input channel (0..target_count-1)
#define CS_TARGET_OUTPUT_CH 2   // target = output channel (0..target_count-1)
#define CS_TARGET_DSP_CH    3   // target = DSP channel (inputs then outputs)
#define CS_TARGET_DSP_BAND  4   // target = DSP channel, index = filter band

// Noun descriptor flags (CsNounDesc.dflags)
#define CS_NDF_DEFERRED  0x01   // apply is deferred; engine steps from a target shadow

// ---------------------------------------------------------------------------
// Actions (the "Parameter" of a binding).  Wire/flash-persistent values.
// Control actions (input components) and indicate actions (output
// components) share one namespace; the type/noun masks keep them apart.
// ---------------------------------------------------------------------------
typedef enum {
    CS_ACT_ADJUST     = 0,  // pot: absolute position maps onto a value range
    CS_ACT_STEP       = 1,  // encoder: +/- `step` per detent (enum: next/prev)
    CS_ACT_INC        = 2,  // button: + `step` per press (enum: next; +WRAP = cycle)
    CS_ACT_DEC        = 3,  // button: - `step` per press (enum: previous)
    CS_ACT_TOGGLE     = 4,  // button: invert a bool per press
    CS_ACT_SET        = 5,  // button: set the noun to `value` per press
    CS_ACT_FOLLOW     = 6,  // switch: bool tracks the switch position
    CS_ACT_TRIGGER    = 7,  // button: fire the noun's command (e.g. clip clear)
    CS_ACT_IND_EQUALS = 8,  // LED: lit while noun value == `value`
    CS_ACT_MOMENTARY  = 9,  // button: hold = set to `value`, release = restore
    CS_ACT_IND_ABOVE  = 10, // LED: lit while noun value >= `value`
    CS_ACT_IND_LEVEL  = 11, // PWM LED: brightness follows the noun value
    CS_ACT_COUNT
} CsAction;

#define CS_ACT_BIT(a)  (1u << (a))

// Button events (CsBinding.event; CS_TYPE_BUTTON only, 0 for other types).
// Bindings of button type may share one GPIO when their events differ.
typedef enum {
    CS_EVT_PRESS  = 0,   // short press (default)
    CS_EVT_LONG   = 1,   // held >= 500 ms
    CS_EVT_DOUBLE = 2,   // two presses within 350 ms
    CS_EVT_COUNT
} CsEvent;

// Binding flags
#define CS_FLAG_INVERT   0x01  // input: active-high w/ pull-down (default is
                               // active-low w/ pull-up); LED: drive low = lit
#define CS_FLAG_REVERSE  0x02  // pot/encoder: invert direction
#define CS_FLAG_WRAP     0x04  // enum STEP/INC/DEC wraps around the ends
#define CS_FLAG_ACCEL    0x08  // encoder: fast rotation multiplies the step
#define CS_FLAG_REPEAT   0x10  // button INC/DEC: auto-repeat while held
#define CS_FLAG_ALL      0x1F

// Pin classes (CsTypeDesc.pin_class)
#define CS_PINCLASS_ANY  0
#define CS_PINCLASS_ADC  1     // GPIO 26..28 (ADC0..2 on both platforms)

#define CS_MAX_BINDINGS  16
#define CS_GPIO_UNUSED   0xFF

// IR remote control.  One CS_TYPE_IR binding (the receiver) may be live at a
// time; its remote-button commands live in a separate table of sub-slots so
// eight commands cost one binding slot and one GPIO.
#define CS_MAX_IR_COMMANDS  8

// IR code protocols (IrCommand.protocol).  Wire/flash-persistent values.
// NONE marks an empty sub-slot.  NEC and RC5/RC6 are decoded properly (NEC
// repeat frames drive hold-to-repeat; the RC5/RC6 toggle bit is masked so a
// learned button matches every press); everything else falls back to a
// timing-signature hash, matched by exact re-transmission.
#define CS_IR_PROTO_NONE   0
#define CS_IR_PROTO_NEC    1
#define CS_IR_PROTO_RC5    2
#define CS_IR_PROTO_RC6    3
#define CS_IR_PROTO_HASH   4
#define CS_IR_PROTO_COUNT  5

// Learn state (CsStatusPacket.ir_learn_state / REQ_CS_IR_LEARN result read)
#define CS_IR_LEARN_IDLE     0
#define CS_IR_LEARN_ARMED    1   // listening; next decoded press is captured
#define CS_IR_LEARN_DONE     2   // result available (protocol + code)
#define CS_IR_LEARN_TIMEOUT  3   // nothing received within the learn window

// Per-slot user label ("Sub Level", "Mute All", ...), NUL-terminated, set by
// the host app and persisted device-global in the preset directory (V10+)
// so external MCUs and apps on other hosts can read what each control is
// for.  Same 32-byte convention as preset and channel names.  Names are
// slot metadata, independent of the binding: they survive binding changes
// and slot clears, and may be set before a binding exists.  Like bindings
// and IR commands, a name SET is an apply-live-only preview; REQ_CS_SAVE
// persists and REQ_CS_REVERT restores the stored names.
#define CS_NAME_LEN      32

// ---------------------------------------------------------------------------
// Wire / flash structures
// ---------------------------------------------------------------------------

// One binding; 24 bytes, identical on the wire (REQ_SET/GET_CS_BINDING
// payload) and in flash.  value/step/range encoding follows the noun's unit
// (CS_UNIT_*); bool/enum values are plain integers.
// A CS_TYPE_IR binding is a container: gpio[0] is the receiver pin, INVERT
// selects an idle-low receiver (default is idle-high, e.g. TSOP38xx), and
// every other field must be 0.  Its commands live in the IrCommand table.
typedef struct __attribute__((packed)) {
    uint8_t type;          // CsType
    uint8_t noun;          // CsNoun
    uint8_t action;        // CsAction
    uint8_t flags;         // CS_FLAG_*
    uint8_t gpio[2];       // gpio[1] = CS_GPIO_UNUSED unless type needs two
    uint8_t event;         // CsEvent (buttons; 0 otherwise)
    uint8_t target;        // channel index for targeted nouns (else 0)
    uint8_t index;         // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    uint8_t reserved;      // write 0
    int16_t value;         // SET/MOMENTARY target, IND_EQUALS/IND_ABOVE comparand
    int16_t step;          // STEP/INC/DEC size; 0 = per-unit default
    int16_t range_min;     // pot/IND_LEVEL span; both 0 = the noun's full range
    int16_t range_max;
    uint8_t reserved2[6];  // write 0
} CsBinding;

// One IR remote command; 16 bytes, identical on the wire (REQ_SET/GET_CS_IR_CMD
// payload) and in flash.  Semantically a button-shaped binding: the same noun /
// action / target / value / step rules apply (actions INC/DEC/TOGGLE/SET/
// TRIGGER/MOMENTARY; flags WRAP and REPEAT), fired by the learned code instead
// of a GPIO edge.  protocol == CS_IR_PROTO_NONE marks the sub-slot empty (all
// other fields must then be 0).
typedef struct __attribute__((packed)) {
    uint8_t  noun;         // CsNoun
    uint8_t  action;       // CsAction (button subset)
    uint8_t  flags;        // CS_FLAG_WRAP | CS_FLAG_REPEAT
    uint8_t  target;       // channel index for targeted nouns (else 0)
    uint8_t  index;        // filter band for CS_TARGET_DSP_BAND nouns (else 0)
    uint8_t  protocol;     // CS_IR_PROTO_*
    int16_t  value;        // SET/MOMENTARY target
    int16_t  step;         // INC/DEC size; 0 = per-unit default
    uint8_t  reserved[2];  // write 0
    uint32_t code;         // learned code (encoding per protocol; see spec)
} IrCommand;

// Directory-persisted blob (device-global, V9+).  All-zero = every slot
// CS_TYPE_NONE = feature idle; a fresh directory needs no special seeding.
#define CS_CONFIG_VERSION  2
typedef struct __attribute__((packed)) {
    uint8_t   version;     // CS_CONFIG_VERSION
    uint8_t   reserved[3];
    CsBinding bindings[CS_MAX_BINDINGS];
} CsFlashConfig;           // 388 bytes

// IR command table, directory-persisted (device-global, V11+) beside
// cs_config.  All-zero = every sub-slot empty; a fresh directory needs no
// seeding (protocol 0 = CS_IR_PROTO_NONE).
#define CS_IR_CONFIG_VERSION  1
typedef struct __attribute__((packed)) {
    uint8_t   version;     // CS_IR_CONFIG_VERSION
    uint8_t   reserved[3];
    IrCommand cmds[CS_MAX_IR_COMMANDS];
} CsIrConfig;              // 132 bytes

// Capability descriptors (REQ_GET_CS_CAPS).  wValue = 0xFFFF returns the
// header + type table; wValue = noun index returns that noun's descriptor.
typedef struct __attribute__((packed)) {
    uint16_t actions;      // CS_ACT_BIT mask this component can drive
    uint8_t  pin_count;    // GPIOs consumed (1 or 2)
    uint8_t  pin_class;    // CS_PINCLASS_*
} CsTypeDesc;

typedef struct __attribute__((packed)) {
    uint8_t  caps_version; // capability format version (3)
    uint8_t  max_bindings; // CS_MAX_BINDINGS
    uint8_t  type_count;   // CS_TYPE_COUNT (table follows, index = CsType)
    uint8_t  noun_count;   // CS_NOUN_COUNT
    CsTypeDesc types[CS_TYPE_COUNT];
    // v3 additions (hosts locate these at offset 4 + 4*type_count).  The
    // CS_TYPE_IR type descriptor's action mask describes what its COMMANDS
    // may do; the container binding itself carries noun/action 0.
    uint8_t  max_ir_commands;  // CS_MAX_IR_COMMANDS
    uint8_t  reserved[3];
} CsCapsHeader;            // 4 + 4*CS_TYPE_COUNT + 4 = 40 bytes

typedef struct __attribute__((packed)) {
    uint8_t  kind;         // CS_KIND_*
    uint8_t  enum_count;   // CS_KIND_ENUM only
    uint16_t actions;      // CS_ACT_BIT mask this noun accepts (0 = unavailable
                           // on this platform, e.g. ADAT_ACTIVE on RP2040)
    int16_t  min_q;        // CS_KIND_CONTINUOUS range, unit-encoded
    int16_t  max_q;
    uint8_t  unit;         // CS_UNIT_*
    uint8_t  target_kind;  // CS_TARGET_*
    uint8_t  target_count; // valid targets 0..target_count-1 (0 if untargeted)
    uint8_t  dflags;       // CS_NDF_*
} CsNounDesc;              // 12 bytes

// REQ_GET_CS_STATUS response.  last_status / last_slot report the most
// recent deferred SET of any CS kind (binding, name, IR command, save,
// revert); IR command slots are reported as 0x80 | sub-slot.
typedef struct __attribute__((packed)) {
    uint8_t  last_status;  // result of the most recent deferred CS SET
    uint8_t  last_slot;    // slot that SET targeted (0x80 | n = IR sub-slot)
    uint8_t  max_bindings; // CS_MAX_BINDINGS
    uint8_t  dirty;        // 1 = live config differs from flash (unsaved preview)
    uint16_t active_mask;  // bit N = binding N live
    uint8_t  slot_status[CS_MAX_BINDINGS];  // per-slot apply status
    // v3 additions
    uint8_t  ir_active_mask;   // bit N = IR command N live (component up)
    uint8_t  ir_learn_state;   // CS_IR_LEARN_*
    uint8_t  ir_cmd_status[CS_MAX_IR_COMMANDS];  // per-sub-slot apply status
} CsStatusPacket;          // 32 bytes

// Status codes.  0x00..0x05 reuse the shared PIN_CONFIG_* namespace
// (config.h); Control Surfaces extends it from 0x10.
#define CS_STATUS_INVALID_SLOT    0x10
#define CS_STATUS_INVALID_TYPE    0x11
#define CS_STATUS_INVALID_NOUN    0x12
#define CS_STATUS_INVALID_ACTION  0x13  // action not allowed for type+noun
#define CS_STATUS_INVALID_VALUE   0x14  // value/step/range out of bounds
#define CS_STATUS_PIN_NOT_ADC     0x15  // pot on a non-ADC GPIO
#define CS_STATUS_PENDING         0x16  // SET accepted, apply not yet run
#define CS_STATUS_INVALID_TARGET  0x17  // target/index out of range for the noun
#define CS_STATUS_INVALID_EVENT   0x18  // bad event, or event on a non-button
#define CS_STATUS_PWM_CONFLICT    0x19  // PWM LED collides with another on the
                                        // same PWM slice+channel
#define CS_STATUS_EVENT_IN_USE    0x1A  // another button binding already has
                                        // this GPIO+event pair
#define CS_STATUS_BUSY            0x1B  // a previous binding SET is still
                                        // queued for apply; retry shortly
#define CS_STATUS_FLASH_ERROR     0x1C  // directory persist failed (REQ_CS_SAVE)
#define CS_STATUS_IR_IN_USE       0x1D  // another slot already holds the IR
                                        // component (one receiver per device)
#define CS_STATUS_NO_IR           0x1E  // IR command/learn needs a live
                                        // CS_TYPE_IR binding first

// ---------------------------------------------------------------------------
// Public API (all main-loop context)
// ---------------------------------------------------------------------------

// Boot init: load the persisted config and bring up every valid binding.
// Call at the END of core0_init, after preset_boot_load, all audio pin
// claims, notify_init, and the UART/I2C control interfaces, so pin-conflict
// results are truthful and dispatches see initialised state.  A stored
// binding whose pins now collide is kept down (visible in slot_status).
void control_surfaces_init(void);

// 1 kHz poll: debounce buttons/switches, decode gestures, decode encoders,
// read pots, drive LEDs, and dispatch resulting parameter changes.
// Self-throttled with time_us_64; cheap no-op when no binding is active.
// Call once per main loop iteration.
void control_surfaces_tick(void);

// Validate and apply one binding (type CS_TYPE_NONE clears the slot).
// Releases the slot's old pins, claims the new ones, and activates the
// runtime state.  Returns PIN_CONFIG_* / CS_STATUS_*; on failure the old
// binding is restored.  Caller persists on success.
uint8_t control_surfaces_apply_binding(uint8_t slot, const CsBinding *b);

// True if `pin` is claimed by any live binding.  Wired into
// pin_used_by_fixed_peripheral so no other subsystem can take a CS pin.
bool control_surfaces_owns_pin(uint8_t pin);

// Live config (the persistence source for REQ_CS_SAVE) and read-only
// accessors for the vendor GET handlers.
const CsFlashConfig *control_surfaces_config(void);
const CsIrConfig *control_surfaces_ir_config(void);
const char (*control_surfaces_names(void))[CS_NAME_LEN];
const CsBinding *control_surfaces_get_binding(uint8_t slot);  // NULL if bad slot
const IrCommand *control_surfaces_get_ir_cmd(uint8_t sub);    // NULL if bad sub-slot
const char *control_surfaces_get_name(uint8_t slot);          // NULL if bad slot

// Update one live slot name (copied, NUL termination guaranteed).  Live-only,
// like apply_binding; the caller marks the config dirty.  Returns
// PIN_CONFIG_SUCCESS or CS_STATUS_INVALID_SLOT.
uint8_t control_surfaces_apply_name(uint8_t slot, const char *name);
void control_surfaces_get_status(CsStatusPacket *out);
const CsCapsHeader *control_surfaces_caps_header(void);
const CsNounDesc *control_surfaces_noun_desc(uint8_t noun);   // NULL if bad noun

// Validate and apply one IR command (protocol CS_IR_PROTO_NONE clears the
// sub-slot).  Live-only, like apply_binding; the caller marks the config
// dirty.  Commands may be set before the IR component exists; they activate
// when it comes up.  Returns PIN_CONFIG_* / CS_STATUS_*.
uint8_t control_surfaces_apply_ir_cmd(uint8_t sub, const IrCommand *c);

// Re-apply the persisted config (bindings + IR commands + slot names) from
// the directory cache, discarding the live preview.  Per-slot failures land
// in slot_status exactly as at boot.  Main-loop only (releases and reclaims
// GPIOs).
void control_surfaces_revert(void);

// Preview dirty flag: set when a live apply diverges from flash, cleared by
// save / revert (main.c owns the transitions around the flash write).
bool control_surfaces_dirty(void);
void control_surfaces_set_dirty(bool dirty);

// Learn result snapshot for the REQ_CS_IR_LEARN result read:
// {state, protocol, 0, 0, code_le32}, 8 bytes.
void control_surfaces_get_ir_learn(uint8_t out[8]);

// Deferred SET (written by the vendor handler, consumed by the main loop;
// same shape as ctrl_set_uart_pending).  cs_last_status / cs_last_slot feed
// REQ_GET_CS_STATUS and report both binding and name SETs.
extern volatile bool    cs_set_binding_pending;
extern uint8_t          cs_set_binding_slot;
extern CsBinding        cs_set_binding_val;
extern volatile uint8_t cs_last_status;
extern volatile uint8_t cs_last_slot;

// Deferred slot-name SET (REQ_SET_CS_NAME); live-only preview like the
// binding SET, consumed in the main loop for the shared status channel.
extern volatile bool    cs_set_name_pending;
extern uint8_t          cs_set_name_slot;
extern char             cs_set_name_val[CS_NAME_LEN];

// Deferred IR-command SET (REQ_SET_CS_IR_CMD); same single-deep handoff
// shape as the binding SET.  Results land in cs_last_status with
// cs_last_slot = 0x80 | sub-slot, plus the per-sub-slot status array.
extern volatile bool    cs_set_ir_cmd_pending;
extern uint8_t          cs_set_ir_cmd_slot;
extern IrCommand        cs_set_ir_cmd_val;

// Deferred save / revert (REQ_CS_SAVE / REQ_CS_REVERT).  Save persists the
// whole live CS config (bindings + IR commands + slot names) in one
// directory write; revert re-applies the stored config.  Results land in
// cs_last_status (cs_last_slot = 0xFF).
extern volatile bool    cs_save_pending;
extern volatile bool    cs_revert_pending;

// Learn control (REQ_CS_IR_LEARN): op 1 = arm, anything else = cancel.
// Direct call; every dispatch transport runs on the core0 main loop.
// Returns PIN_CONFIG_SUCCESS or CS_STATUS_NO_IR (arm without a live IR
// component).  Completion is pushed on the notify EP and readable via
// control_surfaces_get_ir_learn.
uint8_t control_surfaces_ir_learn_control(uint8_t op);

// ---------------------------------------------------------------------------
// Internal interface between the engine (control_surfaces.c) and the noun
// catalog (control_surfaces_nouns.c).  Not part of the host-facing API.
// ---------------------------------------------------------------------------

// The noun descriptor table (indexed by CsNoun).
extern const CsNounDesc cs_noun_table[CS_NOUN_COUNT];

// Live value of a noun in its natural units (bool 0/1, enum index,
// continuous dB/Hz/Q/percent).  target/index are pre-validated.
float cs_noun_get(uint8_t noun, uint8_t target, uint8_t index);

// Dispatch a resolved absolute target through the shared vendor-command
// surface.  Returns false only on CTRL_DISPATCH_BUSY (caller retries next
// tick); OK and ERROR both count as done.
bool cs_noun_dispatch(uint8_t noun, uint8_t target, uint8_t index, float value);

// Validate a binding's target/index against the noun's addressing kind and
// the platform's live channel/band layout.  Returns PIN_CONFIG_SUCCESS or
// CS_STATUS_INVALID_TARGET.
uint8_t cs_noun_validate_target(const CsBinding *b);

#endif // CONTROL_SURFACES_H
