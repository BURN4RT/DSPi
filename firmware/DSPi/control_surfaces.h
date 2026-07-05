/*
 * control_surfaces.h; user-wired physical controls and indicators on spare GPIOs.
 *
 * A Control Surface binding attaches one physical component (push button,
 * toggle switch, potentiometer, rotary encoder, LED) to one firmware
 * parameter (a "noun") through one operation (an "action").  Bindings are
 * configured via vendor commands 0x84-0x87, persisted device-global in the
 * preset directory (V7+), and processed by a 1 kHz main-loop tick.
 *
 * Every control action is applied by dispatching the SAME vendor command a
 * host would send, through vendor_dispatch_set/get with CTRL_SOURCE_GPIO.
 * That reuses all existing validation, deferred-apply safety, and host
 * notifications (tagged PARAM_SRC_GPIO); nothing in the apply path is
 * duplicated here.
 *
 * Validity is table-driven, not hand-enumerated: each component type
 * declares the actions it can drive (plus pin count and pin class) and each
 * noun declares the actions it accepts.  A binding is valid iff its action
 * is in both masks and its pins pass the shared conflict checks.  Hosts read
 * the same tables via REQ_GET_CS_CAPS, so new types/nouns/actions appear in
 * the UI without host-side hardcoding.  Future indicator types (character
 * displays) extend CsType + the tables; the engine and wire format are
 * unchanged.
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
    CS_TYPE_BUTTON  = 1,   // momentary push button (1 GPIO, edge-triggered)
    CS_TYPE_SWITCH  = 2,   // latching toggle switch (1 GPIO, level-follow)
    CS_TYPE_POT     = 3,   // potentiometer / fader (1 ADC-capable GPIO)
    CS_TYPE_ENCODER = 4,   // quadrature rotary encoder (2 GPIOs)
    CS_TYPE_LED     = 5,   // indicator LED (1 GPIO, output)
    CS_TYPE_COUNT
} CsType;

// ---------------------------------------------------------------------------
// Nouns; the firmware parameters a surface can control or indicate.
// Values are wire/flash-persistent; never renumber, append only.
// ---------------------------------------------------------------------------
typedef enum {
    CS_NOUN_USER_VOLUME   = 0,  // continuous dB, shared with the host OS slider
    CS_NOUN_MASTER_VOLUME = 1,  // continuous dB, device output ceiling
    CS_NOUN_USER_MUTE     = 2,  // bool
    CS_NOUN_LOUDNESS      = 3,  // bool (loudness compensation enable)
    CS_NOUN_CROSSFEED     = 4,  // bool (crossfeed enable)
    CS_NOUN_LEVELLER      = 5,  // bool (volume leveller enable)
    CS_NOUN_PRESET        = 6,  // enum 0..9 (active preset slot)
    CS_NOUN_INPUT_SOURCE  = 7,  // enum 0..2 (USB / SPDIF / I2S)
    CS_NOUN_CLIP          = 8,  // bool latch (any-channel clip); trigger clears
    CS_NOUN_COUNT
} CsNoun;

// Value kinds (CsNounDesc.kind)
#define CS_KIND_CONTINUOUS  0   // float dB, 8.8 fixed point on the wire
#define CS_KIND_BOOL        1
#define CS_KIND_ENUM        2

// ---------------------------------------------------------------------------
// Actions (the "Parameter" of a binding).  Wire/flash-persistent values.
// Control actions (input components) and indicate actions (output
// components) share one namespace; the type/noun masks keep them apart.
// ---------------------------------------------------------------------------
typedef enum {
    CS_ACT_ADJUST     = 0,  // pot: absolute position maps onto a value range
    CS_ACT_STEP       = 1,  // encoder: +/- `step` per detent (enum: next/prev)
    CS_ACT_INC        = 2,  // button: + `step` per press (enum: next)
    CS_ACT_DEC        = 3,  // button: - `step` per press (enum: previous)
    CS_ACT_TOGGLE     = 4,  // button: invert a bool per press
    CS_ACT_SET        = 5,  // button: set the noun to `value` per press
    CS_ACT_FOLLOW     = 6,  // switch: bool tracks the switch position
    CS_ACT_TRIGGER    = 7,  // button: fire the noun's command (e.g. clip clear)
    CS_ACT_IND_EQUALS = 8,  // LED: lit while noun value == `value`
    CS_ACT_COUNT
} CsAction;

#define CS_ACT_BIT(a)  (1u << (a))

// Binding flags
#define CS_FLAG_INVERT   0x01  // input: active-high w/ pull-down (default is
                               // active-low w/ pull-up); LED: drive low = lit
#define CS_FLAG_REVERSE  0x02  // pot/encoder: invert direction
#define CS_FLAG_WRAP     0x04  // enum STEP/INC/DEC wraps around the ends

// Pin classes (CsTypeDesc.pin_class)
#define CS_PINCLASS_ANY  0
#define CS_PINCLASS_ADC  1     // GPIO 26..28 (ADC0..2 on both platforms)

#define CS_MAX_BINDINGS  8
#define CS_GPIO_UNUSED   0xFF

// ---------------------------------------------------------------------------
// Wire / flash structures
// ---------------------------------------------------------------------------

// One binding; 16 bytes, identical on the wire (REQ_SET/GET_CS_BINDING
// payload) and in flash.  Continuous values/steps/ranges are dB in 8.8
// signed fixed point (1 dB = 256); bool/enum values are plain integers.
typedef struct __attribute__((packed)) {
    uint8_t type;          // CsType
    uint8_t noun;          // CsNoun
    uint8_t action;        // CsAction
    uint8_t flags;         // CS_FLAG_*
    uint8_t gpio[2];       // gpio[1] = CS_GPIO_UNUSED unless type needs two
    uint8_t reserved[2];
    int16_t value;         // CS_ACT_SET target / CS_ACT_IND_EQUALS comparand
    int16_t step;          // STEP/INC/DEC size; 0 = default (1 dB / 1 step)
    int16_t range_min;     // pot span; both 0 = the noun's full range
    int16_t range_max;
} CsBinding;

// Directory-persisted blob (device-global, V7+).  All-zero = every slot
// CS_TYPE_NONE = feature idle; a fresh directory needs no special seeding.
#define CS_CONFIG_VERSION  1
typedef struct __attribute__((packed)) {
    uint8_t   version;     // CS_CONFIG_VERSION
    uint8_t   reserved[3];
    CsBinding bindings[CS_MAX_BINDINGS];
} CsFlashConfig;           // 132 bytes

// Capability descriptors (REQ_GET_CS_CAPS).  wValue = 0xFFFF returns the
// header + type table; wValue = noun index returns that noun's descriptor.
typedef struct __attribute__((packed)) {
    uint16_t actions;      // CS_ACT_BIT mask this component can drive
    uint8_t  pin_count;    // GPIOs consumed (1 or 2)
    uint8_t  pin_class;    // CS_PINCLASS_*
} CsTypeDesc;

typedef struct __attribute__((packed)) {
    uint8_t  caps_version; // capability format version (1)
    uint8_t  max_bindings; // CS_MAX_BINDINGS
    uint8_t  type_count;   // CS_TYPE_COUNT (table follows, index = CsType)
    uint8_t  noun_count;   // CS_NOUN_COUNT
    CsTypeDesc types[CS_TYPE_COUNT];
} CsCapsHeader;            // 4 + 4*CS_TYPE_COUNT bytes

typedef struct __attribute__((packed)) {
    uint8_t  kind;         // CS_KIND_*
    uint8_t  enum_count;   // CS_KIND_ENUM only
    uint16_t actions;      // CS_ACT_BIT mask this noun accepts
    int16_t  min_q8;       // CS_KIND_CONTINUOUS range, dB 8.8
    int16_t  max_q8;
} CsNounDesc;              // 8 bytes

// REQ_GET_CS_STATUS response
typedef struct __attribute__((packed)) {
    uint8_t last_status;   // result of the most recent REQ_SET_CS_BINDING
    uint8_t last_slot;     // slot that SET targeted
    uint8_t max_bindings;  // CS_MAX_BINDINGS
    uint8_t active_mask;   // bit N = binding N live
    uint8_t slot_status[CS_MAX_BINDINGS];  // per-slot apply status
} CsStatusPacket;          // 12 bytes

// Status codes.  0x00..0x05 reuse the shared PIN_CONFIG_* namespace
// (config.h); Control Surfaces extends it from 0x10.
#define CS_STATUS_INVALID_SLOT    0x10
#define CS_STATUS_INVALID_TYPE    0x11
#define CS_STATUS_INVALID_NOUN    0x12
#define CS_STATUS_INVALID_ACTION  0x13  // action not allowed for type+noun
#define CS_STATUS_INVALID_VALUE   0x14  // value/step/range out of bounds
#define CS_STATUS_PIN_NOT_ADC     0x15  // pot on a non-ADC GPIO
#define CS_STATUS_PENDING         0x16  // SET accepted, apply not yet run

// ---------------------------------------------------------------------------
// Public API (all main-loop context)
// ---------------------------------------------------------------------------

// Boot init: load the persisted config and bring up every valid binding.
// Call at the END of core0_init, after preset_boot_load, all audio pin
// claims, notify_init, and the UART/I2C control interfaces, so pin-conflict
// results are truthful and dispatches see initialised state.  A stored
// binding whose pins now collide is kept down (visible in slot_status).
void control_surfaces_init(void);

// 1 kHz poll: debounce buttons/switches, decode encoders, read pots, drive
// LEDs, and dispatch resulting parameter changes.  Self-throttled with
// time_us_64; cheap no-op when no binding is active.  Call once per main
// loop iteration.
void control_surfaces_tick(void);

// Validate and apply one binding (type CS_TYPE_NONE clears the slot).
// Releases the slot's old pins, claims the new ones, and activates the
// runtime state.  Returns PIN_CONFIG_* / CS_STATUS_*; on failure the old
// binding is restored.  Caller persists on success.
uint8_t control_surfaces_apply_binding(uint8_t slot, const CsBinding *b);

// True if `pin` is claimed by any live binding.  Wired into
// pin_used_by_fixed_peripheral so no other subsystem can take a CS pin.
bool control_surfaces_owns_pin(uint8_t pin);

// Live config (the persistence source for preset_set_cs_config) and
// read-only accessors for the vendor GET handlers.
const CsFlashConfig *control_surfaces_config(void);
const CsBinding *control_surfaces_get_binding(uint8_t slot);  // NULL if bad slot
void control_surfaces_get_status(CsStatusPacket *out);
const CsCapsHeader *control_surfaces_caps_header(void);
const CsNounDesc *control_surfaces_noun_desc(uint8_t noun);   // NULL if bad noun

// Deferred SET (written by the vendor handler, consumed by the main loop;
// same shape as ctrl_set_uart_pending).  cs_last_status / cs_last_slot feed
// REQ_GET_CS_STATUS.
extern volatile bool    cs_set_binding_pending;
extern uint8_t          cs_set_binding_slot;
extern CsBinding        cs_set_binding_val;
extern volatile uint8_t cs_last_status;
extern volatile uint8_t cs_last_slot;

#endif // CONTROL_SURFACES_H
