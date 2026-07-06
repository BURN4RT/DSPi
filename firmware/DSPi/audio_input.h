/*
 * audio_input.h — Input source abstraction for DSPi
 *
 * Defines the input source enum and switching infrastructure.
 * Currently supports USB and S/PDIF inputs; designed for future
 * extensibility to I2S and ADAT without restructuring.
 */

#ifndef AUDIO_INPUT_H
#define AUDIO_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Input source identifiers (extensible — leave gaps for future types)
typedef enum {
    INPUT_SOURCE_USB    = 0,
    INPUT_SOURCE_SPDIF  = 1,   // SPDIF input 1 (always enabled)
    INPUT_SOURCE_I2S    = 2,
    // 3 reserved: future INPUT_SOURCE_ADAT
    INPUT_SOURCE_SPDIF2 = 4,   // Optional SPDIF input 2 (disabled until enabled by host)
    INPUT_SOURCE_SPDIF3 = 5,   // Optional SPDIF input 3 (disabled until enabled by host)
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_SPDIF3   // Highest valid value (3 is a gap)

// Number of selectable SPDIF inputs. Input 0 (INPUT_SOURCE_SPDIF) is the
// always-present one; inputs 1..2 (SPDIF2/SPDIF3) are optional and share the
// single RX PIO state machine; only the active one ever claims its GPIO.
#define SPDIF_RX_NUM_INPUTS 3

// Default SPDIF RX GPIO pin.  GPIO 5 sits just below the output-pin
// neighborhood (SPDIF outs on 6-9, PDM on 10) and is unused by any
// default output, leaving GPIO 11 free for the DAC hardware-mute
// default (see DAC_HW_MUTE_DEFAULT_PIN in dac_hw_mute.h).
#define PICO_SPDIF_RX_PIN_DEFAULT  5

// Default GPIOs for the optional SPDIF inputs 2 and 3.  Both are free of
// every default assignment on RP2350; GPIO 21 is the RP2040 MCK default, so
// enable-time validation rejects the clash if MCK is enabled there.
#define PICO_SPDIF_RX_PIN2_DEFAULT 20
#define PICO_SPDIF_RX_PIN3_DEFAULT 21

// Default I2S RX data GPIO (stereo pair 0).  The four data-pin defaults are the
// contiguous block GPIO 1/2/3/4 (pairs 0/1/2/3), all unused by any default
// assignment (SPDIF RX 5, outputs 6-9, PDM 10, DAC mute 11, BCK 14, LRCLK 15,
// MCK 21 on RP2040 / 13 on RP2350), so enabling 4/6/8-channel input
// out of the box never self-collides.  Real boards override these per wiring.
#define PICO_I2S_RX_PIN_DEFAULT    1

// Maximum I2S RX stereo pairs.  Each pair is one PIO state machine + one DMA
// ring + one serial-data pin, sharing the single BCK/LRCLK.  RP2350 fans out to
// 4 pairs (8 channels), matching its 8 input channels and the freed DMA budget;
// RP2040's unified channel model has only the stereo pair and no spare PIO SM /
// DMA channels for more, so it stays at one pair.
#if PICO_RP2350
#define I2S_RX_MAX_PAIRS    4
#else
#define I2S_RX_MAX_PAIRS    1
#endif
#define I2S_RX_MAX_CHANNELS (I2S_RX_MAX_PAIRS * 2)

// SPDIF RX lock debounce — firmware constant, not configurable via vendor command.
// After the library reports lock, wait this many ms before unmuting output.
#define SPDIF_RX_LOCK_DEBOUNCE_MS  100

// Current active input source (definition in audio_input.c)
extern volatile uint8_t active_input_source;

// SPDIF RX pin (device-level setting, stored in PresetDirectory)
extern uint8_t spdif_rx_pin;

// GPIOs for the optional SPDIF inputs 2 and 3 ([0] = SPDIF2, [1] = SPDIF3).
// Same persistence model as spdif_rx_pin.  A disabled input's pin is only a
// stored preference: it is not reserved against other functions and is never
// claimed in hardware until the input is enabled AND selected.
extern uint8_t spdif_rx_pin_ext[SPDIF_RX_NUM_INPUTS - 1];

// Enable mask for the optional SPDIF inputs: bit 0 = SPDIF2, bit 1 = SPDIF3.
// 0 by default; both extra inputs absent from the source list and invisible
// to pin-conflict validation.  SPDIF input 1 is always enabled.
extern uint8_t spdif_rx_enabled_ext;

// I2S RX serial-data pins, one per stereo pair, each independently
// configurable (same per-pin persistence model as spdif_rx_pin).  [0] is the
// always-present stereo pair (default PICO_I2S_RX_PIN_DEFAULT); [1..3] are used
// only when i2s_input_channels selects more than 2 channels and are assigned by
// the host (multichannel input requires wiring one ADC data line per pair).
extern uint8_t i2s_rx_pin[I2S_RX_MAX_PAIRS];

// Active I2S input channel count: 2/4/6/8 on RP2350, always 2 on RP2040.
// Determines how many stereo pairs (SMs / DMA rings / data pins) the input
// claims.  Changing it requires a full input restart (i2s_input_restart_pending).
extern uint8_t i2s_input_channels;

// Selected sample rate for I2S input (device is the rate authority in
// I2S input mode; 44100 / 48000 / 96000)
extern uint32_t i2s_input_rate;

// Deferred input source switch (set by vendor command, handled in main loop)
extern volatile bool input_source_change_pending;
extern volatile uint8_t pending_input_source;

// Deferred I2S RX hot-swaps (set by vendor commands / bulk apply, handled
// in main loop): data-pin change, and full restart after a BCK pin change
// while the input SM is the clock master
extern volatile bool i2s_rx_pin_change_pending;
extern volatile bool i2s_input_restart_pending;

// I2S clock mode: who owns BCK/LRCLK while I2S is the input source.
// MASTER (default) = the device generates BCK/LRCLK and is the rate
// authority (selected via REQ_SET_INPUT_RATE).  SLAVE = an external master
// drives BCK/LRCLK (both pins become inputs); the rate is auto-detected
// from the external clocks and every output is slaved to them (I2S slots
// edge-driven, SPDIF/ADAT servo rate-matched, MCK forced off).  Has no
// effect while the input source is not I2S.
typedef enum {
    I2S_CLOCK_MODE_MASTER = 0,
    I2S_CLOCK_MODE_SLAVE  = 1,
} I2sClockMode;

extern uint8_t i2s_clock_mode;

// Deferred clock-mode apply: a mode change while I2S input is live needs a
// full input restart plus an output-side rebuild (I2S output SMs switch
// between clkout/dataout and the external-clock program), handled in the
// main loop.  The handler copies pending_i2s_clock_mode into i2s_clock_mode
// at apply time; boot/preset paths may also write the global directly when
// nothing I2S is live (dormant apply).
extern volatile bool i2s_clock_mode_change_pending;
extern volatile uint8_t pending_i2s_clock_mode;

// True when I2S slave clocking is in force (SLAVE selected AND I2S is the
// active input source)
static inline bool i2s_slave_mode_active(void) {
    return i2s_clock_mode == I2S_CLOCK_MODE_SLAVE &&
           active_input_source == INPUT_SOURCE_I2S;
}

// Structurally valid input source value (3 is the reserved ADAT gap)
static inline bool input_source_valid(uint8_t src) {
    return src <= INPUT_SOURCE_MAX && src != 3;
}

// True for any of the three SPDIF inputs
static inline bool input_source_is_spdif(uint8_t src) {
    return src == INPUT_SOURCE_SPDIF || src == INPUT_SOURCE_SPDIF2 ||
           src == INPUT_SOURCE_SPDIF3;
}

// SPDIF input index (0..2) for a source; 0 for non-SPDIF sources
static inline uint8_t spdif_index_for_source(uint8_t src) {
    if (src == INPUT_SOURCE_SPDIF2) return 1;
    if (src == INPUT_SOURCE_SPDIF3) return 2;
    return 0;
}

// InputSource value for a SPDIF input index (0..2)
static inline uint8_t spdif_source_for_index(uint8_t idx) {
    if (idx == 1) return INPUT_SOURCE_SPDIF2;
    if (idx == 2) return INPUT_SOURCE_SPDIF3;
    return INPUT_SOURCE_SPDIF;
}

// Configured GPIO for a SPDIF input index (0..2)
static inline uint8_t spdif_rx_pin_for_index(uint8_t idx) {
    return (idx == 0) ? spdif_rx_pin : spdif_rx_pin_ext[idx - 1];
}

// True if SPDIF input `idx` is enabled (index 0 is always enabled)
static inline bool spdif_input_enabled(uint8_t idx) {
    return idx == 0 || ((spdif_rx_enabled_ext >> (idx - 1)) & 1u) != 0;
}

// Valid AND currently offered in the source list (disabled SPDIF 2/3 are not)
static inline bool input_source_selectable(uint8_t src) {
    if (!input_source_valid(src)) return false;
    if (!input_source_is_spdif(src)) return true;
    return spdif_input_enabled(spdif_index_for_source(src));
}

// GPIO the RX library should run on for the currently-active SPDIF source
static inline uint8_t spdif_rx_active_pin(void) {
    return spdif_rx_pin_for_index(spdif_index_for_source(active_input_source));
}

// Validate a proposed I2S RX data-pin set (the first `n_pairs` entries of
// `pins[]` are the pairs that will be active) against the effective `bck_pin`:
// each active pin must be a valid GPIO, not a clock pin (BCK/LRCLK), not used by
// a fixed peripheral, and mutually distinct.  The bulk/preset restore paths use
// this to reject an inconsistent pushed/stored I2S config as a unit before it
// can reach i2s_input_start().  Defined in vendor_commands.c (alongside the
// other pin helpers); declared here so the restore paths can call it without
// pulling in the TinyUSB-heavy vendor_commands.h.
bool i2s_rx_pin_set_acceptable(const uint8_t *pins, uint8_t n_pairs, uint8_t bck_pin);

// True if `bck_pin` (LRCLK = bck_pin + 1) is acceptable as the I2S clock pair:
// both valid GPIOs and neither colliding with a fixed peripheral.  The
// bulk/preset restore paths use this to reject a pushed/stored BCK they would
// otherwise install raw (BCK/LRCLK are clock OUTPUTS — a collision is driver
// contention and an invalid GPIO can fault pio_gpio_init()).  Defined in
// vendor_commands.c; declared here for the same reason as above.
bool i2s_bck_pin_acceptable(uint8_t bck_pin);

// True if SPDIF input `idx` (1..2) could be enabled right now: its configured
// GPIO is valid and not claimed by any other function.  Defined in
// vendor_commands.c (alongside the other pin helpers); declared here so the
// bulk/preset restore paths can validate a stored enable before applying it.
bool spdif_input_enable_acceptable(uint8_t idx);

// I2S input rate wire/flash encoding (1 byte): 0 = 44100, 1 = 48000,
// 2 = 96000. Unknown values decode to 48000.
static inline uint8_t i2s_rate_encode(uint32_t hz) {
    return (hz == 44100) ? 0 : ((hz == 96000) ? 2 : 1);
}
static inline uint32_t i2s_rate_decode(uint8_t enc) {
    return (enc == 0) ? 44100 : ((enc == 2) ? 96000 : 48000);
}

#endif // AUDIO_INPUT_H
