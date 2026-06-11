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
    INPUT_SOURCE_USB   = 0,
    INPUT_SOURCE_SPDIF = 1,
    INPUT_SOURCE_I2S   = 2,
    // Future: INPUT_SOURCE_ADAT = 3
} InputSource;

#define INPUT_SOURCE_MAX    INPUT_SOURCE_I2S   // Highest valid value

// Default SPDIF RX GPIO pin.  GPIO 5 sits just below the output-pin
// neighborhood (SPDIF outs on 6-9, PDM on 10) and is unused by any
// default output, leaving GPIO 11 free for the DAC hardware-mute
// default (see DAC_HW_MUTE_DEFAULT_PIN in dac_hw_mute.h).
#define PICO_SPDIF_RX_PIN_DEFAULT  5

// Default I2S RX data GPIO. GPIO 4 is unused by any default assignment
// (SPDIF RX 5, outputs 6-9, PDM 10, DAC mute 11, UART TX 12, BCK 14,
// LRCLK 15, MCK 21 on RP2040 / 13 on RP2350).
#define PICO_I2S_RX_PIN_DEFAULT    4

// SPDIF RX lock debounce — firmware constant, not configurable via vendor command.
// After the library reports lock, wait this many ms before unmuting output.
#define SPDIF_RX_LOCK_DEBOUNCE_MS  100

// Current active input source (definition in audio_input.c)
extern volatile uint8_t active_input_source;

// SPDIF RX pin (device-level setting, stored in PresetDirectory)
extern uint8_t spdif_rx_pin;

// I2S RX data pin (same persistence model as spdif_rx_pin)
extern uint8_t i2s_rx_pin;

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

// Validate an input source value
static inline bool input_source_valid(uint8_t src) {
    return src <= INPUT_SOURCE_MAX;
}

// I2S input rate wire/flash encoding (1 byte): 0 = 44100, 1 = 48000,
// 2 = 96000. Unknown values decode to 48000.
static inline uint8_t i2s_rate_encode(uint32_t hz) {
    return (hz == 44100) ? 0 : ((hz == 96000) ? 2 : 1);
}
static inline uint32_t i2s_rate_decode(uint8_t enc) {
    return (enc == 0) ? 44100 : ((enc == 2) ? 96000 : 48000);
}

#endif // AUDIO_INPUT_H
