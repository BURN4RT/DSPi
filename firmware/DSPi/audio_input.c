/*
 * audio_input.c — Input source state for DSPi
 *
 * Global definitions for the input source abstraction layer.
 * Phase 2 adds SPDIF RX lifecycle functions here.
 */

#include "audio_input.h"

// Active input source — default to USB
volatile uint8_t active_input_source = INPUT_SOURCE_USB;

// SPDIF RX GPIO pin — device-level setting (not per-preset)
uint8_t spdif_rx_pin = PICO_SPDIF_RX_PIN_DEFAULT;

// I2S RX data pin (same persistence model as spdif_rx_pin)
uint8_t i2s_rx_pin = PICO_I2S_RX_PIN_DEFAULT;

// Selected sample rate while I2S input is active (device is the rate
// authority in I2S input mode)
uint32_t i2s_input_rate = 48000;

// Deferred input source switch
volatile bool input_source_change_pending = false;
volatile uint8_t pending_input_source = INPUT_SOURCE_USB;

// Deferred I2S RX hot-swaps (handled in main loop)
volatile bool i2s_rx_pin_change_pending = false;
volatile bool i2s_input_restart_pending = false;
