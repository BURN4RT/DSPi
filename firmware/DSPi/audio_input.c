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

// I2S RX serial-data pins, one per stereo pair.  [0] is the always-present
// stereo pair; [1..3] (RP2350) are placeholders the host assigns when enabling
// >2-channel I2S input.  Defaults are the contiguous block GPIO 1/2/3/4 (pairs
// 0/1/2/3): all free of every default assignment, so enabling 4/6/8-channel
// input out of the box never self-collides.  They are still expected to be
// overridden per board, since multichannel input requires explicit wiring of
// each ADC data line.
uint8_t i2s_rx_pin[I2S_RX_MAX_PAIRS] = {
    PICO_I2S_RX_PIN_DEFAULT,            // pair 0 = GPIO 1
#if I2S_RX_MAX_PAIRS > 1
    2, 3, 4,                            // pairs 1/2/3 = GPIO 2/3/4
#endif
};

// Active I2S input channel count (2/4/6/8 on RP2350, 2 on RP2040)
uint8_t i2s_input_channels = 2;

// Selected sample rate while I2S input is active (device is the rate
// authority in I2S input mode)
uint32_t i2s_input_rate = 48000;

// Deferred input source switch
volatile bool input_source_change_pending = false;
volatile uint8_t pending_input_source = INPUT_SOURCE_USB;

// Deferred I2S RX hot-swaps (handled in main loop)
volatile bool i2s_rx_pin_change_pending = false;
volatile bool i2s_input_restart_pending = false;
