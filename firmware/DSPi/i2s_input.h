/*
 * i2s_input.h - I2S receiver integration for DSPi
 *
 * I2S input is synchronous to the device's own clock domain (master
 * architecture: the external source slaves to our BCK/LRCLK), so unlike
 * SPDIF input there is no clock servo, no rate detection and no lock
 * state machine. Two roles exist:
 *
 *   clock master - no output slot is I2S; the input SM drives BCK/LRCLK
 *                  via side-set while sampling data
 *   slave        - at least one output slot is I2S; the input SM samples
 *                  data against the BCK/LRCLK pads driven by the output
 *                  clock master
 *
 * Reuses the SPDIF RX PIO state machine and DMA channels, which are free
 * whenever SPDIF input is inactive (inputs are switched, never mixed).
 */

#ifndef I2S_INPUT_H
#define I2S_INPUT_H

#include <stdint.h>
#include <stdbool.h>

// Default data pin: PICO_I2S_RX_PIN_DEFAULT in audio_input.h

// I2S RX state. No lock concept: clocks are ours, so the input is either
// running or not selected.
typedef enum {
    I2S_INPUT_INACTIVE = 0,   // Hardware stopped (not selected as input)
    I2S_INPUT_RUNNING  = 1,   // Receiving and processing audio
} I2sInputState;

// Initialize the subsystem (called once at boot, no hardware claimed)
void i2s_input_init(void);

// Start I2S RX hardware in the given role. Claims the SPDIF RX PIO SM and
// DMA channels; caller must ensure SPDIF RX is inactive.
void i2s_input_start(bool clock_master);

// Stop I2S RX hardware and release all claimed resources
void i2s_input_stop(void);

// Re-phase a running slave-role input after the I2S TX clock master has
// been restarted (which resets LRCLK phase). No-op unless RUNNING in the
// slave role. Called at the end of complete_pipeline_reset() and
// enable_outputs_in_sync().
void i2s_input_resync(void);

// Main-loop poll: drain the DMA ring, apply preamp, feed the pipeline.
// Returns number of stereo frames processed.
uint32_t i2s_input_poll(void);

// Push one silent block through the pipeline to prefill the output consumer
// pools. Used only during a slave-role prefill, where the input is clocked by
// an I2S output and so cannot supply samples while the outputs are drained.
void i2s_input_prefill_silence(uint32_t frames);

// Get current state
I2sInputState i2s_input_get_state(void);

// True if RUNNING in the clock-master role
bool i2s_input_is_clock_master(void);

#endif // I2S_INPUT_H
