#ifndef PDM_GENERATOR_H
#define PDM_GENERATOR_H

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// Enable/disable PDM hardware (PIO + DMA) from Core 0
void pdm_set_enabled(bool enabled);

// Functions exposed to main.c
void pdm_setup_hw(uint8_t pin);
void pdm_core1_entry(void);
void pdm_update_clock(uint32_t freq);
void pdm_push_sample(int32_t sample, bool reset);
void pdm_change_pin(uint8_t new_pin);

// Fill the free-running PDM DMA ring with true silence and arm a write-lead
// re-anchor for the Core 1 processing loop.  Called from Core 0 immediately
// before a flash blackout, with Core 1 parked (no concurrent writer): the ring
// DMA keeps clocking through the window, so without this it would loop the
// last ~45 ms of modulator output.  Touches content only; DMA, PIO, and
// pointers are untouched, so PDM stays phase-continuous with the other slots.
void pdm_flash_silence(void);

// Buffer fill level accessors (called from Core 0 for stats)
uint8_t pdm_get_dma_fill_pct(void);
uint8_t pdm_get_ring_fill_pct(void);

// Core 1 mode and EQ worker state (written by Core 0, read by Core 1)
extern volatile Core1Mode core1_mode;
extern Core1EqWork core1_eq_work;
extern volatile bool pdm_enabled;

#endif // PDM_GENERATOR_H
