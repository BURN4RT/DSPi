PROJECT(DSPi)

add_compile_options(-fcommon)
add_compile_options(-O2)

# Use -O3 for DSP-critical files
set_source_files_properties(
    dsp_pipeline.c usb_audio.c vendor_commands.c audio_pipeline.c crossfeed.c loudness.c leveller.c crossover.c spdif_input.c i2s_input.c lg_sound_sync.c dac_hw_mute.c
    PROPERTIES COMPILE_FLAGS "-O3"
)

add_executable(DSPi
    audio_input.c
    audio_input.h
    audio_pipeline.c
    audio_pipeline.h
    bulk_params.c
    bulk_params.h
    config.h
    crossfeed.c
    crossfeed.h
    crossover.c
    crossover.h
    dac_hw_mute.c
    dac_hw_mute.h
    dcp_inline.h
    dsp_pipeline.c
    dsp_pipeline.h
    flash_clkdiv.c
    flash_clkdiv.h
    flash_storage.c
    flash_storage.h
    i2s_input.c
    i2s_input.h
    leveller.c
    leveller.h
    lg_sound_sync.c
    lg_sound_sync.h
    loudness.c
    loudness.h
    main.c
    notify.c
    notify.h
    pdm_generator.c
    pdm_generator.h
    spdif_input.c
    spdif_input.h
    usb_audio.c
    usb_audio.h
    vendor_commands.c
    vendor_commands.h
    usb_descriptors.c
    usb_descriptors.h
    usb_feedback_controller.c
    usb_feedback_controller.h
    tusb_config.h
)

if (PICO_PLATFORM STREQUAL "rp2040")
    target_sources(DSPi PRIVATE dsp_process_rp2040.S)
endif()

# DSPI_LOOPBACK (debug/verification only): fold a USB audio CAPTURE function
# into DSPi that streams output slot 0 back to the host (replaces the external
# USBrx recorder).  Configure dedicated build dirs with -DDSPI_LOOPBACK=ON;
# normal build dirs leave it OFF so release artifacts are unaffected.
option(DSPI_LOOPBACK "Build the USB audio loopback capture (output slot 0 -> host)" OFF)
if (DSPI_LOOPBACK)
    target_sources(DSPi PRIVATE loopback.c loopback.h)
    set_source_files_properties(loopback.c PROPERTIES COMPILE_FLAGS "-O3")
    target_compile_definitions(DSPi PRIVATE DSPI_LOOPBACK)
endif()

if (PICO_PLATFORM STREQUAL "rp2040")
    # RP2040: SPDIF RX on PIO1 SM2 (SM0=PDM; SM1 free since MCK moved to
    # CLK_GPOUT). I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=1)
else()
    # RP2350: SPDIF RX on PIO2 (dedicated block, proven working).
    # I2S RX reuses the same SM when SPDIF RX is inactive.
    target_compile_definitions(DSPi PRIVATE PICO_SPDIF_RX_PIO=2)
endif()
# PICO_FLASH_SPI_CLKDIV=6 is set at the top-level CMakeLists so it reaches
# the boot2 build (target-level defines don't propagate to it).

pico_set_binary_type(DSPi copy_to_ram)

pico_generate_pio_header(DSPi ${CMAKE_CURRENT_SOURCE_DIR}/i2s_input.pio)

target_include_directories(DSPi PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

target_compile_definitions(DSPi PRIVATE
    AUDIO_FREQ_MAX=48000

    # S/PDIF output configuration (pins defined in config.h)
    PICO_AUDIO_SPDIF_PIO=0
    PICO_AUDIO_SPDIF_DMA_IRQ=1
    # I2S TX on DMA_IRQ_0 — must stay separate from SPDIF TX (DMA_IRQ_1)
    # to avoid cross-driver IRQ gating between the two output libraries.
    PICO_AUDIO_I2S_DMA_IRQ=0

    # S/PDIF RX on DMA_IRQ_1 with SPDIF TX. Our library patches ensure
    # safe coexistence: private irq_handler_registered flag (not
    # irq_has_shared_handler), removed irq_set_enabled in teardown,
    # interrupt-safe _spdif_rx_common_end.
    PICO_SPDIF_RX_DMA_IRQ=1

    # Debug UART0 parked at the numeric top of the exposed GPIOs (16/17), clear
    # of every default audio pin (I2S-in 1-4, SPDIF-in 5, outputs 6-10, mute 11,
    # clocks 13-15) and of the ADC pins (26-28).  Both TX and RX are overridden
    # so the SDK default (RX on GPIO 1) can't steal the I2S-RX pair-0 data line.
    PICO_DEFAULT_UART_TX_PIN=16
    PICO_DEFAULT_UART_RX_PIN=17
)

target_link_libraries(DSPi
    pico_stdlib
    pico_multicore
    pico_unique_id
    hardware_pwm
    hardware_flash
    hardware_adc
    pico_audio_spdif_multi
    pico_audio_i2s_multi
    pico_spdif_rx
    tinyusb_device
)

pico_add_extra_outputs(DSPi)
