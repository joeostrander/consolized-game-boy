/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 * 
 */

#include <stdlib.h>
#include <string.h>

#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/regs/intctrl.h"
#include "shared_dma_handler.h"
#include <stdio.h>
#include "analog_microphone.h"

// SDK compatibility: RP2350 adds DMA IRQ2/IRQ3 but the helper functions may be absent.
#if PICO_RP2350
static inline void dma_channel_set_irq3_enabled_compat(uint channel, bool enabled)
{
    if (enabled)
        dma_hw->inte3 |= (1u << channel);
    else
        dma_hw->inte3 &= ~(1u << channel);
}
#endif

#define ANALOG_RAW_BUFFER_COUNT 2

static int audio_dma_chan = -1;

static struct {
    struct analog_microphone_config config;
    int dma_channel;
    uint16_t* raw_buffer[ANALOG_RAW_BUFFER_COUNT];
    volatile int raw_buffer_write_index;
    volatile int raw_buffer_read_index;
    uint buffer_size;
    int16_t bias;
    int dma_irq;
    analog_samples_ready_handler_t samples_ready_handler;
} analog_mic;

// static void analog_dma_handler();
static void __no_inline_not_in_flash_func(analog_dma_handler)(void);

int analog_microphone_init(const struct analog_microphone_config* config) {
    memset(&analog_mic, 0x00, sizeof(analog_mic));
    memcpy(&analog_mic.config, config, sizeof(analog_mic.config));

    if (config->gpio < 26 || config->gpio > 29) {
        return -1;
    }

    size_t raw_buffer_size = config->sample_buffer_size * sizeof(analog_mic.raw_buffer[0][0]);

    analog_mic.buffer_size = config->sample_buffer_size;
    // analog_mic.bias = ((int16_t)((config->bias_voltage * 4095) / 3.3));

    for (int i = 0; i < ANALOG_RAW_BUFFER_COUNT; i++) {
        analog_mic.raw_buffer[i] = malloc(raw_buffer_size);
        if (analog_mic.raw_buffer[i] == NULL) {
            analog_microphone_deinit();

            return -1;   
        }
    }    analog_mic.dma_channel = dma_claim_unused_channel(true);
    if (analog_mic.dma_channel < 0) {
        analog_microphone_deinit();

        return -1;
    }
    
    // Store in global for shared handler
    audio_dma_chan = analog_mic.dma_channel;
    if (!SHARED_DMA_RegisterCallback(audio_dma_chan, analog_dma_handler))
    {
        printf("ERROR: Failed to register DMA callback for channel %d (shared handler not initialized yet?)\n", audio_dma_chan);
        analog_microphone_deinit();
        return -1;
    }
    printf("Registered DMA callback for channel: %d\n", audio_dma_chan);
    

    float clk_div = (clock_get_hz(clk_adc) / (1.0 * config->sample_rate)) - 1;    
    dma_channel_config dma_channel_cfg = dma_channel_get_default_config(analog_mic.dma_channel);

    channel_config_set_transfer_data_size(&dma_channel_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&dma_channel_cfg, false);
    channel_config_set_write_increment(&dma_channel_cfg, true);
    channel_config_set_dreq(&dma_channel_cfg, DREQ_ADC);
    // Keep audio DMA lower priority so video DMA can complete without being starved
    channel_config_set_high_priority(&dma_channel_cfg, false);

    // Normalize the DMA IRQ (in case caller used -1 for default)
    analog_mic.dma_irq = (config->dma_irq >= 0) ? config->dma_irq : DMA_IRQ_1;

    dma_channel_configure(
        analog_mic.dma_channel,
        &dma_channel_cfg,
        analog_mic.raw_buffer[0],
        &adc_hw->fifo,
        analog_mic.buffer_size,
        false
    );

    adc_gpio_init(config->gpio);

    adc_init();
    adc_select_input(config->gpio - 26);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false    // Don't shift each sample to 8 bits when pushing to FIFO
    );

    adc_set_clkdiv(clk_div);

    return 0;
}

void analog_microphone_deinit() {
    for (int i = 0; i < ANALOG_RAW_BUFFER_COUNT; i++) {
        if (analog_mic.raw_buffer[i]) {
            free(analog_mic.raw_buffer[i]);

            analog_mic.raw_buffer[i] = NULL;
        }
    }

    if (analog_mic.dma_channel > -1) {
        dma_channel_unclaim(analog_mic.dma_channel);

        analog_mic.dma_channel = -1;
    }
}

int analog_microphone_start() 
{
    // Don't set handler here - the shared handler will be set up in main.c
    // irq_set_enabled(analog_mic.dma_irq, true);
    // irq_set_exclusive_handler(analog_mic.dma_irq, analog_dma_handler);

    if (analog_mic.dma_irq == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(analog_mic.dma_channel, true);
    } else if (analog_mic.dma_irq == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(analog_mic.dma_channel, true);
#if PICO_RP2350
    } else if (analog_mic.dma_irq == DMA_IRQ_3) {
        dma_channel_set_irq3_enabled_compat(analog_mic.dma_channel, true);
#endif
    } else {
        return -1;
    }

    analog_mic.raw_buffer_write_index = 0;
    analog_mic.raw_buffer_read_index = 0;

    dma_channel_transfer_to_buffer_now(
        analog_mic.dma_channel,
        analog_mic.raw_buffer[0],
        analog_mic.buffer_size
    );

    adc_run(true); // start running the adc

    return 0;
}

void analog_microphone_stop() {
    adc_run(false); // stop running the adc

    dma_channel_abort(analog_mic.dma_channel);

    if (analog_mic.dma_irq == DMA_IRQ_0) {
        dma_channel_set_irq0_enabled(analog_mic.dma_channel, false);
    } else if (analog_mic.dma_irq == DMA_IRQ_1) {
        dma_channel_set_irq1_enabled(analog_mic.dma_channel, false);
#if PICO_RP2350
    } else if (analog_mic.dma_irq == DMA_IRQ_3) {
        dma_channel_set_irq3_enabled_compat(analog_mic.dma_channel, false);
#endif
    }

    irq_set_enabled(analog_mic.dma_irq, false);
}

// static void analog_dma_handler() 
static void __no_inline_not_in_flash_func(analog_dma_handler)(void)
{
    // clear IRQ
    if (analog_mic.dma_irq == DMA_IRQ_0) {
        dma_hw->ints0 = (1u << analog_mic.dma_channel);
    } else if (analog_mic.dma_irq == DMA_IRQ_1) {
        dma_hw->ints1 = (1u << analog_mic.dma_channel);
#if PICO_RP2350
    } else if (analog_mic.dma_irq == DMA_IRQ_3) {
        dma_hw->ints3 = (1u << analog_mic.dma_channel);
#endif
    }

    // get the next capture index to send the dma to start
    analog_mic.raw_buffer_write_index = (analog_mic.raw_buffer_write_index + 1) % ANALOG_RAW_BUFFER_COUNT;

    // give the channel a new buffer to write to and re-trigger it
    dma_channel_transfer_to_buffer_now(
        analog_mic.dma_channel,
        analog_mic.raw_buffer[analog_mic.raw_buffer_write_index],
        analog_mic.buffer_size
    );

    if (analog_mic.samples_ready_handler) {
        analog_mic.samples_ready_handler();
    }
}

void analog_microphone_set_samples_ready_handler(analog_samples_ready_handler_t handler) {
    analog_mic.samples_ready_handler = handler;
}

int analog_microphone_read(int16_t* buffer, size_t samples) {
    if (samples > analog_mic.config.sample_buffer_size) {
        samples = analog_mic.config.sample_buffer_size;
    }

    // Use local copy to avoid race with ISR
    int current_read_index = analog_mic.raw_buffer_read_index;
    int current_write_index = analog_mic.raw_buffer_write_index;
    
    if (current_write_index == current_read_index) {
        return 0;
    }

    uint16_t* in = analog_mic.raw_buffer[current_read_index];
    int16_t* out = buffer;

    // Wrap the read index properly
    analog_mic.raw_buffer_read_index = (current_read_index + 1) % ANALOG_RAW_BUFFER_COUNT;

    // for (int i = 0; i < samples; i++) {
    //     *out++ = *in++;
    // }
    memcpy(out, in, samples * sizeof(uint16_t));

    return samples;
}