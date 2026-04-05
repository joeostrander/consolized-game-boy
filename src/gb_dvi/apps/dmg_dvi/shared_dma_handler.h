// shared_dma_handler.h
#ifndef SHARED_DMA_HANDLER_H
#define SHARED_DMA_HANDLER_H

#include <stddef.h>
#include "pico/types.h"

typedef void (*dma_irq_callback_t)(void);

void SHARED_DMA_Init(int irq_num);
bool SHARED_DMA_RegisterCallback(int dma_channel, dma_irq_callback_t callback);

// Debug helpers to observe IRQ contention (counts times multiple DMA channels were pending per IRQ).
void SHARED_DMA_GetStats(uint32_t* irq0_multi_pending, uint32_t* irq1_multi_pending);
void SHARED_DMA_ResetStats(void);

#endif  // SHARED_DMA_HANDLER_H