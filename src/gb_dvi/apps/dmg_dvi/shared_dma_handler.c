#include "shared_dma_handler.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "hardware/regs/intctrl.h"
#include "hardware/clocks.h"

#define MAX_DMA_CHANNELS 32
static dma_irq_callback_t dma_callbacks[MAX_DMA_CHANNELS] = { NULL };
static bool initialized_irq0 = false;
static bool initialized_irq1 = false;
#if PICO_RP2350
static bool initialized_irq3 = false;
#endif
static void __isr shared_dma_irq_handler(void);
static volatile uint32_t irq0_multi_pending = 0;
static volatile uint32_t irq1_multi_pending = 0;
#if PICO_RP2350
static volatile uint32_t irq3_multi_pending = 0;
#endif

// Can be called for IRQ0 and/or IRQ1; uses one handler for both.
void SHARED_DMA_Init(int irq_num)
{
    // Normalize the DMA IRQ (in case caller used -1 for default)
    irq_num = (irq_num >= 0) ? irq_num : DMA_IRQ_1;

    if (irq_num == DMA_IRQ_0 && !initialized_irq0) {
        irq_set_exclusive_handler(DMA_IRQ_0, shared_dma_irq_handler);
        irq_set_priority(DMA_IRQ_0, 0x00);  // Max priority
        irq_set_enabled(DMA_IRQ_0, true);
        initialized_irq0 = true;
    }
    if (irq_num == DMA_IRQ_1 && !initialized_irq1) {
        irq_set_exclusive_handler(DMA_IRQ_1, shared_dma_irq_handler);
        // Lower priority than VSYNC GPIO and DVI to reduce drift impact
        irq_set_priority(DMA_IRQ_1, 0xC0);
        irq_set_enabled(DMA_IRQ_1, true);
        initialized_irq1 = true;
    }
#if PICO_RP2350
    if (irq_num == DMA_IRQ_3 && !initialized_irq3) {
        irq_set_exclusive_handler(DMA_IRQ_3, shared_dma_irq_handler);
        irq_set_priority(DMA_IRQ_3, 0xC0);
        irq_set_enabled(DMA_IRQ_3, true);
        initialized_irq3 = true;
    }
#endif
}

bool SHARED_DMA_RegisterCallback(int dma_channel, dma_irq_callback_t callback)
{
    if (dma_channel < 0 || dma_channel >= MAX_DMA_CHANNELS) 
        return false; // Invalid channel

    dma_callbacks[dma_channel] = callback;

    return true;
}

static void __isr shared_dma_irq_handler(void)
{
    uint32_t pending0 = dma_hw->ints0;
    uint32_t pending1 = dma_hw->ints1;
#if PICO_RP2350
    uint32_t pending3 = dma_hw->ints3;
#endif

    if (__builtin_popcount(pending0) > 1)
        irq0_multi_pending++;
    if (__builtin_popcount(pending1) > 1)
        irq1_multi_pending++;
#if PICO_RP2350
    if (__builtin_popcount(pending3) > 1)
        irq3_multi_pending++;
#endif

    // Service all pending on IRQ0
    while (pending0)
    {
        int channel = __builtin_ctz(pending0);
        dma_hw->ints0 = (1u << channel);
        if (dma_callbacks[channel])
            dma_callbacks[channel]();
        pending0 &= pending0 - 1;
    }

    // Service all pending on IRQ1
    while (pending1)
    {
        int channel = __builtin_ctz(pending1);
        dma_hw->ints1 = (1u << channel);
        if (dma_callbacks[channel])
            dma_callbacks[channel]();
        pending1 &= pending1 - 1;
    }

#if PICO_RP2350
    // Service all pending on IRQ3
    while (pending3)
    {
        int channel = __builtin_ctz(pending3);
        dma_hw->ints3 = (1u << channel);
        if (dma_callbacks[channel])
            dma_callbacks[channel]();
        pending3 &= pending3 - 1;
    }
#endif    
}

void SHARED_DMA_GetStats(uint32_t* out_irq0_multi_pending, uint32_t* out_irq1_multi_pending)
{
    if (out_irq0_multi_pending) *out_irq0_multi_pending = irq0_multi_pending;
    if (out_irq1_multi_pending) *out_irq1_multi_pending = irq1_multi_pending;
}

void SHARED_DMA_ResetStats(void)
{
    irq0_multi_pending = 0;
    irq1_multi_pending = 0;
#if PICO_RP2350
    irq3_multi_pending = 0;
#endif
}