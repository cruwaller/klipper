// SAM3x8e timer interrupt scheduling
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "autoconf.h"
#include "board/irq.h" // irq_disable
#include "board/misc.h" // timer_read_time
#include "board/timer_irq.h" // timer_dispatch_many
#include "command.h" // DECL_SHUTDOWN
#include "sched.h" // DECL_INIT

#include <LPC17xx.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_clkpwr.h>

#define TC_CHANNEL 0

static inline void tc_clear_irq(uint32_t const timer) {
    LPC_TIM0->IR |= (1 << timer); // read to clear irq pending
}

// Set the next irq time
static void
timer_set(uint32_t value)
{
#if (TC_CHANNEL == 0)
    LPC_TIM0->MR0 = value;
#elif (TC_CHANNEL == 1)
    LPC_TIM0->MR1 = value;
#elif (TC_CHANNEL == 2)
    LPC_TIM0->MR2 = value;
#elif (TC_CHANNEL == 3)
    LPC_TIM0->MR3 = value;
#endif
}

// Return the current time (in absolute clock ticks).
uint32_t
timer_read_time(void)
{
#if (TC_CHANNEL == 0)
    return LPC_TIM0->MR0;
#elif (TC_CHANNEL == 1)
    return LPC_TIM0->MR1;
#elif (TC_CHANNEL == 2)
    return LPC_TIM0->MR2;
#elif (TC_CHANNEL == 3)
    return LPC_TIM0->MR3;
#endif
}

// Activate timer dispatch as soon as possible
void
timer_kick(void)
{
    timer_set(timer_read_time() + 50);
    tc_clear_irq(TC_CHANNEL);
}

void
timer_init(void)
{
    /* Enable clocks just in case */
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCTIM0, ENABLE);
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_TIMER0, CLKPWR_PCLKSEL_CCLK_DIV_1);

    LPC_TIM0->TCR |=  (1 << 1); // Set timer to reset

    // Count and Capture Control Register
    LPC_TIM0->CTCR = 0;
    LPC_TIM0->CCR  = 0; // = timer mode

    /********************************
      PR = 4
      FCPU / ( CCLK_DIV_1 (1) * PR ) -->
      LPC1768 -> 100MHz / 4 = 25MHz
      LPC1769 -> 120MHz / 4 = 30MHz
    *********************************/
    LPC_TIM0->PR   = 3;
    LPC_TIM0->TC   = 0;
    LPC_TIM0->PC   = 0;
    LPC_TIM0->MR0  = 0x7FFFFFFF;
    LPC_TIM0->MR1  = 0x7FFFFFFF;
    LPC_TIM0->MR2  = 0x7FFFFFFF;
    LPC_TIM0->MR3  = 0x7FFFFFFF;

    // ISR on match + Clear TC
    LPC_TIM0->MCR  = (0b011 << (TC_CHANNEL * 3));

    // No external actions
    LPC_TIM0->EMR  = 0;

    // Clear pending interrupts
    LPC_TIM0->IR = 0xFFFFFFFF;

    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_ClearPendingIRQ(TIMER0_IRQn); // Clear existings
    NVIC_EnableIRQ(TIMER0_IRQn);       // Enable IRQ

    LPC_TIM0->TCR |=  (1);      // Make sure the counter is enabled
    LPC_TIM0->TCR &= ~(1 << 1); // Release reset
    timer_kick();
}
DECL_INIT(timer_init);

// IRQ handler
void __visible __aligned(16) // aligning helps stabilize perf benchmarks
TC0_Handler(void)
{
    irq_disable();
    uint32_t status = LPC_TIM0->IR; // Read pending IRQ
    tc_clear_irq(TC_CHANNEL);
    if (likely(status & (1 << TC_CHANNEL))) {
        uint32_t next = timer_dispatch_many();
        timer_set(next);
    }
    irq_enable();
}
