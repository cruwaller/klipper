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

#include "pins_MKS.h"

#include <LPC17xx.h>
#include <lpc17xx_timer.h>
#include <lpc17xx_clkpwr.h>
#include <cmsis_nvic.h>

#define TC_CHANNEL 0

void timer_isr(void);

static __attribute__((always_inline)) inline void
tc_clear_irq(uint32_t const timer) {
//    LPC_TIM0->IR |= (1 << timer); // read to clear irq pending
    LPC_TIM0->IR = 0xFFFFFFFF;
    (void)timer;
}

// Set the next irq time
static void
timer_set(uint32_t const value) {
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
timer_read_time(void) {
    return LPC_TIM0->TC; // Return current timer counter value
}

// Activate timer dispatch as soon as possible
void
timer_kick(void) {
    /*
      LPC1768 : 50  = 2.00us
      LPC1769 : 50 ~= 1.67us
     */
    //timer_set(LPC_TIM0->TC + 50);  // + ~2us  << cause reschedule failure
    //timer_set(LPC_TIM0->TC + 200); // + ~8us  << cause reschedule failure
    //timer_set(LPC_TIM0->TC + 400); // + ~16us << cause reschedule failure
    timer_set(LPC_TIM0->TC + 1000); // TODO FIXME : Need to be fine tuned!!
    tc_clear_irq(TC_CHANNEL);
}

void
timer_init(void)
{
    NVIC_DisableIRQ(TIMER0_IRQn);

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
    //LPC_TIM0->MCR  = (0b011 << (TC_CHANNEL * 3));
    // ISR on match
    LPC_TIM0->MCR  = (0b001 << (TC_CHANNEL * 3));

    // No external actions
    LPC_TIM0->EMR  = 0;

    // Clear pending interrupts
    LPC_TIM0->IR = 0xFFFFFFFF;

    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_ClearPendingIRQ(TIMER0_IRQn); // Clear existings
    //NVIC_SetVector(TIMER0_IRQn, (uint32_t)&timer_isr);

    LPC_TIM0->TCR |=  (1);       // Make sure the counter is enabled

    timer_kick();

    // Start timer
    LPC_TIM0->TCR &= ~(1 << 1);  // Release reset
    NVIC_EnableIRQ(TIMER0_IRQn); // Enable IRQ
}
DECL_INIT(timer_init);

// IRQ handler
//void __visible __aligned(16) // aligning helps stabilize perf benchmarks
void TIMER0_IRQHandler(void)
//void timer_isr(void)
{
    irq_disable();
    uint32_t const status = LPC_TIM0->IR; // Read pending IRQ
    tc_clear_irq(TC_CHANNEL);
    if (likely(status & (1 << TC_CHANNEL))) {
        uint32_t const next = timer_dispatch_many();
        timer_set(next);
    }
    irq_enable();
}
