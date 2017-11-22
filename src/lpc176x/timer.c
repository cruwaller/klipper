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
    TIM_TIMERCFG_Type cfg;
    cfg.PrescaleOption = TIM_PRESCALE_USVAL;
    cfg.PrescaleValue  = CONFIG_CLOCK_FREQ;
    TIM_Init(LPC_TIM0, TIM_TIMER_MODE, &cfg);

    TIM_MATCHCFG_Type match;
    match.MatchChannel       = TC_CHANNEL;
    match.IntOnMatch         = ENABLE;
    match.StopOnMatch        = DISABLE;
    match.ResetOnMatch       = ENABLE;
    match.ExtMatchOutputType = TIM_EXTMATCH_NOTHING; // Don't touch any pin
    match.MatchValue         = 0x7FFFFFFF; // int32_t max
    TIM_ConfigMatch(LPC_TIM0, &match);

    NVIC_SetPriority(TIMER0_IRQn, 1);
    NVIC_ClearPendingIRQ(TIMER0_IRQn); // Clear existings
    NVIC_EnableIRQ(TIMER0_IRQn);       // Enable IRQ

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
