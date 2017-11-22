// Main starting point for SAM3x8e boards.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_main

#include <system_LPC17xx.h>
#include <lpc17xx_wdt.h>


DECL_CONSTANT(MCU, "lpc176x");


void SystemPostInit(void) {
    /* This is run after SystemInit and before main */
}


/****************************************************************
 * watchdog handler
 ****************************************************************/

void
watchdog_reset(void)
{
    WDT_Feed();
}
DECL_TASK(watchdog_reset);

void
watchdog_init(void)
{
    uint32_t const timeout = 500 * 1000;  // 500ms timeout
    WDT_Init(WDT_CLKSRC_IRC, WDT_MODE_RESET);
    WDT_Start(timeout);
}
DECL_INIT(watchdog_init);


/****************************************************************
 * misc functions
 ****************************************************************/

void
command_reset(uint32_t *args)
{
    NVIC_SystemReset();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

// Main entry point
int
main(void)
{
    sched_main();
    return 0;
}
