// Main starting point for SAM3x8e boards.
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_main
#include "mempool.h"

#include "pins_MKS.h"

#include <system_LPC17xx.h>
#include <lpc17xx_wdt.h>
#include <lpc17xx_clkpwr.h>
#include <mpu.h>
#include <string.h>

extern void serial_init(void);


DECL_CONSTANT(MCU, "lpc176x");

struct gpio_out SBASE_LED0;
struct gpio_out SBASE_LED1;
struct gpio_out SBASE_LED2;
struct gpio_out SBASE_LED3;
struct gpio_out SBASE_LED4;

#if 0
/* Default error handler for undefined ISR vectors */
void DEF_IRQHandler(void) {
#if 0
    gpio_out_write(SBASE_LED0, 0);
    gpio_out_write(SBASE_LED1, 1);
    gpio_out_write(SBASE_LED2, 1);
    gpio_out_write(SBASE_LED3, 1);
    while(1) {}
#endif
}
void Default_Handler(void) {
    //
}


void SystemPostInit(void) {
}
#endif

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
int main(void) {
    //CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCGPIO, ENABLE);

#if 1
    // Switch off the MKS SBase leds
    SBASE_LED0 = gpio_out_setup(pin_SBASE_LED0, 0);
    SBASE_LED1 = gpio_out_setup(pin_SBASE_LED1, 1);
    SBASE_LED2 = gpio_out_setup(pin_SBASE_LED2, 0);
    SBASE_LED3 = gpio_out_setup(pin_SBASE_LED3, 1);
    SBASE_LED4 = gpio_out_setup(pin_SBASE_LED4, 0);
#endif

    serial_uart_init();
    serial_init();

    /* This is run after SystemInit and before main */
    serial_uart_put('.');
    serial_uart_put('.');
    serial_uart_put('.');
    serial_uart_put('.');
    /*while(1) {
        serial_uart_put('.');
        }*/

    //serial_uart_init();
    //serial_init();

    DEBUG_OUT("main loop\n");
    //gpio_out_write(SBASE_LED0, 1);
    sched_main();
    //gpio_out_write(SBASE_LED0, 0);
    return 0;
}

/****************************************************************
 * startup init functions
 ****************************************************************/

POOL * _AHB0 = NULL;
POOL * _AHB1 = NULL;

extern unsigned int __bss_start__;
extern unsigned int __bss_end__;
extern unsigned int __StackTop;
extern unsigned int __end__;

extern uint8_t __AHB0_block_start;
extern uint8_t __AHB0_dyn_start;
extern uint8_t __AHB0_end;
extern uint8_t __AHB1_block_start;
extern uint8_t __AHB1_dyn_start;
extern uint8_t __AHB1_end;

static void fillUnusedRAM(void);


/********* Code Read Protection ***********/
#if 0
#define NO_CRP 0xFFFFFFFF
#define NO_ISP_MAGIC 0x4E697370
#define CRP1_MAGIC 0x12345678
#define CRP2_MAGIC 0x87654321
/**** DANGER CRP3 WILL LOCK PART TO ALL READS and WRITES ****/
/*********** #define CRP3_MAGIC xxxx 0x43218765 *************/

#define CURRENT_CRP_SETTING NO_CRP

__attribute__  ((section(".crp"))) const uint32_t CRP_WORD = CURRENT_CRP_SETTING;
#endif
/******************************************/

void exit (int status);

/*void _exit(int status) {
    exit(status);
    }*/

void __libc_init_array(void);
void _start(void)
{
    size_t bssSize = (uintptr_t)&__bss_end__ - (uintptr_t)&__bss_start__;
    int mainReturnValue;

    memset(&__bss_start__, 0, bssSize);
    fillUnusedRAM();

    // zero the data sections in AHB0 and AHB1
    //memset(&__AHB0_block_start, 0, &__AHB0_dyn_start - &__AHB0_block_start);
    //memset(&__AHB1_block_start, 0, &__AHB1_dyn_start - &__AHB1_block_start);

    // MemoryPool stuff - needs to be initialised before __libc_init_array
    // so static ctors can use them
    //_AHB0 = pool_create(&__AHB0_dyn_start, ((uintptr_t)&__AHB0_end - (uintptr_t)&__AHB0_dyn_start));
    //_AHB1 = pool_create(&__AHB1_dyn_start, ((uintptr_t)&__AHB1_end - (uintptr_t)&__AHB1_dyn_start));

    //__libc_init_array();
    mainReturnValue = main();
    exit(mainReturnValue);
    exit(0);
}

static __attribute__((naked)) void fillUnusedRAM(void)
{
    __asm (
        ".syntax unified\n"
        ".thumb\n"
        // Fill 2 words (8 bytes) at a time with 0xdeadbeef.
        " ldr   r2, =__FillStart\n"
        " movw  r0, #0xbeef\n"
        " movt  r0, #0xdead\n"
        " mov   r1, r0\n"
        // Don't fill past current stack pointer value.
        " mov   r3, sp\n"
        " bics  r3, r3, #7\n"
        "1$:\n"
        " strd  r0, r1, [r2], #8\n"
        " cmp   r2, r3\n"
        " blo   1$\n"
        " bx    lr\n"
    );
}
