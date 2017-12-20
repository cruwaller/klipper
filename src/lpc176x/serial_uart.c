#include <string.h>     // memmove
#include "autoconf.h"   // CONFIG_SERIAL_BAUD
#include "board/gpio.h" // gpio_peripheral
#include "board/io.h"   // readl
#include "board/irq.h"  // irq_save
#include "board/misc.h" // console_sendf
#include "command.h"    // DECL_CONSTANT
#include "sched.h"      // DECL_INIT

//#include "pins_MKS.h"

#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_uart.h>
#include <cmsis_nvic.h>


// UART0
// P0_02 = TXD0, FUNC_1
// P0_03 = RXD0, FUNC_1
// UART1
// P0_15 = TXD1, FUNC_1
// P0_16 = RXD1, FUNC_1
// UART2
// P0_10 = TXD2, FUNC_1
// P0_11 = RXD2, FUNC_1
// UART3
// P0_00 = TXD3, FUNC_2
// P0_01 = RXD3, FUNC_2
// ==== Pins config ====
_gpio_peripheral_t TXD = {0, 2, PINSEL_FUNC_1};
_gpio_peripheral_t RXD = {0, 3, PINSEL_FUNC_1};

static uint32_t initdone = 0;


void calc_baudrate(uint32_t baud)
{
    uint32_t pclk, reg;

    /** Baud Rate Calculation :
        PCLKSELx registers contains the PCLK info for all the clock dependent peripherals.
        Bit6,Bit7 contains the Uart Clock(ie.UART_PCLK) information.
        The UART_PCLK and the actual Peripheral Clock(PCLK) is calculated as below.
        (Refer data sheet for more info)

        UART_PCLK    PCLK
        0x00       SystemFreq/4
        0x01       SystemFreq
        0x02       SystemFreq/2
        0x03       SystemFreq/8
    **/

    LPC_UART0->LCR |= UART_LCR_DLAB_EN;

    pclk = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
    switch( pclk )
    {
        case 0x00:
            pclk = SystemCoreClock/4;
            break;
        case 0x01:
            pclk = SystemCoreClock;
            break;
        case 0x02:
            pclk = SystemCoreClock/2;
            break;
        case 0x03:
            pclk = SystemCoreClock/8;
            break;
    }

    reg = ( pclk / (16 * baud ));

    LPC_UART0->DLL  = (reg & 0xFF);
    LPC_UART0->DLM  = ((reg >> 8) & 0xFF);
    LPC_UART0->LCR &= ~(UART_LCR_DLAB_EN);
}


void serial_uart_init(void) {
    if (initdone) return;

    // Init pins
    gpio_peripheral(&TXD, 0);
    gpio_peripheral(&RXD, 0);

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0, ENABLE);

    LPC_UART0->FCR = UART_FCR_FIFO_EN | UART_FCR_RX_RS   | UART_FCR_TX_RS;
    LPC_UART0->LCR = UART_LCR_WLEN8;

    calc_baudrate(115200);

    /*UART_CFG_Type initStruct;
    initStruct.Baud_rate = 115200;
    initStruct.Databits  = UART_DATABIT_8;
    initStruct.Parity    = UART_PARITY_NONE;
    initStruct.Stopbits  = UART_STOPBIT_1;
    UART_Init(LPC_UART0, &initStruct);*/
}

// Process any incoming commands
/*void serial_uart_task(void) {
}
DECL_TASK(serial_uart_task);*/

void serial_uart_put(char c) {
    if (initdone) {
        //UART_SendByte(LPC_UART0, (uint8_t)c);
        while( !(LPC_UART0->LSR & UART_LSR_THRE) );
        LPC_UART0->THR = c;
    }
}

void serial_uart_puts(char * str) {
    if (initdone) {
#if 0
        uint32_t iter, len = strlen(str);
        for (iter = 0; iter < len; iter++) {
            //UART_Send(LPC_UART0, (uint8_t*)str, strlen(str), /*NONE_*/BLOCKING);
            serial_uart_put(str[iter]);
        }
        serial_uart_put('\r');
        serial_uart_put('\n');
#else
        while(*str) serial_uart_put(*str++);
#endif
    }
}

uint8_t serial_uart_get(void) {
    if (initdone) {
        while( !(LPC_UART0->LSR & UART_LSR_RDR));
        return (uint8_t)LPC_UART0->RBR;
    }
    return 0;
}


void UART0_IRQHandler(void) {
    ;
}
