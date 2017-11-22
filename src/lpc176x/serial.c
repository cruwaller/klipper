// sam3x8e serial port
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h> // memmove
#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "board/gpio.h" // gpio_peripheral
#include "board/io.h" // readl
#include "board/irq.h" // irq_save
#include "board/misc.h" // console_sendf
#include "command.h" // DECL_CONSTANT
#include "sched.h" // DECL_INIT

#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_uart.h>


#define SERIAL_BUFFER_SIZE 96
static char receive_buf[SERIAL_BUFFER_SIZE];
static uint32_t receive_pos;
static char transmit_buf[SERIAL_BUFFER_SIZE];
static uint32_t transmit_pos, transmit_max;


/*
 * IMPLEMENT USB SERIAL!!!!!!
 */



/****************************************************************
 * Serial hardware
 ****************************************************************/

DECL_CONSTANT(SERIAL_BAUD, CONFIG_SERIAL_BAUD);

void
serial_init(void)
{

}
DECL_INIT(serial_init);

void __visible
UART0_IRQHandler(void)
{
    //uint32_t status = *LPC_UART0;
    uint32_t status = 0;
    if (status & /*UART_SR_RXRDY*/1) {
        //uint8_t data = UART->UART_RHR;
        uint8_t data = 0;
        if (data == MESSAGE_SYNC)
            sched_wake_tasks();
        if (receive_pos >= sizeof(receive_buf))
            // Serial overflow - ignore it as crc error will force retransmit
            return;
        receive_buf[receive_pos++] = data;
        return;
    }
    if (status & /*UART_SR_TXRDY*/2) {
        /*
        if (transmit_pos >= transmit_max)
            UART->UART_IDR = UART_IDR_TXRDY;
        else
            UART->UART_THR = transmit_buf[transmit_pos++];
        */
    }
}

// Enable tx interrupts
static void
enable_tx_irq(void)
{
}


/****************************************************************
 * Console access functions
 ****************************************************************/

// Remove from the receive buffer the given number of bytes
static void
console_pop_input(uint32_t len)
{
    uint32_t copied = 0;
    for (;;) {
        uint32_t rpos = readl(&receive_pos);
        uint32_t needcopy = rpos - len;
        if (needcopy) {
            memmove(&receive_buf[copied], &receive_buf[copied + len]
                    , needcopy - copied);
            copied = needcopy;
            sched_wake_tasks();
        }
        irqstatus_t flag = irq_save();
        if (rpos != readl(&receive_pos)) {
            // Raced with irq handler - retry
            irq_restore(flag);
            continue;
        }
        receive_pos = needcopy;
        irq_restore(flag);
        break;
    }
}

// Process any incoming commands
void
console_task(void)
{
    uint8_t pop_count;
    uint32_t rpos = readl(&receive_pos);
    int8_t ret = command_find_block(receive_buf, rpos, &pop_count);
    if (ret > 0)
        command_dispatch(receive_buf, pop_count);
    if (ret)
        console_pop_input(pop_count);
}
DECL_TASK(console_task);

// Encode and transmit a "response" message
void
console_sendf(const struct command_encoder *ce, va_list args)
{
    // Verify space for message
    uint32_t tpos = readl(&transmit_pos), tmax = readl(&transmit_max);
    if (tpos >= tmax) {
        tpos = tmax = 0;
        writel(&transmit_max, 0);
        writel(&transmit_pos, 0);
    }
    uint32_t max_size = ce->max_size;
    if (tmax + max_size > sizeof(transmit_buf)) {
        if (tmax + max_size - tpos > sizeof(transmit_buf))
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writel(&transmit_max, 0);
        tpos = readl(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writel(&transmit_pos, 0);
        writel(&transmit_max, tmax);
        enable_tx_irq();
    }

    // Generate message
    char *buf = &transmit_buf[tmax];
    uint32_t msglen = command_encodef(buf, ce, args);
    command_add_frame(buf, msglen);

    // Start message transmit
    writel(&transmit_max, tmax + msglen);
    enable_tx_irq();
}
