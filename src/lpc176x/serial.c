// sam3x8e serial port
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <string.h>     // memmove
#include "autoconf.h"   // CONFIG_SERIAL_BAUD
#include "board/gpio.h" // gpio_peripheral
#include "board/io.h"   // readl
#include "board/irq.h"  // irq_save
#include "board/misc.h" // console_sendf
#include "command.h"    // DECL_CONSTANT
#include "sched.h"      // DECL_INIT

#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>

#include <usbapi.h>


DECL_CONSTANT(SERIAL_BAUD, CONFIG_SERIAL_BAUD);


/****************************************************************
 * RX and TX FIFOs
 ****************************************************************/
#define SERIAL_BUFFER_SIZE 256
static char receive_buf[SERIAL_BUFFER_SIZE];
static uint32_t receive_pos = 0;
static char transmit_buf[SERIAL_BUFFER_SIZE];
static uint32_t transmit_pos = 0, transmit_max = 0;



/****************************************************************
 * Serial hardware
 ****************************************************************/

#define MAX_PACKET_SIZE         128 // 64
#define ASSERT(_x)

// data structure for GET_LINE_CODING / SET_LINE_CODING class requests
typedef struct {
    U32       dwDTERate;
    U8        bCharFormat;
    U8        bParityType;
    U8        bDataBits;
} TLineCoding;

static TLineCoding LineCoding = {CONFIG_SERIAL_BAUD, 0, 0, 8};
static U8 abBulkBuf[64];
static U8 abClassReqData[8];

static const U8 abDescriptors[] = {

// device descriptor
    0x12,
    DESC_DEVICE,
    LE_WORD(0x0101),            // bcdUSB
    0x02,                       // bDeviceClass
    0x00,                       // bDeviceSubClass
    0x00,                       // bDeviceProtocol
    MAX_PACKET_SIZE0,           // bMaxPacketSize
    LE_WORD(0xFFFF),            // idVendor
    LE_WORD(0x0005),            // idProduct
    LE_WORD(0x0100),            // bcdDevice
    0x01,                       // iManufacturer
    0x02,                       // iProduct
    0x03,                       // iSerialNumber
    0x01,                       // bNumConfigurations

// configuration descriptor
    0x09,
    DESC_CONFIGURATION,
    LE_WORD(67),                // wTotalLength
    0x02,                       // bNumInterfaces
    0x01,                       // bConfigurationValue
    0x00,                       // iConfiguration
    0xC0,                       // bmAttributes
    0x32,                       // bMaxPower
// control class interface
    0x09,
    DESC_INTERFACE,
    0x00,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x01,                       // bNumEndPoints
    0x02,                       // bInterfaceClass
    0x02,                       // bInterfaceSubClass
    0x01,                       // bInterfaceProtocol, linux requires value of 1 for the cdc_acm module
    0x00,                       // iInterface
// header functional descriptor
    0x05,
    CS_INTERFACE,
    0x00,
    LE_WORD(0x0110),
// call management functional descriptor
    0x05,
    CS_INTERFACE,
    0x01,
    0x01,                       // bmCapabilities = device handles call management
    0x01,                       // bDataInterface
// ACM functional descriptor
    0x04,
    CS_INTERFACE,
    0x02,
    0x02,                       // bmCapabilities
// union functional descriptor
    0x05,
    CS_INTERFACE,
    0x06,
    0x00,                       // bMasterInterface
    0x01,                       // bSlaveInterface0
// notification EP
    0x07,
    DESC_ENDPOINT,
    INT_IN_EP,                  // bEndpointAddress
    0x03,                       // bmAttributes = intr
    LE_WORD(8),                 // wMaxPacketSize
    0x0A,                       // bInterval
// data class interface descriptor
    0x09,
    DESC_INTERFACE,
    0x01,                       // bInterfaceNumber
    0x00,                       // bAlternateSetting
    0x02,                       // bNumEndPoints
    0x0A,                       // bInterfaceClass = data
    0x00,                       // bInterfaceSubClass
    0x00,                       // bInterfaceProtocol
    0x00,                       // iInterface
// data EP OUT
    0x07,
    DESC_ENDPOINT,
    BULK_OUT_EP,                // bEndpointAddress
    0x02,                       // bmAttributes = bulk
    LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
    0x00,                       // bInterval
// data EP in
    0x07,
    DESC_ENDPOINT,
    BULK_IN_EP,                 // bEndpointAddress
    0x02,                       // bmAttributes = bulk
    LE_WORD(MAX_PACKET_SIZE),   // wMaxPacketSize
    0x00,                       // bInterval

    // string descriptors
    0x04,
    DESC_STRING,
    LE_WORD(0x0409),

    0x0E,
    DESC_STRING,
    'L', 0, 'P', 0, 'C', 0, 'U', 0, 'S', 0, 'B', 0,

    0x14,
    DESC_STRING,
    'U', 0, 'S', 0, 'B', 0, 'S', 0, 'e', 0, 'r', 0, 'i', 0, 'a', 0, 'l', 0,

    0x12,
    DESC_STRING,
    'D', 0, 'E', 0, 'A', 0, 'D', 0, 'C', 0, '0', 0, 'D', 0, 'E', 0,

// terminating zero
    0
};


/**
    Local function to handle incoming bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkOut(U8 bEP, U8 bEPStatus)
{
    int i, iLen;
    bEPStatus = bEPStatus;
    if ((SERIAL_BUFFER_SIZE - receive_pos) < MAX_PACKET_SIZE) {
        // may not fit into fifo
        return;
    }

    // get data from USB into intermediate buffer
    iLen = USBHwEPRead(bEP, abBulkBuf, sizeof(abBulkBuf));
    // in case of serial overflow - ignore it as crc error will force retransmit
    for (i = 0; i < iLen && receive_pos < SERIAL_BUFFER_SIZE; i++) {
        if (abBulkBuf[i] == MESSAGE_SYNC)
            sched_wake_tasks();
        receive_buf[receive_pos++] = abBulkBuf[i];
    }
}


/**
    Local function to handle outgoing bulk data

    @param [in] bEP
    @param [in] bEPStatus
 */
static void BulkIn(U8 bEP, U8 bEPStatus)
{
    int iLen;
    bEPStatus = bEPStatus;
    if (transmit_max <= transmit_pos) {
        // no more data, disable further NAK interrupts until next USB frame
        USBHwNakIntEnable(0);
        return;
    }

    // get bytes from transmit FIFO into intermediate buffer
    for (iLen = 0;
         iLen < MAX_PACKET_SIZE && transmit_pos < transmit_max; // TODO  : onko <= vai < !!!!
         iLen++, transmit_pos++) {

        abBulkBuf[iLen] = transmit_buf[transmit_pos++];
    }
    // send over USB
    if (iLen > 0) {
        USBHwEPWrite(bEP, abBulkBuf, iLen);
    }
}


/**
    Local function to handle the USB-CDC class requests

    @param [in] pSetup
    @param [out] piLen
    @param [out] ppbData
 */
static BOOL HandleClassRequest(TSetupPacket *pSetup, int *piLen, U8 **ppbData)
{
    int i;
    switch (pSetup->bRequest) {
        // set line coding
        case SET_LINE_CODING:
            *piLen = 7;
            for (i = 0; i < 7; i++)
                ((U8 *)&LineCoding)[i] = (*ppbData)[i];
            break;

            // get line coding
        case GET_LINE_CODING:
            *ppbData = (U8 *)&LineCoding;
            *piLen = 7;
            break;

            // set control line state
        case SET_CONTROL_LINE_STATE:
            // bit0 = DTR, bit = RTS
            break;

        default:
            return FALSE;
    }
    return TRUE;
}


static void USBFrameHandler(U16 wFrame) {
    wFrame = wFrame;
    if (transmit_pos < transmit_max) {
        // data available, enable NAK interrupt on bulk in
        USBHwNakIntEnable(INACK_BI);
    }
}


/**
   Interrupt handler
*/
void USB_IRQHandler(void) {
    // Simply calls the USB ISR
    USBHwISR();
}


void init_usb_cdc(void) {
    // initialise stack
    USBInit();

    // register descriptors
    USBRegisterDescriptors(abDescriptors);

    // register class request handler
    USBRegisterRequestHandler(REQTYPE_TYPE_CLASS, HandleClassRequest, abClassReqData);

    // register endpoint handlers
    USBHwRegisterEPIntHandler(INT_IN_EP, NULL);
    USBHwRegisterEPIntHandler(BULK_IN_EP, BulkIn);
    USBHwRegisterEPIntHandler(BULK_OUT_EP, BulkOut);

    // register frame handler
    USBHwRegisterFrameHandler(USBFrameHandler);

    // enable bulk-in interrupts on NAKs
    USBHwNakIntEnable(INACK_BI);

    // CodeRed - add in interrupt setup code for RDB1768
#ifndef POLLED_USBSERIAL
    NVIC_EnableIRQ(USB_IRQn);
#endif

    // connect to bus
    USBHwConnect(TRUE);
}


void serial_init(void) {
    // Power on the USB
    //CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART0, ENABLE);
    //CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART1, ENABLE);
    //CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART2, ENABLE);
    //CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUART3, ENABLE);
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCUSB, ENABLE);

    receive_pos = 0;
    transmit_pos = transmit_max = 0;

    init_usb_cdc();
}
DECL_INIT(serial_init);


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
    if (tmax + max_size > SERIAL_BUFFER_SIZE) {
        if (tmax + max_size - tpos > SERIAL_BUFFER_SIZE)
            // Not enough space for message
            return;
        // Disable TX irq and move buffer
        writel(&transmit_max, 0);
        tpos = readl(&transmit_pos);
        tmax -= tpos;
        memmove(&transmit_buf[0], &transmit_buf[tpos], tmax);
        writel(&transmit_pos, 0);
        writel(&transmit_max, tmax);
        //enable_tx_irq();
    }

    // Generate message
    char *buf = &transmit_buf[tmax];
    uint32_t msglen = command_encodef(buf, ce, args);
    command_add_frame(buf, msglen);

    // Start message transmit
    writel(&transmit_max, tmax + msglen);
    //enable_tx_irq();
}
