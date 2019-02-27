#include <stdio.h>
#include <string.h>
#include <driver/uart.h>

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include "sched.h"
#include "command.h" // MESSAGE_SYNC
#include "serial_gen.h"

#include <sdkconfig.h>

/****************************************************************
 * UART hardware
 ****************************************************************/

#if (CONFIG_SERIAL_UART0_USED)
// Default pins: RX GPIO3 , TX GPIO1
#  define EX_UART_NUM     UART_NUM_0
#  define DR_REG_BASE     (volatile uart_dev_t *)DR_REG_UART_BASE
#  define UART_TX_PIN     1
#  define UART_RX_PIN     3
#elif (CONFIG_SERIAL_UART1_USED)
// Default pins: RX GPIO9 , TX GPIO10
#  define EX_UART_NUM     UART_NUM_1
#  define DR_REG_BASE     (volatile uart_dev_t *)DR_REG_UART1_BASE
#  define UART_TX_PIN     10
#  define UART_RX_PIN     9
#elif (CONFIG_SERIAL_UART2_USED)
// Default pins: RX GPIO16 , TX GPIO17
#  define EX_UART_NUM     UART_NUM_2
#  define DR_REG_BASE     (volatile uart_dev_t *)DR_REG_UART2_BASE
#  define UART_TX_PIN     17
#  define UART_RX_PIN     16
#endif

#if (CONFIG_SERIAL_CUSTOM_USED)
// Use UART1 for custom pins
#  define EX_UART_NUM     UART_NUM_1
#  define DR_REG_BASE     (volatile uart_dev_t *)DR_REG_UART1_BASE
#  unset UART_TX_PIN
#  unset UART_RX_PIN
#  define UART_TX_PIN     CONFIG_SERIAL_CUSTOM_TXD
#  define UART_RX_PIN     CONFIG_SERIAL_CUSTOM_RXD
#  if (CONFIG_SERIAL_CUSTOM_TXD < 0 || CONFIG_SERIAL_CUSTOM_RXD < 0)
#    error "TXD or RXD pin is not set correctly!"
#  endif
#endif

#define UART_RTS_PIN    UART_PIN_NO_CHANGE
#define UART_CTS_PIN    UART_PIN_NO_CHANGE

/********************************************************************************
 *                                   PRIVATE
 ********************************************************************************/

static xSemaphoreHandle uart_lock = NULL;
static volatile uart_dev_t * uart_dev = DR_REG_BASE;

#define MUTEX_LOCK(_lock)    while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS);
#define MUTEX_UNLOCK(_lock)  xSemaphoreGive(_lock)


static void IRAM_ATTR uart_rx_isr(void *arg) {
    xQueueHandle queue = (xQueueHandle)arg;
    BaseType_t prio_task_woken;
    uint8_t c;

    uart_dev->int_clr.rxfifo_full = 1;
    uart_dev->int_clr.frm_err     = 1;
    uart_dev->int_clr.rxfifo_tout = 1;
    while (uart_dev->status.rxfifo_cnt) {
        c = uart_dev->fifo.rw_byte;
        if (!xQueueIsQueueFullFromISR(queue)) {
            xQueueSendFromISR(queue, &c, &prio_task_woken);
        }
    }
    if (prio_task_woken) portYIELD_FROM_ISR();
}

static void uartFlush(void) {
    if (uart_dev) {
        MUTEX_LOCK(uart_lock);
        while (uart_dev->status.txfifo_cnt);
        uart_dev->conf0.txfifo_rst = 1;
        uart_dev->conf0.txfifo_rst = 0;
        uart_dev->conf0.rxfifo_rst = 1;
        uart_dev->conf0.rxfifo_rst = 0;
        MUTEX_UNLOCK(uart_lock);
    }
}


/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/

void serial_send(uint8_t * buff, uint8_t count) {
    if (buff && count && uart_dev) {
        MUTEX_LOCK(uart_lock);
        while (count) {
            while (count && uart_dev->status.txfifo_cnt < 0x7F) {
                uart_dev->fifo.rw_byte = *buff++;
                count--;
            }
        }
        MUTEX_UNLOCK(uart_lock);
    }
}

#define UART_EMPTY_THRESH_DEFAULT  (10)
#define UART_FULL_THRESH_DEFAULT  (120)
#define UART_TOUT_THRESH_DEFAULT   (10)

void serial_init(QueueHandle_t rx_queue) {
    if (rx_queue == NULL || uart_dev == NULL)
        shutdown("Init failure");

    if (uart_lock == NULL) uart_lock = xSemaphoreCreateMutex();

    /* Configure parameters of an UART driver,
     * communication pins and install the driver */
    uart_config_t uart_config = {
        .baud_rate = CONFIG_SERIAL_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_TX_PIN, UART_RX_PIN,
                 UART_RTS_PIN, UART_CTS_PIN);
    static uart_isr_handle_t uart_isr_handle = NULL;
    uart_isr_register(EX_UART_NUM, uart_rx_isr, (void*)rx_queue,
                      (int)ESP_INTR_FLAG_IRAM, &uart_isr_handle);
    static intr_handle_t uart_intr_handle = NULL;
    MUTEX_LOCK(uart_lock);
    uart_dev->conf1.rxfifo_full_thrhd = 112;
    uart_dev->conf1.rx_tout_thrhd     = 2;
    uart_dev->conf1.rx_tout_en        = 1;
    uart_dev->int_ena.rxfifo_full     = 1;
    uart_dev->int_ena.frm_err         = 1;
    uart_dev->int_ena.rxfifo_tout     = 1;
    uart_dev->int_clr.val             = 0xffffffff;
    esp_intr_alloc(ETS_UART0_INTR_SOURCE, (int)ESP_INTR_FLAG_IRAM,
                   uart_rx_isr, NULL, &uart_intr_handle);
    MUTEX_UNLOCK(uart_lock);
    uartFlush();
}
