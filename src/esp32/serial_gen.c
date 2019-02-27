#include <string.h> // memmove
#include "serial_gen.h"
#include "board/misc.h" // console_sendf
#include "command.h" // DECL_CONSTANT, MESSAGE_MAX
#include "sched.h" // DECL_INIT
#include "board/io.h" // readl
#include "board/internal.h"


#define RX_SERIAL_BUFFER_SIZE 512


DECL_CONSTANT(SERIAL_BAUD, CONFIG_SERIAL_BAUD);
DECL_CONSTANT(RECEIVE_WINDOW, RX_SERIAL_BUFFER_SIZE);


//static portMUX_TYPE lock_tx = portMUX_INITIALIZER_UNLOCKED;


/********************************************************************************
 *                                   PRIVATE
 ********************************************************************************/

// Remove from the receive buffer the given number of bytes
void console_pop_input(uint32_t   const len,
                       uint32_t * const rpos,
                       uint8_t  * const rbuf)
{
    uint32_t const needcopy = *rpos - len;
    if (needcopy) {
        memmove(&rbuf[0], &rbuf[len], needcopy);
    }
    *rpos = needcopy;
}

// Process any incoming commands
static void console_task(void * arg) {
    extern void wd_reset(void);
    (void)arg;
    static uint8_t receive_buf[RX_SERIAL_BUFFER_SIZE];
    QueueHandle_t command_queue =
        xQueueCreate(RX_SERIAL_BUFFER_SIZE, sizeof(char));
    uint_fast32_t receive_pos = 10;
    //portTickType timeout = 100 / portTICK_RATE_MS;
    portTickType timeout = portMAX_DELAY;
    uint_fast8_t pop_count;
    int_fast8_t ret = 0;
    char rx_c;
    serial_init(command_queue);
    for (;;) {
        do {
            /* Check received command */
            ret = command_find_block(receive_buf, receive_pos, &pop_count);
            if (ret > 0) {
                command_dispatch(receive_buf, pop_count);
                wd_reset();
            }
            if (ret) {
                console_pop_input(pop_count, &receive_pos, receive_buf);
                if (ret > 0)
                    command_send_ack();
            }
        } while(ret && receive_pos > 0);
        if (xQueueReceive(command_queue, (void*)&rx_c, timeout) == pdTRUE) {
            receive_buf[receive_pos++] = rx_c;
        }
    }
    /* Delete queue */
    while(xQueueReceive(command_queue, &rx_c, 0));
    vQueueDelete(command_queue);
    command_queue = NULL;
    /* Delete task context */
    vTaskDelete(NULL);
}

/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/
// TODO : Add separate TX queue

// Encode and transmit a "response" message
void console_sendf(const struct command_encoder *ce, va_list args) {
    //portENTER_CRITICAL(&lock_tx); // Make sure the one core can send at the time
    uint8_t tx_buff[MESSAGE_MAX];
    uint_fast32_t msglen = command_encodef(tx_buff, ce, args);
    command_add_frame(tx_buff, msglen);
    serial_send(tx_buff, msglen);
    //portEXIT_CRITICAL(&lock_tx);
}

void serial_gen_init(void) {
    xTaskCreatePinnedToCore(
            console_task, "console_task", 4096, NULL,
            TASK_PRIO_SERIAL, NULL, 0);
}
