#ifndef SERIAL_GEN_H_
#define SERIAL_GEN_H_

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/ringbuf.h>
#include <stdint.h>

#include "autoconf.h" // CONFIG_SERIAL_BAUD
#include <sdkconfig.h>

void serial_init(QueueHandle_t rx_queue);
void serial_send(uint8_t * buff, uint8_t count);

#endif /* SERIAL_GEN_H_ */
