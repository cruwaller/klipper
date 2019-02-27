// Copyright 2018 Evandro Luis Copercini
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Code modified to fit this purpose. 2019.01 P. Honkala
//

#include "serial_gen.h"
#include "board/internal.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#  error "BT must be enabled if used!"
#endif
#if (CONFIG_BLUEDROID_PINNED_TO_CORE != 0)
#error "BT must running on Core0"
#endif

#include "command.h" // MESSAGE_SYNC, MESSAGE_MAX
#include "sched.h" // sched_shutdown

#include <esp_bt.h>
#include <api/esp_bt_main.h>
#include <api/esp_gap_bt_api.h>
#include <api/esp_bt_device.h>
#include <api/esp_spp_api.h>
#include <freertos/event_groups.h>
#include <string.h>


/********************************************************************************
 *                                   HAL
 ********************************************************************************/
#ifdef CONFIG_CLASSIC_BT_ENABLED
#define BT_MODE ESP_BT_MODE_BTDM
#else
#define BT_MODE ESP_BT_MODE_BLE
#endif

static inline uint8_t btStarted(void) {
    return (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
}

bool btStart(void) {
    esp_bt_controller_config_t cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        return true;
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        esp_bt_controller_init(&cfg);
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {}
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        if (esp_bt_controller_enable(BT_MODE)) {
            return false;
        }
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        return true;
    }
    return false;
}

uint8_t btStop(void) {
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_IDLE) {
        return true;
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        if (esp_bt_controller_disable()) {
            return false;
        }
        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);
    }
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        return true;
    }
    return false;
}


/********************************************************************************
 *                                   PRIVATE
 ********************************************************************************/

#define SERVER_NAME   "KLIPPER_SPP_SERVER"

#define TX_QUEUE_SIZE 32

static uint32_t _spp_client;

static xQueueHandle _spp_rx_queue = NULL;
static xQueueHandle _spp_tx_queue = NULL;
static SemaphoreHandle_t _spp_tx_done = NULL; // wait until tx is done
static TaskHandle_t _spp_tx_task_handle = NULL;
static EventGroupHandle_t _spp_event_group = NULL;
// used to discard incoming connections is client already registered
static uint_fast8_t secondConnectionAttempt;

#define SPP_RUNNING     0x01
#define SPP_CONNECTED   0x02
#define SPP_CONGESTED   0x04

typedef struct spp_packet_t {
    struct spp_packet_t * next;
    size_t len;
    uint8_t data[MESSAGE_MAX];
} spp_packet_t;

spp_packet_t spp_packet_q[TX_QUEUE_SIZE];
spp_packet_t *spp_packet_q_head = NULL, *spp_packet_q_tail = NULL;
static portMUX_TYPE spp_packet_lock = portMUX_INITIALIZER_UNLOCKED;

static inline spp_packet_t* pop_spp_packet(void) {
    // pop from head
    portENTER_CRITICAL(&spp_packet_lock);
    spp_packet_t * tmp = spp_packet_q_head;
    if (tmp)
        spp_packet_q_head = tmp->next;
    portEXIT_CRITICAL(&spp_packet_lock);
    return tmp;
}

static inline void put_spp_packet(spp_packet_t* packet) {
    // add to tail
    packet->next = NULL;
    portENTER_CRITICAL(&spp_packet_lock);
    if (!spp_packet_q_head) {
        spp_packet_q_head = spp_packet_q_tail = packet;
    } else {
        spp_packet_q_tail->next = packet;
        spp_packet_q_tail = packet;
    }
    portEXIT_CRITICAL(&spp_packet_lock);
}

static inline void _spp_queue_packet(uint8_t *data, size_t len) {
    spp_packet_t * packet = pop_spp_packet();
    if (!packet) {
        return;
    }
    packet->len = len;
    memcpy(packet->data, data, len);
    if (xQueueSend(_spp_tx_queue, &packet,
#ifdef BT_WAIT_TX
                   portMAX_DELAY
#else
                   (TickType_t)0
#endif
                   ) != pdPASS) {
        free(packet);
    }
}

static bool _spp_send_buffer(uint8_t * _tx_buffer, uint16_t * const _tx_buffer_len) {
    if ((xEventGroupWaitBits(_spp_event_group, SPP_CONGESTED,
                             pdFALSE, pdTRUE, portMAX_DELAY) & SPP_CONGESTED)) {
        esp_err_t err = esp_spp_write(_spp_client, *_tx_buffer_len, _tx_buffer);
        if (err != ESP_OK) {
            return false;
        }
        *_tx_buffer_len = 0;
        if (xSemaphoreTake(_spp_tx_done, portMAX_DELAY) != pdTRUE) {
            return false;
        }
        return true;
    }
    return false;
}

static void _spp_tx_task(void * arg){
    spp_packet_t *packet = NULL;
    size_t len = 0, to_send = 0;
    uint8_t * data = NULL;

#define SPP_TX_MAX 330
    uint8_t _spp_tx_buffer[SPP_TX_MAX];
    uint16_t _spp_tx_buffer_len = 0;

    for (;;) {
        if (_spp_tx_queue &&
            xQueueReceive(_spp_tx_queue, &packet, portMAX_DELAY) == pdTRUE &&
            packet) {
            if (packet->len <= (SPP_TX_MAX - _spp_tx_buffer_len)) {
                memcpy(_spp_tx_buffer + _spp_tx_buffer_len,
                       packet->data,
                       packet->len);
                _spp_tx_buffer_len += packet->len;
                put_spp_packet(packet);
                packet = NULL;
                if (SPP_TX_MAX == _spp_tx_buffer_len ||
                    uxQueueMessagesWaiting(_spp_tx_queue) == 0) {
                    _spp_send_buffer(_spp_tx_buffer, &_spp_tx_buffer_len);
                }
            } else {
                len = packet->len;
                data = packet->data;
                to_send = SPP_TX_MAX - _spp_tx_buffer_len;
                memcpy(_spp_tx_buffer + _spp_tx_buffer_len, data, to_send);
                _spp_tx_buffer_len = SPP_TX_MAX;
                data += to_send;
                len -= to_send;
                _spp_send_buffer(_spp_tx_buffer, &_spp_tx_buffer_len);
                while (len >= SPP_TX_MAX) {
                    memcpy(_spp_tx_buffer, data, SPP_TX_MAX);
                    _spp_tx_buffer_len = SPP_TX_MAX;
                    data += SPP_TX_MAX;
                    len -= SPP_TX_MAX;
                    _spp_send_buffer(_spp_tx_buffer, &_spp_tx_buffer_len);
                }
                if (len) {
                    memcpy(_spp_tx_buffer, data, len);
                    _spp_tx_buffer_len += len;
                    put_spp_packet(packet);
                    packet = NULL;
                    if (uxQueueMessagesWaiting(_spp_tx_queue) == 0) {
                        _spp_send_buffer(_spp_tx_buffer, &_spp_tx_buffer_len);
                    }
                }
            }
        }
    }
    vTaskDelete(NULL);
    _spp_tx_task_handle = NULL;
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
    switch (event) {
        case ESP_SPP_INIT_EVT:
            //esp_bt_dev_set_device_name(CONFIG_SERIAL_BT_NAME);
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_NONE, ESP_SPP_ROLE_SLAVE, 0, SERVER_NAME);
            xEventGroupSetBits(_spp_event_group, SPP_RUNNING);
            break;
        case ESP_SPP_SRV_OPEN_EVT: //Server connection open
            if (!_spp_client) {
                _spp_client = param->open.handle;
            } else {
                secondConnectionAttempt = true;
                esp_spp_disconnect(param->open.handle);
            }
            xEventGroupSetBits(_spp_event_group, SPP_CONNECTED);
            //_spp_client = param->open.handle;
            break;
        case ESP_SPP_CLOSE_EVT: //Client connection closed
            if (secondConnectionAttempt) {
                secondConnectionAttempt = false;
            } else {
                _spp_client = 0;
            }
            xEventGroupClearBits(_spp_event_group, SPP_CONNECTED);
            break;
        case ESP_SPP_CONG_EVT: //connection congestion status changed
            if (param->cong.cong) {
                xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
            } else {
                xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
            }
            break;
        case ESP_SPP_WRITE_EVT: //write operation completed
            if (param->write.cong) {
                xEventGroupClearBits(_spp_event_group, SPP_CONGESTED);
            }
            xSemaphoreGive(_spp_tx_done); //we can try to send another packet
            break;
        case ESP_SPP_DATA_IND_EVT: { //connection received data
#if 0
            BaseType_t prio_task_woken;
            uint8_t data;
            for (int i = 0; i < param->data_ind.len; i++) {
                data = param->data_ind.data[i];
                if (_spp_rx_queue != NULL &&
                    !xQueueIsQueueFullFromISR(_spp_rx_queue)) {
                    xQueueSendFromISR(_spp_rx_queue, &data, &prio_task_woken);
                }
            }
#else
            if (_spp_rx_queue != NULL) {
                for (int i = 0; i < param->data_ind.len; i++) {
                    xQueueSend(_spp_rx_queue,
                               &param->data_ind.data[i],
                               portMAX_DELAY /*(TickType_t)0*/);
                }
            }
#endif
            break;
        }
        case ESP_SPP_DISCOVERY_COMP_EVT: //discovery complete
            break;
        case ESP_SPP_OPEN_EVT: //Client connection open
            break;
        case ESP_SPP_START_EVT: // server started
            break;
        case ESP_SPP_CL_INIT_EVT: // client initiated a connection
            break;
        default:
            break;
    }
}

#ifdef CONFIG_SERIAL_BT_PIN
void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_BT_GAP_AUTH_CMPL_EVT: {
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS) {
                // authentication ok
            }
            break;
        }
        case ESP_BT_GAP_PIN_REQ_EVT: {
            esp_bt_pin_code_t pin_code = {0};
            size_t len = sizeof(CONFIG_SERIAL_BT_PIN);
            len = len > 16 ? 16 : len;
            memcpy(pin_code, CONFIG_SERIAL_BT_PIN, len);
            esp_bt_gap_pin_reply(param->pin_req.bda, true,
                                 param->pin_req.min_16_digit ? 16 : 4,
                                 pin_code);
            break;
        }
        case ESP_BT_GAP_CFM_REQ_EVT:
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;
        case ESP_BT_GAP_KEY_NOTIF_EVT:
            break;
        case ESP_BT_GAP_KEY_REQ_EVT:
            break;

        default:
            break;
    }
    return;
}
#endif

static bool _init_bt(void) {
    /* Initialize local queue */
    uint_fast8_t iter;
    memset(spp_packet_q, 0, sizeof(spp_packet_q));
    spp_packet_q_head = spp_packet_q;
    spp_packet_t * temp = spp_packet_q;
    for (iter = 1; iter < (sizeof(spp_packet_q)/sizeof(spp_packet_t)); iter++) {
        temp = temp->next = &spp_packet_q[iter];
    }
    spp_packet_q_tail = temp;

    if (!_spp_event_group) {
        _spp_event_group = xEventGroupCreate();
        if (!_spp_event_group) {
            return false;
        }
        xEventGroupClearBits(_spp_event_group, 0xFFFFFF);
        xEventGroupSetBits(_spp_event_group, SPP_CONGESTED);
    }

    if (_spp_tx_queue == NULL) {
        _spp_tx_queue = xQueueCreate(
                TX_QUEUE_SIZE, sizeof(spp_packet_t*)); //initialize the queue
        if (_spp_tx_queue == NULL) {
            return false;
        }
    }
    if (_spp_tx_done == NULL) {
        _spp_tx_done = xSemaphoreCreateBinary();
        if (_spp_tx_done == NULL) {
            return false;
        }
        xSemaphoreTake(_spp_tx_done, 0);
    }

    if (!_spp_tx_task_handle) {
        xTaskCreatePinnedToCore(_spp_tx_task, "spp_tx", 4096,
                                NULL, TASK_PRIO_BT_TX,
                                &_spp_tx_task_handle, 0);
        if (!_spp_tx_task_handle) {
            return false;
        }
    }

    if (!btStarted() && !btStart()){
        return false;
    }

    esp_bluedroid_status_t bt_state = esp_bluedroid_get_status();
    if (bt_state == ESP_BLUEDROID_STATUS_UNINITIALIZED) {
        if (esp_bluedroid_init()) {
            return false;
        }
    }

    if (bt_state != ESP_BLUEDROID_STATUS_ENABLED) {
        if (esp_bluedroid_enable()) {
            return false;
        }
    }

#ifdef CONFIG_SERIAL_BT_PIN
    if (esp_bt_gap_register_callback(esp_bt_gap_cb) != ESP_OK) {
        return false;
    }
#endif

    if (esp_spp_register_callback(esp_spp_cb) != ESP_OK) {
        return false;
    }

    if (esp_spp_init(ESP_SPP_MODE_CB) != ESP_OK) {
        return false;
    }

    esp_bt_dev_set_device_name(CONFIG_SERIAL_BT_NAME);

    esp_bt_cod_t cod;
    cod.major = 0b00001;
    cod.minor = 0b000100;
    cod.service = 0b00000010110;
    if (esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD) != ESP_OK) {
        return false;
    }

    return true;
}

static void _stop_bt(void) {
    if (btStarted()) {
        if (_spp_client)
            esp_spp_disconnect(_spp_client);
        esp_spp_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();
        btStop();
    }
    _spp_client = 0;

    if (_spp_tx_task_handle) {
        vTaskDelete(_spp_tx_task_handle);
        _spp_tx_task_handle = NULL;
    }
    if (_spp_event_group) {
        vEventGroupDelete(_spp_event_group);
        _spp_event_group = NULL;
    }
    if (_spp_tx_queue) {
        spp_packet_t *packet = NULL;
        while (xQueueReceive(_spp_tx_queue, &packet, 0) == pdTRUE) {
            free(packet);
        }
        vQueueDelete(_spp_tx_queue);
        _spp_tx_queue = NULL;
    }
    if (_spp_tx_done) {
        vSemaphoreDelete(_spp_tx_done);
        _spp_tx_done = NULL;
    }
}


/********************************************************************************
 *                                   PUBLIC
 ********************************************************************************/

void serial_send(uint8_t * buff, uint8_t count) {
    if (!_spp_client) return; // No client connected....
    if (likely(buff) && likely(count)) {
        //esp_spp_write(_spp_client, count, buff);
        _spp_queue_packet(buff, count);
    }
}

void serial_init(QueueHandle_t rx_queue) {
    _spp_rx_queue = rx_queue;
    _stop_bt();
    if (!_init_bt())
        shutdown("BT init failure!");
}
