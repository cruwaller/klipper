#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <esp_spi_flash.h>
#include <esp_ipc.h>
#include <nvs_flash.h>
//#include "esp_wifi.h"
//#include "esp_bt.h" // Bluetooth
//#include "esp_bt_main.h" // Bluedroid


#include "autoconf.h"
#include <sdkconfig.h>
#include <esp_log.h>

#include <stdio.h>

#include "command.h" // DECL_CONSTANT
#include "sched.h" // sched_main
#include "serial_gen.h"
#include "board/internal.h"


DECL_CONSTANT(MCU, "esp32");

/*

  xtensa-esp32-elf-gdb out/klipper.elf
  > info symbol <addr>
  > list *<addr>

  xtensa-esp32-elf-gdb out/klipper.elf -b 115200 -ex 'target remote /dev/ttyUSB0'
  > bt

 */


/****************************************************************
 * misc functions
 ****************************************************************/

void
command_reset(uint32_t *args)
{
    (void)args;
    esp_restart();
}
DECL_COMMAND_FLAGS(command_reset, HF_IN_SHUTDOWN, "reset");

// Main entry point
void
main_task(void * arg) {
    (void)arg;
    sched_main();
    vTaskDelete(NULL);
}


void
app_main(void) {
    nvs_flash_init();

#if (CONFIG_LOG_DEFAULT_LEVEL > 0)
    //printf("Klipper init starting...\n");
#else
    esp_log_level_set("*", ESP_LOG_NONE);
#endif

    gpio_adc_init(NULL);

    // Disable radios by default
    //esp_bluedroid_disable();
    //esp_bt_controller_disable();
    //esp_wifi_stop();

    // Initialize serial connection
    extern void serial_gen_init(void);
    serial_gen_init();
    // Start main task
    xTaskCreatePinnedToCore(
            main_task, "mainTask", 8192, NULL,
            TASK_PRIO_MAIN, NULL, 1);
}
