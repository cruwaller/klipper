#include "autoconf.h"

#include "board/gpio.h"   // gpio_out_write
#include "board/irq.h"    // irq_disable
#include "generic/spi.h"

#include "basecmd.h"      // oid_alloc
#include "command.h"      // DECL_COMMAND
#include "sched.h"        // DECL_TASK

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
#include <stdio.h>
#endif

struct spibus_t {
    struct spi_config spi_config;
    struct gpio_out pin;
};

/* Config slave select pin and SPI bus */
void command_config_spibus_ss_pin(uint32_t *args) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("config_spibus_ss_pin oid=%u pin=%u spi_mode=%u spi_speed=%u\n",
           args[0], args[1], args[2], args[3]);
#endif
    struct spibus_t *spi =
        oid_alloc(args[0],
                  command_config_spibus_ss_pin,
                  sizeof(*spi));
    spi->pin        = gpio_out_setup(args[1], 1); // CS pin
    spi->spi_config = spi_get_config(args[2], args[3]);
}
DECL_COMMAND(command_config_spibus_ss_pin,
             "config_spibus_ss_pin oid=%c pin=%u spi_mode=%u spi_speed=%u");

void command_spibus_transfer(uint32_t *args) {
    struct spibus_t *spi =
        oid_lookup(args[0], command_config_spibus_ss_pin);
    uint8_t status = 1;
    uint8_t len = args[1];
    uint8_t *msg = (uint8_t*)(uintptr_t)args[2];

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    uint8_t iter;
    printf("spibus_transfer oid=%u len=%u\n",
           args[0], args[1]);
    printf("    sent: ");
    for (iter = 0; iter < len; iter++)
        printf("0x%02X,", msg[iter]);
    printf("\n");
#endif

    /* Read data from SPI slave */
    if (len) {
        while (!spi_set_config(spi->spi_config));
        gpio_out_write(spi->pin, 0); // Enable slave
        spi_transfer_len((char*)msg, len);
        spi_set_ready();
        gpio_out_write(spi->pin, 1); // Disable slave
        status = 0;
    }
    sendf("spibus_transfer_resp oid=%c status=%i data=%*s",
          args[0], status, len, msg);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("    status: %u\n", status);
    if (status == 0) {
        printf("    recv: ");
        for (iter = 0; iter < len; iter++)
            printf("0x%02X,", msg[iter]);
        printf("\n");
    }
#endif
}
DECL_COMMAND(command_spibus_transfer,
             "spibus_transfer oid=%c cmd=%*s");

/* Shutdown task */
void spibus_shutdown(void) {
    uint8_t oid;
    struct spibus_t *spi;
    foreach_oid(oid, spi, command_config_spibus_ss_pin) {
        gpio_out_write(spi->pin, 1); // Disable slaves
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
        printf("spibus shutdown! oid %u\n", oid);
#endif
    }
}
DECL_SHUTDOWN(spibus_shutdown);
