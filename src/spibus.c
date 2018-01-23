#include "autoconf.h"

#include "board/gpio.h"   // gpio_out_write
#include "board/irq.h"    // irq_disable
#include "generic/spi.h"

#include "basecmd.h"      // oid_alloc
#include "command.h"      // DECL_COMMAND
#include "sched.h"        // DECL_TASK

#if (CONFIG_SIMULATOR == 1)
#define SPI_READ_CMD (0x40)
#else
#define SPI_READ_CMD (0x00)
#endif

#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
#include <stdio.h>
extern int SIMULATOR_MODE;
#undef SPI_READ_CMD
#define SPI_READ_CMD (SIMULATOR_MODE ? 0x40 : 0x00)
#endif

struct spibus_t {
    SPI_t    spi_config;
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

/* write value to SPI slave */
void command_spibus_write(uint32_t *args) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("spibus_write oid=%u cmd=%u len=%u\n",
           args[0], args[1], args[2]);
#endif
    struct spibus_t *spi =
        oid_lookup(args[0], command_config_spibus_ss_pin);

    uint8_t  status = 1;
    uint8_t  cmd = args[1];
    uint16_t len = args[2];
    uint16_t iter = 0;
    uint8_t  resp[32] = {0};

    //shutdown("spibus error!");

    /* Write data to SPI slave */
    uint8_t *msg = (uint8_t*)(size_t)args[3];
    if (1 <= len) {
        spi_set_config(spi->spi_config);
        gpio_out_write(spi->pin, 0); // Enable slave
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
        printf("spibus_write - cmd: 0x%X (%d)\n", cmd, cmd);
#endif
        resp[iter++] = spi_transfer(cmd, 0);
        while (len--) {
            uint8_t const data = (uint8_t)(*msg++);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
            printf("spibus_write - config: 0x%X (%d)\n", data, data);
#endif
            resp[iter++] = spi_transfer(data, (len == 0));
        }
        gpio_out_write(spi->pin, 1); // Disable slave
        status = 0;
    }
    sendf("spibus_write_resp oid=%c status=%i data=%*s",
          args[0], status, iter, resp);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("spibus_write_resp sent [status: %u]!\n", status);
#endif
}
DECL_COMMAND(command_spibus_write,
             "spibus_write oid=%c cmd=%c cfg=%*s");

/* read from SPI slave */
void command_spibus_read(uint32_t *args) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    printf("spibus_read oid=%u cmd=%u len=%u\n",
           args[0], args[1], args[2]);
#endif
    struct spibus_t *spi =
        oid_lookup(args[0], command_config_spibus_ss_pin);

    uint8_t  status = 1;
    uint8_t  cmd = args[1];
    uint8_t  data;
    uint16_t len = args[2];
    uint16_t iter = 0;
    uint8_t  resp[32] = {0};

    /* Write data to SPI slave */
    if (1 <= len && len <= 32) {
        spi_set_config(spi->spi_config);
        gpio_out_write(spi->pin, 0); // Enable slave
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
        printf("spibus_read - cmd: 0x%X\n", cmd);
#endif
        resp[0] = spi_transfer(cmd, 0);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
            printf("spibus_read - recv: 0x%X\n", resp[0]);
#endif
        for (iter = 1; iter < len; iter++) {
            data = spi_transfer(SPI_READ_CMD, 0);
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
            printf("spibus_read - recv: 0x%X\n", data);
#endif
            resp[iter] = data;
        }
        data = spi_transfer(SPI_READ_CMD, 1); // Last part
        resp[iter++] = data;
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
        printf("spibus_read - recv: 0x%X\n", data);
#endif
        gpio_out_write(spi->pin, 1); // Disable slave
        status = 0;
    }
    sendf("spibus_read_resp oid=%c status=%i data=%*s",
          args[0], status, iter, resp);
}
DECL_COMMAND(command_spibus_read,
             "spibus_read oid=%c cmd=%c len=%hu");

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
