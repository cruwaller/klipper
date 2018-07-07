// Communicating with an SPI device via linux spidev
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include "autoconf.h"

#include <fcntl.h> // open
#include <stdio.h> // snprintf
#include <unistd.h> // write
#include "command.h" // DECL_COMMAND
#include "gpio.h" // spi_setup
#include "internal.h" // report_errno
#include "sched.h" // shutdown

struct spi_s {
    uint32_t bus, dev;
    int fd;
};
static struct spi_s devices[16];
static uint32_t devices_count = 0;

static int
spi_open(uint32_t bus, uint32_t dev)
{
    // Find existing device (if already opened)
    uint32_t i;
    for (i=0; i<devices_count; i++)
        if (devices[i].bus == bus && devices[i].dev == dev)
            return devices[i].fd;

    // Setup new SPI device
    if (devices_count >= ARRAY_SIZE(devices))
        shutdown("Too many spi devices");
    char fname[256];
    snprintf(fname, sizeof(fname), "/dev/spidev%d.%d", bus, dev);
    int fd = open(fname, O_RDWR|O_CLOEXEC);
    if (fd < 0) {
        report_errno("open spi", fd);
        shutdown("Unable to open spi device");
    }
    int ret = set_non_blocking(fd);
    if (ret < 0)
        shutdown("Unable to set non-blocking on spi device");

    devices[devices_count].bus = bus;
    devices[devices_count].dev = dev;
    devices[devices_count].fd = fd;
    return fd;
}

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    (void)mode, (void)rate;
#if (!CONFIG_SIMULATOR)
    int bus_id = (bus >> 8) & 0xff, dev_id = bus & 0xff;
    int fd = spi_open(bus_id, dev_id);
    return (struct spi_config) { fd };
#else
    (void)bus;
    return (struct spi_config) { 0 };
#endif
}

void
spi_prepare(struct spi_config config)
{
}

void
spi_transfer(struct spi_config config, uint8_t receive_data
             , uint8_t len, uint8_t *data)
{
    (void)receive_data;
#if (!CONFIG_SIMULATOR)
    int ret = write(config.fd, data, len);
    if (ret < 0) {
        report_errno("write spi", ret);
        shutdown("Unable to write to spi");
    }
#else
    (void)config, (void)len, (void)data;
#endif
}
