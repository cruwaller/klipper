// Read analog values from Linux IIO device
//
// Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.
#include "autoconf.h"

#include <fcntl.h> // open
#include <stdio.h> // snprintf
#include <stdlib.h> // atoi
#include <unistd.h> // read
#include "command.h" // shutdown
#include "gpio.h" // gpio_adc_setup
#include "internal.h" // report_errno
#include "sched.h" // sched_shutdown

#if (CONFIG_SIMULATOR == 1)
#include <stdio.h>
extern int SIMULATOR_MODE;
#endif


DECL_CONSTANT(ADC_MAX, CONFIG_ADC_MAX_VALUE); // Assume 12bit adc

#define IIO_PATH "/sys/bus/iio/devices/iio:device0/in_voltage%d_raw"

struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    int fd;
#if (CONFIG_SIMULATOR == 1)
    if (SIMULATOR_MODE == 0)
#endif
    {
        char fname[256];
        snprintf(fname, sizeof(fname), IIO_PATH, pin);

        fd = open(fname, O_RDONLY|O_CLOEXEC);
        if (fd < 0) {
            report_errno("analog open", fd);
            goto fail;
        }
        int ret = set_non_blocking(fd);
        if (ret < 0)
            goto fail;
#if (CONFIG_SIMULATOR == 1)
    } else {
        fd = pin;
#endif
    }
    return (struct gpio_adc){ .fd = fd };
fail:
    if (fd >= 0
#if (CONFIG_SIMULATOR == 1)
       && SIMULATOR_MODE == 0
#endif
       )
        close(fd);
    shutdown("Unable to open adc device");
}

uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    (void)g;
    return 0;
}

uint16_t
gpio_adc_read(struct gpio_adc g)
{
#if (CONFIG_SIMULATOR == 1)
    if (SIMULATOR_MODE == 0)
#endif
    {
        char buf[64];
        int ret = pread(g.fd, buf, sizeof(buf)-1, 0);
        if (ret <= 0) {
            report_errno("analog read", ret);
            try_shutdown("Error on analog read");
            return 0;
        }
        buf[ret] = '\0';
        return atoi(buf);
#if (CONFIG_SIMULATOR == 1)
    } else {
        (void)g;
        return (uint16_t)(0.94f * CONFIG_ADC_MAX_VALUE);
#endif
    }
}

void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    (void)g;
}


/********************************************************************************/

struct gpio_out gpio_out_setup(uint8_t pin, uint8_t val) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_out_setup: pin %d value %u\n", pin, val);
#endif
    return (struct gpio_out){.fd = pin, .val = val};
}
void gpio_out_toggle_noirq(struct gpio_out g) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_out_toggle_noirq: pin %d\n", g.fd);
#endif
    (void)g;
}
void gpio_out_toggle(struct gpio_out g) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_out_toggle: pin %d\n", g.fd);
#endif
    (void)g;
}
void gpio_out_write(struct gpio_out g, uint8_t val) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_out_write: pin %d value %u\n", g.fd, val);
#endif
    (void)g;
    (void)val;
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
}

/********************************************************************************/

struct gpio_in gpio_in_setup(uint8_t pin, int8_t pull_up) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_in_setup: pin %d pull_up %u\n", pin, pull_up);
#endif
    return (struct gpio_in){.fd = pin, .val = !pull_up};
}
uint8_t gpio_in_read(struct gpio_in g) {
#if (CONFIG_SIMULATOR == 1 && CONFIG_MACH_LINUX == 1)
    //printf("gpio_in_read: pin %d value %u\n", g.fd, g.val);
#endif
    return g.val; //(g.fd & 1);
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
}

/********************************************************************************/

struct gpio_pwm gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint8_t val) {
    (void)cycle_time;
    return (struct gpio_pwm){.fd = pin, .val = val};
}
void gpio_pwm_write(struct gpio_pwm g, uint8_t val) {
    (void)g;
    (void)val;
}

/********************************************************************************/
