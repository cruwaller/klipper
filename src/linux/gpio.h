#ifndef __LINUX_GPIO_H
#define __LINUX_GPIO_H

#include "autoconf.h"

#include <stdint.h> // uint8_t

/********************************************************************************/
#if (CONFIG_SIMULATOR == 1)
struct gpio_out {
    int fd;
    int val;
};
void gpio_out_reset(struct gpio_out g, uint8_t val);
#else // CONFIG_SIMULATOR
struct gpio_out {
    uint32_t pin;
};
#endif // CONFIG_SIMULATOR
struct gpio_out gpio_out_setup(uint8_t pin, uint8_t val);
void gpio_out_toggle_noirq(struct gpio_out g);
void gpio_out_toggle(struct gpio_out g);
void gpio_out_write(struct gpio_out g, uint8_t val);

#if (CONFIG_SIMULATOR == 1)
struct gpio_in {
    int fd;
    int val;
};
struct gpio_in gpio_in_setup(uint8_t pin, int8_t pull_up);
void gpio_in_reset(struct gpio_in g, int8_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);
#endif // CONFIG_SIMULATOR

struct gpio_adc {
    int fd;
};
struct gpio_adc gpio_adc_setup(uint8_t pin);
uint32_t gpio_adc_sample(struct gpio_adc g);
uint16_t gpio_adc_read(struct gpio_adc g);
void gpio_adc_cancel_sample(struct gpio_adc g);

struct spi_config {
    int fd;
    int rate;
};
struct spi_config spi_setup(uint32_t bus, uint8_t mode, uint32_t rate);
void spi_prepare(struct spi_config config);
void spi_transfer(struct spi_config config, uint8_t receive_data
                  , uint8_t len, uint8_t *data);

struct gpio_pwm {
    int fd;
    uint32_t period;
};
struct gpio_pwm gpio_pwm_setup(uint8_t pin, uint32_t cycle_time, uint16_t val);
void gpio_pwm_write(struct gpio_pwm g, uint16_t val);

#endif // gpio.h
