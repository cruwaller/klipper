#ifndef __GPIO_H
#define __GPIO_H

#include <stdint.h>


struct gpio_out {
    uint32_t pin;
    uint32_t mask;
    struct gpio_expander * io_exp;
};
struct gpio_out gpio_out_setup(uint8_t pin, uint8_t val);
void gpio_out_reset(struct gpio_out g, uint8_t val);
void gpio_out_toggle(struct gpio_out g);
#define gpio_out_toggle_noirq(_x) gpio_out_toggle(_x)
void gpio_out_write(struct gpio_out g, uint8_t val);

struct gpio_in {
    uint32_t pin;
    uint32_t mask;
    struct gpio_expander * io_exp;
};
struct gpio_in gpio_in_setup(uint8_t pin, int8_t pull_up);
void gpio_in_reset(struct gpio_in g, int8_t pull_up);
uint8_t gpio_in_read(struct gpio_in g);

struct gpio_adc {
    uint16_t channel;
    uint8_t adc_num;
};
struct gpio_adc gpio_adc_setup(uint8_t pin);
uint32_t gpio_adc_sample(struct gpio_adc g);
uint16_t gpio_adc_read(struct gpio_adc g);
void gpio_adc_cancel_sample(struct gpio_adc g);

struct spi_config {
    void * dev;
    uint32_t pin_reg;
    uint32_t user_reg;
    uint32_t ctrl_reg;
    uint32_t clock_reg;
};
struct spi_config spi_setup(uint32_t bus, uint8_t mode, uint32_t rate);
void spi_prepare(struct spi_config config);
void spi_transfer(struct spi_config config, uint8_t receive_data
                  , uint8_t len, uint8_t *data);

struct gpio_pwm {
    uint8_t ch;
};
struct gpio_pwm gpio_pwm_setup(uint32_t pin, uint32_t cycle_time, uint32_t val);
void gpio_pwm_write(struct gpio_pwm g, uint32_t val);

struct i2c_config {
    void *twi;
    uint8_t addr;
};

struct i2c_config i2c_setup(uint32_t bus, uint32_t rate, uint8_t addr);
void i2c_write(struct i2c_config config, uint8_t write_len, uint8_t *write);
void i2c_read(struct i2c_config config, uint8_t reg_len, uint8_t *reg
              , uint8_t read_len, uint8_t *read);

#endif // gpio.h
