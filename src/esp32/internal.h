#ifndef INTERNAL_H
#define INTERNAL_H

#include <soc/gpio_sig_map.h>
#include <stdint.h>
#include <stdbool.h>


/********************************************************************************
 * Tasks
 ********************************************************************************/
enum {
    TASK_PRIO_IDLE   = 0, // Don't use!
    TASK_PRIO_MAIN   = 1,
    TASK_PRIO_BT_TX  = 2,
    TASK_PRIO_SERIAL = 3,
    TASK_PRIO_TIMER  = 31,
    TASK_PRIO_MAX    = 32
};

/********************************************************************************
 * Init
 ********************************************************************************/
void gpio_adc_init(void* arg);


/********************************************************************************
 * GPIO Internal
 ********************************************************************************/
void __gpio_set_pull_mode(uint8_t pin, int8_t pull_mode); // -1=pulldown, 0=float, 1=pullup
void __gpio_set_dir(uint8_t pin, int8_t dir); // -1=in, 0=in/out, 1=out

/********************************************************************************
 * GPIO Expander
 ********************************************************************************/
uint8_t gpio_expander_is_valid(uint8_t pin);

struct gpio_out gpio_expander_out_setup(uint8_t pin, uint8_t val);
void gpio_expander_out_reset(struct gpio_out g, uint8_t val);
void gpio_expander_out_toggle(struct gpio_out g);
void gpio_expander_out_write(struct gpio_out g, uint8_t val);

struct gpio_in gpio_expander_in_setup(uint8_t pin, int8_t pull_up);
void gpio_expander_in_reset(struct gpio_in g, int8_t pull_up);
uint8_t gpio_expander_in_read(struct gpio_in g);

#endif
