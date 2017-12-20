#ifndef LPC176X_PINS_H_
#define LPC176X_PINS_H_

#include "gpio.h"

typedef enum {
    pin_SBASE_LED0 = GPIO('B', 18), // P1_18
    pin_SBASE_LED1 = GPIO('B', 19), // P1_19
    pin_SBASE_LED2 = GPIO('B', 20), // P1_20
    pin_SBASE_LED3 = GPIO('B', 21), // P1_21
    pin_SBASE_LED4 = GPIO('E', 28), // P4_28
} MKS_SBASE_LED_pins;

extern struct gpio_out SBASE_LED0;
extern struct gpio_out SBASE_LED1;
extern struct gpio_out SBASE_LED2;
extern struct gpio_out SBASE_LED3;
extern struct gpio_out SBASE_LED4;

// gpio_out_write(SBASE_LED0, 0);

// #include "pins_MKS.h"

// Debug UART0
extern void serial_uart_init(void);
extern void serial_uart_put(char c);
extern void serial_uart_puts(char * str);

#define DEBUG_OUT(_x) serial_uart_puts(_x)

#endif // LPC176X_PINS_H_
