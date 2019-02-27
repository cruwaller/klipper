#include "autoconf.h"

#include <driver/gpio.h>
#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "gpio.h" // gpio_out_setup
#include "sched.h" // sched_shutdown
#include "board/internal.h"



/********************************************************************************
 * SPI IO Expander MCP23S17
 ********************************************************************************/

// Move to config
#define EXPANDER_SPI_BUS  CONFIG_KLIPPER_IO_EXPANDER_BUS
#define EXPANDER_SPI_MODE CONFIG_KLIPPER_IO_EXPANDER_MODE
#define EXPANDER_SPI_RATE CONFIG_KLIPPER_IO_EXPANDER_SPEED

struct gpio_expander {
    struct spi_config spi_config;
    struct gpio_out pin;
    uint16_t iodir;
    uint16_t gpio;
    uint16_t gppu;
    uint8_t read_cmd;
    uint8_t write_cmd;
};

struct gpio_expander expanders[
        1
#if (CONFIG_KLIPPER_IO_EXPANDER_1_CS != -1)
        +1
#endif
#if (CONFIG_KLIPPER_IO_EXPANDER_2_CS != -1)
        +1
#endif
                              ];

#define NUM_EXPANDERS (sizeof(expanders)/sizeof(struct gpio_expander))

/***** Registers *****/
enum {
    IODIRA,     IODIRB,
    IPOLA,      IPOLB,
    GPINTENA,   GPINTENB,
    DEFVALA,    DEFVALB,
    INTCONA,    INTCONB,
    IOCONA,     IOCONB,
    GPPUA,      GPPUB,
    INTFA,      INTFB,
    INTCAPA,    INTCAPB,
    GPIOA,      GPIOB,
    OLATA,      OLATB
};

static uint16_t read_addr(struct gpio_expander *expander,
                          uint8_t addr)
{
    uint8_t msg[4] = { expander->read_cmd, addr, 0x0, 0x0 };
    gpio_out_write(expander->pin, 0);
    spi_transfer(expander->spi_config, 1, 4, msg);
    gpio_out_write(expander->pin, 1);
    uint16_t resp = msg[2];
    resp = (resp << 8) + msg[3];
    return resp;
}

static void write_addr(struct gpio_expander *expander,
                       uint8_t addr, uint16_t data)
{
    uint8_t msg[4] = { expander->write_cmd, addr,
                       (uint8_t)(data), (uint8_t)(data >> 8) };
    gpio_out_write(expander->pin, 0);
    spi_transfer(expander->spi_config, 0, 4, msg);
    gpio_out_write(expander->pin, 1);
}

#define EXPANDER_PIN_BASE 100
#define EXPANDER_GET_DEV(_pin) (((_pin) - EXPANDER_PIN_BASE) / 32)
#define EXPANDER_GET_PIN(_pin) (((_pin) - EXPANDER_PIN_BASE) & 31)


uint8_t
gpio_expander_is_valid(uint8_t pin)
{
#if (CONFIG_KLIPPER_IO_EXPANDER_0_CS == -1)
    return 0; // Not a valid config
#else
    return (EXPANDER_GET_DEV(pin) < NUM_EXPANDERS);
#endif
}

struct gpio_out
gpio_expander_out_setup(uint8_t pin, uint8_t val)
{
    struct gpio_expander *expander = &expanders[EXPANDER_GET_DEV(pin)];
    pin = 1 << EXPANDER_GET_PIN(pin);
    struct gpio_out g = { .pin=pin, .io_exp=expander };
    gpio_expander_out_reset(g, val);
    return g;
}

void
gpio_expander_out_reset(struct gpio_out g, uint8_t val)
{
    struct gpio_expander *expander = g.io_exp;
    expander->iodir &= ~(g.pin);
    write_addr(expander, IODIRA, expander->iodir);
    if (val) {
        expander->gpio |= g.pin;
    } else {
        expander->gpio &= ~g.pin;
    }
    write_addr(expander, GPIOA, expander->gpio);
}

void
gpio_expander_out_toggle(struct gpio_out g)
{
    struct gpio_expander *expander = g.io_exp;
    if (expander->gpio & g.pin) // If set
        expander->gpio &= ~g.pin; // Clear
    else
        expander->gpio |= g.pin; // Set
    write_addr(expander, GPIOA, expander->gpio);
}

void
gpio_expander_out_write(struct gpio_out g, uint8_t val)
{
    struct gpio_expander *expander = g.io_exp;
    if (val)
        expander->gpio |= g.pin; // Set
    else
        expander->gpio &= ~g.pin; // Clear
    write_addr(expander, GPIOA, expander->gpio);
}


struct gpio_in
gpio_expander_in_setup(uint8_t pin, int8_t pull_up)
{
    struct gpio_expander *expander = &expanders[EXPANDER_GET_DEV(pin)];
    pin = EXPANDER_GET_PIN(pin);
    struct gpio_in g = { .pin=pin, .io_exp=expander };
    gpio_expander_in_reset(g, pull_up);
    return g;
}

void
gpio_expander_in_reset(struct gpio_in g, int8_t pull_up)
{
    struct gpio_expander *expander = g.io_exp;
    expander->iodir |= 1 << g.pin;
    write_addr(expander, IODIRA, expander->iodir);
    if (pull_up) {
        expander->gppu |= 1 << g.pin;
        write_addr(expander, GPPUA, expander->gppu);
    }
}

uint8_t
gpio_expander_in_read(struct gpio_in g)
{
    return !!(read_addr(g.io_exp, GPIOA) & 1 << g.pin);
}


void gpio_io_expander_init(void* arg) {
    (void)arg;
#if (CONFIG_KLIPPER_IO_EXPANDER_0_CS == -1)
    return; // Not a valid config
#else // CONFIG_KLIPPER_IO_EXPANDER_0_CS
    uint8_t iter;
    uint8_t cs_pins[] = {
        CONFIG_KLIPPER_IO_EXPANDER_0_CS
#if (CONFIG_KLIPPER_IO_EXPANDER_1_CS != -1)
        ,CONFIG_KLIPPER_IO_EXPANDER_1_CS
#endif
#if (CONFIG_KLIPPER_IO_EXPANDER_2_CS != -1)
        ,CONFIG_KLIPPER_IO_EXPANDER_2_CS
#endif
    };
    for (iter = 0; iter < NUM_EXPANDERS; iter++) {
        struct gpio_expander *expander = &expanders[iter];
        expander->pin = gpio_out_setup(cs_pins[iter], 1);
        expander->spi_config = spi_setup(
                EXPANDER_SPI_BUS, EXPANDER_SPI_MODE,
                EXPANDER_SPI_RATE);
        expander->write_cmd = 0b01000000 | ((iter && 0b111) << 1);
        expander->read_cmd = expander->write_cmd | 1<<0;
        // Disable address pins
        write_addr(expander, IOCONA,
                   (read_addr(expander, IOCONA) & ~(0b1000))); // Clear HAEN
        // Make all input and low by default
        expander->iodir = 0xFFFF;
        expander->gpio = 0;
        expander->gppu = 0;
        write_addr(expander, IODIRA, 0xFFFF);
        write_addr(expander, GPIOA, 0);
        write_addr(expander, GPPUA, 0);
    }
#endif // CONFIG_KLIPPER_IO_EXPANDER_0_CS
}
DECL_INIT(gpio_io_expander_init);
