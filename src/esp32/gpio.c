#include "autoconf.h"
#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "gpio.h" // gpio_out_setup
#include "sched.h" // sched_shutdown
#include "board/internal.h"

#include <driver/gpio.h>
#include <driver/rtc_io.h>
#include <soc/soc.h>
#include <soc/gpio_periph.h> // GPIO_PIN_MUX_REG
#include <soc/io_mux_reg.h>
#include <soc/gpio_struct.h>


// Internal
void
__gpio_set_pull_mode(uint8_t pin, int8_t pull_mode)
{
    if (pin >= GPIO_PIN_COUNT)
        return;
    if (pull_mode < 0) {
        // pull down enabled
        gpio_pullup_dis(pin);
        gpio_pulldown_en(pin);
        //gpio_set_pull_mode(pin, GPIO_PULLDOWN_ONLY);
    } else if (pull_mode > 0) {
        // pull up enabled
        gpio_pulldown_dis(pin);
        gpio_pullup_en(pin);
        //gpio_set_pull_mode(pin, GPIO_PULLUP_ONLY);
    } else {
        // pin is floating
        gpio_pullup_dis(pin);
        gpio_pulldown_dis(pin);
        //gpio_set_pull_mode(pin, GPIO_FLOATING);
    }
}

void
__gpio_set_dir(uint8_t pin, int8_t dir)
{
    if (pin >= GPIO_PIN_COUNT)
        return;
    uint32_t mask = 0x1 << ((pin < 32) ? pin : (pin - 32));
    uint32_t io_reg = GPIO_PIN_MUX_REG[pin];
    if (dir <= 0) {
        // intput or input_output
        PIN_INPUT_ENABLE(io_reg);
    }
    if (dir >= 0) {
        // output or input_output
        GPIO.pin[pin].pad_driver = 0; // Normal gpio, 1 = open-drain
        if (pin < 32) {
            GPIO.enable_w1ts = mask;
        } else {
            GPIO.enable1_w1ts.data = mask;
        }
        gpio_matrix_out(pin, SIG_GPIO_OUT_IDX, false, false);
    } else {
        // Disable output for inputs
        GPIO.pin[pin].pad_driver = 0; // Normal gpio, 1 = open-drain
        if (pin < 32) {
            GPIO.enable_w1tc = mask;
        } else {
            GPIO.enable1_w1tc.data = mask;
        }
        // Ensure no other output signal is routed via GPIO matrix to this pin
        REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (pin * 4), SIG_GPIO_OUT_IDX);
    }
}

/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

struct gpio_out
gpio_out_setup(uint8_t pin, uint8_t val)
{
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin)) {
        if (gpio_expander_is_valid(pin))
            return gpio_expander_out_setup(pin, val);
        else
            goto out_fail;
    }
    uint32_t mask = 0x1 << ((pin < 32) ? pin : (pin - 32));
    struct gpio_out g = { .pin=pin, .mask=mask, .io_exp=NULL };
    gpio_out_reset(g, val);
    return g;
out_fail:
    shutdown("Not an output pin");
}

void
gpio_out_reset(struct gpio_out g, uint8_t val)
{
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
    if (g.io_exp) {
        gpio_expander_out_reset(g, val);
        return;
    }
#endif

    // Configure pin to input_output mode
    uint32_t io_reg = GPIO_PIN_MUX_REG[g.pin];
    if (RTC_GPIO_IS_VALID_GPIO(g.pin))
        rtc_gpio_deinit(g.pin);
    /*PIN_INPUT_ENABLE(io_reg);
    GPIO.pin[g.pin].pad_driver = 0; // Normal gpio, 1 = open-drain
    if (g.pin < 32) {
        GPIO.enable_w1ts = g.mask;
    } else {
        GPIO.enable1_w1ts.data = g.mask;
    }
    gpio_matrix_out(g.pin, SIG_GPIO_OUT_IDX, false, false);*/
    __gpio_set_dir(g.pin, 0);

    gpio_pullup_dis(g.pin);
    gpio_pulldown_dis(g.pin);
    gpio_intr_disable(g.pin);
    PIN_FUNC_SELECT(io_reg, PIN_FUNC_GPIO);
    gpio_out_write(g, val);
}

void
gpio_out_toggle(struct gpio_out g)
{
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
    if (g.io_exp) {
        gpio_expander_out_toggle(g);
        return;
    }
#endif
    uint8_t const val =
        ((g.pin < 32 ? GPIO.in : GPIO.in1.data) & g.mask);
    gpio_out_write(g, !val);
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
    if (g.io_exp) {
        gpio_expander_out_write(g, val);
        return;
    }
#endif

    if (g.pin < 32) {
        volatile uint32_t * ptr = (uint32_t*)((val) ? &GPIO.out_w1ts : &GPIO.out_w1tc);
        *ptr = g.mask;
    } else {
        volatile uint32_t * ptr = (uint32_t*)(
                (val) ? &GPIO.out1_w1ts.val : &GPIO.out1_w1tc.val);
        *ptr = g.mask;
    }
}

struct gpio_in
gpio_in_setup(uint8_t pin, int8_t pull_up)
{
    if (!GPIO_IS_VALID_GPIO(pin)) {
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
        if (gpio_expander_is_valid(pin))
            return gpio_expander_in_setup(pin, pull_up);
        else
#endif
            goto in_fail;
    }
    uint32_t mask = 0x1 << (pin < 32 ? pin : (pin - 32));
    struct gpio_in g = { .pin=pin, .mask=mask, .io_exp=NULL };
    gpio_in_reset(g, pull_up);
    return g;
in_fail:
    shutdown("Not an input pin");
}

void
gpio_in_reset(struct gpio_in g, int8_t pull_up)
{
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
    if (g.io_exp) {
        gpio_expander_in_reset(g, pull_up);
        return;
    }
#endif

    uint32_t io_reg = GPIO_PIN_MUX_REG[g.pin];
    if(RTC_GPIO_IS_VALID_GPIO(g.pin)){
        rtc_gpio_deinit(g.pin);
    }
    /*
    PIN_INPUT_ENABLE(io_reg);
    GPIO.pin[g.pin].pad_driver = 0; // Normal gpio, 1 = open-drain
    //gpio_output_disable(g.pin);
    if (g.pin < 32) {
        GPIO.enable_w1tc = g.mask;
    } else {
        GPIO.enable1_w1tc.data = g.mask;
    }
    // Ensure no other output signal is routed via GPIO matrix to this pin
    REG_WRITE(GPIO_FUNC0_OUT_SEL_CFG_REG + (g.pin * 4), SIG_GPIO_OUT_IDX);
    */
    __gpio_set_dir(g.pin, -1);
    /*
    if (pull_up) {
        gpio_pulldown_dis(g.pin);
        gpio_pullup_en(g.pin);
    } else {
        gpio_pullup_dis(g.pin);
        gpio_pulldown_en(g.pin);
    }
    */
    __gpio_set_pull_mode(g.pin, pull_up ? 1 : -1);
    gpio_intr_disable(g.pin);
    PIN_FUNC_SELECT(io_reg, PIN_FUNC_GPIO);
}

uint8_t
gpio_in_read(struct gpio_in g)
{
#if (CONFIG_KLIPPER_HAS_IO_EXPANDERS)
    if (g.io_exp)
        return gpio_expander_in_read(g);
#endif
    return !!((g.pin < 32 ? GPIO.in : GPIO.in1.data) & g.mask);
}
