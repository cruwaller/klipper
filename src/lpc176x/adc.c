#include "autoconf.h"
#include "board/irq.h" // irq_save
#include "command.h"   // shutdown
#include "sched.h"     // shutdown
#include "compiler.h"  // ARRAY_SIZE
#include "gpio.h"

#include <stdint.h>
#include <LPC17xx.h>
#include <lpc17xx_adc.h>
#include <lpc17xx_pinsel.h>
#include <lpc17xx_clkpwr.h>


/****************************************************************
 * Analog to Digital Converter (ADC) pins
 ****************************************************************/

/*
 * Analog Pins
 */
static const _gpio_peripheral_t adc_pins[8] = {
  // ADC0...7
  { 0, 23, PINSEL_FUNC_1 }, // ADC0
  { 0, 24, PINSEL_FUNC_1 }, // ADC1
  { 0, 25, PINSEL_FUNC_1 }, // ADC2
  { 0, 26, PINSEL_FUNC_1 }, // ADC3
  { 1, 30, PINSEL_FUNC_3 }, // ADC4
  { 1, 31, PINSEL_FUNC_3 }, // ADC5
  { 0,  3, PINSEL_FUNC_2 }, // ADC6
  { 0,  2, PINSEL_FUNC_2 }  // ADC7
};

#define ADC_FREQ_MAX 2000000 // 2MHz (should be less than or equal to 13MHz)
DECL_CONSTANT(ADC_MAX, 4095);

void
gpio_adc_init(void) {
    /* Init ADC HW */
    uint32_t rate, reg;

    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCAD, ENABLE);
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_ADC, CLKPWR_PCLKSEL_CCLK_DIV_2);
    LPC_ADC->ADCR = 0;
    //Enable PDN bit
    reg = ADC_CR_PDN;

    // Set clock frequency
    rate = (SystemCoreClock / (2 * ADC_FREQ_MAX)) - 1;
    reg |=  ADC_CR_CLKDIV(rate);

    LPC_ADC->ADCR = reg;
}
DECL_INIT(gpio_adc_init);

struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    uint8_t const in_port = GPIO2PORT(pin);
    uint8_t const in_pin  = GPIO2PIN(pin);
    // Find pin in adc_pins table

    int chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan].port == in_port &&
            adc_pins[chan].pin == in_pin)
            break;
    }
    gpio_peripheral(&adc_pins[chan], 0);
    return (struct gpio_adc){ .channel = chan };
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    uint32_t const chsr = LPC_ADC->ADGDR; // read global status reg
    if (! (LPC_ADC->ADCR & ADC_CR_START_MASK)) {
        // Start sample
        //LPC_ADC->ADCR &= ~ADC_CR_START_MASK;
        LPC_ADC->ADCR |= ADC_CR_CH_SEL(g.channel);
        LPC_ADC->ADCR |= ADC_CR_START_MODE_SEL((uint32_t)ADC_START_NOW);
        goto need_delay;
    }
    if (ADC_GDR_CH(chsr) != g.channel)
        // Sampling in progress on another channel
        goto need_delay;
    if (! (chsr & ADC_GDR_DONE_FLAG))
        // Conversion still in progress
        goto need_delay;
    // Conversion ready
    return 0;
need_delay:
    return (CONFIG_CLOCK_FREQ / (ADC_FREQ_MAX * 5)); // 5th of the ADC time
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
    gpio_adc_cancel_sample(g);
    return ADC_GDR_RESULT(LPC_ADC->ADGDR);
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    irqstatus_t flag = irq_save();
    //need to stop START bits before disable channel
    LPC_ADC->ADCR &= ~ADC_CR_START_MASK;
    LPC_ADC->ADCR &= ~ADC_CR_CH_SEL(g.channel);
    irq_restore(flag);
}
