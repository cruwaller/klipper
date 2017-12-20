// GPIO functions on sam3x8e
//
// Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdint.h> // uint32_t
#include "autoconf.h" // CONFIG_CLOCK_FREQ
#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "gpio.h" // gpio_out_setup
#include <sam3x8e.h> // Pio
#include "sched.h" // sched_shutdown


/****************************************************************
 * Pin mappings
 ****************************************************************/

#define GPIO(PORT, NUM) (((PORT)-'A') * 32 + (NUM))
#define GPIO2PORT(PIN) ((PIN) / 32)
#define GPIO2BIT(PIN) (1<<((PIN) % 32))

static Pio * const digital_regs[] = {
    PIOA, PIOB, PIOC, PIOD
};


/****************************************************************
 * General Purpose Input Output (GPIO) pins
 ****************************************************************/

void
gpio_peripheral(char bank, uint32_t bit, char ptype, uint32_t pull_up)
{
    Pio *regs = digital_regs[bank - 'A'];
    if (ptype == 'A')
        regs->PIO_ABSR &= ~bit;
    else
        regs->PIO_ABSR |= bit;
    if (pull_up)
        regs->PIO_PUER = bit;
    else
        regs->PIO_PUDR = bit;
    regs->PIO_PDR = bit;
}


struct gpio_out
gpio_out_setup(uint8_t pin, uint8_t val)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    Pio *regs = digital_regs[GPIO2PORT(pin)];
    uint32_t bit = GPIO2BIT(pin);
    irqstatus_t flag = irq_save();
    if (val)
        regs->PIO_SODR = bit;
    else
        regs->PIO_CODR = bit;
    regs->PIO_OER = bit;
    regs->PIO_OWER = bit;
    regs->PIO_PER = bit;
    irq_restore(flag);
    return (struct gpio_out){ .regs=regs, .bit=bit };
fail:
    shutdown("Not an output pin");
}

void
gpio_out_toggle(struct gpio_out g)
{
    Pio *regs = g.regs;
    regs->PIO_ODSR ^= g.bit;
}

void
gpio_out_write(struct gpio_out g, uint8_t val)
{
    Pio *regs = g.regs;
    if (val)
        regs->PIO_SODR = g.bit;
    else
        regs->PIO_CODR = g.bit;
}


struct gpio_in
gpio_in_setup(uint8_t pin, int8_t pull_up)
{
    if (GPIO2PORT(pin) >= ARRAY_SIZE(digital_regs))
        goto fail;
    uint32_t port = GPIO2PORT(pin);
    Pio *regs = digital_regs[port];
    uint32_t bit = GPIO2BIT(pin);
    irqstatus_t flag = irq_save();
    PMC->PMC_PCER0 = 1 << (ID_PIOA + port);
    if (pull_up)
        regs->PIO_PUER = bit;
    else
        regs->PIO_PUDR = bit;
    regs->PIO_ODR = bit;
    regs->PIO_PER = bit;
    irq_restore(flag);
    return (struct gpio_in){ .regs=regs, .bit=bit };
fail:
    shutdown("Not an input pin");
}

uint8_t
gpio_in_read(struct gpio_in g)
{
    Pio *regs = g.regs;
    return !!(regs->PIO_PDSR & g.bit);
}


/****************************************************************
 * Analog to Digital Converter (ADC) pins
 ****************************************************************/
// Total 15 ADC lines
static const uint8_t adc_pins[] = {
    GPIO('A', 2), GPIO('A', 3), GPIO('A', 4), GPIO('A', 6),
    GPIO('A', 22), GPIO('A', 23), GPIO('A', 24), GPIO('A', 16),
    GPIO('B', 12), GPIO('B', 13), GPIO('B', 17), GPIO('B', 18),
    GPIO('B', 19), GPIO('B', 20) /* , GPIO('B', 21) = temp sensor */
};

#define ADC_FREQ_MAX 20000000u
//#define ADC_FREQ_MAX 1000000UUL
DECL_CONSTANT(ADC_MAX, 4095);

#define USE_ISR 0

static int32_t _adc_data[15];
void ADC_Handler(void) {     // move DMA pointers to next buffer
    // OVER_RUN STATUS : ADC->ADC_OVER --> shutdown?
    uint32_t const reg = ADC->ADC_ISR;
#if 1
    if (likely(reg & ADC_ISR_DRDY)) {
        uint32_t const status = ADC->ADC_LCDR;
        _adc_data[((status >> 12) & 0xF)] = (status & 0xFFF);
    }
#else
    if (likely(reg & ADC_ISR_EOC0))
        _adc_data[0] = ADC->ADC_CDR0;
    if (likely(reg & ADC_ISR_EOC1))
        _adc_data[1] = ADC->ADC_CDR1;
    if (likely(reg & ADC_ISR_EOC2))
        _adc_data[2] = ADC->ADC_CDR2;
    if (likely(reg & ADC_ISR_EOC3))
        _adc_data[3] = ADC->ADC_CDR3;
    if (likely(reg & ADC_ISR_EOC4))
        _adc_data[4] = ADC->ADC_CDR4;
    if (likely(reg & ADC_ISR_EOC5))
        _adc_data[5] = ADC->ADC_CDR5;
    if (likely(reg & ADC_ISR_EOC6))
        _adc_data[6] = ADC->ADC_CDR6;
    if (likely(reg & ADC_ISR_EOC7))
        _adc_data[7] = ADC->ADC_CDR7;
    if (likely(reg & ADC_ISR_EOC8))
        _adc_data[8] = ADC->ADC_CDR8;
    if (likely(reg & ADC_ISR_EOC9))
        _adc_data[9] = ADC->ADC_CDR9;
    if (likely(reg & ADC_ISR_EOC10))
        _adc_data[10] = ADC->ADC_CDR10;
    if (likely(reg & ADC_ISR_EOC11))
        _adc_data[11] = ADC->ADC_CDR11;
    if (likely(reg & ADC_ISR_EOC12))
        _adc_data[12] = ADC->ADC_CDR12;
    if (likely(reg & ADC_ISR_EOC13))
        _adc_data[13] = ADC->ADC_CDR13;
    if (likely(reg & ADC_ISR_EOC14))
        _adc_data[14] = ADC->ADC_CDR14;
#endif
}

void
gpio_adc_init(void)
{
#if (USE_ISR == 1)
    uint8_t iter;
    for (iter = 0; iter < 8; iter++) _adc_data[iter] = -1;
#endif
    // <core_cm3.h>
    /* Init ADC HW */
    NVIC_DisableIRQ(ADC_IRQn);
    NVIC_ClearPendingIRQ(ADC_IRQn);

    // Setup ADC
    PMC->PMC_PCER1 = 1 << (ID_ADC-32);
    uint32_t prescal = SystemCoreClock / (2 * ADC_FREQ_MAX) - 1;
    ADC->ADC_MR = (ADC_MR_PRESCAL(prescal)
                  | ADC_MR_STARTUP_SUT768
                  | ADC_MR_TRANSFER(1));
    ADC->ADC_ACR &= ~(1u << 4); // Disable temperature sensor

#if (USE_ISR == 1)
    /*
      ADC->ADC_IER = 1 << chan; // EOCx: End of Conversion Interrupt Enable x
      ADC->ADC_IDR = 1 << chan;
     */
    //Enable TAG
    ADC->ADC_EMR = ADC_EMR_TAG;
    // Configure ISR
    ADC->ADC_IDR |= 0xFFFFFFFF;
    //ADC->ADC_IER |= ADC_ISR_DRDY; // DRDY Data Ready Interrupt Enable
    //ADC->ADC_IER = 1 << chan; // EOCx: End of Conversion Interrupt Enable x

    ADC->ADC_MR |= ADC_MR_FREERUN_ON; // Never wait any trigger

    ADC->ADC_CR = ADC_CR_START;
    NVIC_EnableIRQ(ADC_IRQn);
#endif
}
DECL_INIT(gpio_adc_init);


struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    // Find pin in adc_pins table
    int chan;
    for (chan=0; ; chan++) {
        if (chan >= ARRAY_SIZE(adc_pins))
            shutdown("Not a valid ADC pin");
        if (adc_pins[chan] == pin)
            break;
    }
#if (USE_ISR == 1)
    ADC->ADC_IER |= 1u << chan;
    return (struct gpio_adc){ .bit = chan };
#else
    return (struct gpio_adc){ .bit = 1 << chan };
#endif
}

// Try to sample a value. Returns zero if sample ready, otherwise
// returns the number of clock ticks the caller should wait before
// retrying this function.
uint32_t
gpio_adc_sample(struct gpio_adc g)
{
#if (USE_ISR == 1)
    if (_adc_data[g.bit] < 0) goto need_delay;
#else
    uint32_t chsr = ADC->ADC_CHSR & 0xffff;
    if (!chsr) {
        // Start sample
        ADC->ADC_CHER = g.bit;
        ADC->ADC_CR = ADC_CR_START;
        goto need_delay;
    }
    if (chsr != g.bit)
        // Sampling in progress on another channel
        goto need_delay;
    if (!(ADC->ADC_ISR & ADC_ISR_DRDY))
        // Conversion still in progress
        goto need_delay;
#endif
    // Conversion ready
    return 0;
need_delay:
    //return ADC_FREQ_MAX * 1000ULL / CONFIG_CLOCK_FREQ;
    return (CONFIG_CLOCK_FREQ / (ADC_FREQ_MAX * 2)); // Half of the ADC time
}

// Read a value; use only after gpio_adc_sample() returns zero
uint16_t
gpio_adc_read(struct gpio_adc g)
{
#if (USE_ISR == 1)
    return (uint16_t)_adc_data[g.bit];
#else
    ADC->ADC_CHDR = g.bit;
    return ADC->ADC_LCDR;
#endif
}

// Cancel a sample that may have been started with gpio_adc_sample()
void
gpio_adc_cancel_sample(struct gpio_adc g)
{
#if (USE_ISR == 1)
    irqstatus_t flag = irq_save();
    _adc_data[g.bit] = -1;
    irq_restore(flag);
#else
    //irqstatus_t flag = irq_save();
    //if ((ADC->ADC_CHSR & 0xffff) == g.bit)
    //    gpio_adc_read(g);
    //irq_restore(flag);
    ADC->ADC_CHDR = g.bit;
#endif
}
