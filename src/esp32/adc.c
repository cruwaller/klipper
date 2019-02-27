#include "autoconf.h"

#include "board/internal.h"
#include "board/irq.h" // irq_save
#include "command.h" // shutdown
#include "compiler.h" // ARRAY_SIZE
#include "gpio.h" // gpio_out_setup
#include "sched.h" // sched_shutdown

#include <esp_attr.h> // DRAM_ATTR
#include <esp_err.h>
#include <soc/soc.h>
#include <soc/sens_struct.h>
#include <soc/sens_reg.h>
#include <soc/rtc_io_struct.h>
#include <soc/rtc_periph.h>
#include <soc/syscon_struct.h>
#include <driver/adc.h>
#include <driver/gpio.h>
#include <driver/rtc_io.h>


/****************************************************************
 * Analog to Digital Converter (ADC) pins
 ****************************************************************/

#define ADC_FREQ_MAX 20000000 // 20MHz
#define ADC_DELAY    (ADC_FREQ_MAX * 1000ULL / CONFIG_CLOCK_FREQ)

DECL_CONSTANT(ADC_MAX, 4095);

#define INVALID_ADC (struct gpio_adc){ .channel = 0, .adc_num = 0 }
DRAM_ATTR static struct gpio_adc adc_started = INVALID_ADC;

struct gpio_adc
gpio_adc_setup(uint8_t pin)
{
    int rtc_num = rtc_gpio_desc[pin].rtc_num;
    if (rtc_gpio_init(pin) != 0 || rtc_num < 0)
        shutdown("Not a valid ADC pin");
    // rtc_gpio_output_disable(pin);
    CLEAR_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TS_REG, (1 << (rtc_num + RTC_GPIO_ENABLE_W1TS_S)));
    SET_PERI_REG_MASK(RTC_GPIO_ENABLE_W1TC_REG, (1 << ( rtc_num + RTC_GPIO_ENABLE_W1TC_S)));
    //rtc_gpio_input_disable(pin);
    CLEAR_PERI_REG_MASK(rtc_gpio_desc[pin].reg, rtc_gpio_desc[pin].ie);
    __gpio_set_pull_mode(pin, 0); // floating
    uint8_t adc_num = 2, channel = 0;
    if (32 <= pin && pin <= 39) {
        // ADC2
        adc_num = 1;
        channel = (pin >= 36) ? (pin - 36) : (pin - 28);
        // Set attenuation to 11dB ( = full 3V3 scale)
        SET_PERI_REG_BITS(SENS_SAR_ATTEN1_REG, SENS_SAR1_ATTEN_VAL_MASK,
                          ADC_ATTEN_DB_11, (channel * 2));
    } else {
        // ADC2
        switch(pin) {
            case  0: channel = 1; break;
            case  2: channel = 2; break;
            case  4: channel = 0; break;
            case 12: channel = 5; break;
            case 13: channel = 4; break;
            case 14: channel = 6; break;
            case 15: channel = 3; break;
            case 25: channel = 8; break;
            case 26: channel = 9; break;
            case 27: channel = 7; break;
            default: channel = 0xff; break;
        };
        if (channel > 7)
            shutdown("Not a valid ADC pin");
        // Set attenuation to 11dB ( = full 3V3 scale)
        SET_PERI_REG_BITS(SENS_SAR_ATTEN2_REG, SENS_SAR2_ATTEN_VAL_MASK,
                          ADC_ATTEN_DB_11, (channel * 2));
    }
    return (struct gpio_adc){ .channel = 0x1 << channel, .adc_num = adc_num };
}

static void
gpio_adc_start(struct gpio_adc g)
{
    if (g.adc_num == 1) {
        // Start ADC conversion
        SENS.sar_meas_start1.meas1_start_sar = 0;
        SENS.sar_meas_start1.sar1_en_pad = g.channel;
        //while (SENS.sar_slave_addr1.meas_status != 0);
        SENS.sar_meas_start1.meas1_start_sar = 1;
    } else {
        // Start ADC conversion
        SENS.sar_meas_start2.meas2_start_sar = 0;
        SENS.sar_meas_start2.sar2_en_pad = g.channel;
        SENS.sar_meas_start2.meas2_start_sar = 1;
    }
}

static inline uint8_t
gpio_adc_is_ready(struct gpio_adc g)
{
    return (g.adc_num == 1) ?
        SENS.sar_meas_start1.meas1_done_sar :
        SENS.sar_meas_start2.meas2_done_sar;
}

uint32_t
gpio_adc_sample(struct gpio_adc g)
{
    if (!adc_started.adc_num) {
        // Start sample
        gpio_adc_start(g);
        adc_started = g;
        goto need_delay;
    }
    if (g.adc_num != adc_started.adc_num ||
        // Sampling in progress on another channel
        g.channel != adc_started.channel)
        goto need_delay;
    if (!gpio_adc_is_ready(g))
        // Conversion still in progress
        goto need_delay;
    // Conversion ready
    return 0;
need_delay:
    return ADC_DELAY;
}

uint16_t
gpio_adc_read(struct gpio_adc g)
{
    adc_started = INVALID_ADC;
    return (g.adc_num == 1) ?
        SENS.sar_meas_start1.meas1_data_sar :
        SENS.sar_meas_start2.meas2_data_sar;
}

void
gpio_adc_cancel_sample(struct gpio_adc g)
{
    (void)g;
    adc_started = INVALID_ADC;
}

void
gpio_adc_init(void* arg) {
    (void)arg;

    // Disable touch
    SENS.sar_touch_enable.val = 0;
    // Disable DAC (power off)
    RTCIO.pad_dac[0].dac_xpd_force = 0;
    RTCIO.pad_dac[0].xpd_dac = 0;
    RTCIO.pad_dac[1].dac_xpd_force = 0;
    RTCIO.pad_dac[1].xpd_dac = 0;

    // Invert, default = 1
    SENS.sar_read_ctrl.sar1_data_inv = 1;
    SENS.sar_read_ctrl2.sar2_data_inv = 1;
    // Set adc to 12bits
    SENS.sar_start_force.sar1_bit_width = 3;
    SENS.sar_read_ctrl.sar1_sample_bit = 3;
    SENS.sar_start_force.sar2_bit_width = 3;
    SENS.sar_read_ctrl2.sar2_sample_bit = 3;
    // Set cycles
    SENS.sar_read_ctrl.sar1_sample_cycle = 8;
    SENS.sar_read_ctrl2.sar2_sample_cycle = 8;
    // Samples to one
    SENS.sar_read_ctrl.sar1_sample_num = 1;
    SENS.sar_read_ctrl2.sar2_sample_num = 1;
    // Set clock div
    uint8_t div = APB_CLK_FREQ / ADC_FREQ_MAX;
    //SENS.sar_read_ctrl.sar1_clk_div = div;
    //SENS.sar_read_ctrl2.sar2_clk_div = div;
    SYSCON.saradc_ctrl.sar_clk_div = div;

    RTCIO.hall_sens.xpd_hall = 0;

    SENS.sar_meas_wait2.force_xpd_sar = SENS_FORCE_XPD_SAR_PU;
    // fsm disable
    //channel is set in the  convert function
    SENS.sar_meas_wait2.force_xpd_amp = SENS_FORCE_XPD_AMP_PD;
    //disable FSM, it's only used by the LNA.
    SENS.sar_meas_ctrl.amp_rst_fb_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_fsm = 0;
    SENS.sar_meas_ctrl.amp_short_ref_gnd_fsm = 0;
    SENS.sar_meas_wait1.sar_amp_wait1 = 1;
    SENS.sar_meas_wait1.sar_amp_wait2 = 1;
    SENS.sar_meas_wait2.sar_amp_wait3 = 1;

    // ADC1 started by SW
    SENS.sar_meas_start1.meas1_start_force = true;  // RTC controller controls the ADC, not ulp coprocessor
    SENS.sar_meas_start1.sar1_en_pad_force = true;  // RTC controller controls the data port, not ulp coprocessor
    SENS.sar_read_ctrl.sar1_dig_force = false;      // RTC controller controls the ADC, not digital controller
    SENS.sar_touch_ctrl1.xpd_hall_force = true;     // RTC controller controls the hall sensor power, not ulp coprocessor
    SENS.sar_touch_ctrl1.hall_phase_force = true;   // RTC controller controls the hall sensor phase, not ulp coprocessor

    // ADC2 started by SW
    SENS.sar_meas_start2.meas2_start_force = true;  // RTC controller controls the ADC, not ulp coprocessor
    SENS.sar_meas_start2.sar2_en_pad_force = true;  // RTC controller controls the data port, not ulp coprocessor
    SENS.sar_read_ctrl2.sar2_dig_force = false;     // RTC controller controls the ADC, not digital controller
    SENS.sar_read_ctrl2.sar2_pwdet_force = false;   // RTC controller controls the ADC, not PWDET
    SYSCON.saradc_ctrl.sar2_mux = true;             // RTC controller controls the ADC, not PWDET
}
//DECL_INIT(gpio_adc_init);
