#include <stddef.h>   // NULL
#include "sched.h"
#include "autoconf.h"
#include "gpio.h"
#include "generic/spi.h"


#include <LPC17xx.h>
#include <lpc17xx_clkpwr.h>
#include <lpc17xx_spi.h>

typedef struct ADC_mapping_t {
    _gpio_peripheral_t clk;
    _gpio_peripheral_t mosi;
    _gpio_peripheral_t miso;
} ADC_mapping_t;

// TODO FIXME: SPI mapping to SSP1 (EXT-2 on Smoothie)

/*
 * SPI or SSP0/1
 *   * SSEL is handled by application using GPIO
 */
static const ADC_mapping_t g_pinsSPI[] = {
    {
        // SPI ( overlap pins with SSP0 )
        { 0, 15, PINSEL_FUNC_3 }, // CLK  - P0.15
        { 0, 18, PINSEL_FUNC_3 }, // MOSI - P0.18
        { 0, 17, PINSEL_FUNC_3 }  // MISO - P0.17
    }
#if 0
    ,{
        // SSP0 ( overlap pins with SPI )
        { 0, 15, PINSEL_FUNC_2 }, // CLK  - P0.15
        { 0, 18, PINSEL_FUNC_2 }, // MOSI - P0.18
        { 0, 17, PINSEL_FUNC_2 }  // MISO - P0.17
    },
    {
        // SSP1
        { 0,  7, PINSEL_FUNC_2 }, // CLK  - P0.07
        { 0,  9, PINSEL_FUNC_2 }, // MOSI - P0.09
        { 0,  8, PINSEL_FUNC_2 }  // MISO - P0.08
    }
#endif
};


SPI_t spi_basic_config = 0;

static uint32_t
spi_get_clock(uint32_t const target_clock)
{
    uint32_t prescale = 8;
    uint32_t const spi_pclk = CLKPWR_GetPCLK(CLKPWR_PCLKSEL_SPI);

    // Find closest clock to target clock
    while (1) {
        if (spi_pclk <= (target_clock * prescale)) {
            break;
        }
        prescale += 2;
        if (254 < prescale) {
            break;
        }
    }
    return SPI_SPCCR_COUNTER(prescale); // same as & SPI_SPCCR_BITMASK, 0xFF
}


void
spi_init(void)
{
    /* Configure SCK, MISO and MOSI */
    gpio_peripheral(&g_pinsSPI[0].clk,  0);
    gpio_peripheral(&g_pinsSPI[0].mosi, 0);
    gpio_peripheral(&g_pinsSPI[0].miso, 0);

    // Power on the SPI
    CLKPWR_ConfigPPWR(CLKPWR_PCONP_PCSPI, ENABLE);
    CLKPWR_SetPCLKDiv(CLKPWR_PCLKSEL_SPI, CLKPWR_PCLKSEL_CCLK_DIV_1);

    // Set SPI default settings
    spi_basic_config = spi_get_config(0, 4000000); // 4MHz, SPI Mode0
    spi_set_config(spi_basic_config); // Set default SPI config
}
DECL_INIT(spi_init);

SPI_t
spi_get_config(uint8_t const mode, uint32_t const clock)
{
    SPI_t config = 0;
    config |= (SPI_MASTER_MODE);
    config |= (SPI_DATA_MSB_FIRST);
    config |= (SPI_DATABIT_8);

    switch(mode) {
        case 0:
            config |= (SPI_CPHA_FIRST);
            config |= (SPI_CPOL_HI);
            break;
        case 1:
            config |= (SPI_CPHA_SECOND);
            break;
        case 2:
            config |= (SPI_CPOL_LO);
            break;
        case 3:
            config |= (SPI_CPHA_SECOND);
            config |= (SPI_CPOL_LO);
            break;
    };

    config &= SPI_SPCR_BITMASK; // Store config , 0xFFC

    // Calculate SPI prescaler
    config |= (spi_get_clock(clock) << 24);

    return config;
}

void
spi_set_config(SPI_t const config)
{
    LPC_SPI->SPCR  = (config & SPI_SPCR_BITMASK);     // Set config
    LPC_SPI->SPCCR = SPI_SPCCR_COUNTER(config >> 24); // Set SPI clock
}

void
spi_transfer_len(char *data, uint8_t len)
{
    uint16_t i;
    for (i = 0; i < len; i++) {
        data[i] = spi_transfer(data[i], 0);
    }
}

uint8_t
spi_transfer(uint8_t const data, uint8_t const last)
{
    (void)last;
    // write byte with address and end transmission flag
    LPC_SPI->SPDR = (data & SPI_SPDR_BITMASK);
    // wait for transmit register empty
    while (!(LPC_SPI->SPSR & SPI_SPSR_SPIF));
    // get data
    return (uint8_t)(LPC_SPI->SPDR & SPI_SPDR_BITMASK);
}
