#include <sdkconfig.h>
#include "autoconf.h"

#include <stddef.h>   // NULL
#include <string.h>
#include "command.h" // DECL_SHUTDOWN
#include "gpio.h"
#include "sched.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include <rom/ets_sys.h>
#include <esp_attr.h>
#include <esp_intr.h>
#include <rom/gpio.h>
#include <soc/spi_reg.h>
#include <soc/spi_struct.h>
#include <soc/io_mux_reg.h>
#include <soc/gpio_sig_map.h>
#include <soc/dport_reg.h>
#include <soc/rtc.h>
#include <soc/soc.h> //APB_CLK_FREQ


#define FSPI  1
#define HSPI  2
#define VSPI  3

#define MUTEX_LOCK(_lock)    while (xSemaphoreTake(_lock, portMAX_DELAY) != pdPASS);
#define MUTEX_UNLOCK(_lock)  xSemaphoreGive(_lock)

struct spi_struct_t {
    spi_dev_t * dev;
    xSemaphoreHandle lock;
    uint32_t clk_en;
    uint32_t rst;
    uint8_t num, sck, miso, mosi;
    uint8_t sck_func, miso_func, mosi_func;
};
typedef struct spi_struct_t spi_t;


static spi_t _spi_bus_array[3] = {
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), NULL, DPORT_SPI_CLK_EN_1, DPORT_SPI_RST_1,
     FSPI,  6,  7,  8, SPICLK_OUT_IDX, SPIQ_OUT_IDX, SPID_IN_IDX},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), NULL, DPORT_SPI_CLK_EN,   DPORT_SPI_RST,
     HSPI, 14, 12, 13, HSPICLK_OUT_IDX, HSPIQ_OUT_IDX, HSPID_IN_IDX},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), NULL, DPORT_SPI_CLK_EN_2, DPORT_SPI_RST_2,
     VSPI, 18, 19, 23, VSPICLK_OUT_IDX, VSPIQ_OUT_IDX, VSPID_IN_IDX}
};

struct spi_config
spi_setup(uint32_t bus, uint8_t mode, uint32_t rate)
{
    if (bus > 2 || mode > 3)
        shutdown("Invalid spi_setup parameters");

    spi_t * spi = &_spi_bus_array[bus];

    if (spi->lock == NULL) {
        spi->lock = xSemaphoreCreateMutex();
        if (spi->lock == NULL) {
            shutdown("SPI init failure");
        }
    }

    MUTEX_LOCK(spi->lock);

    /* Enable peripheral */
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, spi->clk_en);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, spi->rst);

    /* Clear */
    spi->dev->slave.trans_done = 0;
    spi->dev->slave.slave_mode = 0;
    spi->dev->pin.val = 0;
    spi->dev->user.val = 0;
    spi->dev->user1.val = 0;
    spi->dev->ctrl.val = 0;
    spi->dev->ctrl1.val = 0;
    spi->dev->ctrl2.val = 0;
    spi->dev->clock.val = 0;

    /* Calculate data mode */
    switch (mode) {
        case 1:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 1;
            break;
        case 2:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 1;
            break;
        case 3:
            spi->dev->pin.ck_idle_edge = 1;
            spi->dev->user.ck_out_edge = 0;
            break;
        case 0:
        default:
            spi->dev->pin.ck_idle_edge = 0;
            spi->dev->user.ck_out_edge = 0;
            break;
    }

    // Bit order to SPI_MSBFIRST
    spi->dev->ctrl.wr_bit_order = 0;
    spi->dev->ctrl.rd_bit_order = 0;

    // Clock divider
    // BASE_CLK = APB_CLK_FREQ = 80MHz
    // f_spi = BASE_CLK / ( (clkcnt_n + 1) * (clkdiv_pre + 1) )
    //  * fix clkcnt_n to 3 => f_spi = BASE_CLK / (clkdiv_pre * 4 + 4)
    //     ==> clkdiv_pre * 4 = BASE_CLK / f_spi - 4
    //     ==> clkdiv_pre = BASE_CLK / (f_spi * 4) - 1
    //
    spi->dev->clock.clk_equ_sysclk = 0;
    spi->dev->clock.clkdiv_pre = (APB_CLK_FREQ / (rate * 4));
    spi->dev->clock.clkcnt_n = 3;
    spi->dev->clock.clkcnt_h = 1;
    spi->dev->clock.clkcnt_l = 3;

    spi->dev->user.usr_mosi = 1;
    spi->dev->user.usr_miso = 1;
    spi->dev->user.doutdin = 1; // Full duplex mode
    // Disable CS auto control
    spi->dev->user.cs_setup = 0;
    spi->dev->user.cs_hold = 0;
    spi->dev->pin.cs0_dis = 1;
    spi->dev->pin.cs1_dis = 1;
    spi->dev->pin.cs2_dis = 1;

    struct spi_config cfg = {
        .dev=spi,
        .pin_reg=spi->dev->pin.val,
        .user_reg=spi->dev->user.val,
        .ctrl_reg=spi->dev->ctrl.val,
        .clock_reg=spi->dev->clock.val
    };

    MUTEX_UNLOCK(spi->lock);

    // SCK pin config
    gpio_out_setup(spi->sck, 1);
    gpio_matrix_out(spi->sck, spi->sck_func, false, false);
    // MISO pin config
    gpio_in_setup(spi->miso, 1);
    gpio_matrix_in(spi->miso, spi->miso_func, false);
    // MOSI pin config
    gpio_out_setup(spi->mosi, 1);
    gpio_matrix_out(spi->mosi, spi->mosi_func, false, false);

    return cfg;
}

void spi_prepare(struct spi_config config)
{
    spi_t * spi = (spi_t*)config.dev;
    MUTEX_LOCK(spi->lock);
    spi->dev->pin.val = config.pin_reg;
    spi->dev->user.val = config.user_reg;
    spi->dev->ctrl.val = config.ctrl_reg;
    spi->dev->clock.val = config.clock_reg;
}

void
spi_transfer(struct spi_config config, uint8_t receive_data
             , uint8_t len, uint8_t *data)
{
    int i;
    spi_t * spi = (spi_t*)config.dev;
    uint32_t * tx_ptr = (uint32_t*)data;
    uint32_t * rx_ptr = tx_ptr;

    while (len) {
        uint32_t bytes = 64 < len ? 64 : len;
        uint32_t words = (bytes + 3) / 4;

        spi->dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
        spi->dev->miso_dlen.usr_miso_dbitlen = receive_data ? ((bytes * 8) - 1) : 0;

        // Fill data to be sent
        for (i = 0; i < words; i++) {
            spi->dev->data_buf[i] = *tx_ptr++;
        }

        // Send and wait for ready
        spi->dev->cmd.usr = 1;
        while (spi->dev->cmd.usr);

        // Read received data
        if (receive_data) {
            for (i = 0; i < words; i++, rx_ptr++) {
                *rx_ptr = spi->dev->data_buf[i];
            }
        }

        len -= bytes;
    }
    MUTEX_UNLOCK(spi->lock);
}
