# Additional sam3x8e build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/lpc176x src/generic
dirs-y += lib/lpc176x/driver lib/lpc176x/startup
dirs-y += lib/lpc176x/lib/chanfs lib/lpc176x/lib/usb
#dirs-y += lib/lpc176x/VCOM_lib/src/usbserial

#CFLAGS += -std=c99
CFLAGS += -std=gnu99
CFLAGS += -mthumb -mcpu=cortex-m3 #-mthumb-interwork
CFLAGS += -D__LPC176x__ -D__LPC1768__ -Dprintf=iprintf
CFLAGS += -DTARGET_LPC1768 -DTOOLCHAIN_GCC_ARM
CFLAGS += -Ilib/cmsis-include
CFLAGS += -Ilib/lpc176x/include
#CFLAGS += -Ilib/lpc176x/VCOM_lib/inc/usbserial

CFLAGS_klipper.elf += -Llib/lpc176x/startup
CFLAGS_klipper.elf += -TLPC1768.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
#CFLAGS_klipper.elf += --no-undefined

ASFLAGS += -mcpu=cortex-m3 -mthumb #-mthumb-interwork

# Add source files
src-y += lpc176x/main.c lpc176x/timer.c lpc176x/gpio.c lpc176x/spi.c lpc176x/adc.c
src-y += generic/crc16_ccitt.c generic/alloc.c
src-y += generic/armcm_irq.c generic/timer_irq.c
src-$(CONFIG_SERIAL) += lpc176x/serial.c

# Add driver sources
#src-y += ../lib/lpc176x/driver/lpc17xx_uart.c
src-y += ../lib/lpc176x/driver/lpc17xx_clkpwr.c
src-y += ../lib/lpc176x/driver/lpc17xx_pinsel.c
src-y += ../lib/lpc176x/driver/lpc17xx_gpio.c
src-y += ../lib/lpc176x/driver/lpc17xx_spi.c
src-y += ../lib/lpc176x/driver/lpc17xx_adc.c
src-y += ../lib/lpc176x/driver/lpc17xx_wdt.c
src-y += ../lib/lpc176x/driver/lpc17xx_timer.c
#src-y += ../lib/lpc176x/driver/lpc17xx_.c

src-y += ../lib/lpc176x/startup/system_LPC17xx.c
asm-y += ../lib/lpc176x/startup/startup_LPC17xx.s

# Build the additional hex output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

flash: $(OUT)klipper.bin
	@echo " [LPC176x] Sorry! Flashing is not supported"
