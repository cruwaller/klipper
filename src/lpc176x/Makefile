# Additional sam3x8e build rules

USBLIBDIR = lib/lpc176x/VCOM_lib/

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/lpc176x src/generic
dirs-y += lib/lpc176x/driver lib/lpc176x/startup
dirs-y += lib/lpc176x/cmsis
dirs-y += lib/lpc176x/VCOM_lib/src/usbSerial

#CFLAGS += -std=c99
CFLAGS += -std=gnu99
CFLAGS += -mthumb -mcpu=cortex-m3 -mthumb-interwork
CFLAGS += -D__LPC176x__ -D__LPC1768__ -Dprintf=iprintf
CFLAGS += -DTARGET_LPC1768 -DTOOLCHAIN_GCC_ARM
CFLAGS += -Ilib/cmsis-include
CFLAGS += -Ilib/lpc176x/include
CFLAGS += -Ilib/lpc176x/cmsis
CFLAGS += -Ilib/lpc176x/VCOM_lib/inc/usbSerial

CFLAGS_klipper.elf += --specs=nano.specs #--specs=nosys.specs #-nostdlib
CFLAGS_klipper.elf += --specs=nosys.specs
#CFLAGS_klipper.elf += -specs=nano.specs -lstdc++ -lsupc++ -lm -lgcc -lc -lnosys
CFLAGS_klipper.elf += -Wl,-Map=$(OUT)klipper.map,--cref
CLFAGS_klipper.elf += -specs=lib/lpc176x/startup/startfile.spec
CFLAGS_klipper.elf += -Tlib/lpc176x/startup/LPC1768.ld
CFLAGS_klipper.elf += -nostartfiles
#CFLAGS_klipper.elf += -Xlinker #--gc-sections

ASFLAGS += -mcpu=cortex-m3 -mthumb

# Add driver sources
src-y += ../lib/lpc176x/driver/lpc17xx_clkpwr.c
src-y += ../lib/lpc176x/driver/lpc17xx_pinsel.c
src-y += ../lib/lpc176x/driver/lpc17xx_gpio.c
src-y += ../lib/lpc176x/driver/lpc17xx_spi.c
src-y += ../lib/lpc176x/driver/lpc17xx_adc.c
src-y += ../lib/lpc176x/driver/lpc17xx_wdt.c
src-y += ../lib/lpc176x/driver/lpc17xx_timer.c
src-y += ../lib/lpc176x/driver/lpc17xx_uart.c

src-y += ../lib/lpc176x/cmsis/cmsis_nvic.c

src-y += ../lib/lpc176x/startup/system_LPC17xx.c
asm-y += ../lib/lpc176x/startup/startup_LPC17xx.s

# usb serial:
src-$(CONFIG_SERIAL) += ../lib/lpc176x/VCOM_lib/src/usbSerial/usbcontrol.c
src-$(CONFIG_SERIAL) += ../lib/lpc176x/VCOM_lib/src/usbSerial/usbhw_lpc.c
src-$(CONFIG_SERIAL) += ../lib/lpc176x/VCOM_lib/src/usbSerial/usbinit.c
src-$(CONFIG_SERIAL) += ../lib/lpc176x/VCOM_lib/src/usbSerial/usbstdreq.c

# Add source files
src-y += lpc176x/main.c lpc176x/timer.c lpc176x/gpio.c lpc176x/spi.c lpc176x/adc.c lpc176x/mempool.c
src-y += generic/crc16_ccitt.c generic/alloc.c generic/timer_irq.c generic/armcm_irq.c
src-$(CONFIG_SERIAL) += lpc176x/serial_usb.c lpc176x/serial_uart.c

# Build the additional hex output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@
flash: $(OUT)klipper.bin
	@echo " [LPC176x] Sorry! Flashing is not supported"
