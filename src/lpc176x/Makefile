# lpc176x build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/lpc176x src/generic
dirs-y += lib/lpc176x/device/TOOLCHAIN_GCC_ARM

CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -Ilib/lpc176x/device -Ilib/cmsis-core

# Add source files
src-y += lpc176x/main.c lpc176x/gpio.c
src-$(CONFIG_HAVE_GPIO_ADC) += lpc176x/adc.c
src-$(CONFIG_HAVE_GPIO_I2C) += lpc176x/i2c.c
src-$(CONFIG_HAVE_GPIO_SPI) += lpc176x/spi.c
src-y += generic/crc16_ccitt.c generic/alloc.c
src-y += generic/armcm_irq.c generic/armcm_timer.c
src-y += ../lib/lpc176x/device/system_LPC17xx.c
src-$(CONFIG_USBSERIAL) += lpc176x/usbserial.c generic/usb_cdc.c
src-$(CONFIG_SERIAL) += lpc176x/serial.c generic/serial_irq.c

# Add assembler build rules
$(OUT)%.o: %.S
	@echo "  Assembling $@"
	$(Q)$(AS) $< -o $@

asmsrc-y := ../lib/lpc176x/device/TOOLCHAIN_GCC_ARM/startup_LPC17xx.S
OBJS_klipper.elf += $(patsubst %.S, $(OUT)src/%.o,$(asmsrc-y))

# Build the linker script
$(OUT)LPC1768.ld: lib/lpc176x/device/TOOLCHAIN_GCC_ARM/LPC1768.ld  $(OUT)autoconf.h $(OUT)board-link
	@echo "  Preprocessing $@"
	$(Q)$(CPP) -P -MD -MT $@ -DMBED_APP_START=$(CONFIG_FLASH_START) -DMBED_APP_SIZE="(512K - $(CONFIG_FLASH_START))" $< -o $@

CFLAGS_klipper.elf += -T $(OUT)LPC1768.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs
$(OUT)klipper.elf : $(OUT)LPC1768.ld

# Build the additional bin output file
target-y += $(OUT)klipper.bin
target-y += $(OUT)klipper.hex

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating bin file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

$(OUT)klipper.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -R .stack -O ihex $< $@

$(OUT)klipper.disasm: $(OUT)klipper.elf
	@echo "  Creating disasm file $@"
	$(Q)$(OBJDUMP)  -d -f -M reg-names-std --demangle $< > $@

ifeq ($(CONFIG_SMOOTHIEWARE_BOOTLOADER),y)
flash: $(OUT)klipper.bin
	@echo ""
	@echo "  The LPC176x build does not currently support 'make flash'"
	@echo ""
	@echo "  To flash a Smoothieboard, copy the $(OUT)klipper.bin to a file"
	@echo "  named firmware.bin on an SD card and then restart the"
	@echo "  Smoothieboard with that SD card."
else
LISP := $(shell command -v lpc21isp 2> /dev/null)
flash: $(OUT)klipper.hex
	@echo ""
ifeq ($(LSIP),)
	@echo "  lpc2lisp is not installed. Please install it and try again."
else
	@echo "  Flashing $^ to $(FLASH_DEVICE) via lpc21isp"
	$(Q)if [ -z $(FLASH_DEVICE) ]; then echo "Please specify FLASH_DEVICE"; exit 1; fi
	$(Q)lpc21isp $(OUT)klipper.hex $(FLASH_DEVICE) 57600 12000
endif
endif
