# Additional avr build rules

# Use the avr toolchain
CROSS_PREFIX=avr-

CFLAGS += -mmcu=$(CONFIG_MCU) -O2 -g
CFLAGS += -flto -fwhole-program -fno-use-linker-plugin
CFLAGS_klipper.elf += $(CFLAGS)

# Add avr source files
src-y += avr/main.c avr/timer.c avr/gpio.c
src-$(CONFIG_AVR_WATCHDOG) += avr/watchdog.c
src-$(CONFIG_AVR_USBSERIAL) += avr/usbserial.c ../lib/pjrc_usb_serial/usb_serial.c
src-$(CONFIG_AVR_SERIAL) += avr/serial.c

# Suppress broken "misspelled signal handler" warnings on gcc 4.8.1
CFLAGS_klipper.elf := $(CFLAGS_klipper.elf) $(if $(filter 4.8.1, $(shell $(CC) -dumpversion)), -w)

# Build the additional hex output file
target-y += $(OUT)klipper.elf.hex

$(OUT)klipper.elf.hex: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -j .text -j .data -O ihex $< $@


ifneq ($(CONFIG_AVRDUDE_UPLOADSPEED),)
  uploadSpeed = -b$(CONFIG_AVRDUDE_UPLOADSPEED)
endif
STTYFARG := $(shell stty --help 2>&1 | \
  grep -q 'illegal option' && echo -f || echo -F)

flash: $(OUT)klipper.elf.hex
	@echo "  Flashing $^ to $(FLASH_DEVICE) via avrdude"
	$(Q)if [ -z $(FLASH_DEVICE) ]; then echo "Please specify FLASH_DEVICE"; exit 1; fi
	$(Q)stty $(STTYFARG) $(FLASH_DEVICE) hupcl
	$(Q)avrdude -p$(CONFIG_MCU) -c$(CONFIG_AVRDUDE_UPLOADPROTOCOL) -P"$(FLASH_DEVICE)" -D -U"flash:w:$(OUT)klipper.elf.hex:i" $(uploadSpeed)

# 328p: avrdude -v -D -patmega328p -P/dev/ttyUSB0 -carduino -U"flash:w:out/klipper.elf.hex:i" -b57600
