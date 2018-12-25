# Additional sam3x8e build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

dirs-y += src/sam3x8e src/generic
dirs-y += lib/sam3x/gcc/gcc

CFLAGS += -mthumb -mcpu=cortex-m3 -falign-loops=16
CFLAGS += -Ilib/sam3x/include -Ilib/cmsis-core
CFLAGS += -D__SAM3X8E__

CFLAGS_klipper.elf += -Llib/sam3x/gcc/gcc
CFLAGS_klipper.elf += -T lib/sam3x/gcc/gcc/sam3x8e_flash.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs

# Add source files
src-y += sam3x8e/main.c sam3x8e/timer.c
src-y += sam3x8e/gpio.c sam3x8e/adc.c sam3x8e/spi.c
src-y += generic/crc16_ccitt.c generic/alloc.c
src-y += generic/armcm_irq.c generic/timer_irq.c
src-y += ../lib/sam3x/gcc/system_sam3xa.c
src-y += ../lib/sam3x/gcc/gcc/startup_sam3xa.c
src-$(CONFIG_SERIAL) += sam3x8e/serial.c generic/serial_irq.c

# Build the additional hex output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

STTYFARG := $(shell stty --help 2>&1 | \
  grep -q 'illegal option' && echo -f || echo -F)

# Flash rules
lib/bossac/bin/bossac:
	@echo "  Building bossac"
	$(Q)make -C lib/bossac bin/bossac

ifeq ($(CONFIG_RPI_SERIAL),)
flash: $(OUT)klipper.bin lib/bossac/bin/bossac
	@echo "  Flashing $^ to $(FLASH_DEVICE) via bossac"
	$(Q)if [ -z $(FLASH_DEVICE) ]; then echo "Please specify FLASH_DEVICE"; exit 1; fi
	$(Q)lib/bossac/bin/bossac -U -p "$(FLASH_DEVICE)" -a -e -w $(OUT)klipper.bin -v -b
	$(Q)lib/bossac/bin/bossac -p "$(FLASH_DEVICE)" -R > /dev/null 2>&1 || true
# This triggers flash erase
#	$(Q)stty $(STTYFARG) $(FLASH_DEVICE) 1200
else
flash: $(OUT)klipper.bin lib/bossac/bin/bossac
	@echo "  Flashing $^ to $(FLASH_DEVICE) via bossac (RS-232)"
	$(Q)if [ -z $(FLASH_DEVICE) ]; then echo "Please specify FLASH_DEVICE"; exit 1; fi
	$(Q)python src/sam3x8e/due_erase.py erase $(CONFIG_RPI_RESET_PIN) $(CONFIG_RPI_ERASE_PIN)
	$(Q)lib/bossac/bin/bossac --usb-port=0 -p "$(FLASH_DEVICE)" -a -e -w $(OUT)klipper.bin -v -b
	$(Q)lib/bossac/bin/bossac -p "$(FLASH_DEVICE)" -R > /dev/null 2>&1 || true
endif
