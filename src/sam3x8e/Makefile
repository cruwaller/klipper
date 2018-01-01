# Additional sam3x8e build rules

# Setup the toolchain
CROSS_PREFIX=arm-none-eabi-

CFLAGS += -mthumb -mcpu=cortex-m3
CFLAGS += -D__SAM3X8E__
CFLAGS += -Ilib/sam3x8e/include -Ilib/cmsis-include
CFLAGS += -O2 -g
CFLAGS += -flto -fwhole-program -fno-use-linker-plugin

CFLAGS_klipper.elf += $(CFLAGS)
CFLAGS_klipper.elf += -Llib/sam3x8e/source/gcc
CFLAGS_klipper.elf += -Tsam3x8e_flash.ld
CFLAGS_klipper.elf += --specs=nano.specs --specs=nosys.specs

# Add source files
src-y += sam3x8e/main.c sam3x8e/timer.c sam3x8e/gpio.c sam3x8e/spi.c
src-y += generic/crc16_ccitt.c generic/alloc.c
src-y += generic/armcm_irq.c generic/timer_irq.c
src-y += ../lib/sam3x8e/source/system_sam3xa.c
src-y += ../lib/sam3x8e/source/gcc/startup_sam3xa.c
src-$(CONFIG_SERIAL) += sam3x8e/serial.c

# Build the additional hex output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf
	@echo "  Creating hex file $@"
	$(Q)$(OBJCOPY) -O binary $< $@

STTYFARG := $(shell stty --help 2>&1 | \
  grep -q 'illegal option' && echo -f || echo -F)

flash: $(OUT)klipper.bin
	@echo "  Flashing $^ to $(FLASH_DEVICE) via bossac"
	$(Q)stty $(STTYFARG) $(FLASH_DEVICE) 1200
	$(Q)bossac -i -p "$(notdir $(FLASH_DEVICE))" -R -e -w -v -b $(OUT)klipper.bin
