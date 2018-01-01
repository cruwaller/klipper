# Additional linux build rules

src-y += linux/main.c linux/timer.c linux/console.c linux/watchdog.c
src-y += linux/pca9685.c linux/spidev.c linux/analog.c
src-y += generic/crc16_ccitt.c generic/alloc.c

CFLAGS += -O2 -g
CFLAGS += -flto -fwhole-program -fno-use-linker-plugin
CFLAGS_klipper.elf += $(CFLAGS) -lutil

flash: $(OUT)klipper.elf
	@echo "  Flashing"
	$(Q)sudo ./scripts/flash-linux.sh
