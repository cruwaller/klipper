# Additional simulator build rules

CFLAGS += -O2 -g
CFLAGS += -flto -fwhole-program -fno-use-linker-plugin
CFLAGS_klipper.elf += $(CFLAGS)

src-y += simulator/main.c simulator/gpio.c simulator/timer.c simulator/serial.c
src-y += generic/crc16_ccitt.c generic/alloc.c
src-y += generic/timer_irq.c generic/serial_irq.c
