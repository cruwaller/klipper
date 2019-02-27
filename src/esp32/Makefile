# Additional ESP32 build rules

# Setup the toolchain
CROSS_PREFIX = xtensa-esp32-elf-
#CROSS_PREFIX := $(TOOLSDIR)/$(CROSS_PREFIX)

# IDF environment paths
ifeq ($(IDF_BUILD_PATH),)
  $(error IDF_BUILD_PATH is not defined)
else
  ifneq ($(wildcard $(IDF_BUILD_PATH)/sdkconfig),)
    # Point to root of the build folder, change it to /build
    IDF_BUILD_PATH := $(IDF_BUILD_PATH)/build
  endif
endif
ifeq ($(IDF_PATH),)
  $(error IDF_PATH is not defined!)
else
  ifeq ($(wildcard $(IDF_PATH)/components/soc),)
    $(error IDF_PATH does not point to root of the IDF)
  endif
endif

ESPPART = $(IDF_PATH)/components/partition_table/gen_esp32part.py
ESPTOOL = $(IDF_PATH)/components/esptool_py/esptool/esptool.py
PARTTOOL = python $(IDF_PATH)/components/partition_table/parttool.py -q

# include idf sdkconfig
-include $(IDF_BUILD_PATH)/../sdkconfig

include src/esp32/esp32.mk

CFLAGS += -Wno-old-style-definition
CFLAGS += -Ilib/esp32/hal -DESP32 -DESP_PLATFORM
CFLAGS += $(ESP32_CPPFLAGS)
CFLAGS += $(ESP32_CFLAGS)
CFLAGS_klipper.elf := $(ESP32_LDFLAGS)
CFLAGS_klipper.elf += -flto -fwhole-program
CFLAGS_klipper.elf += -Wl,-Map=$(OUT)klipper.map

# Add source files
src-y += esp32/main.c esp32/irq.c esp32/alloc.c esp32/timer.c
src-y += esp32/serial_gen.c esp32/spi.c esp32/gpio_expander.c
src-y += generic/timer_irq.c generic/crc16_ccitt.c
# Add configuration specific files
src-$(CONFIG_SERIAL_UART) += esp32/serial.c
src-$(CONFIG_SERIAL_BT) += esp32/serial_bt.c
src-$(CONFIG_HAVE_GPIO) += esp32/gpio.c
src-$(CONFIG_HAVE_GPIO_ADC) += esp32/adc.c

# Build the additional hex output file
target-y += $(OUT)klipper.bin

$(OUT)klipper.bin: $(OUT)klipper.elf $(ESPTOOL)
	@echo "  Creating klipper.bin"
	$(Q)python $(ESPTOOL) --chip $(CONFIG_MCU) elf2image \
        --flash_mode $(CONFIG_ESPTOOLPY_FLASHMODE) --flash_freq $(CONFIG_ESPTOOLPY_FLASHFREQ) \
        --flash_size $(CONFIG_ESPTOOLPY_FLASHSIZE) -o $@ $<

# Build partition file
PARTITION_FILE := $(IDF_PATH)/components/partition_table/$(subst ",,$(CONFIG_PARTITION_TABLE_FILENAME))
$(OUT)klipper.partition.bin: $(ESPPART) $(PARTITION_FILE)
	@echo "  Creating klipper partition table"
	$(Q)python $(ESPPART) -q $(PARTITION_FILE) $@

BOOTLOADER_BIN = $(IDF_BUILD_PATH)/bootloader/bootloader.bin

ESPTOOL_FLAGS := \
  --chip $(CONFIG_MCU) \
  --baud $(CONFIG_ESPTOOLPY_BAUD) \
  --before default_reset \
  --after hard_reset write_flash -z \
  --flash_mode $(CONFIG_ESPTOOLPY_FLASHMODE) \
  --flash_freq $(CONFIG_ESPTOOLPY_FLASHFREQ) \
  --flash_size detect

# Flash rules
flash: $(OUT)klipper.bin $(OUT)klipper.partition.bin
	@echo "  Flashing to $(FLASH_DEVICE) via esptool"
	$(Q)if [ -z $(FLASH_DEVICE) ]; then echo "Please specify FLASH_DEVICE"; exit 1; fi
	$(eval PHY_DATA_OFFSET:=$(shell $(PARTTOOL) --partition-type data --partition-subtype phy --partition-table-file $(OUT)klipper.partition.bin get_partition_info --info offset))
	$(eval APP_OFFSET:=$(shell $(PARTTOOL) --partition-boot-default --partition-table-file $(OUT)klipper.partition.bin get_partition_info --info offset))
	$(eval OTA_DATA_OFFSET:=$(shell $(PARTTOOL) --partition-type data --partition-subtype ota --partition-table-file $(OUT)klipper.partition.bin get_partition_info --info offset))
	$(eval OTA_DATA_SIZE:=$(shell $(PARTTOOL) --partition-type data --partition-subtype ota --partition-table-file $(OUT)klipper.partition.bin get_partition_info --info size))
	$(Q)python $(ESPTOOL) --port $(FLASH_DEVICE) $(ESPTOOL_FLAGS) \
        $(OTA_DATA_OFFSET) $(IDF_BUILD_PATH)/ota_data_initial.bin \
        0x1000 $(BOOTLOADER_BIN) \
        $(APP_OFFSET) $(OUT)klipper.bin \
        $(CONFIG_PARTITION_TABLE_OFFSET) $(OUT)klipper.partition.bin
