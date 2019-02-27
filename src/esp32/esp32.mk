
ESP32_CPPFLAGS = \
  -I$(IDF_PATH)/components \
  -I$(IDF_BUILD_PATH)/include \
  -I$(IDF_PATH)/components/bluedroid \
  -I$(IDF_PATH)/components/bluedroid/api \
  -I$(IDF_PATH)/components/app_trace \
  -I$(IDF_PATH)/components/app_update \
  -I$(IDF_PATH)/components/bootloader_support \
  -I$(IDF_PATH)/components/bt/include \
  -I$(IDF_PATH)/components/bt/bluedroid/api/include/ \
  -I$(IDF_PATH)/components/driver/include \
  -I$(IDF_PATH)/components/esp32/include \
  -I$(IDF_PATH)/components/esp_adc_cal \
  -I$(IDF_PATH)/components/esp_http_client \
  -I$(IDF_PATH)/components/esp-tls \
  -I$(IDF_PATH)/components/esp_ringbuf/include \
  -I$(IDF_PATH)/components/ethernet \
  -I$(IDF_PATH)/components/fatfs \
  -I$(IDF_PATH)/components/freertos/include \
  -I$(IDF_PATH)/components/heap/include \
  -I$(IDF_PATH)/components/jsmn \
  -I$(IDF_PATH)/components/log/include \
  -I$(IDF_PATH)/components/mdns \
  -I$(IDF_PATH)/components/mbedtls \
  -I$(IDF_PATH)/components/mbedtls_port \
  -I$(IDF_PATH)/components/newlib \
  -I$(IDF_PATH)/components/nvs_flash/include \
  -I$(IDF_PATH)/components/openssl \
  -I$(IDF_PATH)/components/spi_flash/include \
  -I$(IDF_PATH)/components/sdmmc \
  -I$(IDF_PATH)/components/smartconfig_ack \
  -I$(IDF_PATH)/components/spiffs \
  -I$(IDF_PATH)/components/tcpip_adapter \
  -I$(IDF_PATH)/components/ulp \
  -I$(IDF_PATH)/components/vfs \
  -I$(IDF_PATH)/components/wear_levelling \
  -I$(IDF_PATH)/components/xtensa-debug-module \
  -I$(IDF_PATH)/components/coap \
  -I$(IDF_PATH)/components/console \
  -I$(IDF_PATH)/components/expat \
  -I$(IDF_PATH)/components/json \
  -I$(IDF_PATH)/components/lwip \
  -I$(IDF_PATH)/components/newlib \
  -I$(IDF_PATH)/components/nghttp \
  -I$(IDF_PATH)/components/soc/include \
  -I$(IDF_PATH)/components/soc/esp32/include \
  -I$(IDF_PATH)/components/wpa_supplicant \

# -ffunction-sections -fdata-sections
ESP32_CFLAGS = \
  -fstack-protector -fstrict-volatile-bitfields \
  -mlongcalls -nostdlib -Wpointer-arith \
  -Wno-error=unused-function -Wno-error=unused-but-set-variable -Wno-error=unused-variable \
  -Wno-error=deprecated-declarations -Wno-unused-parameter -Wno-sign-compare \
  -Wno-old-style-declaration
# -MMD -c


ESP32_LDFLAGS := \
  -nostdlib -u call_user_start_cpu0 -Wl,--gc-sections -Wl,-static \
  -Wl,--start-group \
  -L $(IDF_PATH)/components/esp32/ld -T esp32_out.ld \
  -T $(IDF_BUILD_PATH)/esp32/esp32.common.ld \
  -T esp32.rom.ld -T esp32.peripherals.ld -T esp32.rom.libgcc.ld  \
  -L$(IDF_BUILD_PATH)/app_trace -lapp_trace \
  -L$(IDF_BUILD_PATH)/app_update -lapp_update -u esp_app_desc \
  -L$(IDF_BUILD_PATH)/asio -lasio \
  -L$(IDF_BUILD_PATH)/aws_iot  \
  -L$(IDF_BUILD_PATH)/bootloader_support -lbootloader_support \
  -L$(IDF_BUILD_PATH)/bt -lbt \
  -L$(IDF_PATH)/components/bt/lib -lbtdm_app \
  -L$(IDF_BUILD_PATH)/coap -lcoap \
  -L$(IDF_BUILD_PATH)/console -lconsole \
  -L$(IDF_BUILD_PATH)/cxx -lcxx -u __cxa_guard_dummy -u __cxx_fatal_exception \
  -L$(IDF_BUILD_PATH)/driver -ldriver \
  -L$(IDF_BUILD_PATH)/esp-tls -lesp-tls \
  -L$(IDF_BUILD_PATH)/esp32 -lesp32 \
  $(IDF_PATH)/components/esp32/libhal.a \
  -L$(IDF_PATH)/components/esp32/lib -lcore -lrtc -lnet80211 -lpp -lwpa -lsmartconfig -lcoexist -lwps -lwpa2 -lespnow -lphy -lmesh \
  -u ld_include_panic_highint_hdl \
  -L$(IDF_BUILD_PATH)/esp_adc_cal -lesp_adc_cal \
  -L$(IDF_BUILD_PATH)/esp_event -lesp_event \
  -L$(IDF_BUILD_PATH)/esp_http_client -lesp_http_client \
  -L$(IDF_BUILD_PATH)/esp_http_server -lesp_http_server \
  -L$(IDF_BUILD_PATH)/esp_https_ota -lesp_https_ota \
  -L$(IDF_BUILD_PATH)/esp_https_server -lesp_https_server \
  -L$(IDF_BUILD_PATH)/esp_ringbuf -lesp_ringbuf \
  -L$(IDF_BUILD_PATH)/ethernet -lethernet \
  -L$(IDF_BUILD_PATH)/expat -lexpat \
  -L$(IDF_BUILD_PATH)/fatfs -lfatfs \
  -L$(IDF_BUILD_PATH)/freemodbus -lfreemodbus \
  -L$(IDF_BUILD_PATH)/freertos -lfreertos -Wl,--undefined=uxTopUsedPriority \
  -L$(IDF_BUILD_PATH)/heap -lheap \
  -L$(IDF_BUILD_PATH)/idf_test -lidf_test \
  -L$(IDF_BUILD_PATH)/jsmn -ljsmn \
  -L$(IDF_BUILD_PATH)/json -ljson \
  -L$(IDF_BUILD_PATH)/libsodium -llibsodium \
  -L$(IDF_BUILD_PATH)/log -llog \
  -L$(IDF_BUILD_PATH)/lwip -llwip \
  -L$(IDF_BUILD_PATH)/main -lmain \
  -L$(IDF_BUILD_PATH)/mbedtls -lmbedtls \
  -L$(IDF_BUILD_PATH)/mdns -lmdns \
  -L$(IDF_BUILD_PATH)/micro-ecc -lmicro-ecc \
  -L$(IDF_BUILD_PATH)/mqtt -lmqtt \
  -L$(IDF_BUILD_PATH)/newlib \
  $(IDF_PATH)/components/newlib/lib/libc-psram-workaround.a \
  $(IDF_PATH)/components/newlib/lib/libm-psram-workaround.a \
  -lnewlib \
  -L$(IDF_BUILD_PATH)/nghttp -lnghttp \
  -L$(IDF_BUILD_PATH)/nvs_flash -lnvs_flash \
  -L$(IDF_BUILD_PATH)/openssl -lopenssl \
  -L$(IDF_BUILD_PATH)/protobuf-c -lprotobuf-c \
  -L$(IDF_BUILD_PATH)/protocomm -lprotocomm \
  -L$(IDF_BUILD_PATH)/pthread -lpthread \
  -L$(IDF_BUILD_PATH)/sdmmc -lsdmmc \
  -L$(IDF_BUILD_PATH)/smartconfig_ack -lsmartconfig_ack \
  -L$(IDF_BUILD_PATH)/soc -lsoc \
  -L$(IDF_BUILD_PATH)/spi_flash -lspi_flash \
  -L$(IDF_BUILD_PATH)/spiffs -lspiffs \
  -L$(IDF_BUILD_PATH)/tcp_transport -ltcp_transport \
  -L$(IDF_BUILD_PATH)/tcpip_adapter -ltcpip_adapter \
  -L$(IDF_BUILD_PATH)/ulp -lulp \
  -L$(IDF_BUILD_PATH)/unity -lunity \
  -L$(IDF_BUILD_PATH)/vfs -lvfs \
  -L$(IDF_BUILD_PATH)/wear_levelling -lwear_levelling \
  -L$(IDF_BUILD_PATH)/wifi_provisioning -lwifi_provisioning \
  -L$(IDF_BUILD_PATH)/wpa_supplicant -lwpa_supplicant \
  -L$(IDF_BUILD_PATH)/xtensa-debug-module -lxtensa-debug-module \
  -lgcc -lstdc++ -lgcov \
  -Wl,--end-group -Wl,-EL

ifeq ($(CONFIG_NEWLIB_NANO_FORMAT),y)
  ESP32_LDFLAGS += -T esp32.rom.nanofmt.ld
endif
