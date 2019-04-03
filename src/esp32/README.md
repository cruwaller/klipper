## Building the mcu code for esp32

#### 1. Clone ESP IDF on your computer
reference: [espressif get started](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/)
```
mkdir ~/esp && cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
export ESP_IDF=~/esp/esp-idf
```

#### 2. Create a hello world example project
```
cp -r $IDF_PATH/examples/get-started/hello_world ~/esp/
```

#### 3. Build a required IDF libraries
```
cp <klipper path>/src/esp32/esp_idf.sdkconfig ~/esp/hello_world/sdkconfig
cd ~/esp/hello_world/
make
```

#### 4. Define env variable for klipper use
```
export IDF_BUILD_PATH=~/esp/hello_world/build
```

#### 5. Build the klipper ESP32 binary normally
```
make menuconfig
make
make flash FLASH_DEVICE=<path>
```
