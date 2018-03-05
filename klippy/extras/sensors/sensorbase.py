# Printer heater support
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import pins

SAMPLE_TIME_DEFAULT    = 0.001
SAMPLE_COUNT_DEFAULT   = 8

class SensorBase(object):
    spi          = False
    sample_time  = SAMPLE_TIME_DEFAULT
    sample_count = SAMPLE_COUNT_DEFAULT
    def __init__(self,
                 config,
                 is_spi = False,
                 sample_time  = SAMPLE_TIME_DEFAULT,
                 sample_count = SAMPLE_COUNT_DEFAULT):
        self.printer = config.get_printer()
        self.spi = is_spi # remove?
        self.sample_time = sample_time
        self.sample_count = sample_count
        self.min_temp = config.getfloat('min_temp', minval=0., default=0.)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        sensor_pin = config.get('sensor_pin')
        adc_range = [self.calc_adc(self.min_temp),
                     self.calc_adc(self.max_temp)]

        if is_spi:
            self.mcu = pins.setup_pin(
                self.printer, 'temp_spi', sensor_pin)
            self.mcu.setup_spi_settings(
                config.getint('spi_mode', minval=0, maxval=3),
                config.getint('spi_speed', minval=0))
            self.mcu.setup_minmax(
                sample_time, 32767,
                minval=min(adc_range), maxval=max(adc_range))
            self.mcu.setup_read_command(
                self.get_read_cmd(), self.get_read_bytes(),
                self.get_configs(), self.get_fault_filter())

        else:
            self.mcu = pins.setup_pin(
                self.printer, 'adc', sensor_pin)
            self.mcu.setup_minmax(
                sample_time, sample_count,
                minval=min(adc_range), maxval=max(adc_range))
    def get_mcu(self):
        return self.mcu
    def get_min_max_temp(self):
        return self.min_temp, self.max_temp
