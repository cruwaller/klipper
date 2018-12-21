# Printer heater support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import extras.bus as bus

SAMPLE_TIME_DEFAULT    = 0.001
SAMPLE_COUNT_DEFAULT   = 8
REPORT_TIME_DEFAULT    = 0.300

VALID_SPI_SENSORS = {
    'MAX31855' : 1,
    'MAX31856' : 2,
    'MAX31865' : 4,
    'MAX6675'  : 8,
}

class SensorBase(object):
    min_temp = max_temp = .0
    min_sample_value = max_sample_value = 0
    def __init__(self, config,
                 sample_time  = SAMPLE_TIME_DEFAULT,
                 sample_count = SAMPLE_COUNT_DEFAULT,
                 report_time  = REPORT_TIME_DEFAULT,
                 chip_type=None, config_cmd=None):
        self.printer = config.get_printer()
        self.oid = None
        self.sample_time = sample_time
        self.sample_count = sample_count
        self.report_time = report_time
        self._callback = self.__default_callback
        self._report_clock = 0
        if chip_type in VALID_SPI_SENSORS:
            # SPI configuration
            spi = bus.MCU_SPI_from_config(
                config, 1, pin_option="sensor_pin", default_speed=4000000)
            if config_cmd is not None:
                spi.spi_send(config_cmd)
            self.mcu = mcu = spi.get_mcu()
            # Reader chip configuration
            self.oid = oid = mcu.create_oid()
            mcu.add_config_cmd(
                "config_thermocouple oid=%u spi_oid=%u chip_type=%u" % (
                    oid, spi.get_oid(), VALID_SPI_SENSORS[chip_type]))
            mcu.register_msg(self._handle_thermocouple_result,
                "thermocouple_result", oid)
            mcu.register_config_callback(self._build_config_cb)
        else:
            ppins = self.printer.lookup_object('pins')
            self.mcu = ppins.setup_pin('adc', config.get('sensor_pin'))
            self.mcu.setup_callback(
                self.report_time, self._handle_adc_result)
        min_temp = config.getfloat('min_temp', minval=0., default=0.)
        max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.__setup_minmax(min_temp, max_temp)
    def fault(self, msg):
        self.printer.invoke_async_shutdown(msg)
    def get_mcu(self):
        if self.oid is not None:
            return self.mcu
        return self.mcu.get_mcu()
    def setup_minmax(self, min_temp, max_temp):
        # Set heaters min and max temperatures
        if min_temp < self.min_temp:
            raise self.printer.config_error("Min temp below sensor's lowest")
        if max_temp > self.max_temp:
            raise self.printer.config_error("Max temp over sensor's lowest")
        self.__setup_minmax(min_temp, max_temp)
    def __setup_minmax(self, min_temp, max_temp):
        self.min_temp = min_temp
        self.max_temp = max_temp
        adc_range = [self.calc_adc(min_temp), self.calc_adc(max_temp)]
        self.min_sample_value = min(adc_range)
        self.max_sample_value = max(adc_range)
        if hasattr(self.mcu, "setup_minmax"):
            self.mcu.setup_minmax(
                self.sample_time, self.sample_count,
                minval=min(adc_range), maxval=max(adc_range))
    def get_min_max_temp(self):
        return self.min_temp, self.max_temp
    def setup_callback(self, cb):
        self._callback = cb
    def get_report_delta(self):
        return self.report_time
    # ============ INTERNAL ===============
    def _build_config_cb(self):
        clock = self.mcu.get_query_slot(self.oid)
        self._report_clock = self.mcu.seconds_to_clock(self.report_time)
        self.mcu.add_config_cmd(
            "query_thermocouple oid=%u clock=%u rest_ticks=%u"
            " min_value=%u max_value=%u" % (
                self.oid, clock, self._report_clock,
                self.min_sample_value, self.max_sample_value), is_init=True)
    def _handle_thermocouple_result(self, params):
        temp = self.calc_temp(params['value'], params['fault'])
        next_clock      = self.mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self._report_clock
        last_read_time  = self.mcu.clock_to_print_time(last_read_clock)
        self._callback(last_read_time, temp)
    def _handle_adc_result(self, last_read_time, adc_val):
        self._callback(last_read_time, self.calc_temp(adc_val))
    # ============ VIRTUAL ===============
    def calc_temp(self, read_value, fault=0):
        raise NotImplementedError("calc_temp must to be implemented in parent class")
    def calc_adc(self, temp):
        raise NotImplementedError("calc_adc must to be implemented in parent class")
    def __default_callback(self, arg1, arg2):
        raise NotImplementedError("Temp comtrol callback is not set!")
