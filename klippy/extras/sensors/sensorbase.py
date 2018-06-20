# Printer heater support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

SAMPLE_TIME_DEFAULT    = 0.001
SAMPLE_COUNT_DEFAULT   = 8
REPORT_TIME_DEFAULT    = 0.300

VALID_SPI_SENSORS = {
    'MAX6675' : 1, 'MAX31855' : 1,
    'MAX31856' : 2,
    'MAX31865' : 4
}

class error(Exception):
    pass

class SensorBase(object):
    error = error
    def __init__(self,
                 config,
                 sample_time  = SAMPLE_TIME_DEFAULT,
                 sample_count = SAMPLE_COUNT_DEFAULT,
                 report_time  = REPORT_TIME_DEFAULT,
                 chip_type    = None):
        if chip_type:
            self.logger = config.get_printer().logger.getChild(chip_type)
        else:
            self.logger = config.get_printer().logger.getChild("sensor")
        self.oid = None
        self.sample_time = sample_time
        self.sample_count = sample_count
        self.report_time = report_time
        self.min_temp = config.getfloat('min_temp', minval=0., default=0.)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self._callback = None
        sensor_pin = config.get('sensor_pin')
        adc_range = [self.calc_adc(self.min_temp),
                     self.calc_adc(self.max_temp)]
        self.min_sample_value = min(adc_range)
        self.max_sample_value = max(adc_range)
        self._report_clock = 0
        ppins = config.get_printer().lookup_object('pins')
        if chip_type is not None:
            pin_params = ppins.lookup_pin('digital_out', sensor_pin)
            self.mcu = mcu = pin_params['chip']
            pin = pin_params['pin']
            # SPI bus configuration
            spi_oid = mcu.create_oid()
            spi_mode = config.getint('spi_mode', minval=0, maxval=3)
            spi_speed = config.getint('spi_speed', minval=0)
            mcu.add_config_cmd(
                "config_spi oid=%u bus=%u pin=%s inverted=%u"
                " mode=%u rate=%u shutdown_msg=" % (
                    spi_oid, 0, pin, pin_params['invert'],
                    spi_mode, spi_speed))
            config_cmd = "".join("%02x" % b for b in self.get_configs())
            mcu.add_config_cmd("spi_send oid=%u data=%s" % (
                spi_oid, config_cmd), is_init=False)
            # Reader chip configuration
            self.oid = oid = mcu.create_oid()
            mcu.add_config_cmd(
                "config_thermocouple oid=%u spi_oid=%u chip_type=%u" % (
                    oid, spi_oid, VALID_SPI_SENSORS[chip_type]))
            mcu.register_msg(self._handle_thermocouple_result,
                "thermocouple_result", oid)
            mcu.add_config_object(self)
        else:
            self.mcu = ppins.setup_pin('adc', sensor_pin)
            self.mcu.setup_minmax(
                sample_time, sample_count,
                minval=min(adc_range), maxval=max(adc_range))
    def get_mcu(self):
        if self.oid is not None:
            return self.mcu
        return self.mcu.get_mcu()
    def get_min_max_temp(self):
        return self.min_temp, self.max_temp
    def setup_callback(self, cb):
        if self.oid is not None:
            self._callback = cb
        else:
            self.mcu.setup_callback(self.report_time, cb)
    def get_report_delta(self):
        return self.report_time
    def build_config(self):
        clock = self.mcu.get_query_slot(self.oid)
        self._report_clock = self.mcu.seconds_to_clock(self.report_time)
        self.mcu.add_config_cmd(
            "query_thermocouple oid=%u clock=%u rest_ticks=%u"
            " min_value=%u max_value=%u" % (
                self.oid, clock, self._report_clock,
                self.min_sample_value, self.max_sample_value))
    def _handle_thermocouple_result(self, params):
        #self.logger.debug("value: %s [0x%08X], fault: %s" % (
        #    params['value'], int(params['value']), params['fault']))
        last_value      = params['value']
        next_clock      = self.mcu.clock32_to_clock64(params['next_clock'])
        last_read_clock = next_clock - self._report_clock
        last_read_time  = self.mcu.clock_to_print_time(last_read_clock)
        self.check_faults(params['fault'])
        if self._callback is not None:
            self._callback(last_read_time, last_value)
    # ============ VIRTUAL ===============
    def check_faults(self, fault):
        pass
    def calc_adc(self, min_temp):
        raise NotImplementedError("calc_adc must to be implemented in parent class")
    def get_configs(self):
        raise NotImplementedError("calc_adc must to be implemented in parent class")
