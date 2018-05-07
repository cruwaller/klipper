# Printer heater support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from sensorbase import SensorBase

# Thermistor calibrated with three temp measurements
class ThermistorDummy(SensorBase):
    def __init__(self, config, params):
        self.temperature = params['t1']
        SensorBase.__init__(self, config)
    def calc_temp(self, adc):
        return self.temperature
    def calc_adc(self, temp):
        return 0 # TODO FIXME calc ADC value!
    def check_faults(self, fault):
        pass
