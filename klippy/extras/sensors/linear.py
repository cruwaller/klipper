# Obtain temperature using linear interpolation of ADC values
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, bisect
from sensorbase import SensorBase

# Linear style conversion chips calibrated with two temp measurements
class Linear(SensorBase):
    def __init__(self, config, params):
        self.adc_samples = []
        self.slope_samples = []
        adc_voltage = config.getfloat('adc_voltage', 5., above=0.)
        last_volt = last_temp = None
        for volt, temp in sorted([(v, t) for t, v in params['values']]):
            adc = volt / adc_voltage
            if adc < 0. or adc > 1.:
                logging.warn("Ignoring adc sample %.3f/%.3f in heater %s",
                             temp, volt, config.get_name())
                continue
            if last_volt is None:
                last_volt = volt
                last_temp = temp
                continue
            if volt <= last_volt:
                raise config.error("adc_temperature duplicate voltage")
            slope = (temp - last_temp) / (volt - last_volt)
            gain = adc_voltage * slope
            offset = last_temp - last_volt * slope
            if self.slope_samples and self.slope_samples[-1] == (gain, offset):
                continue
            last_temp = temp
            last_volt = volt
            self.adc_samples.append(adc)
            self.slope_samples.append((gain, offset))
        if not self.adc_samples:
            raise config.error(
                "adc_temperature needs two volt and temperature measurements")
        self.adc_samples[-1] = 1.
        SensorBase.__init__(self, config)
    def calc_temp(self, read_value):
        pos = bisect.bisect(self.adc_samples, read_value)
        gain, offset = self.slope_samples[pos]
        temp = read_value * gain + offset
        return temp
    def calc_adc(self, temp):
        temps = [adc * gain + offset for adc, (gain, offset) in zip(
            self.adc_samples, self.slope_samples)]
        if temps[0] < temps[-1]:
            pos = min([i for i in range(len(temps)) if temps[i] >= temp]
                      + [len(temps) - 1])
        else:
            pos = min([i for i in range(len(temps)) if temps[i] <= temp]
                      + [len(temps) - 1])
        gain, offset = self.slope_samples[pos]
        return (temp - offset) / gain

# Custom defined sensors from the config file
class CustomLinear(Linear):
    def __init__(self, config, params):
        self.name = " ".join(config.get_name().split()[1:])
        self.params = []
        for i in range(1, 1000):
            t = config.getfloat("temperature%d" % (i,), None)
            if t is None:
                break
            v = config.getfloat("voltage%d" % (i,))
            self.params.append((t, v))
        Linear.__init__(self, config, {'values': self.params})
