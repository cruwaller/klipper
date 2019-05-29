# Printer heater support
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging
from sensorbase import SensorBase

KELVIN_TO_CELCIUS = -273.15

# Analog voltage to temperature converter for thermistors
class Thermistor(SensorBase):
    def __init__(self, config, params):
        self.name = config.get_name()
        self.pullup = config.getfloat('pullup_resistor', 4700., above=0.)
        self.inline_resistor = config.getfloat('inline_resistor', 0., minval=0.)
        self.c1 = self.c2 = self.c3 = 0.
        if 'beta' in params:
            self.setup_coefficients_beta(params['t1'], params['r1'], params['beta'])
        else:
            self.setup_coefficients(params['t1'], params['r1'],
                params['t2'], params['r2'], params['t3'], params['r3'], self.name)
        SensorBase.__init__(self, config)
    def setup_coefficients(self, t1, r1, t2, r2, t3, r3, name=""):
        # Calculate Steinhart-Hart coefficents from temp measurements.
        # Arrange samples as 3 linear equations and solve for c1, c2, and c3.
        inv_t1 = 1. / (t1 - KELVIN_TO_CELCIUS)
        inv_t2 = 1. / (t2 - KELVIN_TO_CELCIUS)
        inv_t3 = 1. / (t3 - KELVIN_TO_CELCIUS)
        ln_r1 = math.log(r1)
        ln_r2 = math.log(r2)
        ln_r3 = math.log(r3)
        ln3_r1, ln3_r2, ln3_r3 = ln_r1**3, ln_r2**3, ln_r3**3

        inv_t12, inv_t13 = inv_t1 - inv_t2, inv_t1 - inv_t3
        ln_r12, ln_r13 = ln_r1 - ln_r2, ln_r1 - ln_r3
        ln3_r12, ln3_r13 = ln3_r1 - ln3_r2, ln3_r1 - ln3_r3

        self.c3 = ((inv_t12 - inv_t13 * ln_r12 / ln_r13)
                   / (ln3_r12 - ln3_r13 * ln_r12 / ln_r13))
        if self.c3 <= 0.:
            beta = ln_r13 / inv_t13
            logging.warn("Using thermistor beta %.3f in heater %s", beta, name)
            self.setup_coefficients_beta(t1, r1, beta)
            return
        self.c2 = (inv_t12 - self.c3 * ln3_r12) / ln_r12
        self.c1 = inv_t1 - self.c2 * ln_r1 - self.c3 * ln3_r1
    def setup_coefficients_beta(self, t1, r1, beta):
        # Calculate equivalent Steinhart-Hart coefficents from beta
        inv_t1 = 1. / (t1 - KELVIN_TO_CELCIUS)
        ln_r1 = math.log(r1)
        self.c3 = 0.
        self.c2 = 1. / beta
        self.c1 = inv_t1 - self.c2 * ln_r1
    def calc_temp(self, adc, fault=0):
        # Calculate temperature from adc
        adc = max(.00001, min(.99999, adc))
        r = self.pullup * adc / (1.0 - adc)
        ln_r = math.log(r - self.inline_resistor)
        inv_t = self.c1 + self.c2 * ln_r + self.c3 * ln_r**3
        return 1.0/inv_t + KELVIN_TO_CELCIUS
    def calc_adc(self, temp):
        # Calculate adc reading from a temperature
        if temp <= KELVIN_TO_CELCIUS:
            return 1.
        inv_t = 1. / (temp - KELVIN_TO_CELCIUS)
        if self.c3:
            # Solve for ln_r using Cardano's formula
            y = (self.c1 - inv_t) / (2. * self.c3)
            x = math.sqrt((self.c2 / (3. * self.c3))**3 + y**2)
            ln_r = math.pow(x - y, 1./3.) - math.pow(x + y, 1./3.)
        else:
            ln_r = (inv_t - self.c1) / self.c2
        r = math.exp(ln_r) + self.inline_resistor
        return r / (self.pullup + r)

# Custom defined thermistors from the config file
class CustomThermistor(Thermistor):
    def __init__(self, config, params):
        self.name = " ".join(config.get_name().split()[1:])
        t1 = config.getfloat("temperature1", minval=KELVIN_TO_CELCIUS)
        r1 = config.getfloat("resistance1", minval=0.)
        beta = config.getfloat("beta", None, above=0.)
        if beta is not None:
            self.params = {'t1': t1, 'r1': r1, 'beta': beta}
        else:
            t2 = config.getfloat("temperature2", minval=KELVIN_TO_CELCIUS)
            r2 = config.getfloat("resistance2", minval=0.)
            t3 = config.getfloat("temperature3", minval=KELVIN_TO_CELCIUS)
            r3 = config.getfloat("resistance3", minval=0.)
            (t1, r1), (t2, r2), (t3, r3) = sorted([(t1, r1), (t2, r2), (t3, r3)])
            self.params = {'t1': t1, 'r1': r1, 't2': t2, 'r2': r2,
                           't3': t3, 'r3': r3}
        Thermistor.__init__(self, config, self.params)
