# Printer heater support
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
from sensorbase import SensorBase

MAX31856_CR0_REG           = 0x00
MAX31856_CR0_AUTOCONVERT   = 0x80
MAX31856_CR0_1SHOT         = 0x40
MAX31856_CR0_OCFAULT1      = 0x20
MAX31856_CR0_OCFAULT0      = 0x10
MAX31856_CR0_CJ            = 0x08
MAX31856_CR0_FAULT         = 0x04
MAX31856_CR0_FAULTCLR      = 0x02
MAX31856_CR0_FILT50HZ      = 0x01
MAX31856_CR0_FILT60HZ      = 0x00

MAX31856_CR1_REG           = 0x01
MAX31856_CR1_AVGSEL1       = 0x00
MAX31856_CR1_AVGSEL2       = 0x10
MAX31856_CR1_AVGSEL4       = 0x20
MAX31856_CR1_AVGSEL8       = 0x30
MAX31856_CR1_AVGSEL16      = 0x70

MAX31856_MASK_REG                          = 0x02
MAX31856_MASK_COLD_JUNCTION_HIGH_FAULT     = 0x20
MAX31856_MASK_COLD_JUNCTION_LOW_FAULT      = 0x10
MAX31856_MASK_THERMOCOUPLE_HIGH_FAULT      = 0x08
MAX31856_MASK_THERMOCOUPLE_LOW_FAULT       = 0x04
MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT     = 0x02
MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT      = 0x01

MAX31856_CJHF_REG          = 0x03
MAX31856_CJLF_REG          = 0x04
MAX31856_LTHFTH_REG        = 0x05
MAX31856_LTHFTL_REG        = 0x06
MAX31856_LTLFTH_REG        = 0x07
MAX31856_LTLFTL_REG        = 0x08
MAX31856_CJTO_REG          = 0x09
MAX31856_CJTH_REG          = 0x0A
MAX31856_CJTL_REG          = 0x0B
MAX31856_LTCBH_REG         = 0x0C
MAX31856_LTCBM_REG         = 0x0D
MAX31856_LTCBL_REG         = 0x0E

MAX31856_SR_REG            = 0x0F
MAX31856_FAULT_CJRANGE     = 0x80  # Cold Junction out of range
MAX31856_FAULT_TCRANGE     = 0x40  # Thermocouple out of range
MAX31856_FAULT_CJHIGH      = 0x20  # Cold Junction High
MAX31856_FAULT_CJLOW       = 0x10  # Cold Junction Low
MAX31856_FAULT_TCHIGH      = 0x08  # Thermocouple Low
MAX31856_FAULT_TCLOW       = 0x04  # Thermocouple Low
MAX31856_FAULT_OVUV        = 0x02  # Under Over Voltage
MAX31856_FAULT_OPEN        = 0x01

class Thermocouple(SensorBase):
    tc_type         = 100;
    use_50Hz_filter = False;
    average_count   = 1;

    types = {
        "B" : 0b0000,
        "E" : 0b0001,
        "J" : 0b0010,
        "K" : 0b0011,
        "N" : 0b0100,
        "R" : 0b0101,
        "S" : 0b0110,
        "T" : 0b0111,
    }

    averages = {
        1  : MAX31856_CR1_AVGSEL1,
        2  : MAX31856_CR1_AVGSEL2,
        4  : MAX31856_CR1_AVGSEL4,
        8  : MAX31856_CR1_AVGSEL8,
        16 : MAX31856_CR1_AVGSEL16
    }

    is_k_simple = False # Check faults

    def __init__(self, config, params):
        self.tc_type         = types[config.get('tc_type', "K")]
        self.use_50Hz_filter = config.getboolean('tc_use_50Hz_filter', False)
        self.average_count   = averages[config.getint('tc_averaging_count', 1)]
        self.is_k_simple     = params["simple"]

        if (self.is_k_simple): # MAX6675/MAX31855
            self.val_a = 0.25
            self.scale = 18
        else:
            self.val_a = 0.0078125
            self.scale = 5
        SensorBase.__init__(self, config, is_spi = True, sample_count = 1)

    def _check_faults_simple(self, val):
        if self.is_k_simple:
            if (val & 0x1):
                raise error("MAX6675/MAX31855 : Open Circuit")
            if (val & 0x2):
                raise error("MAX6675/MAX31855 : Short to GND")
            if (val & 0x4):
                raise error("MAX6675/MAX31855 : Short to Vcc")
        else:
            if (val & 0x1):
                pass

    def check_faults(self, fault):
        if self.is_k_simple == False:
            if (fault & MAX31856_FAULT_CJRANGE):
                raise error("Max31856: Cold Junction Range Fault")
            if (fault & MAX31856_FAULT_TCRANGE):
                raise error("Max31856: Thermocouple Range Fault")
            if (fault & MAX31856_FAULT_CJHIGH):
                raise error("Max31856: Cold Junction High Fault")
            if (fault & MAX31856_FAULT_CJLOW):
                raise error("Max31856: Cold Junction Low Fault")
            if (fault & MAX31856_FAULT_TCHIGH):
                raise error("Max31856: Thermocouple High Fault")
            if (fault & MAX31856_FAULT_TCLOW):
                raise error("Max31856: Thermocouple Low Fault")
            if (fault & MAX31856_FAULT_OVUV):
                raise error("Max31856: Over/Under Voltage Fault")
            if (fault & MAX31856_FAULT_OPEN):
                raise error("Max31856: Thermocouple Open Fault")

    def calc_temp(self, adc):
        if self.is_k_simple:
            self._check_faults_simple(adc)
        adc = adc >> self.scale
        # Fix sign bit:
        if self.is_k_simple:
            if (adc & 0x2000):
                adc = ((adc & 0x1FFF) + 1) * -1
        else:
            if (adc & 0x40000):
                adc = ((adc & 0x3FFFF) + 1) * -1
        temp = self.val_a * adc;
        return (temp);

    def calc_adc(self, temp):
        adc = int ( ( temp / self.val_a ) + 0.5 ) # convert to ADC value
        adc = adc << self.scale
        return adc

    def get_read_cmd(self):
        if self.is_k_simple == False:
            return MAX31856_LTCBH_REG
        return 0x00
    def get_read_bytes(self):
        if self.is_k_simple == False:
            return 3 # 24bit value (MAX31856)
        return 4 # 32bit (MAX6675 / MAX31855)
    def get_configs(self):
        cmds = []
        if self.is_k_simple == False:
            value = MAX31856_CR0_AUTOCONVERT
            if (self.use_50Hz_filter):
                value |= MAX31856_CR0_FILT50HZ
            if (self.num_wires == 3):
                value |= MAX31865_CONFIG_3WIRE
            cmds.append(0x80 + MAX31856_CR0_REG)
            cmds.append(value)

            value  = self.tc_type
            value |= self.average_count
            cmds.append(0x80 + MAX31856_CR1_REG)
            cmds.append(value)

            value = (MAX31856_MASK_VOLTAGE_UNDER_OVER_FAULT |
                     MAX31856_MASK_THERMOCOUPLE_OPEN_FAULT)
            cmds.append(0x80 + MAX31856_MASK_REG)
            cmds.append(value)
        return cmds
    def get_fault_filter(self):
        if self.is_k_simple:
            return 0x4;
        return 0;
