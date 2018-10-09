# Printer heater support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
from sensorbase import SensorBase

######################################################################
# MAX31865 (RTD sensor)
######################################################################
MAX31865_CONFIG_REG            = 0x00
MAX31865_RTDMSB_REG            = 0x01
MAX31865_RTDLSB_REG            = 0x02
MAX31865_HFAULTMSB_REG         = 0x03
MAX31865_HFAULTLSB_REG         = 0x04
MAX31865_LFAULTMSB_REG         = 0x05
MAX31865_LFAULTLSB_REG         = 0x06
MAX31865_FAULTSTAT_REG         = 0x07

MAX31865_CONFIG_BIAS           = 0x80
MAX31865_CONFIG_MODEAUTO       = 0x40
MAX31865_CONFIG_1SHOT          = 0x20
MAX31865_CONFIG_3WIRE          = 0x10
MAX31865_CONFIG_FAULTCLEAR     = 0x02
MAX31865_CONFIG_FILT50HZ       = 0x01

MAX31865_FAULT_HIGHTHRESH      = 0x80
MAX31865_FAULT_LOWTHRESH       = 0x40
MAX31865_FAULT_REFINLOW        = 0x20
MAX31865_FAULT_REFINHIGH       = 0x10
MAX31865_FAULT_RTDINLOW        = 0x08
MAX31865_FAULT_OVUV            = 0x04

VAL_A = 0.00390830
VAL_B = 0.0000005775
VAL_C = -0.00000000000418301
VAL_ADC_MAX = 32768.0 # 2^15

class MAX31865(SensorBase):
    def __init__(self, config, params):
        self.rtd_nominal_r = config.getint('rtd_nominal_r', 100)
        self.reference_r = config.getfloat('rtd_reference_r', 430., above=0.)
        SensorBase.__init__(self, config, sample_count=1,
                            chip_type="MAX31865",
                            config_cmd=self.build_spi_init(config))
    def calc_temp(self, adc, fault=0):
        if fault & 0x80:
            self.fault("MAX31865 RTD input is disconnected")
        if fault & 0x40:
            self.fault("MAX31865 RTD input is shorted")
        if fault & 0x20:
            self.fault("MAX31865 VREF- is greater than 0.85 * VBIAS, FORCE- open")
        if fault & 0x10:
            self.fault("MAX31865 VREF- is less than 0.85 * VBIAS, FORCE- open")
        if fault & 0x08:
            self.fault("MAX31865 VRTD- is less than 0.85 * VBIAS, FORCE- open")
        if fault & 0x04:
            self.fault("MAX31865 Overvoltage or undervoltage fault")
        if fault & 0x03:
            self.fault("MAX31865 Unspecified error")
        rtd_nominal_r = self.rtd_nominal_r
        adc = adc >> 1 # remove fault bit
        R_rtd = (self.reference_r * adc) / VAL_ADC_MAX
        temp = (
            (( ( -1 * rtd_nominal_r ) * VAL_A ) +
             math.sqrt( ( rtd_nominal_r * rtd_nominal_r * VAL_A * VAL_A ) -
                        ( 4 * rtd_nominal_r * VAL_B * ( rtd_nominal_r - R_rtd ) )))
            / (2 * rtd_nominal_r * VAL_B))
        return temp
    def calc_adc(self, temp):
        rtd_nominal_r = self.rtd_nominal_r
        R_rtd = temp * ( 2 * rtd_nominal_r * VAL_B )
        R_rtd = math.pow( ( R_rtd + ( rtd_nominal_r * VAL_A ) ), 2)
        R_rtd = -1 * ( R_rtd - ( rtd_nominal_r * rtd_nominal_r * VAL_A * VAL_A ) )
        R_rtd = R_rtd / ( 4 * rtd_nominal_r * VAL_B )
        R_rtd = ( -1 * R_rtd ) + rtd_nominal_r
        adc = int( ( ( R_rtd * VAL_ADC_MAX ) / self.reference_r) + 0.5 )
        adc = adc << 1 # Add fault bit
        return adc
    def build_spi_init(self, config):
        value = (MAX31865_CONFIG_BIAS |
                 MAX31865_CONFIG_MODEAUTO |
                 MAX31865_CONFIG_FAULTCLEAR)
        if config.getboolean('rtd_use_50Hz_filter', False):
            value |= MAX31865_CONFIG_FILT50HZ
        if config.getint('rtd_num_of_wires', 2) == 3:
            value |= MAX31865_CONFIG_3WIRE
        cmd = 0x80 + MAX31865_CONFIG_REG
        return [cmd, value]
