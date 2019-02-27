# TMC2208 UART communication and configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math

######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

# Decode two's complement signed integer
def decode_signed_int(val, bits):
    if (val >> (bits - 1)) & 1:
        return val - (1 << bits)
    return val

class FieldHelper:
    def __init__(self, all_fields, field_formatters={}, registers=None):
        self.all_fields = all_fields
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = {}
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def set_reg_value(self, reg_name, new_value):
        self.registers[reg_name] = new_value
    def get_reg_value(self, reg_name):
        return self.registers.get(reg_name, 0)
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers[reg_name]
        mask = self.all_fields[reg_name][field_name]
        return (reg_value & mask) >> ffs(mask)
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default, config_name=None):
        # Allow a field to be set from the config file
        if config_name is None:
            config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            fval = (value & mask) >> ffs(mask)
            sval = self.field_formatters.get(field_name, str)(fval)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", value, "".join(fields))

######################################################################
# Config reading helpers
######################################################################

def current_bits(current, sense_resistor, vsense_on):
    sense_resistor += 0.020
    vsense = 0.32
    if vsense_on:
        vsense = 0.18
    cs = int(32. * current * sense_resistor * math.sqrt(2.) / vsense - 1. + .5)
    return max(0, min(31, cs))

def get_config_current(config):
    vsense = False
    run_current = config.getfloat('run_current', above=0., maxval=2.)
    hold_current = config.getfloat('hold_current', run_current,
                                   above=0., maxval=2.)
    sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
    irun = current_bits(run_current, sense_resistor, vsense)
    ihold = current_bits(hold_current, sense_resistor, vsense)
    if irun < 16 and ihold < 16:
        vsense = True
        irun = current_bits(run_current, sense_resistor, vsense)
        ihold = current_bits(hold_current, sense_resistor, vsense)
    return vsense, irun, ihold

def get_config_microsteps(config):
    steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
             '8': 5, '4': 6, '2': 7, '1': 8}
    return config.getchoice('microsteps', steps)

def get_config_stealthchop(config, tmc_freq):
    mres = get_config_microsteps(config)
    velocity = config.getfloat('stealthchop_threshold', 0., minval=0.)
    if not velocity:
        return mres, False, 0
    stepper_name = " ".join(config.get_name().split()[1:])
    stepper_config = config.getsection(stepper_name)
    step_dist = stepper_config.getfloat('step_distance')
    step_dist_256 = step_dist / (1 << mres)
    threshold = int(tmc_freq * step_dist_256 / velocity + .5)
    return mres, True, max(0, min(0xfffff, threshold))
