# Stepper driver control base classes
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import extras.bus as bus

######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

# Decode two's complement signed integer
def decode_signed_int(val, bits):
    if ((val >> (bits - 1)) & 1):
        return val - (1 << bits)
    return val

class FieldHelper:
    def __init__(self, all_fields, field_formatters={}):
        self.all_fields = all_fields
        self.field_formatters = field_formatters
    def get_field(self, reg_name, field_name, reg_value):
        # Returns value of the register field
        mask = self.all_fields.get(reg_name, {})[field_name]
        return (reg_value & mask) >> ffs(mask)
    def set_field(self, reg_name, field_name, reg_value, field_value):
        # Returns register value with field bits filled with supplied value
        mask = self.all_fields.get(reg_name, {})[field_name]
        return (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
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
# Driver base handlers
######################################################################

class DriverBase(object):
    __inv_step_dist = __step_dist = microsteps = None
    def __init__(self, config, has_step_dir_pins=True, has_endstop=False):
        self.printer = config.get_printer()
        self.name = name = config.get_name()[7:]
        self.logger = self.printer.logger.getChild("driver.%s" % name)
        self.microsteps = config.getint('microsteps', None)
        # Driver class can override existing stepper config steps
        self.step_dist = config.getfloat('step_distance', default=None, above=0.)
        self.inv_step_dist = config.getfloat('steps_per_mm', default=None, above=0.)
        self.__has_step_dir_pins = has_step_dir_pins
        self.__has_endstop = has_endstop
    @property
    def inv_step_dist(self):
        return self.__inv_step_dist
    @inv_step_dist.setter
    def inv_step_dist(self, dist):
        if dist is not None:
            self.__inv_step_dist = dist
            self.__step_dist = 1. / dist
    @property
    def step_dist(self):
        return self.__step_dist
    @step_dist.setter
    def step_dist(self, dist):
        if dist is not None:
            self.__step_dist = dist
            self.__inv_step_dist = 1. / dist
    def get_step_dist(self):
        return self.__step_dist
    @property
    def has_step_dir_pins(self):
        return self.__has_step_dir_pins
    @property
    def has_endstop(self):
        return self.__has_endstop
    def setup_step_distance(self, step_dist): # needed?
        self.step_dist = step_dist


class SpiDriver(DriverBase):
    def __init__(self, config, has_step_dir_pins=True, has_endstop=False):
        DriverBase.__init__(self, config, has_step_dir_pins, has_endstop)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # ========== SPI config ==========
        spi = bus.MCU_SPI_from_config(
            config, 3, pin_option="ss_pin", default_speed=2000000)
        self.mcu = mcu = spi.get_mcu()
        self._oid = spi.get_oid()
        mcu.register_config_callback(self._build_config_cb)
        self._transfer = (lambda *args: 0)
        self.transfer_cmd = None
    # ============ SETUP ===============
    def handle_ready(self):
        if not self.mcu.is_shutdown():
            self._init_driver() # TODO: change this!
    def _build_config_cb(self):
        self.transfer_cmd = self.mcu.lookup_command(
            "spi_transfer oid=%c data=%*s")
        self._transfer = self.__transfer
        self._build_config()
    def get_mcu(self):
        return self.mcu
    def get_oid(self):
        return self._oid
    # ============ REGISTER HANDLING ===============
    def modify_reg(self, defs, reg, val, addr=None):
        send = False
        for offset, mask, _map in defs:
            try:
                val = _map[val]
            except (KeyError, TypeError):
                pass
            current_val = ((reg >> offset) & mask)
            if (current_val ^ (val & mask)) != 0:
                reg &= ~(mask << offset)
                reg |= ((val & mask) << offset)
                send = True
        if send and addr is not None:
            self._command_write(addr, reg)
        return reg
    def read_reg(self, defs, reg):
        val = 0
        for offset, mask, _map in defs:
            val += ((reg >> offset) & mask)
        return val
    # ============ SPI ===============
    def __transfer(self, cmd):
        params = self.transfer_cmd.send_with_response(
            [self._oid, cmd], response='spi_transfer_response', response_oid=self._oid)
        return list(bytearray(params['response']))
    # ============ VIRTUAL ===============
    def _command_read(self, *args, **kwargs):
        raise NotImplementedError("This need to be implemented in parent class")
    def _command_write(self, *args, **kwargs):
        raise NotImplementedError("This need to be implemented in parent class")
    def _init_driver(self):
        #raise NotImplementedError("This need to be implemented in parent class")
        pass
    def _build_config(self):
        pass
