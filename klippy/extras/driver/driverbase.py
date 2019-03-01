# Stepper driver control base classes
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import extras.bus as bus

######################################################################
# Driver base handlers
######################################################################

class DriverBase(object):
    __inv_step_dist = __step_dist = microsteps = None
    def __init__(self, config, stepper_config,
                 has_step_dir_pins=True, has_endstop=False):
        self.__has_step_dir_pins = has_step_dir_pins
        self.__has_endstop = has_endstop
        self.printer = config.get_printer()
        self.name = name = config.get_name().split()[-1]
        self.logger = self.printer.logger.getChild("driver.%s" % name)
        microsteps = stepper_config.getfloat('microsteps',
            default=None, above=0.)
        self.microsteps = config.getint('microsteps',
            default=microsteps, above=0.)
        # Driver class can override existing stepper config steps
        step_dist = stepper_config.getfloat('step_distance',
            default=None, above=0.)
        self.step_dist = config.getfloat('step_distance',
            default=step_dist, above=0.)
        inv_step_dist = stepper_config.getfloat('steps_per_mm',
            default=None, above=0.)
        self.inv_step_dist = config.getfloat('steps_per_mm',
            default=inv_step_dist, above=0.)
        if self.step_dist is None or self.inv_step_dist is None:
            if self.microsteps is None:
                raise config.error('Cannot detect proper step distance!!')
            self.calculate_steps(stepper_config)
    def calculate_steps(self, config):
        motor_deg = config.getfloat('motor_step_angle', above=0.)
        # Calculate base on settings
        pitch = config.getfloat('pitch', above=0.)
        teeth = config.getfloat('teeths', above=0.)
        ratio = config.getfloat('gear_ratio', above=0., default=1.0)
        motor_rev = 360. / motor_deg
        self.inv_step_dist = \
            motor_rev * self.microsteps / (pitch * teeth) * ratio
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
    def __init__(self, config, stepper_config,
                 has_step_dir_pins=True, has_endstop=False):
        DriverBase.__init__(self, config, stepper_config,
            has_step_dir_pins, has_endstop)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        # ========== SPI config ==========
        self.spi = spi = bus.MCU_SPI_from_config(
            config, 3, pin_option="ss_pin", default_speed=2000000)
        self.mcu = mcu = spi.get_mcu()
        self._oid = spi.get_oid()
        mcu.register_config_callback(self._build_config_cb)
    # ============ SETUP ===============
    def handle_ready(self):
        if not self.mcu.is_shutdown():
            self._init_driver()
    def _build_config_cb(self):
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
    def spi_send(self, data):
        self.spi.spi_send(data)
    def spi_transfer(self, data):
        params = self.spi.spi_transfer(data)
        return list(bytearray(params['response']))
    # ============ VIRTUAL ===============
    def _command_read(self, *args, **kwargs):
        raise NotImplementedError("This need to be implemented in parent class")
    def _command_write(self, *args, **kwargs):
        raise NotImplementedError("This need to be implemented in parent class")
    def _init_driver(self):
        pass
    def _build_config(self):
        pass
