# Stepper driver control base classes
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class DriverBase(object):
    __inv_step_dist = __step_dist = microsteps = None
    def __init__(self, config, has_step_dir_pins=True, has_endstop=False):
        printer = config.get_printer()
        self.name = name = config.get_name()[7:]
        self.logger = printer.logger.getChild("driver.%s" % name)
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
        printer = config.get_printer()
        # ========== SPI config ==========
        cs_pin = config.get('ss_pin')
        spi_mode = config.getint('spi_mode', 3, minval=0, maxval=3)
        spi_speed = config.getint('spi_speed', 2000000)
        spi_bus = config.getint('spi_bus', 0)
        # setup SPI pins and configure mcu
        ppins = printer.lookup_object('pins')
        cs_pin_params = ppins.lookup_pin('digital_out', cs_pin)
        if cs_pin_params['invert']:
            raise config.error("Cannot invert pin")
        self.mcu = mcu = cs_pin_params['chip']
        self._oid = oid = mcu.create_oid()
        mcu.add_config_cmd(
            "config_spi oid=%d bus=%d pin=%s inverted=%u"
            " mode=%u rate=%u shutdown_msg=" % (
                oid, spi_bus, cs_pin_params['pin'],
                cs_pin_params['invert'], spi_mode, spi_speed))
        self._transfer = (lambda *args: 0)
        self.transfer_cmd = None
        mcu.add_config_object(self)
    # ============ SETUP ===============
    def printer_state(self, state):
        if state == 'shutdown':
            pass
        elif state == 'ready':
            if not self.mcu.is_shutdown():
                self._init_driver()
        elif state == 'connect':
            pass
        elif state == 'disconnect':
            pass
    def build_config(self):
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
        raise NotImplementedError("This need to be implemented in parent class")
    def _build_config(self):
        pass