# Stepper driver control base classes
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import extras.bus as bus
from mcu import error
import tmc
import binascii, types, struct, math, collections

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
        self.name = logger_name = config.get_name().split()[-1]
        stepper_name = stepper_config.get_name()
        if logger_name == stepper_name:
            logger_name = 'driver'
        self.logger = self.printer.get_logger("%s.%s" % (
            stepper_name, logger_name))
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
        self.logger.debug("Driver '%s' base loaded", self.name)
        self.logger.debug("step_dist:%s, inv_step_dist:%s" % (self.step_dist,
            self.inv_step_dist))
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


######################################################################
# SPI configured drivers
######################################################################

class SpiDriver(DriverBase):
    def __init__(self, config, stepper_config,
                 has_step_dir_pins=True, has_endstop=False):
        DriverBase.__init__(self, config, stepper_config,
            has_step_dir_pins, has_endstop)
        # ========== SPI config ==========
        self.spi = spi = bus.MCU_SPI_from_config(
            config, 3, pin_option="ss_pin", default_speed=2000000)
        self.mcu = spi.get_mcu()
        self._oid = spi.get_oid()
    # ============ SETUP ===============
    def get_mcu(self):
        return self.mcu
    def get_oid(self):
        return self._oid
    # ============ SPI ===============
    def spi_send(self, data, min_clock=0):
        self.spi.spi_send(data, min_clock)
    def spi_transfer(self, data):
        params = self.spi.spi_transfer(data)
        return list(bytearray(params['response']))


######################################################################
# TMC SPI drivers
######################################################################

class TmcSpiDriver(SpiDriver):
    def __init__(self, config, stepper_config,
                 registers, fields, field_formatters, signed_fields,
                 has_step_dir_pins=True, has_endstop=False,
                 max_current=1000.):
        SpiDriver.__init__(self, config, stepper_config,
            has_step_dir_pins, has_endstop)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)
        self.min_current = 100.
        self.max_current = max_current
        self.registers = registers
        self.vsense = 0
        self.mutex = self.printer.get_reactor().mutex()
        # Create a register handler
        self.regs = collections.OrderedDict()
        self.fields = tmc.FieldHelper(
            fields, signed_fields, field_formatters, self.regs)
        # Read generic configuration
        self.sensor_less_homing = config.getboolean('sensor_less_homing', False)
        mode = { "spreadCycle" : False, "stealthChop" : True }
        self.silent_mode = config.getchoice('mode', mode, default='stealthChop')
        sense_resistor = config.getfloat('sense_R', None, minval=0.09)
        if sense_resistor is None:
            sense_resistor = config.getfloat(
                'sense_resistor', 0.11, minval=0.09)
        self.sense_resistor = sense_resistor + 0.02
        # register command handlers
        self.gcode = gcode = self.printer.lookup_object('gcode')
        cmds = ["DRV_STATUS", "DRV_CURRENT", "DRV_STALLGUARD"]
        for cmd in cmds:
            gcode.register_mux_command(
                cmd, "DRIVER", self.name,
                getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        gcode.register_mux_command(
            "DUMP_TMC", "DRIVER", self.name,
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        gcode.register_mux_command(
            "INIT_TMC", "DRIVER", self.name,
            self.cmd_INIT_TMC, desc=self.cmd_INIT_TMC_help)
    def handle_ready(self):
        if not self.mcu.is_shutdown():
            self._init_driver()

    # **************************************************************************
    # === register helpers ===
    # **************************************************************************
    def get_fields(self):
        return self.fields
    def get_register(self, reg_name):
        reg = self.registers[reg_name]
        with self.mutex:
            self.spi.spi_send([reg, 0x00, 0x00, 0x00, 0x00])
            if self.printer.get_start_args().get('debugoutput') is not None:
                return 0
            params = self.spi.spi_transfer([reg, 0x00, 0x00, 0x00, 0x00])
        pr = bytearray(params['response'])
        return (pr[1] << 24) | (pr[2] << 16) | (pr[3] << 8) | pr[4]
    def set_register(self, reg_name, val, print_time=0.):
        min_clock = self.spi.get_mcu().print_time_to_clock(print_time)
        reg = self.registers[reg_name]
        data = [(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
                (val >> 8) & 0xff, val & 0xff]
        with self.mutex:
            self.spi.spi_send(data, min_clock)

    # **************************************************************************
    # === virtual declarations ===
    # **************************************************************************
    def dump_registers(self):
        return "N/A"
    def set_stallguard(self, sg=None):
        return "N/A"
    def set_dir(self, direction=0):
        pass
    def _init_driver(self):
        pass

    # **************************************************************************
    # === GCode handlers ===
    # **************************************************************************
    cmd_DRV_STATUS_help = "args: DRIVER=driver_name"
    def cmd_DRV_STATUS(self, params):
        self.gcode.respond(self.dump_registers())
    cmd_DRV_CURRENT_help = "args: DRIVER=driver_name [CURRENT=amps]"
    def cmd_DRV_CURRENT(self, params):
        current = self.gcode.get_float('CURRENT', params,default=None,
            minval=(self.min_current/1000.), maxval=(self.max_current/1000.))
        hold = self.gcode.get_float('HOLD_MULTIPLIER', params, default=None,
            minval=0., maxval=1.)
        delay = self.gcode.get_int('HOLD_DELAY', params, default=None,
            minval = 0, maxval = 15)
        msg = self._calc_rms_current(current, hold, delay)
        self.gcode.respond(msg)
    cmd_DRV_SG_help = "args: DRIVER=driver_name [SG=val]"
    def cmd_DRV_STALLGUARD(self, params):
        sg = self.gcode.get_int('SG', params, default=None,
            minval=-64, maxval=63)
        msg = "Stallguard is %s" % self.set_stallguard(sg)
        self.gcode.respond(msg)
    cmd_DUMP_TMC_help = "args: DRIVER=driver_name"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        self.logger.info("DUMP_TMC")
        gcode = self.gcode
        write_only_regs = []
        queried_regs = []
        for reg_name, reg_val in self.registers.items():
            if reg_val[1] == 'W':
                val = self.regs.get(reg_name, None)
                if val is not None:
                    write_only_regs.append(
                        self.fields.pretty_format(reg_name, int(val)))
            else:
                val = self._command_read(reg_name)
                queried_regs.append(self.fields.pretty_format(reg_name, val))
        msg = ["========== Write-only registers =========="]
        msg.extend(write_only_regs)
        msg.append("========== Queried registers ==========")
        msg.extend(queried_regs)
        msg = "\n".join(msg)
        gcode.respond_info(msg)
    cmd_INIT_TMC_help = "args: DRIVER=driver_name"
    def cmd_INIT_TMC(self, params):
        self.logger.info("INIT_TMC")
        self.printer.lookup_object('toolhead').wait_moves()
        self._init_driver()

    # **************************************************************************
    # === Public handlers ===
    # **************************************************************************
    def get_current(self):
        rms = self._get_rms_current()
        self.logger.debug("get_current = %.3fA" % rms)
        return rms * 1000.

    # **************************************************************************
    # === Protected handlers ===
    # **************************************************************************
    '''
    READ FRAME:
    |               40bit                        |
    | S 8bit |      32bit data                   |
    | STATUS |   D    |   D    |   D    |   D    |
    '''
    def _command_read(self, cmd): # 40bits always = 5 x 8bit!
        # cmd, mode = self.registers.get(cmd, (cmd, '')) # map string to value
        cmd, mode = self.registers.get(cmd)
        if mode and 'R' not in mode:
            raise error("TMC register '%s' R/W mode '%s is wrong!" % (
                cmd, mode))
        cmd &= 0x7F # Makesure the MSB is 0
        read_cmd = [cmd, 0, 0, 0, 0]
        with self.mutex:
            self.spi_send(read_cmd)
            values = self.spi_transfer(read_cmd)
        # convert list of bytes to number
        val    = 0
        status = 0
        if values:
            status = int(values[0])
            '''
            if status:
                if status & 0b0001:
                    self.logger.warning("Reset has occurred!")
                if status & 0b0010:
                    self.logger.error("Driver error detected!")
                if status & 0b0100:
                    self.logger.warning("Stallguard active")
                if status & 0x1000:
                    self.logger.info("Motor stand still")
            '''
            for idx in range(1, len(values)):
                val <<= 8
                val |= int(values[idx])
        self.logger.debug("<<== cmd 0x%02X : 0x%08X [status: 0x%02X]" % (
            cmd, val, status))
        return val

    '''
    WRITE FRAME:
    |               40bit                        |
    | A 8bit |      32bit data                   |
    |  ADDR  |   D    |   D    |   D    |   D    |
    '''
    def _command_write(self, cmd, val=None, print_time=0.): # 40bits always = 5 x 8bit!
        if val is not None:
            # cmd, mode = self.registers.get(cmd, (cmd, '')) # map string to value
            cmd, mode = self.registers.get(cmd)
            if mode and 'W' not in mode:
                raise error("TMC register '%s' R/W mode '%s is wrong!" % (
                    cmd, mode))
            if not isinstance(val, types.ListType):
                conv = struct.Struct('>I').pack
                val = list(conv(val))
            if len(val) != 4:
                raise error("TMC driver internal error! len(val) != 4")
            self.logger.debug("==>> cmd 0x%02X : 0x%s" % (
                cmd, binascii.hexlify(bytearray(val))))
            cmd |= 0x80 # Make sure command has write bit set
            min_clock = self.spi.get_mcu().print_time_to_clock(print_time)
            with self.mutex:
                self.spi_send([cmd] + val, min_clock)

    def _calc_rms_current(self,
                          current = None,
                          hold_current_multiplier = None,
                          hold_delay = None,
                          init=False):
        initial_vsense = self.vsense
        if current is not None:
            # Check if current is in mA or A
            if self.min_current < current:
                current /= 1000.
            vsense = 0
            base = 32.0 * math.sqrt(2.) * current * self.sense_resistor
            CS = base / 0.325 - .5
            if CS < 16:
                # If Current Scale is too low, turn on high sensitivity R_sense
                # and calculate again
                vsense = 1
                CS = base / 0.180 - .5
            self.fields.set_field('vsense', vsense)
            self.vsense = vsense
            iRun = max(0, min(int(CS), 31))
            self.fields.set_field('run_current', iRun)
        else:
            iRun = self.fields.get_field('run_current')
            current = self._get_rms_current(cs=iRun)
        if hold_current_multiplier is not None:
            iHold = int(iRun * hold_current_multiplier)
            self.fields.set_field('hold_current', iHold)
        else:
            iHold = self.fields.get_field('hold_current')
        if hold_delay is not None:
            self.fields.set_field('hold_delay', hold_delay)
        else:
            hold_delay = self.fields.get_field('hold_delay')
        if not init:
            if initial_vsense != self.vsense:
                self._command_write('CHOPCONF', self.regs['CHOPCONF'])
            self._command_write('IHOLD_IRUN', self.regs['IHOLD_IRUN'])
        msg = "RMS current %.3fA => IHold %u, IRun %u, IHoldDelay %u" % (
            current, iHold, iRun, hold_delay)
        self.logger.debug(msg)
        return msg

    def _get_rms_current(self, cs=None, vsense=None):
        if vsense is None:
            # vsense = self.fields.get_field('vsense')
            vsense = self.vsense
        if cs is None:
            cs = self.fields.get_field('run_current')
        V_fs   = [0.325, 0.180][bool(vsense)]
        return ( cs + 1. ) / 32.0 * V_fs / self.sense_resistor / math.sqrt(2)
