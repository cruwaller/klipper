# TMC2130 stepper driver control
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import pins
from mcu import error
import extras.bus as bus
import tmc
import binascii, types, struct, math, collections

# **************************************************************************
# Generic value mappings
# **************************************************************************
msteps_map = {
    256: 0b0000, # 0: 256x microstepping (Native)
    128: 0b0001, # 1: 128x microstepping
    64:  0b0010, # 2:  64x microstepping
    32:  0b0011, # 3:  32x microstepping
    16:  0b0100, # 4:  16x microstepping
    8:   0b0101, # 5:   8x microstepping
    4:   0b0110, # 6:   4x microstepping
    2:   0b0111, # 7:   2x microstepping
    1:   0b1000, # 8:   1x full-step
}
blank_time_map = {
    16: 0b00,
    24: 0b01, # Recommended for most applications.
    36: 0b10, # Recommended for most applications.
    54: 0b11
}
seup_t = {
    1: 0b00, # Current increment steps per measured stallGuard2 value = 1
    2: 0b01, # Current increment steps per measured stallGuard2 value = 2
    4: 0b10, # Current increment steps per measured stallGuard2 value = 4
    8: 0b11  # Current increment steps per measured stallGuard2 value = 8
}
sedn_t = {
    32: 0b00, # %00: For each 32 stallGuard2 values decrease by one
    8:  0b01, # %01: For each 8  stallGuard2 values decrease by one
    2:  0b10, # %10: For each 2  stallGuard2 values decrease by one
    1:  0b11  # %11: For each    stallGuard2 value  decrease by one
}
pwm_freq_t = {
    "2/1024": 0b00, # %00: fPWM = 2/1024 fCLK
    "2/683":  0b01, # %01: fPWM = 2/683  fCLK
    "2/512":  0b10, # %10: fPWM = 2/512  fCLK
    "2/410":  0b11  # %11: fPWM = 2/410  fCLK
}
freewheel_t = {
    "NORMAL":    0b00, # %00: Normal operation
    "FREEWHEEL": 0b01, # %01: Freewheeling
    "SHORT_LS":  0b10, # %10: Coil shorted using LS drivers
    "SHORT_HS":  0b11  # %11: Coil shorted using HS drivers
}

# **************************************************************************
# Registers
# **************************************************************************
Registers = {
    # GENERAL CONFIGURATION REGISTERS (0x00...0x0F)
    "GCONF":      [0x00, 'RW'],
    "GSTAT":      [0x01, 'R'],
    "IOIN":       [0x04, 'R'],
    # VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10...0x1F)
    "IHOLD_IRUN": [0x10, 'W'],
    "TPOWERDOWN": [0x11, 'W'],
    "TSTEP":      [0x12, 'R'],
    "TPWMTHRS":   [0x13, 'W'],
    "TCOOLTHRS":  [0x14, 'W'],
    "THIGH":      [0x15, 'W'],
    # SPI MODE REGISTER (0x2D)
    "XDIRECT":    [0x2D, 'RW'],
    # DCSTEP MINIMUM VELOCITY REGISTER (0x33)
    "VDCMIN":     [0x33, 'W'],
    # MICROSTEPPING CONTROL REGISTER SET (0x60...0x6B)
    "MSLUT0":     [0x60, 'W'],
    "MSLUT1":     [0x61, 'W'],
    "MSLUT2":     [0x62, 'W'],
    "MSLUT3":     [0x63, 'W'],
    "MSLUT4":     [0x64, 'W'],
    "MSLUT5":     [0x65, 'W'],
    "MSLUT6":     [0x66, 'W'],
    "MSLUT7":     [0x67, 'W'],
    "MSLUTSEL":   [0x68, 'W'],
    "MSLUTSTART": [0x69, 'W'],
    "MSCNT":      [0x6A, 'R'],
    "MSCURACT":   [0x6B, 'R'],
    # DRIVER REGISTER SET (0x6C...0x7F)
    "CHOPCONF":   [0x6C, 'RW'],
    "COOLCONF":   [0x6D, 'W'],
    "DCCTRL":     [0x6E, 'W'],
    "DRV_STATUS": [0x6F, 'R'],
    "PWMCONF":    [0x70, 'W'],
    "PWM_SCALE":  [0x71, 'R'],
    "ENCM_CTRL":  [0x72, 'W'],
    "LOST_STEPS": [0x73, 'R'],
}

Fields = {
    # **************************************************************************
    # GENERAL CONFIGURATION REGISTERS (0x00...0x0F)
    # **************************************************************************
    'GCONF': {
        # 'test_mode'           : 1 << 17, # Not allowed to modify
        # 'direct_mode'         : 1 << 16, # Not allowed to modify
        'stop_enable':         1 << 15,
        'small_hysterisis':    1 << 14,
        'diag1_active_high':   1 << 13,
        'diag0_active_high':   1 << 12,
        'diag1_steps_skipped': 1 << 11,
        'diag1_chopper_on':    1 << 10,
        'diag1_index':         1 << 9,
        'diag1_stall':         1 << 8,
        'diag0_stall':         1 << 7,
        'diag0_temp_prewarn':  1 << 6,
        'diag0_errors':        1 << 5,
        'shaft_dir':           1 << 4,
        # 'commutation'         : 1 << 3, # Not allowed to modify
        'stealthChop':         1 << 2,
        'internal_Rsense':     1 << 1,
        'external_ref':        1 << 0,
    },
    'GSTAT': {
        'gstat_reset':   0b001,
        'gstat_drv_err': 0b010,
        'gstat_uv_cp':   0b100,
    },
    'IOIN':  {
        'VERSION':   0xff << 24,
        'DCO':       1 << 5,
        'ENN_CFG6':  1 << 4,
        'DCIN_CFG5': 1 << 3,
        'DCEN_CFG4': 1 << 2,
        'DIR':       1 << 1,
        'STEP':      1 << 0,
    },
    # **************************************************************************
    # VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10...0x1F)
    # **************************************************************************
    'IHOLD_IRUN': {
        'hold_delay':   0b1111 << 16,
        'run_current':  0b11111 << 8,
        'hold_current': 0b11111 << 0,
    },
    'TPOWERDOWN': {'power_down_delay': 0xff},
    'TSTEP':      {'time_microsteps': 0xfffff},
    'TPWMTHRS':   {'hybrid threshold': 0xfffff},
    'TCOOLTHRS':  {'coolstep_min_speed': 0xfffff},
    'THIGH':      {'mode_sw_speed': 0xfffff},
    # **************************************************************************
    # SPI MODE REGISTER (0x2D)
    # **************************************************************************
    'XDIRECT':  {'xdirect': 0x7FFFFF},
    # **************************************************************************
    # DCSTEP MINIMUM VELOCITY REGISTER (0x33)
    # **************************************************************************
    'VDCMIN':   {'dcstep_min_speed': 0x7FFFFF},
    # **************************************************************************
    # MICROSTEPPING CONTROL REGISTER SET (0x60...0x6B)
    # **************************************************************************
    "MSCNT":    {"MSCNT": 0x3ff},
    "MSCURACT": {
        "CUR_A": 0x1ff,
        "CUR_B": 0x1ff << 16
    },
    # **************************************************************************
    # DRIVER REGISTER SET (0X6C...0X7F)
    # **************************************************************************
    'CHOPCONF': {
        'off_time':                 0b1111 << 0,
        # if chopper_mode == 0 :
        'hysterisis_start':         0b111 << 4,
        'hysterisis_end':           0b1111 << 7,
        # if chopper_mode == 1 :
        'fast_decay_time_l':        0b111 << 4,
        'sine_offset':              0b1111 << 7,
        'fast_decay_time_h':        0b1 << 11,
        'disable_I_comparator':     1 << 12,
        'random_off_time':          1 << 13,
        'chopper_mode':             1 << 14,
        'blank_time':               0b11 << 15,  # blank_time_map
        'vsense':                   1 << 17,
        'fullstep_threshold':       1 << 18,
        'high_speed_mode':          1 << 19,
        'sync_phases':              0b1111 << 20,
        'microsteps':               0b1111 << 24,  # msteps_map
        'interpolate':              1 << 28,
        'double_edge_step':         1 << 29,
        'disable_short_protection': 1 << 30,
    },
    'COOLCONF': {
        'sg_min':              0b1111 << 0,
        'sg_step_width':       0b11 << 5,  # seup_t
        'sg_max':              0b1111 << 8,
        'sg_current_decrease': 0b11 << 13,  # sedn_t
        'smart_min_current':   0b1 << 15,
        'sg_stall_value':      0b1111111 << 16,
        'sg_filter':           0b1 << 24,
    },
    'DRV_STATUS': {
        'sg_result':  0b1111111111,
        'fsactive':   1 << 15,
        'cs_actual':  0b11111 << 16,
        'stallGuard': 1 << 24,
        'ot':         1 << 25,
        'otpw':       1 << 26,
        's2ga':       1 << 27,
        's2gb':       1 << 28,
        'ola':        1 << 29,
        'olb':        1 << 30,
        'stst':       1 << 31,
    },
    'PWMCONF':    {
        'stealth_amplitude': 0b11111111 << 0,
        'stealth_gradient':  0b11111111 << 8,
        'stealth_freq':      0b11 << 16,  # pwm_freq_t
        'stealth_autoscale': 0b1 << 18,
        'stealth_symmetric': 0b1 << 19,
        'standstill_mode':   0b11 << 20,  # freewheel_t
    },
    'PWM_SCALE': {'PWM_SCALE': 0xff},
    'LOST_STEPS': {'LOST_STEPS': 0xfffff}
}

SignedFields = ["CUR_A", "CUR_B", "sg_stall_value"]

FieldFormatters = {
    # GCONF
    "external_ref":      (lambda v: "1(ExtVREF)" if v else ""),
    'stealthChop':       (lambda v: '1(stealthChop)' if v else "0(spreadCycle)"),
    "shaft_dir":         (lambda v: "1(Reverse)" if v else ""),
    'diag0_active_high': (lambda v: "0(inverted)" if not v else ""),
    'diag1_active_high': (lambda v: "0(inverted)" if not v else ""),
    # GSTAT
    "gstat_drv_err":    (lambda v: "1(ErrorShutdown!)" if v else ""),
    "gstat_uv_cp":      (lambda v: "1(Undervoltage!)" if v else ""),
    # IOIN
    "VERSION":   (lambda v: "%#x" % v),
    'DCO':       (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'ENN_CFG6':  (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'DCIN_CFG5': (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'DCEN_CFG4': (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'DIR':       (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'STEP':      (lambda v: '1(HIGH)' if v else '0(LOW)'),
    # MSCURACT
    # "CUR_A",
    # "CUR_B",
    # CHOPCONF
    'blank_time': (lambda v: "%d" % {v: k for k, v in blank_time_map.items()}[v]),
    "microsteps": (lambda v: "%d(%dusteps)" % (v, 0x100 >> v)),
    'vsense':     (lambda v: "1(high res)" if v else "0(normal)"),
    # DRV_STATUS
    'fsactive':         (lambda v: "1(Fullstep)" if v else ""),
    'stallGuard':       (lambda v: "1(Stalled)" if v else ""),
    "otpw":             (lambda v: "1(OvertempWarning!)" if v else ""),
    "ot":               (lambda v: "1(OvertempError!)" if v else ""),
    "s2ga":             (lambda v: "1(ShortToGND_A!)" if v else ""),
    "s2gb":             (lambda v: "1(ShortToGND_B!)" if v else ""),
    "ola":              (lambda v: "1(OpenLoad_A!)" if v else ""),
    "olb":              (lambda v: "1(OpenLoad_B!)" if v else ""),
    'stst':             (lambda v: "1(StandStill)" if v else ""),
    # COOLCONF
    # 'sg_stall_value',
    'sg_min':           (lambda v: "%d" % v if v else '0(disabled)'),
    'sg_step_width':    (lambda v: "%s" % {v: k for k, v in seup_t.items()}[v]),
    'sg_current_decrease': (lambda v: "%s" % {v: k for k, v in sedn_t.items()}[v]),
    # PWMCONF
    'stealth_freq':    (lambda v: "%s" % {v: k for k, v in pwm_freq_t.items()}[v]),
    'standstill_mode': (lambda v: "%s" % {v: k for k, v in freewheel_t.items()}[v]),
}


######################################################################
# Driver base handlers
######################################################################

class DriverBase(object):
    def __init__(self, config):
        self.__inv_step_dist = self.__step_dist = self.microsteps = None
        # get stepper config
        stepper_config = config.getsection(config.get_name().split()[-1])
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
        self.logger.info("step in mm: %s, steps per mm: %s" % (
            self.step_dist, self.inv_step_dist))
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


######################################################################
# SPI configured drivers
######################################################################

class SpiDriver(DriverBase):
    def __init__(self, config):
        DriverBase.__init__(self, config)
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
    def __init__(self, config, registers, fields, field_formatters,
                 signed_fields, max_current=1000.):
        SpiDriver.__init__(self, config)
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
                cmd, "STEPPER", self.name,
                getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name,
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        gcode.register_mux_command(
            "INIT_TMC", "STEPPER", self.name,
            self.cmd_INIT_TMC, desc=self.cmd_INIT_TMC_help)
        gcode.register_mux_command(
            "SET_TMC_FIELD", "STEPPER", self.name,
            self.cmd_SET_TMC_FIELD, desc=self.cmd_SET_TMC_FIELD_help)
    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"
    def cmd_SET_TMC_FIELD(self, params):
        if 'FIELD' not in params or 'VALUE' not in params:
            raise self.gcode.error("Invalid command format")
        field_name = self.gcode.get_str('FIELD', params)
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise self.gcode.error("Unknown field name '%s'" % (field_name,))
        value = self.gcode.get_int('VALUE', params)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._command_write(reg_name, reg_val, print_time)
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
    def set_register(self, reg_name, val, print_time=None):
        minclock = 0
        if print_time is not None:
            minclock = self.get_mcu().print_time_to_clock(print_time)
        reg = self.registers[reg_name]
        data = [(reg | 0x80) & 0xff, (val >> 24) & 0xff, (val >> 16) & 0xff,
                (val >> 8) & 0xff, val & 0xff]
        with self.mutex:
            self.spi.spi_send(data, minclock)

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
    cmd_DRV_STATUS_help = "args: STEPPER=name"
    def cmd_DRV_STATUS(self, params):
        self.gcode.respond(self.dump_registers())
    cmd_DRV_CURRENT_help = "args: STEPPER=_name [CURRENT=amps]"
    def cmd_DRV_CURRENT(self, params):
        current = self.gcode.get_float('CURRENT', params,default=None,
            minval=(self.min_current/1000.), maxval=(self.max_current/1000.))
        hold = self.gcode.get_float('HOLD_MULTIPLIER', params, default=None,
            minval=0., maxval=1.)
        delay = self.gcode.get_int('HOLD_DELAY', params, default=None,
            minval = 0, maxval = 15)
        msg = self._calc_rms_current(current, hold, delay)
        self.gcode.respond(msg)
    cmd_DRV_SG_help = "args: STEPPER=name [SG=val]"
    def cmd_DRV_STALLGUARD(self, params):
        sg = self.gcode.get_int('SG', params, default=None,
            minval=-64, maxval=63)
        msg = "Stallguard is %s" % self.set_stallguard(sg)
        self.gcode.respond(msg)
    cmd_DUMP_TMC_help = "args: STEPPER=name"
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
    cmd_INIT_TMC_help = "args: STEPPER=name"
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
    def _command_write(self, cmd, val=None, print_time=None): # 40bits always = 5 x 8bit!
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
            minclock = 0
            if print_time is not None:
                minclock = self.get_mcu().print_time_to_clock(print_time)
            with self.mutex:
                self.spi_send([cmd] + val, minclock)

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


# Endstop wrapper
class VirtualEndstop:
    def __init__(self, tmc):
        self.logger = tmc.logger
        self.tmc = tmc
        if tmc.endstop_pin is None:
            raise pins.error("endstop_pin is not defined")
        ppins = tmc.printer.lookup_object('pins')
        self.mcu_endstop = mcu_endstop = ppins.setup_pin(
            'endstop', tmc.endstop_pin)
        if mcu_endstop.get_mcu() is not tmc.get_mcu():
            raise pins.error("virtual endstop must be on same mcu")
        # Wrappers to MCU_endstop class
        self.get_mcu = mcu_endstop.get_mcu
        self.add_stepper = mcu_endstop.add_stepper
        self.get_steppers = mcu_endstop.get_steppers
        self.home_start = mcu_endstop.home_start
        self.home_wait = mcu_endstop.home_wait
        self.query_endstop = mcu_endstop.query_endstop
        self.TimeoutError = mcu_endstop.TimeoutError
    def home_prepare(self):
        self.tmc.homing_prepare()
        self.mcu_endstop.home_prepare()
    def home_finalize(self):
        self.tmc.homing_ready()
        self.mcu_endstop.home_finalize()


class TMC2130(TmcSpiDriver):
    def __init__(self, config):
        TmcSpiDriver.__init__(self, config, Registers, Fields, FieldFormatters,
                              SignedFields, max_current=1400.)
        printer = config.get_printer()
        # Prepare virtual endstop support
        ppins = printer.lookup_object('pins')
        name_parts = config.get_name().split()
        ppins.register_chip("%s_%s" % (name_parts[0], name_parts[-1]), self)
        self.endstop_pin = config.get('endstop_pin', default=None)
        if self.sensor_less_homing and self.endstop_pin is None:
            printer.register_event_handler("homing:prepare", self.homing_prepare)
            printer.register_event_handler("homing:finalize", self.homing_ready)
        # Driver configuration
        set_field = self.fields.set_field
        # Diag pins configuration
        diag0types = {
            'NA'           : None,
            'errors'       : 'diag0_errors',
            'temp_prewarn' : 'diag0_temp_prewarn',
            'stall'        : 'diag0_stall',
        }
        diag0purpose = config.getchoice('diag0_out', diag0types, default='NA')
        diag1types = {
            'NA'            : None,
            'stall'         : 'diag1_stall',
            'index'         : 'diag1_index',
            'chopper_on'    : 'diag1_chopper_on',
            'steps_skipped' : 'diag1_steps_skipped',
        }
        diag1purpose = config.getchoice('diag1_out', diag1types, default='NA')

        # Motor power down time after last movement (iHOLD current is used)
        t_power_down = config.getint('power_down_delay', 128,
            minval=0., maxval=255) # 128 = ~2sec
        self.fields.set_reg_value('TPOWERDOWN', t_power_down)
        # Set disable stall speed by default
        self.fields.set_reg_value('TCOOLTHRS', 0)
        # CHOPCONF
        set_field('off_time', config.getint('off_time', 4, minval=2, maxval=15))
        set_field('chopper_mode', 0) # Standard mode
        hstart = config.getint('hysterisis_start', 1, minval=1, maxval=8) - 1
        set_field('hysterisis_start', hstart)
        hend = config.getint('hysterisis_end', 2, minval=-3, maxval=12) + 3
        set_field('hysterisis_end', hend)
        set_field('blank_time', config.getchoice('blank_time', blank_time_map, 24))
        set_field('microsteps', msteps_map[self.microsteps])
        set_field('interpolate', config.getboolean('interpolate', True))
        # COOLCONF
        set_field('sg_filter', config.getboolean('stall_filter', False))
        # +1...+63 = less sensitivity, -64...-1 = higher sensitivity
        set_field('sg_stall_value', config.getint('stall_threshold', 10,
            minval=-64, maxval=63))
        set_field('sg_min', config.getint('current_increase_threshold', 0,
            minval=0, maxval=15))
        set_field('sg_max', config.getint('current_decrease_threshold', 0,
            minval=0, maxval=15))
        # GCONF
        set_field('shaft_dir', config.getboolean('dir_invert', False))
        set_field('diag0_active_high',
            config.getboolean('diag0_active_high', True))
        set_field('diag1_active_high',
            config.getboolean('diag1_active_high', True))
        if diag0purpose is not None:
            set_field(diag0purpose, 1)
        if diag1purpose is not None:
            set_field(diag1purpose, 1)
        set_field('stealthChop', self.silent_mode)
        # PWMCONF
        set_field('stealth_autoscale', 1)
        set_field('stealth_gradient', config.getint('stealth_gradient', 5,
            minval=1, maxval=15))
        set_field('stealth_amplitude', config.getint('stealth_amplitude', 180,
            minval=64, maxval=255))
        set_field('stealth_freq', config.getchoice('stealth_freq',
            pwm_freq_t, "2/683"))
        # TPWMTHRS
        # option to manually set a hybrid threshold
        hybrid_threshold = config.getint('hybrid_threshold', None,
            minval=0, maxval=1048575)
        # hybrid threshold, speed is used to derive the threshold
        stealth_max_speed = config.getint('stealth_max_speed', None, minval=10)
        pwm_thrs = 0
        if hybrid_threshold is not None:
            pwm_thrs = hybrid_threshold
            self.logger.info(
                "Hybrid threshold: %s" % (hybrid_threshold, ))
        elif stealth_max_speed is not None:
            # default int clock freq is 13.2MHz @ 50C
            fCLK = config.getfloat('fCLK', 13200000,
                above=4000000, maxval=18000000)
            pwm_thrs = int(fCLK * self.microsteps /
                           (stealth_max_speed * self.inv_step_dist * 256))
            self.logger.debug("Stealth max speed: %u (cfg val: %u)" % (
                stealth_max_speed, pwm_thrs))
        self.fields.set_reg_value('TPWMTHRS', pwm_thrs)
        # ========== Set Current ==========
        current = config.getfloat('current', 1000.0,
            above=self.min_current, maxval=self.max_current)
        hold_multip = config.getfloat('hold_multiplier', 0.5,
            above=0., maxval=1.0)
        hold_delay = config.getint('hold_delay', 10, minval=0., maxval=15)
        self._calc_rms_current(current, hold_multip, hold_delay, init=True)
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise pins.error("virtual endstop is only supported")
        if self.endstop_pin is None:
            raise pins.error("tmc driver endstop_pin is not defined")
        return VirtualEndstop(self)

    #**************************************************************************
    # PUBLIC METHODS
    #**************************************************************************
    def homing_prepare(self):
        self._command_write('TCOOLTHRS', 0xFFFFF)
    def homing_ready(self):
        self._command_write('TCOOLTHRS', 0)

    def dump_registers(self):
        res = [
            "NAME: %s" % self.name,
            "RMS current: %.3fA" % self._get_rms_current(),
            "========== Queried registers =========="
        ]
        dump_regs = [key for key, val in Registers.items() if 'R' in val[1]]
        for reg in dump_regs:
            value = self._command_read(reg)
            res.append(self.fields.pretty_format(reg, value))
        # Print write only config registers
        res.append("========== Write-only registers ==========")
        res.append(self.fields.pretty_format('IHOLD_IRUN', self.regs['IHOLD_IRUN']))
        res.append(self.fields.pretty_format('COOLCONF', self.regs['COOLCONF']))
        res.append(self.fields.pretty_format('PWMCONF', self.regs['PWMCONF']))
        res.append(self.fields.pretty_format('TPWMTHRS', self.regs['TPWMTHRS']))
        res.append(self.fields.pretty_format('TPOWERDOWN', self.regs['TPOWERDOWN']))
        log = "\n".join(sorted(res))
        self.logger.info(log)
        return log
    def set_dir(self, direction=0):
        new_value = self.fields.set_field('shaft_dir', bool(direction))
        self._command_write('GCONF', new_value)
    def set_stallguard(self, sg=None):
        if sg is not None:
            new_val = self.fields.set_field('sg_stall_value', sg)
            self._command_write('COOLCONF', new_val)
        else:
            sg = self.fields.get_field('sg_stall_value')
        return sg

    #**************************************************************************
    # PRIVATE METHODS
    #**************************************************************************
    def _init_driver(self):
        # ----------------------------------------------------------------------
        # Clear errors by reading GSTAT register
        self.__get_global_status()
        # ----------------------------------------------------------------------
        #   Send configurations to driver
        self.logger.debug("==== Send configurations ====")
        for reg_name, val in self.regs.items():
            self._command_write(reg_name, val)
        # ----------------------------------------------------------------------
        # Check errors by reading GSTAT register
        self.__get_global_status()
        # ----------------------------------------------------------------------
        # Verify configurations
        self.logger.debug("Verify configurations")
        self.__validate_cfg()
        # ----------------------------------------------------------------------
        self.logger.debug("Check status")
        self.__get_status()
        self.logger.debug("Init ready")

    def __validate_cfg(self):
        # validate readable configurations
        gconf = self._command_read('GCONF')
        gconf_exp = self.regs.get('GCONF', 0)
        if gconf != gconf_exp:
            self.logger.error("GCONF Configuration error! [was 0x%08X expected 0x%08X]" %
                              (gconf, gconf_exp))
        chopconf = self._command_read('CHOPCONF')
        chopconf_exp = self.regs.get('CHOPCONF', 0)
        if chopconf != chopconf_exp:
            self.logger.error("CHOPCONF Configuration error! [was 0x%08X expected 0x%08X]" %
                              (chopconf, chopconf_exp))

    def __get_global_status(self):
        status = self._command_read('GSTAT')
        log = self.fields.pretty_format('GSTAT', status)
        self.logger.error(log)
        return log

    def __get_io_status(self):
        status = self._command_read('IOIN')
        log = self.fields.pretty_format('IOIN', status)
        self.logger.info(log)
        return log

    def __get_status(self):
        val = self._command_read('DRV_STATUS')
        log = self.fields.pretty_format('DRV_STATUS', val)
        self.logger.info(log)
        return log

    def __get_lost_steps(self):
        val = self._command_read('LOST_STEPS')
        if val:
            dump = "Lost steps: {}".format(val)
            self.logger.error(dump)
        return val

def load_config_prefix(config):
    return TMC2130(config)
