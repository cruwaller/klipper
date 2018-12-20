# TMC51xx stepper driver control
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import types, struct
import binascii
from driverbase import SpiDriver
from mcu import error
import chelper, pins

#***************************************************
# Registers
#***************************************************
# GENERAL CONFIGURATION REGISTERS (0x00...0x0F)
REG_GCONF      = 0x00
REG_GSTAT      = 0x01
#REG_IFCNT      = 0x02 # UART only
#REG_SLAVECONF  = 0x03 # UART only
REG_IOIN       = 0x04 # Reads the state of all input pins available
REG_X_COMPARE  = 0x05

# VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10...0x1F)
REG_IHOLD_IRUN = 0x10
REG_TPOWERDOWN = 0x11
REG_TSTEP      = 0x12
REG_TPWMTHRS   = 0x13
REG_TCOOLTHRS  = 0x14
REG_THIGH      = 0x15

# RAMP GENERATOR MOTION CONTROL REGISTER SET (0x20...0x2D)
REG_RAMPMODE   = 0x20
REG_XACTUAL    = 0x21
REG_VACTUAL    = 0x22
REG_VSTART     = 0x23
REG_A1         = 0x24
REG_V1         = 0x25
REG_AMAX       = 0x26
REG_VMAX       = 0x27
REG_DMAX       = 0x28
REG_D1         = 0x2A
REG_VSTOP      = 0x2B
REG_TZEROWAIT  = 0x2C
REG_XTARGET    = 0x2D

# RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0x30...0x36)
REG_VDCMIN     = 0x33
REG_SW_MODE    = 0x34
REG_RAMP_STAT  = 0x35
REG_XLATCH     = 0x36

# ENCODER REGISTER SET (0x38...0x3C)
REG_ENCMODE    = 0x38
REG_X_ENC      = 0x39
REG_ENC_CONST  = 0x3A
REG_ENC_STATUS = 0x3B
REG_ENC_LATCH  = 0x3C

# MICROSTEPPING CONTROL REGISTER SET (0x60...0x6B)
REG_MSLUT0     = 0x60
REG_MSLUT1     = 0x61
REG_MSLUT2     = 0x62
REG_MSLUT3     = 0x63
REG_MSLUT4     = 0x64
REG_MSLUT5     = 0x65
REG_MSLUT6     = 0x66
REG_MSLUT7     = 0x67
REG_MSLUTSEL   = 0x68
REG_MSLUTSTART = 0x69
REG_MSCNT      = 0x6A
REG_MSCURACT   = 0x6B

# DRIVER REGISTER SET (0x6C...0x7F)
REG_CHOPCONF   = 0x6C
REG_COOLCONF   = 0x6D
REG_DCCTRL     = 0x6E
REG_DRV_STATUS = 0x6F
REG_PWMCONF    = 0x70
REG_PWM_SCALE  = 0x71
REG_ENCM_CTRL  = 0x72
REG_LOST_STEPS = 0x73

# Constants:
MIN_CURRENT = 100.
MAX_CURRENT = 2000.
DEFAULT_SENSE_R = 0.11

class TMC51xx(SpiDriver):
    _stepper_kinematics = None
    toolhead = None
    # Error flags
    isReset      = False
    isError      = False
    isStallguard = False
    isStandstill = False

    val_GCONF = 0
    val_CHOPCONF = 0
    val_COOLCONF = 0
    val_PWMCONF = 0
    val_IHOLD_IRUN = 0
    val_SW_MODE = 0
    val_RAMP_STAT = 0
    val_ENCMODE = 0

    class TimeoutError(Exception):
        pass

    def __init__(self, config):
        self.printer = printer = config.get_printer()
        # init local variables
        self.__ramp_mode = 0
        self._endstop_config = None
        self._commanded_pos = self._mcu_position_offset = 0.
        self._direction = self._homing_speed = 0
        self._last_state = {}
        self._home_cmd = None
        # init
        SpiDriver.__init__(self, config, has_step_dir_pins=False, has_endstop=True)
        self._stepper_oid = stepper_oid = self.mcu.create_oid()
        self.mcu.add_config_cmd(
            "stepper_tmc5x_config oid=%u spi_oid=%u" % (stepper_oid, self._oid,))
        self.mcu.register_msg(self._home_handle_end_stop_state,
                              "stepper_tmc5x_home_status", stepper_oid)
        ffi_main, self._ffi_lib = chelper.get_ffi()
        self._stepqueue = ffi_main.gc(self._ffi_lib.stepcompress_alloc(stepper_oid),
                                      self._ffi_lib.stepcompress_free)
        self.mcu.register_stepqueue(self._stepqueue)
        # configure pins
        ppins = printer.lookup_object('pins')
        self.enable = enable_pin = config.get('enable_pin', None)
        if enable_pin is not None:
            self.enable = ppins.setup_pin('digital_out', enable_pin)
            self.enable.setup_max_duration(0.)
        # register driver as a virtual endstop
        ppins.register_chip('driver', self)
        # Driver configuration
        self._invert_dir = config.getboolean('direction_inverted', False)
        self._endstop_logic = config.getboolean('endstop_inverted', False)
        self.current = config.getfloat('current', 1000.0,
                                       above=MIN_CURRENT, maxval=MAX_CURRENT)
        self.fCLK = config.getfloat(
            'fCLK', 16000000, above=4000000, maxval=32000000)
        self.sense_r = config.getfloat('sense_R', DEFAULT_SENSE_R, above=0.09)
        self.hold_multip = config.getfloat('hold_multiplier', 0.5, above=0., maxval=1.0)
        self.hold_delay = config.getint('hold_delay', 7, minval=0., maxval=15)
        self.t_power_down = config.getint('power_down_delay', 128, minval=0., maxval=255)
        self.interpolate = config.getboolean('interpolate', True)
        self.sensor_less_homing = config.getboolean('sensor_less_homing', False)
        # option to manually set a hybrid threshold
        self.hybrid_threshold = config.getint('hybrid_threshold',
                                              None, minval=0, maxval=1048575)
        # hybrid threshold, speed is used to derive the threshold
        self.stealth_max_speed = config.getint('stealth_max_speed',
                                               None, minval=10)
        # +1...+63 = less sensitivity, -64...-1 = higher sensitivity
        self.sg_stall_value = config.getint('stall_threshold',
                                            10, minval=-64, maxval=63)
        diag0types = {
            'NA'           : None,
            'errors'       : 0,
            'temp_prewarn' : 1,
            'stall'        : 2,
        }
        self.diag0purpose = config.getchoice('diag0_out', diag0types, default='NA')
        self.diag0act_high = config.getboolean('diag0_active_high', default=True)
        diag1types = {
            'NA'            : None,
            'stall'         : 0,
            'index'         : 1,
            'chopper_on'    : 2,
            'steps_skipped' : 3,
        }
        self.diag1purpose = config.getchoice('diag1_out', diag1types, default='NA')
        self.diag1act_high = config.getboolean('diag1_active_high', default=True)
        mode = { "spreadCycle" : False, "stealthChop" : True }
        self.silent_mode = config.getchoice('mode', mode, default='stealthChop')
        self._direction = config.getboolean('dir_invert', default=False)
        # local inits
        self.set_ignore_move(False)
        self.set_homing_dir()
        # register command handlers
        self.gcode = gcode = printer.lookup_object('gcode')
        cmds = ["DRV_STATUS", "DRV_CURRENT", "DRV_STALLGUARD"]
        for cmd in cmds:
            gcode.register_mux_command(
                cmd, "DRIVER", self.name,
                getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual':
            raise pins.error("Probe virtual endstop only useful as endstop pin")
        return self
    # === GCode handlers ===
    cmd_DRV_STATUS_help = "args: DRIVER=driver_name"
    def cmd_DRV_STATUS(self, params):
        self.gcode.respond(self.status())
    cmd_DRV_CURRENT_help = "args: DRIVER=driver_name [CURRENT=amps]"
    def cmd_DRV_CURRENT(self, params):
        current = self.gcode.get_float('CURRENT', params,
                                       default=None,
                                       minval=(MIN_CURRENT/1000.),
                                       maxval=(MAX_CURRENT / 1000.))
        hold = self.gcode.get_float('HOLD_MULTIPLIER', params,
                                    default=self.hold_multip,
                                    minval=0., maxval=1.)
        delay = self.gcode.get_int('HOLD_DELAY', params,
                                   default=self.hold_delay,
                                   minval = 0, maxval = 15)
        self.__calc_rms_current(current, hold, delay)
        msg = "Current is %.3fA, hold current %.3fA, hold delay %s" % (
            self.current, (self.hold_multip * self.current), self.hold_delay)
        self.gcode.respond(msg)
    cmd_DRV_SG_help = "args: DRIVER=driver_name [SG=val]"
    def cmd_DRV_STALLGUARD(self, params):
        sg = self.gcode.get_float('SG', params, default=None)
        self.gcode.respond(self.set_stallguard(sg))
    # === Internal classes ===
    speed_factor = accel_factor = accel_factor_t = 0.
    def _build_config(self):
        self.mcu.add_config_cmd(
            "stepper_tmc5x_reset_step_clock oid=%d clock=0" % (self._stepper_oid,),
            is_init=True)
        self._home_cmd = self.mcu.lookup_command(
            "stepper_tmc5x_home oid=%c clock=%u interval=%u cmd=%*s")
        self._get_position_cmd = self.mcu.lookup_command(
            "stepper_tmc5x_get_position oid=%c")
        self._set_position_cmd = self.mcu.lookup_command(
            "stepper_tmc5x_set_position oid=%c reset_clock=%c position=%u")
        max_error = self.mcu.get_max_stepper_error()
        step_cmd_id = self.mcu.lookup_command_id(
            "stepper_tmc5x_queue oid=%c interval=%u target=%u"
            " amax=%hu dmax=%hu vmax=%u vstart=%u")
        self.speed_factor = float(1 << 24) / self.fCLK * self.microsteps # t = 2^24 / fCLK
        self.accel_factor = float(1 << 41) / self.fCLK**2 * self.microsteps # ta2 = 2^41 / (fCLK^2)
        self.accel_factor_t = float(1 << 17) / self.fCLK # accel_t to AMAX
        self._ffi_lib.stepcompress_fill_tmc5x(
            self._stepqueue, self.mcu.seconds_to_clock(max_error),
            step_cmd_id, self.speed_factor, self.accel_factor, self.accel_factor_t)

    #**************************************************************************
    # PUBLIC METHODS
    #**************************************************************************

    # ============ HOMING ===============
    #def set_homing_speed(self, speed):
    #    self._homing_speed = speed
    def set_homing_dir(self, homedir="min"):
        # TODO: Is it ok to set endstop settings once during init??
        # FIXME : Change to commands
        if homedir in ["min", False]:
            self._endstop_config = 0x21 # 0b10 0001
        else:
            self._endstop_config = 0x11 # 0b01 0001
        self._endstop_config |= 0x0C if not self._endstop_logic else 0x00
        self.logger.info("Homing direction '%s' , inverted %s" % (
            homedir, self._endstop_logic))
    def add_stepper(self, stepper):
        # Just pass to make code more common
        pass
    def get_steppers(self):
        return [self]
    def home_prepare(self, speed):
        self.logger.info("----- HOME INIT ----- speed: %s" % (speed,))
        self._homing_speed = speed
        # Init homing here!
        self.set_ignore_move(True)
        if self.sensor_less_homing:
            self.__set_REG_SW_MODE(0x0)
            # stealthchop off for stallguard homing
            self.__modify_REG_GCONF('stealthChop', 0)
            # Calculate homing speed
            stall_speed = 16777216. / self._homing_speed / self.microsteps
            stall_speed *= 1.10 # +10% tolerance
            self.__set_REG_TCOOLTHRS(stall_speed)
            #self.__set_REG_TCOOLTHRS(0xFFFFF)
            self.__modify_REG_SW_MODE('sg_stop', 1) # Stop to SG
            self.__set_REG_AMAX(500 * self.accel_factor)  # Set lower acceleration
    def home_start(self, print_time, sample_time, sample_count, rest_time):
        clock = self.mcu.print_time_to_clock(print_time)
        self.logger.info("----- HOME START -----")
        self.__set_REG_RAMPMODE('velocity neg')
        self.__set_REG_VMAX(self._homing_speed * self.speed_factor)  # homing_speed
        if not self.sensor_less_homing and \
                self._endstop_config is not None:
            self.__set_REG_SW_MODE(self._endstop_config)
        self._home_cmd.send([self._stepper_oid, clock,
                             self.mcu.seconds_to_clock(.5), []])
    def home_wait(self, home_end_time):
        eventtime = self.mcu.monotonic()
        while self._last_state.get('ready', 0) != 1:
            eventtime = self.mcu.pause(eventtime + 0.5)
            self.logger.info("Wait homing: %s > %s" % (
                    self.mcu.estimated_print_time(eventtime), home_end_time))
            if self.mcu.estimated_print_time(eventtime) > home_end_time:
                raise self.TimeoutError("Timeout during endstop homing")
            if self.mcu.is_shutdown():
                raise error("MCU is shutdown")
    def home_finalize(self):
        self.logger.info("----- HOME FINALIZE ----")
        self._home_cmd.send([self._stepper_oid, 0, 0, []])
        self.__set_REG_SW_MODE(0x0)
        self.__set_REG_RAMPMODE('hold')
        if self.sensor_less_homing:
            self.__modify_REG_GCONF('stealthChop', int(self.silent_mode))
            self.__set_REG_TCOOLTHRS(0)
        self._set_position_cmd.send([self._stepper_oid, 1, 0])
        self.set_ignore_move(False)
        self._ffi_lib.itersolve_set_commanded_pos(
            self._stepper_kinematics, 0 - self._mcu_position_offset)
    def query_endstop(self, print_time):
        self.__get_REG_RAMP_STAT()
    def query_endstop_wait(self):
        # Return current state of the endstop; TRIGGERED (1) or OPEN (0)
        return (self.__read_REG_RAMP_STAT('status_sg') or
                self.__read_REG_RAMP_STAT('status_stop_r') or
                self.__read_REG_RAMP_STAT('status_stop_l'))
    def _home_handle_end_stop_state(self, params):
        """
        {'#receive_time': 662109.504473921, u'oid': 3, u'value': 0,
        '#name': u'stepper_tmc5x_home_status',
        '#sent_time': 662108.664678151, u'ready': 0}
        """
        self._last_state = params

    # ============ STEPPING ===============
    _itersolve_gen_steps = None
    def set_ignore_move(self, ignore_move):
        was_ignore = (self._itersolve_gen_steps
                      is not self._ffi_lib.itersolve_gen_steps)
        if ignore_move:
            self._itersolve_gen_steps = (lambda *args: 0)
        else:
            self._itersolve_gen_steps = self._ffi_lib.itersolve_gen_steps_tmc5x
        self.logger.info("Ignore moves: %s" % ignore_move)
        return was_ignore

    def setup_min_stop_interval(self, min_stop_interval):
        pass

    def calc_position_from_coord(self, coord):
        return self._ffi_lib.itersolve_calc_position_from_coord(
            self._stepper_kinematics, coord[0], coord[1], coord[2])
    def set_position(self, coord):
        # TODO : Handle with driver, no delta needed!
        """
        steppos = newpos * self.inv_step_dist
        self._mcu_position_offset += self._commanded_pos - steppos
        self._commanded_pos = steppos
        self.__set_REG_RAMPMODE('hold')
        self.__set_REG_XTARGET(steppos)
        self.__set_REG_XACTUAL(steppos)
        self.__set_REG_RAMPMODE('positioning')
        """
        self.set_commanded_position(self.calc_position_from_coord(coord))

    def get_commanded_position(self):
        # TODO : Handle with driver!
        return self._ffi_lib.itersolve_get_commanded_pos(
            self._stepper_kinematics)

    def set_commanded_position(self, pos):
        self._mcu_position_offset += self.get_commanded_position() - pos
        self._ffi_lib.itersolve_set_commanded_pos(self._stepper_kinematics, pos)

    def get_mcu_position(self):
        # TODO : Handle with driver, no delta needed!
        pos_delta = self.get_commanded_position() + self._mcu_position_offset
        mcu_pos = pos_delta / self.step_dist
        if mcu_pos >= 0.:
            return int(mcu_pos + 0.5)
        return int(mcu_pos - 0.5)

    def setup_itersolve(self, alloc_func, *params):
        ffi_main, ffi_lib = chelper.get_ffi()
        sk = ffi_main.gc(getattr(ffi_lib, alloc_func)(*params), ffi_lib.free)
        self.set_stepper_kinematics(sk)

    def set_stepper_kinematics(self, sk):
        old_sk = self._stepper_kinematics
        self._stepper_kinematics = sk
        self._ffi_lib.itersolve_set_stepcompress(
            sk, self._stepqueue, self.step_dist)
        return old_sk

    def step_itersolve(self, cmove):
        ret = self._itersolve_gen_steps(self._stepper_kinematics, cmove)
        if ret:
            raise error("Internal error in stepcompress")

    # ============ DRIVER CONTROL ===============
    def status(self):
        # TODO : Update status!
        res = ["NAME: %s" % self.name]
        current = "RMS current: %.3fA" % self.__get_rms_current()
        self.logger.info(current)
        res.append(current)
        self.__get_GSTAT(log=res)
        self.__get_REG_DRV_STATUS(log=res)
        self.__get_IOIN(log=res)
        self.__get_LOST_STEPS(log=res)
        return "\n".join(res)
    def set_dir(self, direction=0):
        self._direction = direction
        self.__modify_REG_GCONF('shaft_dir', direction)
    def has_faults(self):
        return (self.isReset or self.isError or
                self.isStallguard or self.isStandstill)
    def clear_faults(self):
        self.isReset = self.isError = False
        self.isStallguard = self.isStandstill = False
    def get_current(self):
        self.logger.info("get_current = %s" % self.__get_rms_current())
        return self.__get_rms_current() * 1000.
    def set_stallguard(self, sg=None):
        if sg is not None:
            if sg < -64 or 63 < sg:
                raise self.gcode.error("SG out of range (min: -64, max: 63)")
            self.sg_stall_value = sg
            self.__modify_REG_COOLCONF('sg_stall_value', sg)
        return "Stallguard is %d" % self.sg_stall_value
    def get_stallguard(self):
        return self.sg_stall_value
    def get_stall_status(self, *args, **kwargs):
        val = self.__get_REG_DRV_STATUS(check=False)
        if val & 0b00000001000000000000000000000000:
            return True
        return False
    def get_lost_steps(self, *args, **kwargs):
        return self._command_read(REG_LOST_STEPS)

    #**************************************************************************
    # PRIVATE METHODS
    #**************************************************************************
    '''
    ~                    READ / WRITE data transfer example
    =================================================================================
    action                       | data sent to TMC51xx  | data received from TMC51xx
    =================================================================================
    read DRV_STATUS              | --> 0x6F00000000      | <-- 0xSS & unused data
    read DRV_STATUS              | --> 0x6F00000000      | <-- 0xSS & DRV_STATUS
    write CHOPCONF := 0x00ABCDEF | --> 0xEC00ABCDEF      | <-- 0xSS & DRV_STATUS
    write CHOPCONF := 0x00123456 | --> 0xEC00123456      | <-- 0xSS00ABCDEF
    =================================================================================
    '''

    '''
    READ FRAME:
    |               40bit                        |
    | S 8bit |      32bit data                   |
    | STATUS |   D    |   D    |   D    |   D    |
    '''
    def _command_read(self, cmd, cnt=4): # 40bits always = 5 x 8bit!
        cmd &= 0x7F # Makesure the MSB is 0
        read_cmd = [cmd] + cnt*[0]
        self._transfer(read_cmd)
        values = self._transfer(read_cmd)
        # convert list of bytes to number
        val    = 0
        size   = len(values)
        status = 0
        if 0 < size:
            status = int(values[0])
            if status:
                if status & 0b0001:
                    self.isReset = True
                    self.logger.warning("Reset has occurred!")
                if status & 0b0010:
                    self.isError = True
                    self.logger.error("Driver error detected!")
                if status & 0b0100:
                    self.isStallguard = True
                    self.logger.warning("Stallguard active")
                if status & 0x1000:
                    self.isStandstill = True
                    self.logger.info("Motor stand still")
            for idx in range(1, size):
                val <<= 8
                val |= int(values[idx])
        self.logger.debug("<<== cmd 0x%02X : 0x%08X [status: 0x%02X]" % (cmd, val, status))
        return val

    '''
    WRITE FRAME:
    |               40bit                        |
    | A 8bit |      32bit data                   |
    |  ADDR  |   D    |   D    |   D    |   D    |
    '''
    def _command_write(self, cmd, val=None): # 40bits always = 5 x 8bit!
        if val is not None:
            if not isinstance(val, types.ListType):
                conv = struct.Struct('>I').pack
                val = list(conv(val))
            if len(val) != 4:
                raise error("TMC51xx internal error! len(val) != 4")
            self.logger.debug("==>> cmd 0x%02X : 0x%s" % (cmd, binascii.hexlify(bytearray(val))))
            cmd |= 0x80 # Make sure command has write bit set
            self._transfer([cmd] + val)

    def __reset_driver(self):
        self.logger.debug("Reset driver")
        # Clear errors by reading GSTAT register
        self.__get_GSTAT()
        # Init internal values
        self.val_GCONF      = 0
        self.val_CHOPCONF   = 0
        self.val_COOLCONF   = 0
        self.val_PWMCONF    = 0
        self.val_IHOLD_IRUN = 0
        self.val_SW_MODE    = 0
        self.val_RAMP_STAT  = 0
        self.val_ENCMODE    = 0
        # reset registers:
        self._command_write(REG_GCONF,    self.val_GCONF)
        self._command_write(REG_CHOPCONF, self.val_CHOPCONF)
        self._command_write(REG_COOLCONF, self.val_COOLCONF)
        self._command_write(REG_PWMCONF,  self.val_PWMCONF)
        self._command_write(REG_ENCMODE,  self.val_ENCMODE)
        self._command_write(REG_SW_MODE,  self.val_SW_MODE)
        self.__set_REG_TCOOLTHRS(0)

    def __validate_cfg(self):
        self.__get_GSTAT() # read and reset status
        # validate readable configurations
        GCONF      = self._command_read(REG_GCONF)
        CHOPCONF   = self._command_read(REG_CHOPCONF)
        if GCONF != self.val_GCONF:
            self.logger.error("GCONF Configuration error! [was 0x%08X expected 0x%08X]" %
                              (GCONF, self.val_GCONF))
        if CHOPCONF != self.val_CHOPCONF:
            self.logger.error("CHOPCONF Configuration error! [was 0x%08X expected 0x%08X]" %
                              (CHOPCONF, self.val_CHOPCONF))

    def _init_driver(self):
        self.toolhead = toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.enable is not None:
            self.enable.set_digital(print_time, 1)
        _commanded_pos = self._commanded_pos
        self.__reset_driver()

        # Set RMS Current
        self.__calc_rms_current(self.current, self.hold_multip,
                                self.hold_delay, init=True)
        # Set current position
        self.__set_REG_RAMPMODE('hold')
        self.__set_REG_XTARGET(_commanded_pos)
        self.__set_REG_XACTUAL(_commanded_pos)
        # Set mode to positioning
        self.__set_REG_RAMPMODE('positioning')
        # Disables A1 and D1 in position mode, AMAX and VMAX only
        self.__set_REG_V1(0)
        self.__set_REG_D1(0x10)
        # Init Acceleration and Velocity
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.__set_REG_AMAX(max_accel * self.accel_factor) # 0xFFFF
        self.__set_REG_VMAX(max_velocity * self.speed_factor) # 0xFFFF

        # Motor power down time after last movement (iHOLD current is used)
        self.__set_REG_TPOWERDOWN(self.t_power_down)

        # Jerk control:
        self.__set_REG_TZEROWAIT(32) # Delay between moves
        self.__set_REG_VSTART(1) # Start speed
        self.__set_REG_VSTOP(2) # Stop speed

        # TRAMS default 0x140101D5
        # 0b0001 0100 0000 0001 0000 0001 1101 0101
        self.__modify_REG_CHOPCONF('off_time', 5, send=False)
        self.__modify_REG_CHOPCONF('chopper_mode', 0, send=False)
        self.__modify_REG_CHOPCONF('hysterisis_start', 3, send=False) # TRAMS=13
        self.__modify_REG_CHOPCONF('hysterisis_end', 2, send=False) # TRAMS=1
        self.__modify_REG_CHOPCONF('blank_time', 24, send=False) # TRAMS=36
        self.__modify_REG_CHOPCONF('microsteps', self.microsteps, send=False)
        self.__modify_REG_CHOPCONF('interpolate', self.interpolate, send=False)

        self.__modify_REG_COOLCONF('sg_stall_value', self.sg_stall_value, send=False)

        # TRAMS default: 0x1084 | stepper_direction [ Normal = 0x00 or inverse 0x10 ]
        # Default bin: 0b10 000 1000 0100
        self.__modify_REG_GCONF('shaft_dir', self._direction, send=False)
        self.__modify_REG_GCONF('diag0_active_high', self.diag0act_high, send=False)
        self.__modify_REG_GCONF('diag1_active_high', self.diag1act_high, send=False)
        # False = spreadCycle, True = stealthChop
        self.__modify_REG_GCONF('stealthChop', int(self.silent_mode), send=False)
        if self.silent_mode is True:
            # TODO: Verify PWMCONF values!
            self.__modify_REG_PWMCONF('stealth_autoscale', 1, send=False)
            self.__modify_REG_PWMCONF('stealth_gradient', 5, send=False)
            self.__modify_REG_PWMCONF('stealth_amplitude', 255, send=False)
            self.__modify_REG_PWMCONF('stealth_freq', "fPWM_2/683", send=False)

            if self.hybrid_threshold is not None:
                self.__set_REG_TPWMTHRS(self.hybrid_threshold)
                self.logger.info("Hybrid threshold: {}".
                                 format(self.hybrid_threshold))
            elif self.stealth_max_speed is not None:
                speed = int(12650000 * self.microsteps /
                            (self.stealth_max_speed * self.inv_step_dist * 256))
                self.__set_REG_TPWMTHRS(speed)
                self.logger.debug("Stealth max speed: %u (cfg val: %u)" % (
                    self.stealth_max_speed, speed))

        # ========== Set diag pins ==========
        self.__modify_REG_GCONF('diag0_stall', 0, send=False)
        self.__modify_REG_GCONF('diag0_temp_prewarn', 0, send=False)
        self.__modify_REG_GCONF('diag0_errors', 0, send=False)
        if self.diag0purpose is 0:
            self.__modify_REG_GCONF('diag0_errors', 1, send=False)
        elif self.diag0purpose is 1:
            self.__modify_REG_GCONF('diag0_temp_prewarn', 1, send=False)
        elif self.diag0purpose is 2:
            self.__modify_REG_GCONF('diag0_stall', 1, send=False)

        self.__modify_REG_GCONF('diag1_steps_skipped', 0, send=False)
        self.__modify_REG_GCONF('diag1_chopper_on', 0, send=False)
        self.__modify_REG_GCONF('diag1_index', 0, send=False)
        self.__modify_REG_GCONF('diag1_stall', 0, send=False)
        if self.diag1purpose is 0:
            self.__modify_REG_GCONF('diag1_stall', 1, send=False)
        elif self.diag1purpose is 1:
            self.__modify_REG_GCONF('diag1_index', 1, send=False)
        elif self.diag1purpose is 2:
            self.__modify_REG_GCONF('diag1_chopper_on', 1, send=False)
        elif self.diag1purpose is 3:
            self.__modify_REG_GCONF('diag1_steps_skipped', 1, send=False)

        # Send configurations to driver
        self.logger.debug("Send configurations")
        self.__set_REG_CHOPCONF(self.val_CHOPCONF)
        self.__set_REG_COOLCONF(self.val_COOLCONF)
        self.__set_REG_PWMCONF(self.val_PWMCONF)
        self.__set_REG_GCONF(self.val_GCONF)
        # Verify configurations
        self.logger.debug("Verify configurations")
        self.__get_REG_DRV_STATUS()
        self.__validate_cfg()
        self.logger.debug("Init ready")

    def __calc_rms_current(self,
                           current = None,
                           multip_for_holding_current = None,
                           hold_delay = None,
                           init=False):
        # TRAMS defaults: IRun 25, IHold 8, IHoldDelay 7
        CS = None
        if current:
            if MIN_CURRENT < current:
                current /= 1000.
            sense_r = self.sense_r
            CS = 32.0 * 1.41421 * current * (sense_r + 0.02) / 0.325 - 1.
            # If Current Scale is too low, turn on high sensitivity R_sense and calculate again
            if CS < 16:
                self.__modify_REG_CHOPCONF('vsense', 1, send=(not init))
                CS = 32.0 * 1.41421 * current * (sense_r + 0.02) / 0.180 - 1.
            else:
                # If CS >= 16, turn off high_sense_r if it's currently ON
                self.__modify_REG_CHOPCONF('vsense', 0, send=(not init))
            iRun  = int(CS)
            self.__modify_REG_IHOLD_IRUN('IRUN', iRun, send=False)
            self.current = current
        if multip_for_holding_current:
            if CS is None:
                CS = self.__read_REG_IHOLD_IRUN('IRUN')
            iHold = int(CS * multip_for_holding_current)
            self.__modify_REG_IHOLD_IRUN('IHOLD', iHold, send=False)
            self.hold_multip = multip_for_holding_current
        if hold_delay:
            self.__modify_REG_IHOLD_IRUN('IHOLDDELAY', hold_delay, send=False)
            self.hold_delay  = hold_delay
        self.__set_REG_IHOLD_IRUN(self.val_IHOLD_IRUN)
        self.logger.debug("RMS current %.3fA => IHold=%u, IRun=%u, IHoldDelay=%u" % (
            self.current,
            self.__read_REG_IHOLD_IRUN('IHOLD'),
            self.__read_REG_IHOLD_IRUN('IRUN'),
            self.__read_REG_IHOLD_IRUN('IHOLDDELAY')))

    def __get_rms_current(self):
        vsense = self.__read_REG_CHOPCONF('vsense')
        CS     = self.__read_REG_IHOLD_IRUN('IRUN')
        V_fs   = 0.325
        if vsense is 1:
            V_fs = 0.180
        return ( CS + 1. ) / 32.0 * V_fs / (self.sense_r + 0.02) / 1.41421

    #**************************************************************************
    # GENERAL CONFIGURATION REGISTERS (0x00..0x0F)
    #**************************************************************************

    #==================== GCONF ====================
    '''
        external_ref        0/1    Use external voltage reference for coil currents
        internal_Rsense     0/1    Use internal sense resistors
        stealthChop         0/1    Enable stealthChop (dependant on velocity thresholds)
        commutation         0/1    Enable commutation by full step encoder
        shaft_dir           0/1    Inverse motor direction
        diag0_errors        0/1    Enable DIAG0 active on driver errors:
                                       Over temperature (ot),
                                       short to GND (s2g),
                                       undervoltage chargepump (uv_cp)
        diag0_temp_prewarn  0/1    Enable DIAG0 active on driver over temperature prewarning
        diag0_stall         0/1    Enable DIAG0 active on motor stall (set TCOOLTHRS before using this feature)
        diag1_stall         0/1    Enable DIAG1 active on motor stall (set TCOOLTHRS before using this feature)
        diag1_index         0/1    Enable DIAG1 active on index position (microstep look up table position 0)
        diag1_chopper_on    0/1    Enable DIAG1 active when chopper is on
        diag1_steps_skipped 0/1    Enable output toggle when steps are skipped in dcStep mode (increment of LOST_STEPS).
                                   Do not enable in conjunction with other DIAG1 options.
        diag0_active_high   0/1    0: DIAG0 is open collector output (active low)
                                   1: Enable DIAG0 push pull output (active high)
        diag1_active_high   0/1    0: DIAG1 is open collector output (active low)
                                   1: Enable DIAG1 push pull output (active high)
        small_hysterisis    0/1    0: Hysteresis for step frequency comparison is 1/16
                                   1: Hysteresis for step frequency comparison is 1/32
        stop_enable         0/1    Emergency stop: DCIN stops the sequencer when tied high
                                   (no steps become executed by the sequencer, motor goes to standstill state).
        direct_mode         0/1    Motor coil currents and polarity are directly controlled by the SPI interface.
    '''
    GCONF_reg_def = {
        #'test_mode'           : [(17, 0b1, None)], # Not allowed to modify
        #'direct_mode'         : [(16, 0b1, None)], # Not allowed to modify
        'stop_enable'         : [(15, 0b1, None)],
        'small_hysterisis'    : [(14, 0b1, None)],
        'diag1_active_high'   : [(13, 0b1, None)],
        'diag0_active_high'   : [(12, 0b1, None)],
        'diag1_steps_skipped' : [(11, 0b1, None)],
        'diag1_chopper_on'    : [(10, 0b1, None)],
        'diag1_index'         : [( 9, 0b1, None)],
        'diag1_stall'         : [( 8, 0b1, None)],
        'diag0_stall'         : [( 7, 0b1, None)],
        'diag0_temp_prewarn'  : [( 6, 0b1, None)],
        'diag0_errors'        : [( 5, 0b1, None)],
        'shaft_dir'           : [( 4, 0b1, None)],
        #'commutation'         : [( 3, 0b1, None)], # Not allowed to modify
        'stealthChop'         : [( 2, 0b1, None)],
        'internal_Rsense'     : [( 1, 0b1, None)],
        'external_ref'        : [( 0, 0b1, None)],
    }
    def __get_REG_GCONF(self):
        self.val_GCONF = val = self._command_read(REG_GCONF)
        return val
    def __set_REG_GCONF(self, val):
        self.val_GCONF = val
        self._command_write(REG_GCONF, val)
    def __modify_REG_GCONF(self, name, val, send=True):
        reg = REG_GCONF if send else None
        self.val_GCONF = \
            self.modify_reg(self.GCONF_reg_def[name],
                            self.val_GCONF, val, reg)
    def __read_REG_GCONF(self, name):
        return self.read_reg(self.GCONF_reg_def[name], self.val_GCONF)

    #==================== GSTAT ====================
    def __get_GSTAT(self, log=list()):
        # R+C
        status = self._command_read(REG_GSTAT)
        if status:
            if status & 0b001:
                dump = "GSTAT: Reset has occurred!"
                self.logger.error(dump)
                log.append(dump)
            if status & 0b010:
                dump = "GSTAT: Shut down due to overtemperature or short circuit!"
                self.logger.error(dump)
                log.append(dump)
            if status & 0b100:
                dump = "GSTAT: Undervoltage occured - driver is disabled!"
                self.logger.error(dump)
                log.append(dump)
        return status

    #==================== IOIN ====================
    def __get_IOIN(self, log=None):
        # R
        status = self._command_read(REG_IOIN)

        STEP      = 'HIGH' if (status & 0b00000001) else 'LOW'
        DIR       = 'HIGH' if (status & 0b00000010) else 'LOW'
        DCEN      = 'HIGH' if (status & 0b00000100) else 'LOW'
        DCIN      = 'HIGH' if (status & 0b00001000) else 'LOW'
        DRV_ENN   = 'HIGH' if (status & 0b00010000) else 'LOW'
        DCO       = 'HIGH' if (status & 0b00100000) else 'LOW'
        #SD_MODE   = 'HIGH' if (status & 0b01000000) else 'LOW'
        #SWCOMP_IN = 'HIGH' if (status & 0b10000000) else 'LOW'
        VERSION = (status >> 24) & 0xFF

        self.logger.info("DRV_ENN_CFG6 pin %s" % DRV_ENN)
        self.logger.info("STEP pin %s" % STEP)
        self.logger.info("DIR pin %s" % DIR)
        self.logger.info("DCEN_CFG4 pin %s" % DCEN)
        self.logger.info("DCIN_CFG5 pin %s" % DCIN)
        self.logger.info("DCO pin %s" % DCO)
        self.logger.info("IC VERSION %s" % VERSION)
        if log is not None:
            log.append("DRV_ENN_CFG6 pin %s" % DRV_ENN)
            log.append("STEP pin %s" % STEP)
            log.append("DIR pin %s" % DIR)
            log.append("DCEN_CFG4 pin %s" % DCEN)
            log.append("DCIN_CFG5 pin %s" % DCIN)
            log.append("DCO pin %s" % DCO)
            log.append("IC VERSION %s" % VERSION)
        return status

    #==================== X_COMPARE ====================
    def __set_X_COMPARE(self, val):
        # W
        self._command_write(REG_X_COMPARE, val & 0xFFFFFFFF)

    #**************************************************************************
    # VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10..0x1F)
    #**************************************************************************
    #==================== IHOLD_IRUN ====================
    '''
        hold_current 0..31   Standstill current (0=1/32...31=32/32)
        run_current  0..31   Motor run current (0=1/32...31=32/32)
        hold_delay   0..15   Controls the number of clock cycles for motor power down after a
                             motion as soon as standstill is detected (stst=1) and TPOWERDOWN has expired.
    '''
    IHOLD_IRUN_reg_def = {
        'IHOLD'      : [( 0, 0b11111, None)],
        'IRUN'       : [( 8, 0b11111, None)],
        'IHOLDDELAY' : [(16, 0b1111,  None)],
    }
    def __get_REG_IHOLD_IRUN(self):
        self.val_IHOLD_IRUN = val = self._command_read(REG_IHOLD_IRUN)
        return val
    def __set_REG_IHOLD_IRUN(self, val):
        self.val_IHOLD_IRUN = val
        self._command_write(REG_IHOLD_IRUN, val)
    def __modify_REG_IHOLD_IRUN(self, name, val, send=True):
        reg = REG_IHOLD_IRUN if send else None
        self.val_IHOLD_IRUN = \
            self.modify_reg(self.IHOLD_IRUN_reg_def[name],
                            self.val_IHOLD_IRUN, val, reg)
    def __read_REG_IHOLD_IRUN(self, name):
        return self.read_reg(self.IHOLD_IRUN_reg_def[name],
                             self.val_IHOLD_IRUN)

    #========================================
    def __set_REG_TPOWERDOWN(self, power_down_delay):
        """
        Range 0...255
        power_down_delay sets the delay time after stand still (stst) of the motor
        to motor current power down. Time range is about 0 to 4 seconds.
        delay: 0...((2^8)-1) * 2^18 tCLK
        """
        if power_down_delay < 0:
            power_down_delay = 0
        elif 0xFF < power_down_delay:
            power_down_delay = 0xFF
        self._command_write(REG_TPOWERDOWN, power_down_delay)

    #========================================
    def __get_REG_TSTEP(self):
        """
        Read the actual measured time between two 1/256 microsteps
        derived from the step input frequency in units of 1/fCLK.

        microstep velocity time reference t for velocities: TSTEP = fCLK / fSTEP
        """
        return self._command_read(REG_TSTEP) & 0xFFFFF

    #========================================
    def __set_REG_TPWMTHRS(self, stealthchop_max_speed):
        """
        0..1,048,575
        This is the upper velocity for stealthChop voltage PWM mode.
        TSTEP >= TPWMTHRS:
              - stealthChop PWM mode is enabled if configured
              - dcStep is disabled
        """
        if stealthchop_max_speed < 0:
            stealthchop_max_speed = 0
        elif 0xFFFFF < stealthchop_max_speed:
            stealthchop_max_speed = 0xFFFFF
        self._command_write(REG_TPWMTHRS, stealthchop_max_speed)

    #========================================
    def __set_REG_TCOOLTHRS(self, coolstep_min_speed):
        """
        0..1,048,575
        This is the lower threshold velocity for switching on smart
        energy coolStep and stallGuard feature

        Set this parameter to disable coolStep at low speeds, where it
        cannot work reliably. The stall detection and stallGuard output
        signal becomes enabled when exceeding this velocity. In nondcStep
        mode, it becomes disabled again once the velocity falls
        below this threshold.

        TCOOLTHRS >= TSTEP >= THIGH:
          - coolStep is enabled, if configured
          - stealthChop voltage PWM mode is disabled
        TCOOLTHRS >= TSTEP
          - Stop on stall and stall output signal is enabled, if configured
        """
        if coolstep_min_speed < 0:
            coolstep_min_speed = 0
        elif 0xFFFFF < coolstep_min_speed:
            coolstep_min_speed = 0xFFFFF
        self._command_write(REG_TCOOLTHRS, coolstep_min_speed)

    #========================================
    def __set_REG_THIGH(self, mode_sw_speed):
        """
        0..1,048,575
        This velocity setting allows velocity dependent switching into a
        different chopper mode and fullstepping to maximize torque.

        The stall detection feature becomes switched off for 2-3
        electrical periods whenever passing THIGH threshold to
        compensate for the effect of switching modes.

        TSTEP <= THIGH:
          - coolStep is disabled (motor runs with normal current scale)
          - stealthChop voltage PWM mode is disabled
          - If vhighchm is set, the chopper switches to chm=1
            with TFD=0 (constant off time with slow decay, only).
          - chopSync2 is switched off (SYNC=0)
          - If vhighfs is set, the motor operates in fullstep mode
            and the stall detection becomes switched over to
            dcStep stall detection.
        """
        if mode_sw_speed < 0:
            mode_sw_speed = 0
        elif 0xFFFFF < mode_sw_speed:
            mode_sw_speed = 0xFFFFF
        self._command_write(REG_THIGH, mode_sw_speed)

    #**************************************************************************
    # RAMP GENERATOR MOTION CONTROL REGISTER SET (0x20...0x2D)
    #**************************************************************************
    #==================== REG_RAMPMODE ====================
    def __set_REG_RAMPMODE(self, mode):
        """
        :param mode: defines the used ramp calculation mode
            positioning     - using all A, D and V parameters
            velocity pos    - positive VMAX, using AMAX acceleration
            velocity neg    - negative VMAX, using AMAX acceleration
            hold            - velocity remains unchanged, unless stop event occurs
        :return:
        """
        modes = { 'positioning': 0,
                  'velocity pos': 1,
                  'velocity neg': 2,
                  'hold': 3}
        self.__ramp_mode = mode
        self._command_write(REG_RAMPMODE, modes[mode])
    def __get_REG_RAMPMODE(self):
        return self.__ramp_mode

    #==================== REG_XACTUAL ====================
    def __set_REG_XACTUAL(self, val):
        self._command_write(REG_XACTUAL, int(val) & 0xFFFFFFFF)
    def __get_REG_XACTUAL(self):
        return self._command_read(REG_XACTUAL) & 0xFFFFFFFF

    #==================== REG_VACTUAL ====================
    def __get_REG_VACTUAL(self):
        return self._command_read(REG_VACTUAL) & 0x3FFFFF

    #==================== REG_VSTART ====================
    def __set_REG_VSTART(self, val):
        self._command_write(REG_VSTART, int(val) & 0x3FFFF)

    #==================== REG_A1 ====================
    def __set_REG_A1(self, val):
        self._command_write(REG_A1, int(val) & 0xFFFF)

    #==================== REG_V1 ====================
    def __set_REG_V1(self, val):
        self._command_write(REG_V1, int(val) & 0xFFFFF)

    #==================== REG_AMAX ====================
    def __set_REG_AMAX(self, val):
        self._command_write(REG_AMAX, int(val) & 0xFFFF)

    #==================== REG_VMAX ====================
    def __set_REG_VMAX(self, val):
        self._command_write(REG_VMAX, int(val) & 0x7FFFFF)

    #==================== REG_DMAX ====================
    def __set_REG_DMAX(self, val):
        self._command_write(REG_DMAX, int(val) & 0xFFFF)

    #==================== REG_D1 ====================
    def __set_REG_D1(self, val):
        self._command_write(REG_D1, int(val) & 0xFFFF)

    #==================== REG_VSTOP ====================
    def __set_REG_VSTOP(self, val):
        self._command_write(REG_VSTOP, int(val) & 0x3FFFF)

    #==================== REG_TZEROWAIT ====================
    def __set_REG_TZEROWAIT(self, val):
        self._command_write(REG_TZEROWAIT, int(val) & 0xFFFF)

    #==================== REG_XTARGET ====================
    def __set_REG_XTARGET(self, val):
        self._command_write(REG_XTARGET, int(val) & 0xFFFFFFFF)
    def __get_REG_XTARGET(self):
        return self._command_read(REG_XTARGET) & 0xFFFFFFFF

    #**************************************************************************
    # RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0x30...0x36)
    #**************************************************************************
    #==================== VDCMIN ====================
    def __set_REG_VDCMIN(self, dcstep_min_speed):
        """
        0..8,388,607
        The automatic commutation dcStep becomes enabled by the external signal DCEN.
        VDCMIN is used as the minimum step velocity when the motor is heavily loaded.
        Hint: Also set DCCTRL parameters in order to operate dcStep.

        time reference t for VDCMIN: t = 2^24 / fCLK
        """
        if dcstep_min_speed < 0:
            dcstep_min_speed = 0
        elif 0x7FFFFF < dcstep_min_speed:
            dcstep_min_speed = 0x7FFFFF
        self._command_write(REG_VDCMIN, dcstep_min_speed)

    #==================== SW_MODE ====================
    SW_MODE_reg_def = {
        'en_softstop': [(11, 0b1, None)],
        'sg_stop': [(10, 0b1, None)],
        'en_latch_encoder': [(9, 0b1, None)],
        'latch_r_inactive': [(8, 0b1, None)],
        'latch_r_active': [(7, 0b1, None)],
        'latch_l_inactive': [(6, 0b1, None)],
        'latch_l_active': [(5, 0b1, None)],
        'swap_lr': [(4, 0b1, None)],
        'pol_stop_r': [(3, 0b1, None)], # 0 = HIGH, 1 = LOW to stop motor
        'pol_stop_l': [(2, 0b1, None)],
        'stop_r_enable': [(1, 0b1, None)],
        'stop_l_enable': [(0, 0b1, None)],
    }
    def __get_REG_SW_MODE(self):
        self.val_SW_MODE = val = self._command_read(REG_SW_MODE)
        return val
    def __set_REG_SW_MODE(self, val):
        self.val_SW_MODE = val
        self._command_write(REG_SW_MODE, val)
    def __modify_REG_SW_MODE(self, name, val, send=True):
        """
        TRAMS Endstop Config:
           Left endstop:  0x21 = 0b10 0001
           Right endstop: 0x11 = 0b01 0001
           High active |= 0x00
           Low active  |= 0x0C
        """
        reg = REG_SW_MODE if send else None
        self.val_SW_MODE = \
            self.modify_reg(self.SW_MODE_reg_def[name],
                            self.val_SW_MODE, val, reg)
    def __read_REG_SW_MODE(self, name):
        return self.read_reg(self.SW_MODE_reg_def[name], self.val_SW_MODE)

    #==================== RAMP_STAT ====================
    RAMP_STAT_reg_def = {
        'status_sg': [(13, 0b1, None)],
        'second_move': [(12, 0b1, None)],
        't_zerowait_active': [(11, 0b1, None)],
        'vzero': [(10, 0b1, None)],
        'position_reached': [(9, 0b1, None)],
        'velocity_reached': [(8, 0b1, None)],
        'event_pos_reached': [(7, 0b1, None)],
        'event_stop_sg': [(6, 0b1, None)],
        'event_stop_r': [(5, 0b1, None)],
        'event_stop_l': [(4, 0b1, None)],
        'status_latch_r': [(3, 0b1, None)],
        'status_latch_l': [(2, 0b1, None)],
        'status_stop_r': [(1, 0b1, None)],
        'status_stop_l': [(0, 0b1, None)],
    }
    def __get_REG_RAMP_STAT(self):
        # R+C
        self.val_RAMP_STAT = self._command_read(REG_RAMP_STAT)
        return self.val_RAMP_STAT
    def __read_REG_RAMP_STAT(self, name, refresh=False):
        val = self.__get_REG_RAMP_STAT() \
            if refresh else self.val_RAMP_STAT
        return self.read_reg(self.RAMP_STAT_reg_def[name], val)

    #==================== XLATCH ====================
    def __get_REG_XLATCH(self):
        return self._command_read(REG_XLATCH) & 0xFFFFFFFF

    #**************************************************************************
    # ENCODER REGISTER SET (0x38...0x3C)
    #**************************************************************************
    #==================== ENCMODE ====================
    ENCMODE_reg_def = {
        'enc_sel_decimal': [(10, 0b1, None)],
        'latch_x_act': [(9, 0b1, None)],
        'clr_enc_x': [(8, 0b1, None)],
        'neg_edge': [(7, 0b1, None)],
        'pos_edge': [(6, 0b1, None)],
        'clr_once': [(5, 0b1, None)],
        'clr_cont': [(4, 0b1, None)],
        'ignore_AB': [(3, 0b1, None)],
        'pol_N': [(2, 0b1, None)],
        'pol_B': [(1, 0b1, None)],
        'pol_A': [(0, 0b1, None)],
    }
    def __get_REG_ENCMODE(self):
        self.val_ENCMODE = val = self._command_read(REG_ENCMODE)
        return val
    def __set_REG_ENCMODE(self, val):
        self.val_ENCMODE = val
        self._command_write(REG_ENCMODE, val)
    def __modify_REG_ENCMODE(self, name, val, send=True):
        # R+W
        reg = REG_ENCMODE if send else None
        self.val_ENCMODE = \
            self.modify_reg(self.ENCMODE_reg_def[name],
                            self.val_ENCMODE, val, reg)
    def __read_REG_ENCMODE(self, name):
        return self.read_reg(self.ENCMODE_reg_def[name], self.val_ENCMODE)

    #==================== X_ENC ====================
    def __set_REG_X_ENC(self, val):
        return self._command_write(REG_X_ENC, val & 0xFFFFFFFF)
    def __get_REG_X_ENC(self):
        return self._command_read(REG_X_ENC) & 0xFFFFFFFF

    #==================== ENC_LATCH ====================
    def __set_REG_ENC_CONST(self, val):
        # TODO Check integer and fractional parts!
        return self._command_write(REG_ENC_CONST, val)

    #==================== ENC_STATUS ====================
    def __get_REG_ENC_STATUS(self):
        return self._command_read(REG_ENC_STATUS) & 0x1

    #==================== ENC_LATCH ====================
    def __get_REG_ENC_LATCH(self):
        return self._command_read(REG_ENC_LATCH) & 0xFFFFFFFF

    #**************************************************************************
    # MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
    #**************************************************************************
    # TODO FIXME!

    #**************************************************************************
    # DRIVER REGISTER SET (0X6C...0X7F)
    #**************************************************************************

    #==================== CHOPCONF ====================
    '''
        off_time,                  0..15              Off time setting controls duration of slow decay phase
                                                      NCLK=12 + 32*TOFF Initialized to value 2 (NCLK = 76) by begin()
        hysterisis_start,          1..8               Add 1, 2, ..., 8 to hysteresis low value HEND (1/512 of this
                                                      setting adds to current setting)
                                                      Attention: Effective HEND+HSTRT <= 16.
                                                      Hint: Hysteresis decrement is done each 16 clocks
        fast_decay_time,           0..15              Fast decay time setting TFD with NCLK= 32*HSTRT
        hysterisis_end,            -3..12             This is the hysteresis value which becomes used for the hysteresis chopper.
        sine_offset,               -3..12             This is the sine wave offset and 1/512 of the value becomes
                                                      added to the absolute value of each sine wave entry.
        disable_I_comparator,      0/1                1: Disables current comparator usage for termination of the
                                                         fast decay cycle. chopper_mode needs to be 1.
        random_off_time,           0/1                0: Chopper off time is fixed as set by TOFF
                                                      1: Random mode, TOFF is random modulated by dNCLK= -12 ... +3 clocks.
        chopper_mode,              0/1                0: Standard mode (spreadCycle)
                                                      1: Constant off time with fast decay time.
                                                      Fast decay time is also terminated when the negative nominal
                                                      current is reached. Fast decay is after on time.
        blank_time,                16, 24, 36, 54     Set comparator blank time to 16, 24, 36 or 54 clocks.
                                                      Hint: 24 or 36 is recommended for most applications initialized
                                                            to 36 (register value = 3) by begin()
        vsense,                    0/1                0: Low sensitivity, high sense resistor voltage
                                                      1: High sensitivity, low sense resistor voltage
        fullstep_threshold,        0/1                This bit enables switching to fullstep, when VHIGH is exceeded.
                                                      Switching takes place only at 45deg position. The fullstep target
                                                      current uses the current value from the microstep table at
                                                      the 45deg position.
        high_speed_mode,           0/1                This bit enables switching to chm=1 and fd=0, when VHIGH is exceeded.
                                                      This way, a higher velocity can be achieved. Can be combined with
                                                      vhighfs=1. If set, the TOFF setting automatically becomes doubled
                                                      during high velocity operation in order to avoid doubling of the
                                                      chopper frequency.
        sync_phases,               0..15              Synchronization of the chopper for both phases of a two phase motor
                                                      in order to avoid the occurrence of a beat, especially at low motor
                                                      velocities. It is automatically switched off above VHIGH.
        microsteps,                255, 128, 64, 32,  Reduced microstep resolution for Step/Dir operation.
                                   16, 8, 4, 2,         The resolution gives the number of microstep entries
                                   0 (FULLSTEP)         per sine quarter wave.
        interpolate,               0/1                The actual microstep resolution becomes extrapolated
                                                      to 256 microsteps for smoothest motor operation.
        double_edge_step,          0/1                Enable step impulse at each step edge to reduce step frequency requirement.
        disable_short_protection,  0/1                0: Short to GND protection is on
                                                      1: Short to GND protection is disabled
    '''
    msteps_map = {
        256 : 0b0000, # 0 : 256x microstepping (Native)
        128 : 0b0001, # 1 : 128x microstepping
        64  : 0b0010, # 2 :  64x microstepping
        32  : 0b0011, # 3 :  32x microstepping
        16  : 0b0100, # 4 :  16x microstepping
        8   : 0b0101, # 5 :   8x microstepping
        4   : 0b0110, # 6 :   4x microstepping
        2   : 0b0111, # 7 :   2x microstepping
        1   : 0b1000  # 8 :   1x full-step
    }
    blank_time_map = {
        16 : 0b00,
        24 : 0b01,    # Recommended for most applications.
        36 : 0b10,    # Recommended for most applications.
        54 : 0b11
    }
    CHOPCONF_reg_def = {
        'off_time'                 : [ ( 0, 0b1111, None ) ],

        # if chopper_mode == 0 :
        'hysterisis_start'         : [ ( 4, 0b111,  None ) ],
        'hysterisis_end'           : [ ( 7, 0b1111, None ) ],

        # if chopper_mode == 1 :
        'fast_decay_time'          : [ ( 4, 0b0111, None ), ( 8, 0b1000, None ) ],
        'sine_offset'              : [ ( 7, 0b1111, None ) ],

        'disable_I_comparator'     : [ (12, 0b1,    None) ],
        'random_off_time'          : [ (13, 0b1,    None) ],
        'chopper_mode'             : [ (14, 0b1,    None) ],
        'blank_time'               : [ (15, 0b11,   blank_time_map) ],
        'vsense'                   : [ (17, 0b1,    None) ],
        'fullstep_threshold'       : [ (18, 0b1,    None) ],
        'high_speed_mode'          : [ (19, 0b1,    None) ],
        'sync_phases'              : [ (20, 0b1111, None) ],
        'microsteps'               : [ (24, 0b1111, msteps_map) ],
        'interpolate'              : [ (28, 0b1,    None) ],
        'double_edge_step'         : [ (29, 0b1,    None) ],
        'disable_short_protection' : [ (30, 0b1,    None) ],
    }
    def __get_REG_CHOPCONF(self):
        self.val_CHOPCONF = val = self._command_read(REG_CHOPCONF)
        return val
    def __set_REG_CHOPCONF(self, val):
        self.val_CHOPCONF = val
        self._command_write(REG_CHOPCONF, val)
    def __modify_REG_CHOPCONF(self, name, val, send=True):
        reg = REG_CHOPCONF if send else None
        self.val_CHOPCONF = \
            self.modify_reg(self.CHOPCONF_reg_def[name],
                            self.val_CHOPCONF, val, reg)
    def __read_REG_CHOPCONF(self, name):
        return self.read_reg(self.CHOPCONF_reg_def[name], self.val_CHOPCONF)

    #==================== COOLCONF ====================
    '''
        sg_min               0..15       If the stallGuard2 result falls below sg_min*32, the motor current
                                         becomes increased to reduce motor load angle.
        sg_max               0..15       If the stallGuard2 result is equal to or above (sg_min+sg_max+1)*32,
                                         the motor current becomes decreased to save energy.
        sg_step_width        1, 2, 4, 8  Current increment steps per measured stallGuard2 value
        sg_current_decrease  1, 2, 8, 32 For each (value) stallGuard2 values decrease by one
        smart_min_current    0/1         0: 1/2 of current setting (IRUN)
                                         1: 1/4 of current setting (IRUN)
        sg_stall_value       int         This signed value controls stallGuard2 level for stall output and
                                         sets the optimum measurement range for readout. A lower value gives
                                         a higher sensitivity. Zero is the starting value working with most motors.
                                         -64 to +63: A higher value makes stallGuard2 less sensitive and requires
                                         more torque to indicate a stall.
        sg_filter            0/1         0: Standard mode, high time resolution for stallGuard2
                                         1: Filtered mode, stallGuard2 signal updated for each four fullsteps
                                            (resp. six fullsteps for 3 phase motor) only to compensate
                                            for motor pole tolerances
    '''
    seup_t = {
        1 : 0b00, # Current increment steps per measured stallGuard2 value = 1
        2 : 0b01, # Current increment steps per measured stallGuard2 value = 2
        4 : 0b10, # Current increment steps per measured stallGuard2 value = 4
        8 : 0b11  # Current increment steps per measured stallGuard2 value = 8
    }
    sedn_t = {
        32 : 0b00, # %00: For each 32 stallGuard2 values decrease by one
        8  : 0b01, # %01: For each 8  stallGuard2 values decrease by one
        2  : 0b10, # %10: For each 2  stallGuard2 values decrease by one
        1  : 0b11  # %11: For each    stallGuard2 value  decrease by one
    }
    COOLCONF_reg_def = {
        'sg_min'              : [ ( 0, 0b1111,    None  ) ],
        'sg_step_width'       : [ ( 5, 0b11,      seup_t) ],
        'sg_max'              : [ ( 8, 0b1111,    None  ) ],
        'sg_current_decrease' : [ (13, 0b11,      sedn_t) ],
        'smart_min_current'   : [ (15, 0b1,       None  ) ],
        'sg_stall_value'      : [ (16, 0b1111111, None  ) ],
        'sg_filter'           : [ (24, 0b1,       None  ) ],
    }
    def __get_REG_COOLCONF(self):
        self.val_COOLCONF = val = self._command_read(REG_COOLCONF)
        return val
    def __set_REG_COOLCONF(self, val):
        self.val_COOLCONF = val
        self._command_write(REG_COOLCONF, val)
    def __modify_REG_COOLCONF(self, name, val, send=True):
        reg = REG_COOLCONF if send else None
        self.val_COOLCONF = \
            self.modify_reg(self.COOLCONF_reg_def[name],
                            self.val_COOLCONF, val, reg)
    def __read_REG_COOLCONF(self, name):
        return self.read_reg(self.COOLCONF_reg_def[name], self.val_COOLCONF)

    #==================== DRV_STATUS ====================
    def __get_REG_DRV_STATUS(self, log=list(), check=True):
        val = self._command_read(REG_DRV_STATUS)
        if check:
            if val & 0b10000000000000000000000000000000:
                # 31: standstill indicator
                dump = "Stand still"
                self.logger.info(dump)
                log.append(dump)
            if val & 0b01000000000000000000000000000000:
                # 30: open load indicator phase B
                dump = "Phase B open load"
                self.logger.error(dump)
                log.append(dump)
            if val & 0b00100000000000000000000000000000:
                # 29: open load indicator phase A
                dump = "Phase A open load"
                self.logger.error(dump)
                log.append(dump)
            if val & 0b00010000000000000000000000000000:
                # 28: short to ground indicator phase B
                dump = "Phase B short to ground"
                self.logger.error(dump)
                log.append(dump)
            if val & 0b00001000000000000000000000000000:
                # 27: short to ground indicator phase A
                dump = "Phase A short to ground"
                self.logger.error(dump)
                log.append(dump)
            if val & 0b00000100000000000000000000000000:
                # 26: overtemperature prewarning
                dump = "Over temperature prewarning!"
                self.logger.warning(dump)
                log.append(dump)
            if val & 0b00000010000000000000000000000000:
                # 25: overtemperature
                dump = "Over temperature!"
                self.logger.error(dump)
                log.append(dump)
            if val & 0b00000001000000000000000000000000:
                # 24: stallGuard2 status
                dump = "motor stall"
                self.logger.error(dump)
                log.append(dump)

            # 20-16: actual motor current / smart energy current
            CS_ACTUAL = (val & 0b00000000000111110000000000000000) >> 16

            if val & 0b00000000000000001000000000000000:
                # 15: full step active indicator
                dump = "driver has switched to fullstep"
                self.logger.warning(dump)
                log.append(dump)

            # Stall Guard status
            SG_RESULT = (val & 0b00000000000000000000001111111111)

            dump = "DRV_STATUS : Current (CS) = %d, stallGuard2 result (SG) = %d" % \
                   (CS_ACTUAL, SG_RESULT)
            self.logger.info(dump)
            log.append(dump)

        return val

    #==================== PWMCONF ====================
    '''
        stealth_amplitude     0..255  pwm_autoscale = 0
                                        User defined PWM amplitude offset (0-255)
                                        The resulting amplitude (limited to 0...255) is:
                                        PWM_AMPL + PWM_GRAD * 256 / TSTEP
                                      pwm_autoscale = 1
                                        User defined maximum PWM amplitude when switching back
                                        from current chopper mode to voltage PWM mode
                                        (switch over velocity defined by TPWMTHRS).
                                        Do not set too low values, as the regulation cannot
                                        measure the current when the actual PWM value goes below
                                        a setting specific value. Settings above 0x40 recommended.

        stealth_gradient      0..255  pwm_autoscale = 0
                                        Velocity dependent gradient for PWM amplitude:
                                        PWM_GRAD * 256 / TSTEP is added to PWM_AMPL
                                      pwm_autoscale = 1
                                        User defined maximum PWM amplitude change per
                                        half wave (1 to 15)

        stealth_freq          0..3    0: fPWM=2/1024 fCLK
                                      1: fPWM=2/683 fCLK
                                      2: fPWM=2/512 fCLK
                                      3: fPWM=2/410 fCLK
        stealth_autoscale     0/1     0: User defined PWM amplitude. The current settings have no influence.
                                      1: Enable automatic current control Attention: When using a user
                                         defined sine wave table, the amplitude of this sine wave table
                                         should not be less than 244. Best results are obtained
                                         with 247 to 252 as peak values.
        stealth_symmetric     0/1     0: The PWM value may change within each PWM cycle (standard mode)
                                      1: A symmetric PWM cycle is enforced
        standstill_mode       0..3    Stand still option when motor current setting is zero (I_HOLD=0).
                                      0: Normal operation
                                      1: Freewheeling
                                      2: Coil shorted using LS drivers
                                      3: Coil shorted using HS drivers
    '''
    pwm_freq_t = {
        "fPWM_2/1024" : 0b00, # %00: fPWM = 2/1024 fCLK
        "fPWM_2/683"  : 0b01, # %01: fPWM = 2/683  fCLK
        "fPWM_2/512"  : 0b10, # %10: fPWM = 2/512  fCLK
        "fPWM_2/410"  : 0b11  # %11: fPWM = 2/410  fCLK
    }
    freewheel_t = {
        "FREEWHEEL_NORMAL"    : 0b00, # %00: Normal operation
        "FREEWHEEL_FREEWHEEL" : 0b01, # %01: Freewheeling
        "FREEWHEEL_SHORT_LS"  : 0b10, # %10: Coil shorted using LS drivers
        "FREEWHEEL_SHORT_HS"  : 0b11  # %11: Coil shorted using HS drivers
    }
    PWMCONF_reg_def = {
        'stealth_amplitude' : [ (  0, 0b11111111, None ) ],
        'stealth_gradient'  : [ (  8, 0b11111111, None ) ],
        'stealth_freq'      : [ ( 16, 0b11,       pwm_freq_t ) ],
        'stealth_autoscale' : [ ( 18, 0b1,        None ) ],
        'stealth_symmetric' : [ ( 19, 0b1,        None ) ],
        'standstill_mode'   : [ ( 20, 0b11,       freewheel_t ) ],
    }
    def __get_REG_PWMCONF(self):
        self.val_PWMCONF = val = self._command_read(REG_PWMCONF)
        return val
    def __set_REG_PWMCONF(self, val):
        self.val_PWMCONF = val
        self._command_write(REG_PWMCONF, val)
    def __modify_REG_PWMCONF(self, name, val, send=True):
        reg = REG_PWMCONF if send else None
        self.val_PWMCONF = \
            self.modify_reg(self.PWMCONF_reg_def[name],
                            self.val_PWMCONF, val, reg)
    def __read_REG_PWMCONF(self, name):
        return self.read_reg(self.PWMCONF_reg_def[name], self.val_PWMCONF)

    #========================================
    def __get_PWM_SCALE(self):
        val = self._command_read(REG_PWM_SCALE) & 0xFF
        return val

    #========================================
    def __set_ENCM_CTRL(self, inv=False, maxspeed=False):
        val = inv * 1 | maxspeed * 2
        self._command_write(REG_ENCM_CTRL, val)

    #========================================
    def __get_LOST_STEPS(self, log=list()):
        val = self._command_read(REG_LOST_STEPS) & 0xFFFFF
        if val:
            dump = "Lost steps: {}".format(val)
            self.logger.error(dump)
            log.append(dump)
        return val
