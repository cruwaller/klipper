# TMC2130 stepper driver control
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import types, struct
import binascii
from driverbase import SpiDriver
from mcu import error

#***************************************************
# Registers
#***************************************************
REG_GCONF      = 0x00
REG_GSTAT      = 0x01
REG_IOIN       = 0x04

REG_IHOLD_IRUN = 0x10
REG_TPOWERDOWN = 0x11
REG_TSTEP      = 0x12
REG_TPWMTHRS   = 0x13
REG_TCOOLTHRS  = 0x14
REG_THIGH      = 0x15

REG_XDIRECT    = 0x2D
REG_VDCMIN     = 0x33

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
MAX_CURRENT = 1400.
DEFAULT_SENSE_R = 0.11

class TMC2130(SpiDriver):
    # Error flags
    isReset      = False
    isError      = False
    isStallguard = False
    isStandstill = False

    val_GCONF = 0
    val_CHOPCONF = 0
    val_COOLCONF = 0
    val_PWMCONF = 0
    val_IHOLD_IRUN = 0  # read only

    def __init__(self, config):
        self._direction = 0
        SpiDriver.__init__(self, config)
        printer = config.get_printer()
        self.current = config.getfloat('current', 1000.0,
                                       above=MIN_CURRENT, maxval=MAX_CURRENT)
        self.sense_r = config.getfloat('sense_R', DEFAULT_SENSE_R, above=0.09)
        self.hold_multip = config.getfloat('hold_multiplier', 0.5, above=0., maxval=1.0)
        self.hold_delay = config.getint('hold_delay', 0, minval=0., maxval=15)
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
        # register command handlers
        self.gcode = gcode = printer.lookup_object('gcode')
        cmds = ["DRV_STATUS", "DRV_CURRENT", "DRV_STALLGUARD"]
        for cmd in cmds:
            gcode.register_mux_command(
                cmd, "DRIVER", self.name,
                getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
    cmd_DRV_STATUS_help = "args: DRIVER=driver_name"
    def cmd_DRV_STATUS(self, params):
        params['#input'].respond(self.status())
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
        params['#input'].respond(msg)
    cmd_DRV_SG_help = "args: DRIVER=driver_name [SG=val]"
    def cmd_DRV_STALLGUARD(self, params):
        sg = self.gcode.get_float('SG', params, default=None)
        params['#input'].respond(self.set_stallguard(sg))

    #**************************************************************************
    # PUBLIC METHODS
    #**************************************************************************

    def status(self):
        res = ["NAME: %s" % self.name]
        current = "RMS current: %.3fA" % self.__get_rms_current()
        self.logger.info(current)
        res.append(current)
        self.__get_GSTAT(log=res)
        self.__get_REG_DRV_STATUS(log=res)
        self.__get_IOIN(log=res)
        self.__get_LOST_STEPS(log=res)
        return "\n".join(res)

    def init_homing(self, enable=True, *args):
        if self.sensor_less_homing:
            if enable is True:
                self.__set_REG_TCOOLTHRS(0xFFFFF)
            else:
                self.__set_REG_TCOOLTHRS(0)

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
    action                       | data sent to TMC2130  | data received from TMC2130
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
                raise error("TMC2130 internal error! len(val) != 4")
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
        self.val_IHOLD_IRUN = 0 # read only
        # reset registers:
        self._command_write(REG_GCONF,    self.val_GCONF)
        self._command_write(REG_CHOPCONF, self.val_CHOPCONF)
        self._command_write(REG_COOLCONF, self.val_COOLCONF)
        self._command_write(REG_PWMCONF,  self.val_PWMCONF)
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
        self.__reset_driver()

        # Set RMS Current
        self.__calc_rms_current(self.current, self.hold_multip,
                                self.hold_delay, init=True)
        # Motor power down time after last movement (iHOLD current is used)
        self.__set_REG_TPOWERDOWN(self.t_power_down)

        self.__modify_REG_CHOPCONF('microsteps', self.microsteps, send=False)
        self.__modify_REG_CHOPCONF('blank_time', 24, send=False)
        self.__modify_REG_CHOPCONF('off_time', 5, send=False) # 8
        self.__modify_REG_CHOPCONF('interpolate', self.interpolate, send=False)
        self.__modify_REG_CHOPCONF('chopper_mode', 0, send=False)
        self.__modify_REG_CHOPCONF('hysterisis_start', 3, send=False)
        self.__modify_REG_CHOPCONF('hysterisis_end', 2, send=False)

        #self.__modify_REG_GCONF('stop_enable', 0)
        #self.__modify_REG_GCONF('external_ref', 0)
        #self.__modify_REG_GCONF('internal_Rsense', 0)
        self.__modify_REG_GCONF('shaft_dir', self._direction, send=False)
        self.__modify_REG_GCONF('diag0_active_high', self.diag0act_high, send=False)
        self.__modify_REG_GCONF('diag1_active_high', self.diag1act_high, send=False)

        self.__modify_REG_COOLCONF('sg_stall_value', self.sg_stall_value, send=False)

        if self.silent_mode is True:
            # stealthChop
            self.__modify_REG_GCONF('stealthChop', 1, send=False)
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
                self.logger.info("Stealth max speed: {} ({})".
                                 format(self.stealth_max_speed, speed))
        else:
            # spreadCycle
            self.__modify_REG_GCONF('stealthChop', 0, send=False)

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
            self.__modify_REG_IHOLD_IRUN('IRUN', int(CS), send=False)
            self.current = current
        if multip_for_holding_current:
            if CS is None:
                CS = self.__read_REG_IHOLD_IRUN('IRUN')
            iHold = int(CS * multip_for_holding_current)
            self.__modify_REG_IHOLD_IRUN('IHOLD', iHold, send=False)
            self.hold_multip = multip_for_holding_current
        if hold_delay:
            self.__modify_REG_IHOLD_IRUN('IHOLDDELAY', hold_delay, send=False)
            self.hold_delay = hold_delay
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

        STEP    = 'HIGH' if (status & 0b000001) else 'LOW'
        DIR     = 'HIGH' if (status & 0b000010) else 'LOW'
        DCEN    = 'HIGH' if (status & 0b000100) else 'LOW'
        DCIN    = 'HIGH' if (status & 0b001000) else 'LOW'
        DRV_ENN = 'HIGH' if (status & 0b010000) else 'LOW'
        DCO     = 'HIGH' if (status & 0b100000) else 'LOW'

        self.logger.info("DRV_ENN_CFG6 pin %s" % DRV_ENN)
        self.logger.info("STEP pin %s" % STEP)
        self.logger.info("DIR pin %s" % DIR)
        self.logger.info("DCEN_CFG4 pin %s" % DCEN)
        self.logger.info("DCIN_CFG5 pin %s" % DCIN)
        self.logger.info("DCO pin %s" % DCO)
        if log is not None:
            log.append("DRV_ENN_CFG6 pin %s" % DRV_ENN)
            log.append("STEP pin %s" % STEP)
            log.append("DIR pin %s" % DIR)
            log.append("DCEN_CFG4 pin %s" % DCEN)
            log.append("DCIN_CFG5 pin %s" % DCIN)
            log.append("DCO pin %s" % DCO)
        return status

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
        return self.read_reg(self.IHOLD_IRUN_reg_def[name], self.val_IHOLD_IRUN)

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
    # SPI MODE REGISTER SET
    #**************************************************************************
    def __set_REG_XDRIRECT(self, coil_a_current, coil_b_current):
        """
        255..+255
        Specifies Motor coil currents and polarity directly
        programmed via the serial interface. In this mode,
        the current is scaled by IHOLD setting.
        """
        if coil_a_current < -255:
            coil_a_current = -255
        elif 255 < coil_a_current:
            coil_a_current = 255
        if coil_b_current < -255:
            coil_b_current = -255
        elif 255 < coil_b_current:
            coil_b_current = 255
        val = 0
        if coil_b_current < 0:
            val |= 1 << 8
            coil_b_current = -coil_b_current
        val |= -coil_b_current
        val <<= 16
        if coil_a_current < 0:
            val |= 1 << 8
            coil_a_current = -coil_a_current
        val |= coil_a_current
        self._command_write(REG_XDIRECT, val)

    #**************************************************************************
    # dcStep Minimum Velocity Register
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

    #**************************************************************************
    # MICROSTEPPING CONTROL REGISTER SET (0x60..0x6B)
    #**************************************************************************

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
    # reset default 0x00050480
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
    def __get_LOST_STEPS(self, log=list()):
        val = self._command_read(REG_LOST_STEPS)
        if val:
            dump = "Lost steps: {}".format(val)
            self.logger.error(dump)
            log.append(dump)
        return val
