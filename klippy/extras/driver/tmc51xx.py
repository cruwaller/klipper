# TMC51xx stepper driver control
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import types, struct, math, collections, binascii
from driverbase import SpiDriver
import field_helpers
from mcu import error
import chelper, pins

decode_signed_int = field_helpers.decode_signed_int

#***************************************************
# Constants
#***************************************************
MIN_CURRENT = 100.
MAX_CURRENT = 2000.

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
ramp_mode_t = {
    'positioning': 0,
    'velocity pos': 1,
    'velocity neg': 2,
    'hold': 3
}

# **************************************************************************
# Registers
# **************************************************************************
Registers = {
    # GENERAL CONFIGURATION REGISTERS (0x00...0x0F)
    "GCONF":      [0x00, 'RW'],
    "GSTAT":      [0x01, 'R'],
    # "IFCNT"      : [0x02, ''], # UART only
    # "SLAVECONF"  : [0x03, ''], # UART only
    "IOIN":       [0x04, 'R'],
    "X_COMPARE":  [0x05, 'W'],
    # VELOCITY DEPENDENT DRIVER FEATURE CONTROL REGISTER SET (0x10...0x1F)
    "IHOLD_IRUN": [0x10, 'W'],
    "TPOWERDOWN": [0x11, 'W'],
    "TSTEP":      [0x12, 'R'],
    "TPWMTHRS":   [0x13, 'W'],
    "TCOOLTHRS":  [0x14, 'W'],
    "THIGH":      [0x15, 'W'],
    # RAMP GENERATOR MOTION CONTROL REGISTER SET (0x20...0x2D)
    "RAMPMODE":   [0x20, 'RW'],
    "XACTUAL":    [0x21, 'RW'],
    "VACTUAL":    [0x22, 'R'],
    "VSTART":     [0x23, 'W'],
    "A1":         [0x24, 'W'],
    "V1":         [0x25, 'W'],
    "AMAX":       [0x26, 'W'],
    "VMAX":       [0x27, 'W'],
    "DMAX":       [0x28, 'W'],
    "D1":         [0x2A, 'W'],
    "VSTOP":      [0x2B, 'W'],
    "TZEROWAIT":  [0x2C, 'W'],
    "XTARGET":    [0x2D, 'W'],
    # RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0x30...0x36)
    "VDCMIN":     [0x33, 'W'],
    "SW_MODE":    [0x34, 'RW'],
    "RAMP_STAT":  [0x35, 'R'],
    "XLATCH":     [0x36, 'R'],
    # ENCODER REGISTER SET (0x38...0x3C)
    "ENCMODE":    [0x38, 'RW'],
    "X_ENC":      [0x39, 'RW'],
    "ENC_CONST":  [0x3A, 'W'],
    "ENC_STATUS": [0x3B, 'R'],
    "ENC_LATCH":  [0x3C, 'R'],
    # MOTOR DRIVER REGISTER SET (0x60..0x6B)
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
        #'test_mode'           : 1 << 17, # Not allowed to modify
        #'direct_mode'         : 1 << 16, # Not allowed to modify
        'stop_enable'         : 1 << 15,
        'small_hysterisis'    : 1 << 14,
        'diag1_active_high'   : 1 << 13,
        'diag0_active_high'   : 1 << 12,
        'diag1_steps_skipped' : 1 << 11,
        'diag1_chopper_on'    : 1 << 10,
        'diag1_index'         : 1 << 9,
        'diag1_stall'         : 1 << 8,
        'diag0_stall'         : 1 << 7,
        'diag0_temp_prewarn'  : 1 << 6,
        'diag0_errors'        : 1 << 5,
        'shaft_dir'           : 1 << 4,
        #'commutation'         : 1 << 3, # Not allowed to modify
        'stealthChop'         : 1 << 2,
        'internal_Rsense'     : 1 << 1,
        'external_ref'        : 1 << 0,
    },
    'GSTAT': {
        'gstat_reset'   : 0b001,
        'gstat_drv_err' : 0b010,
        'gstat_uv_cp'   : 0b100,
    },
    'IOIN': {
        'VERSION'       : 0xff << 24,
        'SWCOMP_IN'     : 1 << 7,
        'SD_MODE'       : 1 << 6,
        'ENC_N_DCO'     : 1 << 5,
        'DRV_ENN_CFG6'  : 1 << 4,
        'ENCA_DCIN_CFG5': 1 << 3,
        'ENCB_DCEN_CFG4': 1 << 2,
        'REFR_DIR'      : 1 << 1,
        'REFL_STEP'     : 1 << 0,
    },
    'X_COMPARE': {'compare_position': 0xFFFFFFFF},
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
    #**************************************************************************
    # RAMP GENERATOR MOTION CONTROL REGISTER SET (0x20...0x2D)
    #**************************************************************************
    'RAMPMODE':  {'ramp_mode': 0b11},
    'XACTUAL':   {'actual motor position': 0xFFFFFFFF},
    'VACTUAL':   {'actual motor velocity': 0x3FFFFF},
    'VSTART':    {'velocity start': 0x3FFFF},
    'A1':        {'acceleration': 0xFFFF},
    'V1':        {'velocity': 0xFFFFF},
    'AMAX':      {'acceleration max': 0xFFFF},
    'VMAX':      {'velocity max': 0x7FFFFF},
    'DMAX':      {'deceleration max': 0xFFFF},
    'D1':        {'deceleration': 0xFFFF},
    'VSTOP':     {'velocity stop': 0x3FFFF},
    'TZEROWAIT': {'zero wait time': 0xFFFF},
    'XTARGET':   {'target position': 0xFFFFFFFF},
    # **************************************************************************
    # RAMP GENERATOR DRIVER FEATURE CONTROL REGISTER SET (0x30...0x36)
    # **************************************************************************
    'VDCMIN': {'dcstep_min_speed': 0x7FFFFF},
    'SW_MODE': {
        'en_softstop':      1 << 11,
        'sg_stop':          1 << 10,
        'en_latch_encoder': 1 << 9,
        'latch_r_inactive': 1 << 8,
        'latch_r_active':   1 << 7,
        'latch_l_inactive': 1 << 6,
        'latch_l_active':   1 << 5,
        'swap_lr':          1 << 4,
        'pol_stop_r':       1 << 3, # 0 = HIGH, 1 = LOW to stop motor
        'pol_stop_l':       1 << 2,
        'stop_r_enable':    1 << 1,
        'stop_l_enable':    1 << 0,
    },
    'RAMP_STAT': {
        'status_sg':         1 << 13,
        'second_move':       1 << 12,
        't_zerowait_active': 1 << 11,
        'vzero':             1 << 10,
        'position_reached':  1 << 9,
        'velocity_reached':  1 << 8,
        'event_pos_reached': 1 << 7,
        'event_stop_sg':     1 << 6,
        'event_stop_r':      1 << 5,
        'event_stop_l':      1 << 4,
        'status_latch_r':    1 << 3,
        'status_latch_l':    1 << 2,
        'status_stop_r':     1 << 1,
        'status_stop_l':     1 << 0,
    },
    'XLATCH': {'ramp generator latch position': 0xffffffff},
    # **************************************************************************
    # ENCODER REGISTER SET (0x38...0x3C)
    # **************************************************************************
    'ENCMODE': {
        'enc_sel_decimal': 1 << 10,
        'latch_x_act':     1 << 9,
        'clr_enc_x':       1 << 8,
        'neg_edge':        1 << 7,
        'pos_edge':        1 << 6,
        'clr_once':        1 << 5,
        'clr_cont':        1 << 4,
        'ignore_AB':       1 << 3,
        'pol_N':           1 << 2,
        'pol_B':           1 << 1,
        'pol_A':           1 << 0,
    },
    'X_ENC': {'encoder actual position': 0xffffffff},
    'ENC_CONST': {
        'enc_const_factor':   0xffff,
        'enc_const_fraction': 0xffff,
    },
    'ENC_STATUS': {'enc_n_event': 1 << 0},
    'ENC_LATCH': {'encoder latch position': 0xffffffff},
    # **************************************************************************
    # MOTOR DRIVER REGISTER SET (0x60..0x6B)
    # **************************************************************************
    # TODO: missing MSLUT0...7
    'MSLUTSEL': {
        'lut W0': 0b11 << 0,
        'lut W1': 0b11 << 2,
        'lut W2': 0b11 << 4,
        'lut W3': 0b11 << 6,
        'lut X1': 0xff << 8,
        'lut X2': 0xff << 16,
        'lut X3': 0xff << 24,
    },
    'MSLUTSTART': {
        'START_SIN': 0xff,
        'START_SIN90': 0xff << 16,
    },
    'MSCNT': {'microstep counter': 0x3ff},
    'MSCURACT': {
        'CUR_A': 0b111111111,
        'CUR_B': 0b111111111,
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
    'DCCTRL': {
        'DC_TIME': 0b1111111111,
        'DC_SG':   0b11111111 << 16,
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
    'PWMCONF': {
        'stealth_amplitude' : 0b11111111 << 0,
        'stealth_gradient'  : 0b11111111 << 8,
        'stealth_freq'      : 0b11 << 16, # pwm_freq_t
        'stealth_autoscale' : 0b1 << 18,
        'stealth_symmetric' : 0b1 << 19,
        'standstill_mode'   : 0b11 << 20, # freewheel_t
    },
    'PWM_SCALE': {'pwm_scale': 0xff},
    'ENCM_CTRL': {
        'encoder_inverted': 1 << 0,
        'encoder_maxspeed': 1 << 1,
    },
    'LOST_STEPS': {'lost steps': 0xfffff}
}

FieldFormatters = {
    # GCONF
    'shaft_dir':          (lambda v: '1(Reverse)' if v else ''),
    'stealthChop':        (lambda v: '1(stealthChop)' if v else "0(spreadCycle)"),
    "internal_Rsense":    (lambda v: "1(IntR)" if v else ""),
    "external_ref":       (lambda v: "1(ExtVREF)" if v else ""),
    'diag0_active_high':  (lambda v: "0(diag0 inverted)" if not v else ""),
    'diag1_active_high':  (lambda v: "0(diag1 inverted)" if not v else ""),
    "diag1_steps_skipped": (lambda v: "1(Diag1 steps skip)" if v else ""),
    "diag1_chopper_on":   (lambda v: "1(Diag1 chopper on)" if v else ""),
    "diag1_index":        (lambda v: "1(Diag1 index)" if v else ""),
    "diag1_stall":        (lambda v: "1(Diag1 stall)" if v else ""),
    "diag0_stall":        (lambda v: "1(Diag0 stall)" if v else ""),
    "diag0_temp_prewarn": (lambda v: "1(Diag0 temp)" if v else ""),
    "diag0_errors":       (lambda v: "1(Diag0 errors)" if v else ""),
    # GSTAT
    'gstat_reset'   : (lambda v: 'Reset has occurred!' if v else ""),
    'gstat_drv_err' : (lambda v: 'overtemperature or short circuit!' if v else ""),
    'gstat_uv_cp'   : (lambda v: 'Undervoltage on the charge pump!' if v else ""),
    # IOIN
    'VERSION':        (lambda v: '%02X(IC)' % v),
    'SWCOMP_IN':      (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'SD_MODE':        (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'ENC_N_DCO':      (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'DRV_ENN_CFG6':   (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'ENCA_DCIN_CFG5': (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'ENCB_DCEN_CFG4': (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'REFR_DIR':       (lambda v: '1(HIGH)' if v else '0(LOW)'),
    'REFL_STEP':      (lambda v: '1(HIGH)' if v else '0(LOW)'),
    # RAMPMODE
    'ramp_mode': (lambda v: "%s(%s)" % (
        v, {v: k for k, v in ramp_mode_t.items()}[v])),
    # RAMP GENERATOR
    'actual motor position': (lambda v: str(decode_signed_int(v, 32))),
    'actual motor velocity': (lambda v: str(decode_signed_int(v, 32))),
    'target position':       (lambda v: str(decode_signed_int(v, 32))),
    'encoder actual position': (lambda v: str(decode_signed_int(v, 32))),
    'enc_n_event': (lambda v: '1(event detected' if v else ''),
    'encoder latch position': (lambda v: str(decode_signed_int(v, 32))),
    # MOTOR DRIVER REGISTER SET
    'CUR_A': (lambda v: str(decode_signed_int(v, 9))),
    'CUR_B': (lambda v: str(decode_signed_int(v, 9))),
    # DRV_STATUS
    'sg_result': (lambda v: "%s(Current)" % v),
    'fsactive': (lambda v: "1(Fullstep)" if v else ""),
    'cs_actual': (lambda v: "%s(Current)" % v),
    'stallGuard': (lambda v: "1(Stalled)" if v else ""),
    'ot': (lambda v: "1(Overtemperature)" if v else ""),
    's2ga': (lambda v: "1(A Short)" if v else ""),
    's2gb': (lambda v: "1(B Short)" if v else ""),
    'ola': (lambda v: "1(A Open Load)" if v else ""),
    'olb': (lambda v: "1(B Open Load)" if v else ""),
    'stst': (lambda v: "1(StandStill)" if v else ""),
    # CHOPCONF
    'chopper_mode': (lambda v: '1(Fast decay)' if v else '0(Standard)'),
    'interpolate': (lambda v: "1(on)" if v else "0(off)"),
    'off_time': (lambda v: "%d" % v),
    'hysterisis_start': (lambda v: "%d" % (v+1)),
    'hysterisis_end': (lambda v: "%d" % (v-3)),
    'blank_time': (lambda v: "%d" % {v: k for k, v in blank_time_map.items()}[v]),
    'microsteps': (lambda v: "%d" % {v: k for k, v in msteps_map.items()}[v]),
    'vsense': (lambda v: "1(high res)" if v else "0(normal)"),
    # COOLCONF
    'sg_filter': (lambda v: "1(enabled)" if v else '0(disabled)'),
    'sg_stall_value': (lambda v: "%s(SG)" % str(decode_signed_int(v, 7))),
    'sg_min': (lambda v: "%d" % v if v else '0(disabled)'),
    'sg_max': (lambda v: "%d" % v),
    # ENCM_CTRL
    'encoder_inverted': (lambda v: "1(Inverted)" if v else ""),
    'encoder_maxspeed': (lambda v: "1(MaxSpeed)" if v else ""),
}


class TMC51xx(SpiDriver):
    _stepper_kinematics = None
    # Error flags
    isReset      = False
    isError      = False
    isStallguard = False
    isStandstill = False

    class TimeoutError(Exception):
        pass

    def __init__(self, config, stepper_config):
        self.printer = printer = config.get_printer()
        # init local variables
        self.vsense = 0
        self._endstop_config = None
        self._mcu_position_offset = 0.
        self._homing_speed = 0
        self._last_state = {}
        self._home_cmd = None
        # init
        SpiDriver.__init__(self, config, stepper_config,
            has_step_dir_pins=False, has_endstop=True)
        self._stepper_oid = stepper_oid = self.mcu.create_oid()
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
        ppins.register_chip(self.name, self)
        # Create a register handler
        self.regs = collections.OrderedDict()
        self.fields = field_helpers.FieldHelper(
            Fields, FieldFormatters, self.regs)
        set_field = self.fields.set_field
        # Driver configuration
        self.sensor_less_homing = config.getboolean('sensor_less_homing', False)
        # +1...+63 = less sensitivity, -64...-1 = higher sensitivity
        self.sg_stall_value = config.getint('stall_threshold', 10,
            minval=-64, maxval=63)
        # option to manually set a hybrid threshold
        hybrid_threshold = config.getint('hybrid_threshold', None,
            minval=0, maxval=1048575)
        # Max speed is used to derive the threshold
        stealth_max_speed = config.getint('stealth_max_speed', None, minval=10)
        # Diag pins configuration
        diag0types = {
            'NA'           : None,
            'errors'       : 'diag0_errors',
            'temp_prewarn' : 'diag0_temp_prewarn',
            'stall'        : 'diag0_stall',
        }
        diag0purpose = config.getchoice('diag0_out', diag0types, default='NA')
        diag0act_high = config.getboolean('diag0_active_high', default=True)
        diag1types = {
            'NA'            : None,
            'stall'         : 'diag1_stall',
            'index'         : 'diag1_index',
            'chopper_on'    : 'diag1_chopper_on',
            'steps_skipped' : 'diag1_steps_skipped',
        }
        diag1purpose = config.getchoice('diag1_out', diag1types, default='NA')
        diag1act_high = config.getboolean('diag1_active_high', default=True)
        # Driver mode configurations
        mode = { "spreadCycle" : False, "stealthChop" : True }
        self.silent_mode = config.getchoice('mode', mode, default='stealthChop')
        # Current config
        self.sense_r = config.getfloat('sense_R', 0.11, above=0.09) + 0.02
        self.current = config.getfloat('current', 1000.0,
            above=MIN_CURRENT, maxval=MAX_CURRENT)
        self.hold_multip = config.getfloat('hold_multiplier', 0.5,
            above=0., maxval=1.0)
        self.hold_delay = config.getint('hold_delay', 10, minval=0, maxval=15)
        # Calculate step calculation factors
        # default int clock freq is 13.2MHz @ 50C
        self.fCLK = fCLK = config.getfloat('fCLK', 13200000.,
            above=4000000, maxval=18000000)
        # t = 2^24 / fCLK
        self.speed_factor = float(1 << 24) / fCLK * self.microsteps
        # ta2 = 2^41 / (fCLK^2)
        self.accel_factor = float(1 << 41) / fCLK**2 * self.microsteps
        self.accel_factor_t = float(1 << 17) / fCLK # accel_t to AMAX
        # Configure driver registers
        # -- CHOPCONF
        set_field('off_time', config.getint('off_time', 4, minval=2, maxval=15))
        set_field('chopper_mode', 0) # Standard mode
        hstart = config.getint('hysterisis_start', 1, minval=1, maxval=8) - 1
        set_field('hysterisis_start', hstart)
        hend = config.getint('hysterisis_end', 2, minval=-3, maxval=12) + 3
        set_field('hysterisis_end', hend)
        set_field('blank_time', config.getchoice('blank_time', blank_time_map, 24))
        set_field('microsteps', msteps_map[self.microsteps])
        set_field('interpolate', config.getboolean('interpolate', True))
        # -- COOLCONF
        set_field('sg_filter', config.getboolean('stall_filter', default=False))
        set_field('sg_stall_value', self.sg_stall_value)
        set_field('sg_min', config.getint('current_increase_threshold', 0,
            minval=0, maxval=15))
        set_field('sg_max', config.getint('current_decrease_threshold', 0,
            minval=0, maxval=15))
        # -- GCONF
        set_field('shaft_dir', config.getboolean('dir_invert', default=False))
        set_field('diag0_active_high', diag0act_high)
        set_field('diag1_active_high', diag1act_high)
        if diag0purpose is not None:
            set_field(diag0purpose, 1)
        if diag1purpose is not None:
            set_field(diag1purpose, 1)
        # False = spreadCycle, True = stealthChop
        set_field('stealthChop', self.silent_mode)
        # -- PWMCONF
        set_field('stealth_autoscale', 1)
        set_field('stealth_gradient', config.getint('stealth_gradient', 5,
            minval=1, maxval=15))
        set_field('stealth_amplitude', config.getint('stealth_amplitude', 180,
            minval=64, maxval=255))
        set_field('stealth_freq', config.getchoice('stealth_freq', pwm_freq_t, "2/683"))
        # -- TPWMTHRS
        if hybrid_threshold is not None:
            self.fields.set_reg_value('TPWMTHRS', hybrid_threshold)
            self.logger.info(
                "Hybrid threshold: %s" % (hybrid_threshold, ))
        elif stealth_max_speed is not None:
            speed = int(fCLK * self.microsteps /
                        (stealth_max_speed * self.inv_step_dist * 256))
            self.fields.set_reg_value('TPWMTHRS', speed)
            self.logger.debug("Stealth max speed: %u (cfg val: %u)" % (
                stealth_max_speed, speed))
        # Motor power down time after last movement (iHOLD current is used)
        t_power_down = config.getint(
            'power_down_delay', 128, minval=0., maxval=255) # 128 = ~2sec
        self.fields.set_reg_value('TPOWERDOWN', t_power_down)
        # Set disable stall speed by default
        self.fields.set_reg_value('TCOOLTHRS', 0)
        # Jerk control:
        self.fields.set_reg_value('TZEROWAIT', 32) # Delay between moves
        self.fields.set_reg_value('VSTART', 1) # Start speed
        self.fields.set_reg_value('VSTOP', 2) # Stop speed
        # Disables A1 and D1 in position mode, AMAX and VMAX only
        self.fields.set_reg_value('V1', 0)
        self.fields.set_reg_value('D1', 0x10)
        # ========== Set endstop ==========
        #if self.sensor_less_homing:
        #    set_field('sg_stop', True)
        inverted = config.getboolean('endstop_inverted', False)
        set_field('pol_stop_l', inverted)
        set_field('pol_stop_r', inverted)
        set_field('swap_lr', config.getboolean('endstop_swap', False))
        set_field('stop_r_enable', True)
        set_field('stop_l_enable', True)
        # ========== Encoder init ==========
        enc_res = config.getfloat("encoder_resolution", default=None,
            minval=-32767.9999, maxval=32767.9999)
        if enc_res:
            enc_decimal = config.getboolean('encoder_decimal', default=True)
            # Register format is Q16.16 signed
            fraction, factor = math.modf(enc_res)
            if enc_res < 0:
                factor = 65535 - abs(factor)
                fraction *= -1
            fraction = int(fraction * [65536, 10000][enc_decimal])
            if enc_res < 0:
                fraction = [65535, 10000][enc_decimal] - fraction
            set_field('enc_const_factor', int(factor))
            set_field('enc_const_fraction', fraction)
            set_field('enc_sel_decimal', enc_decimal)
        # ========== Set Current ==========
        self.__calc_rms_current(self.current, self.hold_multip,
                                self.hold_delay, init=True)
        # local inits
        self.set_ignore_move(False)
        self.set_homing_dir()
        # register command handlers
        self.gcode = gcode = printer.lookup_object('gcode')
        cmds = ["DRV_STATUS", "DRV_CURRENT", "DRV_STALLGUARD"]
        for cmd in cmds:
            gcode.register_mux_command(
                cmd, "DRIVER", self.name.upper(),
                getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name.upper(),
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
        gcode.register_mux_command(
            "INIT_TMC", "STEPPER", self.name.upper(),
            self.cmd_INIT_TMC, desc=self.cmd_INIT_TMC_help)
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' or pin_params['pin'] != 'virtual_endstop':
            raise pins.error("virtual endstop is only supported")
        return self

    # **************************************************************************
    # === GCode handlers ===
    # **************************************************************************
    cmd_DRV_STATUS_help = "args: DRIVER=driver_name"
    def cmd_DRV_STATUS(self, params):
        self.gcode.respond(self.dump_registers())
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
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        self.printer.lookup_object('toolhead').get_last_move_time()
        self.logger.info("DUMP_TMC")
        gcode = self.gcode
        write_only_regs = []
        queried_regs = []
        for reg_name, val in self.regs.items():
            if Registers[reg_name][1] == 'W':
                write_only_regs.append(self.fields.pretty_format(reg_name, val))
            else:
                val = self._command_read(reg_name)
                queried_regs.append(self.fields.pretty_format(reg_name, val))
        msg = ["========== Write-only registers =========="]
        msg.extend(write_only_regs)
        msg.append("========== Queried registers ==========")
        msg.extend(queried_regs)
        msg = "\n".join(msg)
        self.logger.info(msg)
        gcode.respond_info(msg)
    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"
    def cmd_INIT_TMC(self, params):
        self.logger.info("INIT_TMC")
        self.printer.lookup_object('toolhead').wait_moves()
        self._init_driver()

    # **************************************************************************
    # HOMING
    # **************************************************************************
    homedir = 'hold'
    def set_homing_dir(self, homedir=False):
        # False = min, True = max direction
        self.homedir = ['velocity neg', 'velocity pos'][homedir]
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
            # stealthchop off for stallguard homing
            new_value = self.fields.set_field('stealthChop', 0)
            self._command_write('GCONF', new_value)
            # Calculate homing speed
            stall_speed = 16777216. / self._homing_speed / self.microsteps
            stall_speed *= 1.10 # +10% tolerance
            self._command_write('TCOOLTHRS', stall_speed)
            # Set lower acceleration
            self._command_write('AMAX', 500 * self.accel_factor)
        # enable endstops & stallguard
        sw_mode = self.fields.set_field('sg_stop', self.sensor_less_homing)
        self._command_write('SW_MODE', sw_mode)
    def home_start(self, print_time, sample_time, sample_count, rest_time):
        clock = self.mcu.print_time_to_clock(print_time)
        self.logger.info("----- HOME START -----")
        self._set_rampmode(self.homedir)
        # Limit homing speed
        self._command_write('VMAX', self._homing_speed * self.speed_factor)
        self._home_cmd.send([self._stepper_oid, clock,
                             self.mcu.seconds_to_clock(.5), []])
    def home_wait(self, home_end_time):
        eventtime = self.mcu.monotonic()
        while self._last_state.get('ready', 0) != 1:
            eventtime = self.mcu.pause(eventtime + 0.5)
            if self.mcu.estimated_print_time(eventtime) > home_end_time:
                raise self.TimeoutError("Timeout during endstop homing")
            if self.mcu.is_shutdown():
                raise error("MCU is shutdown")
    def home_finalize(self):
        self.logger.info("----- HOME FINALIZE ----")
        self._home_cmd.send([self._stepper_oid, 0, 0, []])
        self._command_write('SW_MODE', 0)
        self._set_rampmode('hold')
        if self.sensor_less_homing:
            new_value = self.fields.set_field('stealthChop', self.silent_mode)
            self._command_write('GCONF', new_value)
            self._command_write('TCOOLTHRS', 0)
        self._set_position_cmd.send([self._stepper_oid, 1, 0])
        self.set_ignore_move(False)
        self._ffi_lib.itersolve_set_commanded_pos(
            self._stepper_kinematics, 0 - self._mcu_position_offset)
    def query_endstop(self, print_time):
        #self.fields.set_reg_value('RAMP_STAT',
        #    self._command_read('RAMP_STAT'))
        self.fields.set_reg_value('IOIN',
            self._command_read('IOIN'))
    def query_endstop_wait(self):
        get_field = self.fields.get_field
        #return (get_field('status_sg') or
        #        get_field('status_stop_r') or
        #        get_field('status_stop_l'))
        return get_field('REFR_DIR') or get_field('REFL_STEP')
    def _home_handle_end_stop_state(self, params):
        """
        {'#receive_time': 662109.504473921, u'oid': 3, u'value': 0,
        '#name': u'stepper_tmc5x_home_status',
        '#sent_time': 662108.664678151, u'ready': 0}
        """
        self._last_state = params

    # **************************************************************************
    # STEPPING
    # **************************************************************************
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

    # **************************************************************************
    # DRIVER CONTROL
    # **************************************************************************
    def dump_registers(self):
        res = [
            "NAME: %s" % self.name,
            "RMS current: %.3fA" % self.__get_rms_current()
        ]
        dump_regs = [key for key, val in Registers.items() if 'R' in val[1]]
        for reg in dump_regs:
            value = self._command_read(reg)
            res.append(self.fields.pretty_format(reg, value))
        # Print write only config registers
        res.append(self.fields.pretty_format('IHOLD_IRUN', self.regs['IHOLD_IRUN']))
        res.append(self.fields.pretty_format('COOLCONF', self.regs['COOLCONF']))
        res.append(self.fields.pretty_format('PWMCONF', self.regs['PWMCONF']))
        res.append(self.fields.pretty_format('TPWMTHRS', self.regs['TPWMTHRS']))
        res.append(self.fields.pretty_format('TPOWERDOWN', self.regs['TPOWERDOWN']))
        log = "\n".join(sorted(res))
        self.logger.info(log)
        return log
    def set_dir(self, direction=0):
        new_value = self.fields.set_field('shaft_dir', direction)
        self._command_write('GCONF', new_value)
    def has_faults(self):
        return (self.isReset or self.isError or
                self.isStallguard or self.isStandstill)
    def clear_faults(self):
        self.isReset = self.isError = False
        self.isStallguard = self.isStandstill = False
    def get_current(self):
        rms = self.__get_rms_current()
        self.logger.debug("get_current = %.3fA" % rms)
        return rms * 1000.
    def set_stallguard(self, sg=None):
        if sg is not None:
            if sg < -64 or 63 < sg:
                raise self.gcode.error("SG out of range (min: -64, max: 63)")
            self.sg_stall_value = sg
            new_value = self.fields.set_field('sg_stall_value', sg)
            self._command_write('COOLCONF', new_value)
        return "Stallguard is %d" % self.sg_stall_value

    #**************************************************************************
    # PRIVATE METHODS
    #**************************************************************************
    def _build_config(self):
        self.mcu.add_config_cmd(
            "stepper_tmc5x_config oid=%u spi_oid=%u" % (
                self._stepper_oid, self._oid,))
        self.mcu.add_config_cmd(
            "stepper_tmc5x_reset_step_clock oid=%d clock=0" % (
                self._stepper_oid,),
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
        self._ffi_lib.stepcompress_fill_tmc5x(
            self._stepqueue, self.mcu.seconds_to_clock(max_error),
            step_cmd_id, self.speed_factor, self.accel_factor, self.accel_factor_t)

    def _init_driver(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.enable is not None:
            # Disable driver while initializing it
            self.enable.set_digital(print_time, 1)
        # ----------------------------------------------------------------------
        # Clear errors by reading GSTAT register
        self.__get_global_status()
        # ----------------------------------------------------------------------
        # Init Acceleration and Velocity
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.fields.set_reg_value('AMAX', max_accel * self.accel_factor)
        self.fields.set_reg_value('VMAX', max_velocity * self.speed_factor)
        # ----------------------------------------------------------------------
        #   Send configurations to driver
        self.logger.debug("==== Send configurations ====")
        for reg_name, val in self.regs.items():
            if reg_name is 'SW_MODE':
                # Don't enable endstops by default
                val = 0
            self._command_write(reg_name, val)
        # ----------------------------------------------------------------------
        # Set current position to zero
        self._set_rampmode('hold')
        self._set_position(0)
        self._set_rampmode('positioning')
        # ----------------------------------------------------------------------
        # Check errors by reading GSTAT register
        self.__get_global_status()
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
    def _command_read(self, cmd): # 40bits always = 5 x 8bit!
        cmd, mode = Registers.get(cmd, (cmd, ''))  # map string to value
        if 'R' not in mode:
            raise error("TMC51xx: register '%s' R/W mode '%s is wrong!" % (
                cmd, mode))
        cmd &= 0x7F # Makesure the MSB is 0
        read_cmd = [cmd, 0, 0, 0, 0]
        self.spi_send(read_cmd)
        values = self.spi_transfer(read_cmd)
        # convert list of bytes to number
        val    = 0
        size   = len(values)
        status = 0
        if 0 < size:
            status = int(values[0])
            if status:
                if status & 0b0001:
                    self.isReset = True
                    self.logger.warning("  Reset has occurred!")
                if status & 0b0010:
                    self.isError = True
                    self.logger.error("  Driver error detected!")
                if status & 0b0100:
                    self.isStallguard = True
                    self.logger.warning("  Stallguard active")
                if status & 0x1000:
                    self.isStandstill = True
                    self.logger.info("  Motor stand still")
            for idx in range(1, size):
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
    def _command_write(self, cmd, val=None): # 40bits always = 5 x 8bit!
        if val is not None:
            cmd, mode = Registers.get(cmd, (cmd, ''))  # map string to value
            if 'W' not in mode:
                raise error("TMC51xx: register '%s' R/W mode '%s is wrong!" % (
                    cmd, mode))
            if not isinstance(val, types.ListType):
                conv = struct.Struct('>I').pack
                val = list(conv(val))
            if len(val) != 4:
                raise error("TMC51xx internal error! len(val) != 4")
            self.logger.debug("==>> cmd 0x%02X : 0x%s" % (
                cmd, binascii.hexlify(bytearray(val))))
            cmd |= 0x80 # Make sure command has write bit set
            self.spi_send([cmd] + val)

    def __calc_rms_current(self,
                           current = None,
                           multip_for_holding_current = None,
                           hold_delay = None,
                           init=False):
        if current:
            # Check if current is in mA or A
            if MIN_CURRENT < current:
                current /= 1000.
            self.vsense = 0
            CS = 32.0 * math.sqrt(2.) * current * self.sense_r / 0.325 - .5
            # If Current Scale is too low, turn on high sensitivity R_sense
            # and calculate again
            if CS < 16:
                self.vsense = 1
                CS = 32.0 * math.sqrt(2.) * current * self.sense_r / 0.180 - .5
            self.fields.set_field('vsense', self.vsense)
            iRun  = int(CS)
            self.fields.set_field('run_current', iRun)
            self.current = current
        else:
            iRun = self.fields.get_field('run_current')
        if multip_for_holding_current:
            iHold = int(iRun * multip_for_holding_current)
            self.fields.set_field('hold_current', iHold)
            self.hold_multip = multip_for_holding_current
        else:
            iHold = self.fields.get_field('hold_current')
        if hold_delay:
            self.fields.set_field('hold_delay', hold_delay)
            self.hold_delay  = hold_delay
        if not init:
            self._command_write('CHOPCONF', self.regs['CHOPCONF'])
            self._command_write('IHOLD_IRUN', self.regs['IHOLD_IRUN'])
        self.logger.debug("RMS current %.3fA => IHold=%u, IRun=%u, IHoldDelay=%u" % (
            self.current, iHold, iRun, self.hold_delay))

    def __get_rms_current(self, cs=None, vsense=None):
        if vsense is None:
            # vsense = self.fields.get_field('vsense')
            vsense = self.vsense
        if cs is None:
            cs = self.fields.get_field('run_current')
        V_fs = [0.325, 0.180][bool(vsense)]
        return ( cs + 1. ) / 32.0 * V_fs / self.sense_r / math.sqrt(2.)

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

    #**************************************************************************
    # RAMP GENERATOR MOTION CONTROL REGISTER SET (0x20...0x2D)
    #**************************************************************************
    #==================== REG_RAMPMODE ====================
    def _set_rampmode(self, mode):
        """
        :param mode: defines the used ramp calculation mode
            positioning     - using all A, D and V parameters
            velocity pos    - positive VMAX, using AMAX acceleration
            velocity neg    - negative VMAX, using AMAX acceleration
            hold            - velocity remains unchanged, unless stop event occurs
        """
        self._command_write('RAMPMODE', ramp_mode_t[mode])

    def _set_position(self, pos):
        pos = int(pos) & 0xFFFFFFFF
        self._command_write('XACTUAL', pos)
        self._command_write('XTARGET', pos)
