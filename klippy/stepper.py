# Printer stepper support
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import homing, pins
import drivers

# Code storing the definitions for a stepper motor
class PrinterStepper:
    def __init__(self, printer, config, logger=None):
        self.name = config.section
        if self.name.startswith('stepper_'):
            self.name = self.name[8:]
        if logger is None:
            self.logger = printer.logger.getChild('stepper.%s' % self.name)
        else:
            self.logger = logger.getChild('stepper')
        self.is_Z = False
        if ('Z' in self.name.upper()):
            self.is_Z = True

        self.need_motor_enable = True
        # get a driver
        self.driver = \
            drivers.get_driver(printer,
                               config,
                               name=config.get('driver', None),
                               logger=self.logger)
        self.step_dist = self.driver.step_dist
        self.inv_step_dist = self.driver.inv_step_dist
        # Stepper definition
        self.mcu_stepper = pins.setup_pin(
            printer, 'stepper', config.get('step_pin'))
        dir_pin_params = pins.get_printer_pins(printer).parse_pin_desc(
            config.get('dir_pin'), can_invert=True)
        self.mcu_stepper.setup_dir_pin(dir_pin_params)
        self.mcu_stepper.setup_step_distance(self.step_dist)
        self.step_const = self.mcu_stepper.step_const
        self.step_delta = self.mcu_stepper.step_delta
        # Enable pin
        enable_pin = config.get('enable_pin', None)
        self.mcu_enable = None
        if enable_pin is not None:
            self.mcu_enable = pins.setup_pin(printer, 'digital_out', enable_pin)
            self.mcu_enable.setup_max_duration(0.)

        self.logger.info("steps per mm {} , step in mm {}".
                     format(self.inv_step_dist, self.step_dist))

    def _dist_to_time(self, dist, start_velocity, accel):
        # Calculate the time it takes to travel a distance with constant accel
        time_offset = start_velocity / accel
        return math.sqrt(2. * dist / accel + time_offset**2) - time_offset
    def set_max_jerk(self, max_halt_velocity, max_accel):
        # Calculate the firmware's maximum halt interval time
        last_step_time = self._dist_to_time(
            self.step_dist, max_halt_velocity, max_accel)
        second_last_step_time = self._dist_to_time(
            2. * self.step_dist, max_halt_velocity, max_accel)
        min_stop_interval = second_last_step_time - last_step_time
        self.mcu_stepper.setup_min_stop_interval(min_stop_interval)
    def set_position(self, pos):
        self.mcu_stepper.set_position(pos)
    def motor_enable(self, print_time, enable=0):
        if (self.mcu_enable is not None
            and self.need_motor_enable != (not enable)):
            self.mcu_enable.set_digital(print_time, enable)
        self.need_motor_enable = not enable

# Support for stepper controlled linear axis with an endstop
class PrinterHomingStepper(PrinterStepper):
    def __init__(self, printer, config, default_position=None):
        PrinterStepper.__init__(self, printer, config)
        # Endstop and its position
        if default_position is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position)
        # Homing offset will be substracted from homed position
        self.homing_offset = config.getfloat('homing_offset', None)
        if self.homing_offset is None:
            # Try in steps and convert steps to mm
            self.homing_offset = (config.getfloat('homing_offset_steps', 0.) *
                                  self.step_dist)
        # Homing finetune after enstop hit (mainly for deltas)
        self.tune_after_homing = \
            config.getfloat('endstop_correction', None) # in mm
        if (self.tune_after_homing is None):
            self.tune_after_homing = (config.getfloat('endstop_correction_steps', 0.) *
                                      self.step_dist)
        # Axis range
        self.position_min = config.getfloat('position_min', 0.)
        self.position_max = config.getfloat('position_max', 0.,
                                            above=self.position_min)
        # Homing mechanics
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., above=0.)
        homing_dirs = { 'min' : False, 'max' : True, 'NA' : None}
        homing_dir  = config.getchoice('homing_direction',
                                       homing_dirs, 'NA')
        if homing_dir is not None:
            self.homing_positive_dir = homing_dir
        else:
            self.homing_positive_dir = config.getboolean('homing_positive_dir', None)
            if self.homing_positive_dir is None:
                axis_len = self.position_max - self.position_min
                if self.position_endstop <= self.position_min + axis_len / 4.:
                    self.homing_positive_dir = False
                elif self.position_endstop >= self.position_max - axis_len / 4.:
                    self.homing_positive_dir = True
                else:
                    raise config.error(
                        "Unable to infer homing_positive_dir in section '%s'" % (
                            config.section,))
        # Endstop
        endstop_pin = config.get('endstop_pin', None)
        if endstop_pin is None:
            if self.homing_positive_dir is False:
                # min pin
                endstop_pin = config.get('endstop_min_pin')
            else:
                # max pin
                endstop_pin = config.get('endstop_max_pin')
        self.mcu_endstop = pins.setup_pin(printer, 'endstop', endstop_pin)
        self.mcu_endstop.add_stepper(self.mcu_stepper)
        # Endstop stepper phase position tracking
        self.homing_stepper_phases = config.getint(
            'homing_stepper_phases', None, minval=0)
        endstop_accuracy = config.getfloat(
            'homing_endstop_accuracy', None, above=0.)
        self.homing_endstop_accuracy = self.homing_endstop_phase = None
        if self.homing_stepper_phases:
            self.homing_endstop_phase = config.getint(
                'homing_endstop_phase', None, minval=0
                , maxval=self.homing_stepper_phases-1)
            if (self.homing_endstop_phase is not None
                and config.getboolean('homing_endstop_align_zero', False)):
                # Adjust the endstop position so 0.0 is always at a full step
                micro_steps = self.homing_stepper_phases // 4
                phase_offset = (
                    ((self.homing_endstop_phase + micro_steps // 2) % micro_steps)
                    - micro_steps // 2) * self.step_dist
                full_step = micro_steps * self.step_dist
                es_pos = (int(self.position_endstop / full_step + .5) * full_step
                          + phase_offset)
                if es_pos != self.position_endstop:
                    self.logger.info("Changing endstop position to %.3f"
                                     " (from %.3f)", es_pos,
                                     self.position_endstop)
                    self.position_endstop = es_pos
            if endstop_accuracy is None:
                self.homing_endstop_accuracy = self.homing_stepper_phases//2 - 1
            elif self.homing_endstop_phase is not None:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy * .5 / self.step_dist))
            else:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy / self.step_dist))
            if self.homing_endstop_accuracy >= self.homing_stepper_phases // 2:
                self.logger.info("Endstop is not accurate enough for stepper"
                                 " phase adjustment")
                self.homing_stepper_phases = None
            if self.mcu_endstop.get_mcu().is_fileoutput():
                self.homing_endstop_accuracy = self.homing_stepper_phases

        # Valid for CoreXY and Cartesian Z axis
        if (self.is_Z):
            self.homing_pos_x = config.getfloat('homing_pos_x', None,
                                                minval=self.position_min,
                                                maxval=self.position_max)
            self.homing_pos_y = config.getfloat('homing_pos_y', None,
                                                minval=self.position_min,
                                                maxval=self.position_max)
        else:
            # None for X and Y axis
            self.homing_pos_x = None
            self.homing_pos_y = None
        self.retract_after_home = config.getboolean('homing_retract_after',
                                                    False)
    def set_homing_offset(self, offset):
        self.homing_offset = offset
    def get_endstops(self):
        return [(self.mcu_endstop, self.name)]
    def get_homed_offset(self):
        if not self.homing_stepper_phases or self.need_motor_enable:
            return 0.0 - self.homing_offset
        pos = self.mcu_stepper.get_mcu_position()
        pos %= self.homing_stepper_phases
        if self.homing_endstop_phase is None:
            self.logger.info("Setting endstop phase to %d", pos)
            self.homing_endstop_phase = pos
            return 0.0 - self.homing_offset
        delta = (pos - self.homing_endstop_phase) % self.homing_stepper_phases
        if delta >= self.homing_stepper_phases - self.homing_endstop_accuracy:
            delta -= self.homing_stepper_phases
        elif delta > self.homing_endstop_accuracy:
            raise homing.EndstopError(
                "Endstop %s incorrect phase (got %d vs %d)" % (
                    self.name, pos, self.homing_endstop_phase))
        return (delta * self.step_dist) - self.homing_offset

# Wrapper for dual stepper motor support
class PrinterMultiStepper(PrinterHomingStepper):
    def __init__(self, printer, config):
        PrinterHomingStepper.__init__(self, printer, config)
        self.endstops = PrinterHomingStepper.get_endstops(self)
        self.extras = []
        self.all_step_const = [self.step_const]
        for i in range(1, 99):
            if not config.has_section(config.section + str(i)):
                break
            extraconfig = config.getsection(config.section + str(i))
            extra = PrinterStepper(printer, extraconfig)
            self.extras.append(extra)
            self.all_step_const.append(extra.step_const)
            extraendstop = extraconfig.get('endstop_pin', None)
            if extraendstop is not None:
                mcu_endstop = pins.setup_pin(printer, 'endstop', extraendstop)
                mcu_endstop.add_stepper(extra.mcu_stepper)
                self.endstops.append((mcu_endstop, extra.name))
            else:
                self.mcu_endstop.add_stepper(extra.mcu_stepper)
        self.step_const = self.step_multi_const
    def step_multi_const(self, print_time, start_pos, dist, start_v, accel):
        for step_const in self.all_step_const:
            step_const(print_time, start_pos, dist, start_v, accel)
    def set_max_jerk(self, max_halt_velocity, max_accel):
        PrinterHomingStepper.set_max_jerk(self, max_halt_velocity, max_accel)
        for extra in self.extras:
            extra.set_max_jerk(max_halt_velocity, max_accel)
    def set_position(self, pos):
        PrinterHomingStepper.set_position(self, pos)
        for extra in self.extras:
            extra.set_position(pos)
    def motor_enable(self, print_time, enable=0):
        PrinterHomingStepper.motor_enable(self, print_time, enable)
        for extra in self.extras:
            extra.motor_enable(print_time, enable)
    def get_endstops(self):
        return self.endstops

def LookupMultiHomingStepper(printer, config):
    if not config.has_section(config.section + '1'):
        return PrinterHomingStepper(printer, config)
    return PrinterMultiStepper(printer, config)
