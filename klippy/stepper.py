# Printer stepper support
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import homing

# Tracking of shared stepper enable pins
class StepperEnablePin:
    def __init__(self, mcu_enable, enable_count=0):
        self.mcu_enable = mcu_enable
        self.enable_count = enable_count
    def set_enable(self, print_time, enable):
        if enable:
            if not self.enable_count:
                self.mcu_enable.set_digital(print_time, 1)
            self.enable_count += 1
        else:
            self.enable_count -= 1
            if not self.enable_count:
                self.mcu_enable.set_digital(print_time, 0)

def lookup_enable_pin(ppins, pin):
    if pin is None:
        return StepperEnablePin(None, 9999)
    pin_params = ppins.lookup_pin('digital_out', pin, 'stepper_enable')
    enable = pin_params.get('class')
    if enable is None:
        mcu_enable = pin_params['chip'].setup_pin(pin_params)
        mcu_enable.setup_max_duration(0.)
        pin_params['class'] = enable = StepperEnablePin(mcu_enable)
    return enable

def calculate_steps(config, microsteps=None):
    step_dist = inv_step_dist = 0.
    # Read config and send to driver
    step_dist = config.getfloat('step_distance', default=None, above=0.)
    if step_dist is not None:
        inv_step_dist = 1. / step_dist
    else:
        steps_per_mm = config.getfloat('steps_per_mm', default=None, above=0.)
        if steps_per_mm is None:
            if microsteps is None:
                return None, None
            motor_deg = config.getfloat('motor_step_angle', above=0.)
            # Calculate base on settings
            pitch = config.getfloat('pitch', above=0.)
            teeth = config.getfloat('teeths', above=0.)
            ratio = config.getfloat('gear_ratio', above=0., default=1.0)
            motor_rev = 360. / motor_deg
            steps_per_mm = motor_rev * microsteps / (pitch * teeth) * ratio
        inv_step_dist = steps_per_mm
        step_dist = 1.0 / inv_step_dist
    return step_dist, inv_step_dist

# Code storing the definitions for a stepper motor
class PrinterStepper:
    step_dist = inv_step_dist = None
    def __init__(self, printer, config, logger=None):
        self.name = config.get_name()
        if self.name.startswith('stepper_'):
            self.name = self.name[8:]
        if logger is None:
            self.logger = printer.logger.getChild('stepper.%s' % self.name)
        else:
            self.logger = logger.getChild('stepper')
        self.need_motor_enable = True
        # get a driver
        self.driver = driver = printer.lookup_object(
            'driver %s' % config.get('driver', None), None)
        microsteps = None
        if driver is not None:
            microsteps = driver.microsteps
            self.step_dist = driver.step_dist
            self.inv_step_dist = driver.inv_step_dist
        if self.step_dist is None or self.inv_step_dist is None:
            self.step_dist, self.inv_step_dist = calculate_steps(config, microsteps)
            if driver is not None:
                driver.step_dist = self.step_dist
                driver.inv_step_dist = self.inv_step_dist
        # Stepper definition
        ppins = printer.lookup_object('pins')
        self.mcu_stepper = ppins.setup_pin('stepper', config.get('step_pin'))
        dir_pin_params = ppins.lookup_pin('digital_out', config.get('dir_pin'))
        self.mcu_stepper.setup_dir_pin(dir_pin_params)
        self.mcu_stepper.setup_step_distance(self.step_dist)
        self.step = self.mcu_stepper.step
        self.step_const = self.mcu_stepper.step_const
        self.step_delta = self.mcu_stepper.step_delta
        self.enable = lookup_enable_pin(ppins, config.get('enable_pin', None))
        self.logger.info("steps per mm {} , step in mm {}".
                         format(self.inv_step_dist, self.step_dist))
        printer.add_object(config.get_name(), self) # to get printer_state called
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
        if self.need_motor_enable != (not enable):
            self.enable.set_enable(print_time, enable)
        self.need_motor_enable = not enable
    def printer_state(self, state):
        if state == 'ready':
            if self.mcu_stepper.get_mcu().is_shutdown():
                return
            init = getattr(self.driver, "init_driver", None)
            if init is not None:
                init()

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
        self.position_max = config.getfloat(
            'position_max', 0., above=self.position_min)
        # Homing mechanics
        self.homing_slowdown = config.getfloat('homing_slowdown', 5.0)
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
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
                            self.name,))
        # Endstop
        endstop_pin = config.get('endstop_pin', None)
        if endstop_pin is None:
            if self.homing_positive_dir is False:
                # min pin
                endstop_pin = config.get('endstop_min_pin')
            else:
                # max pin
                endstop_pin = config.get('endstop_max_pin')
        self.mcu_endstop = printer.lookup_object('pins').setup_pin('endstop', endstop_pin)
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
                    self.logger.info("Changing %s endstop position to %.3f"
                                     " (from %.3f)", self.name, es_pos,
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
                self.logger.info("Endstop for %s is not accurate enough for stepper"
                                 " phase adjustment", self.name)
                self.homing_stepper_phases = None
            if self.mcu_endstop.get_mcu().is_fileoutput():
                self.homing_endstop_accuracy = self.homing_stepper_phases

        # Valid for CoreXY and Cartesian Z axis
        if 'Z' in self.name.upper():
            self.homing_pos_x = config.getfloat('homing_pos_x', default=None,
                                                minval=self.position_min,
                                                maxval=self.position_max)
            self.homing_pos_y = config.getfloat('homing_pos_y', default=None,
                                                minval=self.position_min,
                                                maxval=self.position_max)
        else:
            # None for X and Y axis
            self.homing_pos_x = None
            self.homing_pos_y = None
        self.retract_after_home = config.getfloat(
            'homing_retract_dist_after', 0., minval=0.)
    def set_homing_offset(self, offset):
        self.homing_offset = offset
    def get_endstops(self):
        return [(self.mcu_endstop, self.name)]
    def get_homed_offset(self):
        if not self.homing_stepper_phases or self.need_motor_enable:
            return 0. - self.homing_offset
        pos = self.mcu_stepper.get_mcu_position()
        pos %= self.homing_stepper_phases
        if self.homing_endstop_phase is None:
            self.logger.info("Setting %s endstop phase to %d", self.name, pos)
            self.homing_endstop_phase = pos
            return 0. - self.homing_offset
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
            if not config.has_section(config.get_name() + str(i)):
                break
            extraconfig = config.getsection(config.get_name() + str(i))
            extra = PrinterStepper(printer, extraconfig)
            self.extras.append(extra)
            self.all_step_const.append(extra.step_const)
            extraendstop = extraconfig.get('endstop_pin', None)
            if extraendstop is not None:
                ppins = printer.lookup_object('pins')
                mcu_endstop = ppins.setup_pin('endstop', extraendstop)
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

class DummyMcu:
    def __init__(self):
        pass
    def get_commanded_position(self):
        return 0
class PrinterDummyStepper:
    position_min = position_max = 0
    need_motor_enable = False
    dummy = True
    def __init__(self, printer):
        self.printer = printer
        self.mcu_stepper = DummyMcu()
    def set_max_jerk(self, max_halt_velocity, max_accel):
        pass
    def set_position(self, pos):
        pass
    def motor_enable(self, print_time, enable=0):
        pass
    def get_endstops(self):
        pass
    def step_const(self, *args, **kwargs):
        pass


def LookupMultiHomingStepper(printer, config):
    if not config.has_section(config.get_name() + '1'):
        if config.has_section(config.get_name()):
            return PrinterHomingStepper(printer, config)
        return PrinterDummyStepper(printer)
    return PrinterMultiStepper(printer, config)
