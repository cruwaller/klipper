# Printer stepper support
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections
import homing
import extras.driver as driver_base


######################################################################
# Stepper enable pins
######################################################################

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
    pin_params = ppins.lookup_pin(pin, can_invert=True,
                                  share_type='stepper_enable')
    enable = pin_params.get('class')
    if enable is None:
        mcu_enable = pin_params['chip'].setup_pin('digital_out', pin_params)
        mcu_enable.setup_max_duration(0.)
        pin_params['class'] = enable = StepperEnablePin(mcu_enable)
    return enable


def calculate_steps(config, microsteps=None):
    # Read config and send to driver
    step_dist = config.getfloat('step_distance', default=None, above=0.)
    steps_per_mm = config.getfloat('steps_per_mm', default=None, above=0.)
    microsteps = config.getfloat('microsteps', default=microsteps, above=0.)
    if step_dist is None and steps_per_mm is None and microsteps is not None:
        motor_deg = config.getfloat('motor_step_angle', above=0.)
        # Calculate base on settings
        pitch = config.getfloat('pitch', above=0.)
        teeth = config.getfloat('teeths', above=0.)
        ratio = config.getfloat('gear_ratio', above=0., default=1.0)
        motor_rev = 360. / motor_deg
        steps_per_mm = motor_rev * microsteps / (pitch * teeth) * ratio
    if steps_per_mm is not None:
        inv_step_dist = steps_per_mm
        step_dist = 1.0 / inv_step_dist
    else:
        inv_step_dist = 1. / step_dist
    return step_dist, inv_step_dist

######################################################################
# Steppers
######################################################################

# Code storing the definitions for a stepper motor
class PrinterStepper:
    driver = mcu_stepper = None
    step_driver = None
    max_velocity = max_accel = 0
    def __init__(self, config, logger=None):
        printer = config.get_printer()
        self.name = config.get_name()
        if logger is None:
            self.logger = printer.logger.getChild(self.name)
        else:
            self.logger = logger.getChild(self.name)
        self.need_motor_enable = True
        step_dist = inv_step_dist = None
        # get a driver
        driver = microsteps = mcu_stepper = None
        driver_name = config.get('driver', None)
        if driver_name is not None:
            driver_section = 'driver %s' % driver_name
            driver = driver_base.load_driver(config.getsection(driver_section))
            self.driver = driver
            if driver is not None:
                microsteps = driver.microsteps
                step_dist = driver.step_dist
                inv_step_dist = driver.inv_step_dist
                if not driver.has_step_dir_pins:
                    self.mcu_stepper = mcu_stepper = driver
        if step_dist is None or inv_step_dist is None:
            step_dist, inv_step_dist = calculate_steps(config, microsteps)
            if driver is not None:
                driver.step_dist = step_dist
        # Stepper definition
        ppins = printer.lookup_object('pins')
        if mcu_stepper is None:
            step_pin = config.get('step_pin')
            self.mcu_stepper = mcu_stepper = ppins.setup_pin('stepper', step_pin)
            dir_pin = config.get('dir_pin')
            dir_pin_params = ppins.lookup_pin(dir_pin, can_invert=True)
            mcu_stepper.setup_dir_pin(dir_pin_params)
            mcu_stepper.setup_step_distance(step_dist)
        self.enable = lookup_enable_pin(ppins, config.get('enable_pin', None))
        # Register STEPPER_BUZZ command
        force_move = printer.try_load_module(config, 'force_move')
        force_move.register_stepper(self)
        # Wrappers
        self.step_itersolve = mcu_stepper.step_itersolve
        self.setup_itersolve = mcu_stepper.setup_itersolve
        self.set_stepper_kinematics = mcu_stepper.set_stepper_kinematics
        self.set_ignore_move = mcu_stepper.set_ignore_move
        self.calc_position_from_coord = mcu_stepper.calc_position_from_coord
        self.set_position = mcu_stepper.set_position
        self.get_commanded_position = mcu_stepper.get_commanded_position
        self.set_commanded_position = mcu_stepper.set_commanded_position
        self.get_mcu_position = mcu_stepper.get_mcu_position
        self.get_step_dist = mcu_stepper.get_step_dist
        self.logger.info("steps per mm {} , step in mm {}".
                         format(inv_step_dist, step_dist))
        printer.add_object(self.name, self) # to get printer_state called
    def get_name(self, short=False):
        if short and self.name.startswith('stepper_'):
            return self.name[8:]
        return self.name
    def add_to_endstop(self, mcu_endstop):
        mcu_endstop.add_stepper(self.mcu_stepper)
    def _dist_to_time(self, dist, start_velocity, accel):
        # Calculate the time it takes to travel a distance with constant accel
        time_offset = start_velocity / accel
        return math.sqrt(2. * dist / accel + time_offset**2) - time_offset
    def set_max_jerk(self, max_halt_velocity, max_accel, max_velocity=0):
        if max_velocity > 0:
            self.max_velocity = max_velocity
        self.max_accel = max_accel
        # Calculate the firmware's maximum halt interval time
        step_dist = self.get_step_dist()
        last_step_time = self._dist_to_time(
            step_dist, max_halt_velocity, max_accel)
        second_last_step_time = self._dist_to_time(
            2. * step_dist, max_halt_velocity, max_accel)
        min_stop_interval = second_last_step_time - last_step_time
        self.mcu_stepper.setup_min_stop_interval(min_stop_interval)
    def motor_enable(self, print_time, enable=0):
        if self.need_motor_enable != (not enable):
            self.enable.set_enable(print_time, enable)
        self.need_motor_enable = not enable
    def is_motor_enabled(self):
        return not self.need_motor_enable
    def get_max_velocity(self):
        return self.max_velocity, self.max_accel
    def get_driver(self):
        return self.driver
    def has_driver_endstop(self):
        return getattr(self.mcu_stepper, "has_endstop", False)
    def set_homing_dir(self, homing_dir):
        self.logger.debug("Homing dir: %s" % ['min', 'max'][homing_dir])
        if hasattr(self.mcu_stepper, 'set_homing_dir'):
            self.mcu_stepper.set_homing_dir(homing_dir)


######################################################################
# Stepper controlled rails
######################################################################

# A motor control "rail" with one (or more) steppers and one (or more)
# endstops.
class PrinterRail:
    @staticmethod
    def __get_endstop_pin(config):
        homing_dirs = { 'min' : False, 'max' : True, 'NA' : None}
        homing_dir  = config.getchoice(
            'homing_direction', homing_dirs, 'NA')
        endstop_pin = {
            False: 'endstop_min_pin',
            True: 'endstop_max_pin',
            None: 'endstop_pin'}[homing_dir]
        if config.get(endstop_pin, default=None) is None:
            endstop_pin = 'endstop_pin'
            homing_dir = None
        return homing_dir, endstop_pin
    def __init__(self, config, need_position_minmax=True,
                 default_position_endstop=None):
        homing_dir, endstop_pin = self.__get_endstop_pin(config)
        # Primary stepper
        stepper = PrinterStepper(config)
        self.logger = stepper.logger
        self.step_driver = stepper.step_driver
        self.steppers = [stepper]
        self.name = stepper.get_name(short=True)
        self.step_itersolve = stepper.step_itersolve
        self.get_commanded_position = stepper.get_commanded_position
        self.is_motor_enabled = stepper.is_motor_enabled
        # Primary endstop and its position
        printer = config.get_printer()
        ppins = printer.lookup_object('pins')
        mcu_endstop = ppins.setup_pin('endstop', config.get(endstop_pin))
        self.endstops = [(mcu_endstop, self.name)]
        stepper.add_to_endstop(mcu_endstop)
        if hasattr(mcu_endstop, "get_position_endstop"):
            self.position_endstop = mcu_endstop.get_position_endstop()
        elif default_position_endstop is None:
            self.position_endstop = config.getfloat('position_endstop')
        else:
            self.position_endstop = config.getfloat(
                'position_endstop', default_position_endstop)
        query_endstops = printer.try_load_module(config, 'query_endstops')
        query_endstops.register_endstop(mcu_endstop, self.name)
        # Axis range
        if need_position_minmax:
            self.position_min = config.getfloat('position_min', 0.)
            self.position_max = config.getfloat(
                'position_max', above=self.position_min)
        else:
            self.position_min = 0.
            self.position_max = self.position_endstop
        if (self.position_endstop < self.position_min
            or self.position_endstop > self.position_max):
            raise config.error(
                "position_endstop in section '%s' must be between"
                " position_min and position_max" % config.get_name())
        # Homing mechanics
        self.homing_slowdown = config.getfloat('homing_slowdown', 5.0)
        self.homing_speed = config.getfloat('homing_speed', 5.0, above=0.)
        self.second_homing_speed = config.getfloat(
            'second_homing_speed', self.homing_speed/self.homing_slowdown, above=0.)
        self.homing_retract_dist = config.getfloat(
            'homing_retract_dist', 5., minval=0.)
        self.homing_positive_dir = config.getboolean(
            'homing_positive_dir', homing_dir)
        if self.homing_positive_dir is None:
            axis_len = self.position_max - self.position_min
            if self.position_endstop <= self.position_min + axis_len / 4.:
                self.homing_positive_dir = False
            elif self.position_endstop >= self.position_max - axis_len / 4.:
                self.homing_positive_dir = True
            else:
                raise config.error(
                    "Unable to infer homing_positive_dir in section '%s'" % (
                        config.get_name(),))
        # Endstop stepper phase position tracking
        self.homing_stepper_phases = config.getint(
            'homing_stepper_phases', None, minval=0)
        endstop_accuracy = config.getfloat(
            'homing_endstop_accuracy', None, above=0.)
        self.homing_endstop_accuracy = self.homing_endstop_phase = None
        if self.homing_stepper_phases:
            self.homing_step_dist = step_dist = stepper.get_step_dist()
            self.homing_endstop_phase = config.getint(
                'homing_endstop_phase', None, minval=0
                , maxval=self.homing_stepper_phases-1)
            if (self.homing_endstop_phase is not None
                and config.getboolean('homing_endstop_align_zero', False)):
                # Adjust the endstop position so 0.0 is always at a full step
                micro_steps = self.homing_stepper_phases // 4
                phase_offset = (
                    ((self.homing_endstop_phase + micro_steps // 2)
                     % micro_steps) - micro_steps // 2) * step_dist
                full_step = micro_steps * step_dist
                es_pos = (int(self.position_endstop / full_step + .5)
                          * full_step + phase_offset)
                if es_pos != self.position_endstop:
                    self.logger.info("Changing %s endstop position to %.3f"
                                     " (from %.3f)", self.name,
                                     es_pos, self.position_endstop)
                    self.position_endstop = es_pos
            if endstop_accuracy is None:
                self.homing_endstop_accuracy = self.homing_stepper_phases//2 - 1
            elif self.homing_endstop_phase is not None:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy * .5 / step_dist))
            else:
                self.homing_endstop_accuracy = int(math.ceil(
                    endstop_accuracy / step_dist))
            if self.homing_endstop_accuracy >= self.homing_stepper_phases // 2:
                self.logger.info("Endstop for %s is not accurate enough for stepper"
                                 " phase adjustment", self.name)
                self.homing_stepper_phases = None
            if mcu_endstop.get_mcu().is_fileoutput():
                self.homing_endstop_accuracy = self.homing_stepper_phases
        # Valid for CoreXY and Cartesian Z axis
        self.homing_position = [None, None, None, None]
        name_test = self.name[:1].upper()
        if 'Z' in name_test:
            self.homing_position[0] = config.getfloat(
                'homing_pos_x', default=None,
                minval=0.,
                maxval=200.)
            self.homing_position[1] = config.getfloat(
                'homing_pos_y', default=None,
                minval=0.,
                maxval=200.)
            # TODO: Add support for Z raise before homing!
            self.homing_position[2] = config.getfloat(
                'homing_z_raise', default=None,
                minval=self.position_min,
                maxval=self.position_max)
        self.retract_after_home = self.homing_travel_speed = .0
        if name_test in 'XYZ':
            # Only for cartesian machines
            self.homing_travel_speed = config.getfloat(
                'homing_travel_speed', default=self.homing_speed, above=0)
            self.retract_after_home = config.getfloat(
                'homing_retract_dist_after', 0., minval=0.)
        # Homing offset will be substracted from homed position
        self.homing_offset = config.getfloat('homing_offset', None)
        if self.homing_offset is None:
            # Try in steps and convert steps to mm
            self.homing_offset = (config.getfloat('homing_offset_steps', 0.) *
                                  stepper.get_step_dist())
        # Homing finetune after enstop hit (mainly for deltas)
        self.tune_after_homing = \
            config.getfloat('endstop_correction', None) # in mm
        if self.tune_after_homing is None:
            self.tune_after_homing = (config.getfloat('endstop_correction_steps', 0.) *
                                      stepper.get_step_dist())
        # Set homing direction to stepper
        stepper.set_homing_dir(self.homing_positive_dir)
    def get_tune_after_homing(self):
        return self.tune_after_homing
    def set_homing_offset(self, offset):
        self.homing_offset = offset
    def get_homed_offset(self):
        if not self.homing_stepper_phases:
            return 0. - self.homing_offset
        pos = self.steppers[0].get_mcu_position()
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
        return (delta * self.homing_step_dist) - self.homing_offset
    def get_range(self):
        return self.position_min, self.position_max
    def get_homing_info(self):
        homing_info = collections.namedtuple('homing_info', [
            'speed', 'position_endstop', 'retract_dist', 'positive_dir',
            'second_homing_speed',
            'homing_pos', 'travel_speed', 'retract_after_home'])(
                self.homing_speed, self.position_endstop,
                self.homing_retract_dist, self.homing_positive_dir,
                self.second_homing_speed,
                self.homing_position,
                self.homing_travel_speed, self.retract_after_home)
        return homing_info
    def get_steppers(self):
        return list(self.steppers)
    def get_endstops(self):
        return list(self.endstops)
    def get_homing_init_func(self):
        return [s.driver.init_homing for s in self.steppers
                if hasattr(s.driver, 'init_homing')]
    def add_extra_stepper(self, config):
        stepper = PrinterStepper(config)
        self.steppers.append(stepper)
        self.step_itersolve = self.step_multi_itersolve
        mcu_endstop = self.endstops[0][0]
        homing_dir, endstop_pin = self.__get_endstop_pin(config)
        endstop_pin = config.get(endstop_pin, None)
        if endstop_pin is not None:
            printer = config.get_printer()
            ppins = printer.lookup_object('pins')
            mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
            name = stepper.get_name(short=True)
            self.endstops.append((mcu_endstop, name))
            query_endstops = printer.try_load_module(config, 'query_endstops')
            query_endstops.register_endstop(mcu_endstop, name)
        stepper.add_to_endstop(mcu_endstop)
        # Set homing direction to stepper
        stepper.set_homing_dir(self.homing_positive_dir)
    def add_to_endstop(self, mcu_endstop):
        for stepper in self.steppers:
            stepper.add_to_endstop(mcu_endstop)
    def step_multi_itersolve(self, cmove):
        for stepper in self.steppers:
            stepper.step_itersolve(cmove)
    def setup_itersolve(self, alloc_func, *params):
        for stepper in self.steppers:
            stepper.setup_itersolve(alloc_func, *params)
    def set_max_jerk(self, max_halt_velocity, max_accel, max_velocity=0):
        for stepper in self.steppers:
            stepper.set_max_jerk(max_halt_velocity, max_accel, max_velocity)
    def set_commanded_position(self, pos):
        for stepper in self.steppers:
            stepper.set_commanded_position(pos)
    def set_position(self, coord):
        for stepper in self.steppers:
            stepper.set_position(coord)
    def motor_enable(self, print_time, enable=0):
        for stepper in self.steppers:
            stepper.motor_enable(print_time, enable)


# Wrapper for dual stepper motor support
def LookupMultiRail(config):
    rail = PrinterRail(config)
    for i in range(1, 99):
        if not config.has_section(config.get_name() + str(i)):
            break
        rail.add_extra_stepper(config.getsection(config.get_name() + str(i)))
    return rail
