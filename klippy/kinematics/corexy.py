# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2019  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math
import stepper, homing

class CoreXYKinematics:
    name = "coreXY"
    def __init__(self, toolhead, config):
        self.printer = config.get_printer()
        self.toolhead = toolhead
        self.logger = self.printer.get_logger(self.name)
        # Setup axis rails
        self.rails = [ stepper.PrinterRail(config.getsection('stepper_x')),
                       stepper.PrinterRail(config.getsection('stepper_y')),
                       stepper.LookupMultiRail(config.getsection('stepper_z')) ]
        # Check if cross connection if necessary
        #    combined endstops are always triggering both
        if not config.getboolean('combined_endstops', False):
            # x/y axes also need to stop on each others endstops
            #   => cross connect endstop and stepper x->y and y->x
            self.rails[0].add_to_endstop(self.rails[1].get_endstops()[0][0])
            self.rails[1].add_to_endstop(self.rails[0].get_endstops()[0][0])
        self.rails[0].setup_itersolve('corexy_stepper_alloc', '+')
        self.rails[1].setup_itersolve('corexy_stepper_alloc', '-')
        self.rails[2].setup_itersolve('cartesian_stepper_alloc', 'z')
        for s in self.get_steppers():
            s.set_trapq(toolhead.get_trapq())
            toolhead.register_step_generator(s.generate_steps)
        # Setup boundary checks
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.max_velocity = max_velocity
        self.max_accel = max_accel
        if toolhead.allow_move_wo_homing is False:
            self.limits = [(1.0, -1.0)] * 3
        else:
            # Just set min and max values for SW limit
            self.limits = [ rail.get_range() for rail in self.rails ]
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        max_xy_halt_velocity = max_halt_velocity * math.sqrt(2.)
        max_xy_accel = max_accel * math.sqrt(2.)
        self.rails[0].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[1].set_max_jerk(max_xy_halt_velocity, max_xy_accel)
        self.rails[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), self.max_z_accel)
    def get_steppers(self, flags=""):
        if flags == "Z":
            return self.rails[2].get_steppers()
        return [s for rail in self.rails for s in rail.get_steppers()]
    def calc_position(self):
        pos = [rail.get_commanded_position() for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    def set_position(self, newpos, homing_axes):
        for i, rail in enumerate(self.rails):
            rail.set_position(newpos)
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine movement
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            homepos = [None, None, None, None]
            homepos[axis] = hi.position_endstop
            forcepos = list(homepos)
            if hi.positive_dir:
                forcepos[axis] -= 1.5 * (hi.position_endstop - position_min)
            else:
                forcepos[axis] += 1.5 * (position_max - hi.position_endstop)
            # Perform homing
            homing_state.home_rails([rail], forcepos, homepos)
    def motor_off(self, print_time):
        if self.toolhead.require_home_after_motor_off \
           and self.toolhead.sw_limit_check_enabled:
            self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)
    def _check_endstops(self, move):
        if not self.toolhead.sw_limit_check_enabled:
            return
        end_pos = move.end_pos
        for i in (0, 1, 2):
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)
    def check_move(self, move):
        limits = self.limits
        xpos, ypos = move.end_pos[:2]
        if (xpos < limits[0][0] or xpos > limits[0][1]
            or ypos < limits[1][0] or ypos > limits[1][1]):
            self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def get_status(self):
        return {'homed_axes': "".join([a
                    for a, (l, h) in zip("XYZ", self.limits) if l <= h])
        }

    def get_rails(self):
        return list(self.rails)
    def is_homed(self):
        ret = [1, 1, 1]
        for i in (0, 1, 2):
            if self.limits[i][0] > self.limits[i][1]:
                ret[i] = 0
        return ret
    def get_max_limits(self):
        return [
            {'rail': self.rails[0],
             'acc': self.max_accel, 'velocity': self.max_velocity},
            {'rail': self.rails[1],
             'acc': self.max_accel, 'velocity': self.max_velocity},
            {'rail': self.rails[2],
             'acc': self.max_z_accel, 'velocity': self.max_z_velocity},
        ]

def load_kinematics(toolhead, config):
    return CoreXYKinematics(toolhead, config)
