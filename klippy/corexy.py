# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, homing

HOMING_SLOWDOWN = 5

StepList = (0, 1, 2)

class CoreXYKinematics:
    name = "coreXY"
    def __init__(self, toolhead, printer, config, coresign=1.):
        self.toolhead = toolhead
        self.logger = printer.logger.getChild(self.name)
        self.steppers = [
            stepper.PrinterHomingStepper(
                printer, config.getsection('stepper_x')),
            stepper.PrinterHomingStepper(
                printer, config.getsection('stepper_y')),
            stepper.LookupMultiHomingStepper(
                printer, config.getsection('stepper_z'))]
        self.combined_endstops = config.getboolean('combined_endstops', False)
        if self.combined_endstops:
            # endstops are always triggered both
            #   => no cross connection necessary
            pass
        else:
            # x/y axes also need to stop on each others endstops
            #   => cross connect endstop and stepper x->y and y->x
            self.steppers[0].mcu_endstop.add_stepper(self.steppers[1].mcu_stepper)
            self.steppers[1].mcu_endstop.add_stepper(self.steppers[0].mcu_stepper)
        max_velocity, max_accel = toolhead.get_max_velocity()
        self.max_z_velocity = config.getfloat(
            'max_z_velocity', max_velocity, above=0., maxval=max_velocity)
        self.max_z_accel = config.getfloat(
            'max_z_accel', max_accel, above=0., maxval=max_accel)
        self.need_motor_enable = True
        if toolhead.allow_move_wo_homing is False:
            self.limits = [(1.0, -1.0)] * 3
        else:
            # Just set min and max values for SW limit
            self.limits = [ (s.position_min, s.position_max)
                            for s in self.steppers ]
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        max_xy_halt_velocity = max_halt_velocity * math.sqrt(2.)
        self.steppers[0].set_max_jerk(max_xy_halt_velocity, max_accel)
        self.steppers[1].set_max_jerk(max_xy_halt_velocity, max_accel)
        self.steppers[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), self.max_z_accel)
        self.coresign = coresign
        self.experimental = config.getboolean(
            'experimental', False)
    def get_steppers(self, flags=""):
        if flags == "Z":
            return [self.steppers[2]]
        return list(self.steppers)
    def get_position(self):
        pos = [s.mcu_stepper.get_commanded_position() for s in self.steppers]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    def set_position(self, newpos, homing_axes):
        pos = (newpos[0] + newpos[1], newpos[0] - newpos[1], newpos[2])
        for i in StepList:
            s = self.steppers[i]
            s.set_position(pos[i])
            if i in homing_axes:
                self.limits[i] = (s.position_min, s.position_max)
    def home(self, homing_state):
        # Each axis is homed independently and in order
        sensor_funcs = [ getattr(s.driver, 'init_home', None) for s in self.steppers ]
        for axis in homing_state.get_axes():
            s = self.steppers[axis]
            # Determine moves
            if s.homing_positive_dir:
                pos = s.position_endstop - 1.5*(
                    s.position_endstop - s.position_min)
                rpos = s.position_endstop - s.homing_retract_dist
                r2pos = rpos - s.homing_retract_dist
            else:
                pos = s.position_endstop + 1.5*(
                    s.position_max - s.position_endstop)
                rpos = s.position_endstop + s.homing_retract_dist
                r2pos = rpos + s.homing_retract_dist
            # Initial homing
            homing_speed = s.homing_speed
            if axis == 2:
                homing_speed = min(homing_speed, self.max_z_velocity)
            homepos = [None, None, None, None]
            # Set Z homing position if defined
            homing_state.retract([s.homing_pos_x, # X axis position
                                  s.homing_pos_y, # Y axis position
                                  None, None],
                                 self.steppers[0].homing_speed)
            homepos[axis] = s.position_endstop
            coord = [None, None, None, None]
            coord[axis] = pos
            if axis < 2 and self.combined_endstops:
                endstops = ( self.steppers[0].get_endstops()
                           + self.steppers[1].get_endstops() )
            else:
                endstops = s.get_endstops()
            homing_state.home(coord, homepos, endstops, homing_speed,
                              init_sensor=sensor_funcs)
            # Retract
            coord[axis] = rpos
            homing_state.retract(coord, homing_speed)
            # Home again
            coord[axis] = r2pos
            homing_state.home(coord, homepos, endstops,
                              homing_speed/HOMING_SLOWDOWN, second_home=True,
                              init_sensor=sensor_funcs)
            if axis == 2:
                # Support endstop phase detection on Z axis
                coord[axis] = s.position_endstop + s.get_homed_offset()
                homing_state.set_homed_position(coord)
            if 0. < s.retract_after_home:
                movepos = [None, None, None, None]
                # Retract
                if s.homing_positive_dir:
                    movepos[axis] = s.position_endstop - s.retract_after_home
                else:
                    movepos[axis] = s.position_endstop + s.retract_after_home
                homing_state.retract(movepos, homing_speed)
    def motor_off(self, print_time):
        if self.toolhead.require_home_after_motor_off is True \
           and self.toolhead.sw_limit_check_enabled is True:
            self.limits = [(1.0, -1.0)] * 3
        for stepper in self.steppers:
            stepper.motor_enable(print_time, 0)
        self.need_motor_enable = True
    def _check_motor_enable(self, print_time, move):
        if move.axes_d[0] or move.axes_d[1]:
            self.steppers[0].motor_enable(print_time, 1)
            self.steppers[1].motor_enable(print_time, 1)
        if move.axes_d[2]:
            self.steppers[2].motor_enable(print_time, 1)
        need_motor_enable = False
        for i in StepList:
            need_motor_enable |= self.steppers[i].need_motor_enable
        self.need_motor_enable = need_motor_enable
    def _check_endstops(self, move):
        end_pos = move.end_pos
        for i in StepList:
            if (move.axes_d[i]
                and (end_pos[i] < self.limits[i][0]
                     or end_pos[i] > self.limits[i][1])):
                if self.limits[i][0] > self.limits[i][1]:
                    raise homing.EndstopMoveError(
                        end_pos, "Must home axis first")
                raise homing.EndstopMoveError(end_pos)
    def check_move(self, move):
        xpos, ypos = move.end_pos[:2]
        if self.toolhead.sw_limit_check_enabled is True:
            limits = self.limits
            if (xpos < limits[0][0] or xpos > limits[0][1]
                or ypos < limits[1][0] or ypos > limits[1][1]):
                self._check_endstops(move)
        if not move.axes_d[2]:
            # Normal XY move - use defaults
            return
        # Move with Z - update velocity and accel for slower Z axis
        if self.toolhead.sw_limit_check_enabled is True:
            self._check_endstops(move)
        z_ratio = move.move_d / abs(move.axes_d[2])
        move.limit_speed(
            self.max_z_velocity * z_ratio, self.max_z_accel * z_ratio)
    def move(self, print_time, move):
        if self.need_motor_enable:
            self._check_motor_enable(print_time, move)

        sxp = move.start_pos[0]
        syp = move.start_pos[1]
        if self.experimental:
            move_start_pos = ((sxp + syp), (sxp - syp), move.start_pos[2])
            exp = (sxp - move.end_pos[0])
            eyp = (syp - move.end_pos[1]) * self.coresign
            axes_d = ((exp + eyp),
                      (exp - eyp),
                      move.start_pos[2])
            core_flag = (self.coresign == -1) # TODO FIXME
        else:
            move_start_pos = (sxp + syp, sxp - syp, move.start_pos[2])
            exp = move.end_pos[0]
            eyp = move.end_pos[1]
            axes_d = ((exp + eyp) - move_start_pos[0],
                      (exp - eyp) - move_start_pos[1], move.axes_d[2])
            core_flag = False
        for i in StepList:
            axis_d = axes_d[i]
            if not axis_d:
                continue
            step_const = self.steppers[i].step_const
            move_time = print_time
            start_pos = move_start_pos[i]
            axis_r = abs(axis_d) / move.move_d
            accel = move.accel * axis_r
            cruise_v = move.cruise_v * axis_r

            # Acceleration steps
            if move.accel_r:
                accel_d = move.accel_r * axis_d
                step_const(move_time, start_pos, accel_d,
                           move.start_v * axis_r, accel, core=core_flag)
                start_pos += accel_d
                move_time += move.accel_t
            # Cruising steps
            if move.cruise_r:
                cruise_d = move.cruise_r * axis_d
                step_const(move_time, start_pos, cruise_d, cruise_v, 0., core=core_flag)
                start_pos += cruise_d
                move_time += move.cruise_t
            # Deceleration steps
            if move.decel_r:
                decel_d = move.decel_r * axis_d
                step_const(move_time, start_pos, decel_d, cruise_v, -accel, core=core_flag)

    def is_homed(self):
        ret = [1, 1, 1]
        if self.toolhead.sw_limit_check_enabled is True:
            for i in StepList:
                if self.limits[i][0] > self.limits[i][1]:
                    ret[i] = 0
        return ret
    def update_velocities(self):
        max_halt_velocity = self.toolhead.get_max_axis_halt()
        max_velocity, max_accel = self.toolhead.get_max_velocity()
        self.steppers[0].set_max_jerk(max_halt_velocity, max_accel)
        self.steppers[1].set_max_jerk(max_halt_velocity, max_accel)

class CoreYXKinematics(CoreXYKinematics):
    name = "coreYX"
    def __init__(self, toolhead, printer, config):
        CoreXYKinematics.__init__(self, toolhead, printer, config, coresign=-1.)
