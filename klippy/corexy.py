# Code for handling the kinematics of corexy robots
#
# Copyright (C) 2017-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import stepper, homing, chelper

StepList = (0, 1, 2)

class CoreXYKinematics:
    name = "coreXY"
    def __init__(self, toolhead, config, coresign=1.):
        self.toolhead = toolhead
        self.logger = config.get_printer().logger.getChild(self.name)
        self.rails = [ stepper.PrinterRail(config.getsection('stepper_x')),
                       stepper.PrinterRail(config.getsection('stepper_y')),
                       stepper.LookupMultiRail(config.getsection('stepper_z')) ]
        self.combined_endstops = config.getboolean('combined_endstops', False)
        if self.combined_endstops:
            # endstops are always triggered both
            #   => no cross connection necessary
            pass
        else:
            # x/y axes also need to stop on each others endstops
            #   => cross connect endstop and stepper x->y and y->x
            self.rails[0].add_to_endstop(self.rails[1].get_endstops()[0][0])
            self.rails[1].add_to_endstop(self.rails[0].get_endstops()[0][0])
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
            self.limits = [ rail.get_range() for rail in self.rails ]
        # Setup iterative solver
        ffi_main, ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(ffi_lib.move_alloc(), ffi_lib.free)
        self.move_fill = ffi_lib.move_fill
        self.rails[0].setup_itersolve(ffi_main.gc(
            ffi_lib.corexy_stepper_alloc('+'), ffi_lib.free))
        self.rails[1].setup_itersolve(ffi_main.gc(
            ffi_lib.corexy_stepper_alloc('-'), ffi_lib.free))
        self.rails[2].setup_cartesian_itersolve('z')
        # Setup stepper max halt velocity
        max_halt_velocity = toolhead.get_max_axis_halt()
        max_xy_halt_velocity = max_halt_velocity * math.sqrt(2.)
        self.rails[0].set_max_jerk(max_xy_halt_velocity, max_accel, max_velocity)
        self.rails[1].set_max_jerk(max_xy_halt_velocity, max_accel, max_velocity)
        self.rails[2].set_max_jerk(
            min(max_halt_velocity, self.max_z_velocity), self.max_z_accel, self.max_z_velocity)
        self.coresign = coresign
        self.experimental = config.getboolean(
            'experimental', False)
    def get_rails(self, flags=""):
        if flags == "Z":
            return [self.rails[2]]
        return list(self.rails)
    def get_position(self):
        pos = [rail.get_commanded_position() for rail in self.rails]
        return [0.5 * (pos[0] + pos[1]), 0.5 * (pos[0] - pos[1]), pos[2]]
    def set_position(self, newpos, homing_axes):
        pos = (newpos[0] + newpos[1], newpos[0] - newpos[1], newpos[2])
        for i in StepList:
            rail = self.rails[i]
            rail.set_position(pos[i])
            if i in homing_axes:
                self.limits[i] = rail.get_range()
    def home(self, homing_state):
        # Each axis is homed independently and in order
        for axis in homing_state.get_axes():
            rail = self.rails[axis]
            # Determine moves
            position_min, position_max = rail.get_range()
            hi = rail.get_homing_info()
            if hi.positive_dir:
                pos = hi.position_endstop - 1.5*(
                    hi.position_endstop - position_min)
                rpos = hi.position_endstop - hi.retract_dist
                r2pos = rpos - hi.retract_dist
            else:
                pos = hi.position_endstop + 1.5*(
                    position_max - hi.position_endstop)
                rpos = hi.position_endstop + hi.retract_dist
                r2pos = rpos + hi.retract_dist
            # Initial homing
            homing_speed = hi.speed
            if axis == 2:
                homing_speed = min(homing_speed, self.max_z_velocity)
            homepos = [None, None, None, None]
            # Set Z homing position if defined
            homing_state.retract(hi.homing_pos, hi.travel_speed)
            homepos[axis] = hi.position_endstop
            coord = [None, None, None, None]
            coord[axis] = pos
            if axis < 2 and self.combined_endstops:
                endstops = ( self.rails[0].get_endstops()
                           + self.rails[1].get_endstops() )
            else:
                endstops = rail.get_endstops()
            homing_state.home(coord, homepos, endstops, homing_speed,
                              init_sensor=hi.init_home_funcs)
            # Retract
            coord[axis] = rpos
            homing_state.retract(coord, homing_speed)
            # Home again
            coord[axis] = r2pos
            homing_state.home(coord, homepos, endstops,
                              hi.speed_slow, second_home=True,
                              init_sensor=hi.init_home_funcs)
            if axis == 2:
                # Support endstop phase detection on Z axis
                coord[axis] = hi.position_endstop + rail.get_homed_offset()
                homing_state.set_homed_position(coord)
            if 0. < hi.retract_after_home:
                movepos = [None, None, None, None]
                # Retract
                if hi.positive_dir:
                    movepos[axis] = hi.position_endstop - hi.retract_after_home
                else:
                    movepos[axis] = hi.position_endstop + hi.retract_after_home
                homing_state.retract(movepos, homing_speed)
    def motor_off(self, print_time):
        if self.toolhead.require_home_after_motor_off is True \
           and self.toolhead.sw_limit_check_enabled is True:
            self.limits = [(1.0, -1.0)] * 3
        for rail in self.rails:
            rail.motor_enable(print_time, 0)
        self.need_motor_enable = True
    def _check_motor_enable(self, print_time, move):
        if move.axes_d[0] or move.axes_d[1]:
            self.rails[0].motor_enable(print_time, 1)
            self.rails[1].motor_enable(print_time, 1)
        if move.axes_d[2]:
            self.rails[2].motor_enable(print_time, 1)
        need_motor_enable = False
        for i in StepList:
            need_motor_enable |= not self.rails[i].is_motor_enabled()
        self.need_motor_enable = need_motor_enable
        self.toolhead.motor_on(print_time)
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

        axes_d = move.axes_d
        cmove = self.cmove
        self.move_fill(
            cmove, print_time,
            move.accel_t, move.cruise_t, move.decel_t,
            move.start_pos[0], move.start_pos[1], move.start_pos[2],
            axes_d[0], axes_d[1], axes_d[2],
            move.start_v, move.cruise_v, move.accel)
        rail_x, rail_y, rail_z = self.rails
        if axes_d[0] or axes_d[1]:
            rail_x.step_itersolve(cmove)
            rail_y.step_itersolve(cmove)
        if axes_d[2]:
            rail_z.step_itersolve(cmove)
        '''
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
        '''
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
        self.rails[0].set_max_jerk(max_halt_velocity, max_accel, max_velocity)
        self.rails[1].set_max_jerk(max_halt_velocity, max_accel, max_velocity)

class CoreYXKinematics(CoreXYKinematics):
    name = "coreYX"
    def __init__(self, toolhead, config):
        CoreXYKinematics.__init__(self, toolhead, config, coresign=-1.)
