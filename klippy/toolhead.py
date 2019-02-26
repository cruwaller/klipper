# Code for coordinating events on the printer toolhead
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, importlib
import mcu, homing, chelper, kinematics.extruder

# Common suffixes: _d is distance (in mm), _v is velocity (in
#   mm/second), _v2 is velocity squared (mm^2/s^2), _t is time (in
#   seconds), _r is ratio (scalar between 0.0 and 1.0)

# Class to track each move request
class Move:
    def __init__(self, toolhead, start_pos, end_pos, speed):
        self.toolhead = toolhead
        self.start_pos = tuple(start_pos)
        self.end_pos = tuple(end_pos)
        self.accel = toolhead.max_accel
        velocity = min(speed, toolhead.max_velocity)
        self.cmove = toolhead.cmove
        self.is_kinematic_move = True
        self.axes_d = axes_d = [end_pos[i] - start_pos[i] for i in (0, 1, 2, 3)]
        self.move_d = move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        if move_d < .000000001:
            # Extrude only move
            self.end_pos = (start_pos[0], start_pos[1], start_pos[2],
                            end_pos[3])
            axes_d[0] = axes_d[1] = axes_d[2] = 0.
            self.move_d = move_d = abs(axes_d[3])
            self.accel = 99999999.9
            velocity = speed
            self.is_kinematic_move = False
        self.min_move_t = move_d / velocity
        # Junction speeds are tracked in velocity squared.  The
        # delta_v2 is the maximum amount of this squared-velocity that
        # can change in this move.
        self.max_start_v2 = 0.
        self.max_cruise_v2 = velocity**2
        self.delta_v2 = 2.0 * move_d * self.accel
        self.max_smoothed_v2 = 0.
        self.smooth_delta_v2 = 2.0 * move_d * toolhead.max_accel_to_decel
    def limit_speed(self, speed, accel):
        speed2 = speed**2
        if speed2 < self.max_cruise_v2:
            self.max_cruise_v2 = speed2
            self.min_move_t = self.move_d / speed
        self.accel = min(self.accel, accel)
        self.delta_v2 = 2.0 * self.move_d * self.accel
        self.smooth_delta_v2 = min(self.smooth_delta_v2, self.delta_v2)
    def calc_junction(self, prev_move):
        if not self.is_kinematic_move or not prev_move.is_kinematic_move:
            return
        # Allow extruder to calculate its maximum junction
        extruder_v2 = self.toolhead.extruder.calc_junction(prev_move, self)
        # Find max velocity using approximated centripetal velocity as
        # described at:
        # https://onehossshay.wordpress.com/2011/09/24/improving_grbl_cornering_algorithm/
        axes_d = self.axes_d
        prev_axes_d = prev_move.axes_d
        junction_cos_theta = -((axes_d[0] * prev_axes_d[0]
                                + axes_d[1] * prev_axes_d[1]
                                + axes_d[2] * prev_axes_d[2])
                               / (self.move_d * prev_move.move_d))
        if junction_cos_theta > 0.999999:
            return
        junction_cos_theta = max(junction_cos_theta, -0.999999)
        sin_theta_d2 = math.sqrt(0.5*(1.0-junction_cos_theta))
        R = self.toolhead.junction_deviation * sin_theta_d2 / (1. - sin_theta_d2)
        tan_theta_d2 = sin_theta_d2 / math.sqrt(0.5*(1.0+junction_cos_theta))
        move_centripetal_v2 = .5 * self.move_d * tan_theta_d2 * self.accel
        prev_move_centripetal_v2 = (.5 * prev_move.move_d * tan_theta_d2
                                    * prev_move.accel)
        self.max_start_v2 = min(
            R * self.accel, R * prev_move.accel,
            move_centripetal_v2, prev_move_centripetal_v2,
            extruder_v2, self.max_cruise_v2, prev_move.max_cruise_v2,
            prev_move.max_start_v2 + prev_move.delta_v2)
        self.max_smoothed_v2 = min(
            self.max_start_v2
            , prev_move.max_smoothed_v2 + prev_move.smooth_delta_v2)
    def set_junction(self, start_v2, cruise_v2, end_v2):
        # Determine accel, cruise, and decel portions of the move distance
        inv_delta_v2 = 1. / self.delta_v2
        self.accel_r = accel_r = (cruise_v2 - start_v2) * inv_delta_v2
        self.decel_r = decel_r = (cruise_v2 - end_v2) * inv_delta_v2
        self.cruise_r = cruise_r = 1. - accel_r - decel_r
        # Determine move velocities
        self.start_v = start_v = math.sqrt(start_v2)
        self.cruise_v = cruise_v = math.sqrt(cruise_v2)
        self.end_v = end_v = math.sqrt(end_v2)
        # Determine time spent in each portion of move (time is the
        # distance divided by average velocity)
        self.accel_t = accel_r * self.move_d / ((start_v + cruise_v) * 0.5)
        self.cruise_t = cruise_r * self.move_d / cruise_v
        self.decel_t = decel_r * self.move_d / ((end_v + cruise_v) * 0.5)
    def move(self):
        # Generate step times for the move
        next_move_time = self.toolhead.get_next_move_time()
        if self.is_kinematic_move:
            self.toolhead.move_fill(
                self.cmove, next_move_time,
                self.accel_t, self.cruise_t, self.decel_t,
                self.start_pos[0], self.start_pos[1], self.start_pos[2],
                self.axes_d[0], self.axes_d[1], self.axes_d[2],
                self.start_v, self.cruise_v, self.accel)
            self.toolhead.kin.move(next_move_time, self)
        if self.axes_d[3]:
            self.toolhead.extruder.move(next_move_time, self)
        self.toolhead.update_move_time(
            self.accel_t + self.cruise_t + self.decel_t)

LOOKAHEAD_FLUSH_TIME = 0.250

# Class to track a list of pending move requests and to facilitate
# "look-ahead" across moves to reduce acceleration between moves.
class MoveQueue:
    def __init__(self):
        self.extruder_lookahead = None
        self.queue = []
        self.leftover = 0
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def reset(self):
        del self.queue[:]
        self.leftover = 0
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
    def set_flush_time(self, flush_time):
        self.junction_flush = flush_time
    def set_extruder(self, extruder):
        self.extruder_lookahead = extruder.lookahead
    def flush(self, lazy=False):
        self.junction_flush = LOOKAHEAD_FLUSH_TIME
        update_flush_count = lazy
        queue = self.queue
        flush_count = len(queue)
        # Traverse queue from last to first move and determine maximum
        # junction speed assuming the robot comes to a complete stop
        # after the last move.
        delayed = []
        next_end_v2 = next_smoothed_v2 = peak_cruise_v2 = 0.
        for i in range(flush_count-1, self.leftover-1, -1):
            move = queue[i]
            reachable_start_v2 = next_end_v2 + move.delta_v2
            start_v2 = min(move.max_start_v2, reachable_start_v2)
            reachable_smoothed_v2 = next_smoothed_v2 + move.smooth_delta_v2
            smoothed_v2 = min(move.max_smoothed_v2, reachable_smoothed_v2)
            if smoothed_v2 < reachable_smoothed_v2:
                # It's possible for this move to accelerate
                if (smoothed_v2 + move.smooth_delta_v2 > next_smoothed_v2
                    or delayed):
                    # This move can decelerate or this is a full accel
                    # move after a full decel move
                    if update_flush_count and peak_cruise_v2:
                        flush_count = i
                        update_flush_count = False
                    peak_cruise_v2 = min(move.max_cruise_v2, (
                        smoothed_v2 + reachable_smoothed_v2) * .5)
                    if delayed:
                        # Propagate peak_cruise_v2 to any delayed moves
                        if not update_flush_count and i < flush_count:
                            for m, ms_v2, me_v2 in delayed:
                                mc_v2 = min(peak_cruise_v2, ms_v2)
                                m.set_junction(min(ms_v2, mc_v2), mc_v2
                                               , min(me_v2, mc_v2))
                        del delayed[:]
                if not update_flush_count and i < flush_count:
                    cruise_v2 = min((start_v2 + reachable_start_v2) * .5
                                    , move.max_cruise_v2, peak_cruise_v2)
                    move.set_junction(min(start_v2, cruise_v2), cruise_v2
                                      , min(next_end_v2, cruise_v2))
            else:
                # Delay calculating this move until peak_cruise_v2 is known
                delayed.append((move, start_v2, next_end_v2))
            next_end_v2 = start_v2
            next_smoothed_v2 = smoothed_v2
        if update_flush_count:
            return
        # Allow extruder to do its lookahead
        move_count = self.extruder_lookahead(queue, flush_count, lazy)
        # Generate step times for all moves ready to be flushed
        for move in queue[:move_count]:
            move.move()
        # Remove processed moves from the queue
        self.leftover = flush_count - move_count
        del queue[:move_count]
    def add_move(self, move):
        self.queue.append(move)
        if len(self.queue) == 1:
            return
        move.calc_junction(self.queue[-2])
        self.junction_flush -= move.min_move_t
        if self.junction_flush <= 0.:
            # Enough moves have been queued to reach the target flush time.
            self.flush(lazy=True)


STALL_TIME = 0.100

# Main code to track events (and their timing) on the printer toolhead
class ToolHead:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.logger.getChild('toolhead')
        self.logger.info("toolhead '{}' created".format(config.section))
        self.reactor = self.printer.get_reactor()
        self.all_mcus = [
            m for n, m in self.printer.lookup_objects(module='mcu')]
        self.mcu = self.all_mcus[0]
        self.move_queue = MoveQueue()
        self.commanded_pos = [0., 0., 0., 0.]
        self.printer.register_event_handler("gcode:request_restart",
                                            self._handle_request_restart)
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
        # Velocity and acceleration control
        self.max_velocity = config.getfloat('max_velocity', above=0.)
        self.max_accel = config.getfloat('max_accel', above=0.)
        decel_ratio = config.getfloat(
            'max_accel_to_decel_ratio', default=1.0, above=0.,
            maxval=1.)
        self.requested_accel_to_decel = config.getfloat(
            'max_accel_to_decel', self.max_accel * decel_ratio, above=0.)
        self.max_accel_to_decel = self.requested_accel_to_decel
        self.square_corner_velocity = config.getfloat(
            'square_corner_velocity', 5., minval=0.)
        self.config_max_velocity = self.max_velocity
        self.config_max_accel = self.max_accel
        self.config_square_corner_velocity = self.square_corner_velocity
        self.junction_deviation = 0.
        self._calc_junction_deviation()
        self.homing_order = config.get('homing_order', 'XYZ').upper()
        self.require_home_after_motor_off = config.getboolean(
            'require_home_after_motor_off', True)
        self.sw_limit_check_enabled = config.getboolean(
            'sw_limit_check_enabled', True)
        self.allow_move_wo_homing = config.getboolean(
            'allow_move_without_home', False)

        # Print time tracking
        self.buffer_time_low = config.getfloat(
            'buffer_time_low', 1.000, above=0.)
        self.buffer_time_high = config.getfloat(
            'buffer_time_high', 2.000, above=self.buffer_time_low)
        self.buffer_time_start = config.getfloat(
            'buffer_time_start', 0.250, above=0.)
        self.move_flush_time = config.getfloat(
            'move_flush_time', 0.050, above=0.)
        self.print_time = 0.
        self.last_print_start_time = 0.
        self.need_check_stall = -1.
        self.print_stall = 0
        self.sync_print_time = True
        self.idle_flush_print_time = 0.
        self.flush_timer = self.reactor.register_timer(self._flush_handler)
        self.move_queue.set_flush_time(self.buffer_time_high)
        self.printer.try_load_module(config, "idle_timeout")
        self.printer.try_load_module(config, "statistics")
        self.printer.try_load_module(config, "manual_probe")
        # Setup iterative solver
        ffi_main, self.ffi_lib = chelper.get_ffi()
        self.cmove = ffi_main.gc(self.ffi_lib.move_alloc(), self.ffi_lib.free)
        self.move_fill = self.ffi_lib.move_fill
        self.accel_order = config.getchoice(
            'acceleration_order', { "2": 2, "4": 4, "6": 6 }, "2")
        self.ffi_lib.move_set_accel_order(self.cmove, self.accel_order)
        # Create kinematics class
        self.extruder = kinematics.extruder.DummyExtruder()
        self.move_queue.set_extruder(self.extruder)
        kin_name = config.get('kinematics')
        try:
            mod = importlib.import_module('kinematics.' + kin_name)
            self.kin = mod.load_kinematics(self, config)
        except config.error as e:
            raise
        except self.printer.lookup_object('pins').error as e:
            raise
        except:
            msg = "Error loading kinematics '%s'" % (kin_name,)
            self.logger.exception(msg)
            raise config.error(msg)
        # Pause/Idle position
        self.idle_position = idle_position = \
            config.get('idle_position', default=None)
        if idle_position is None:
            idle_x = config.getfloat("idle_position_x", default=.0)
            idle_y = config.getfloat("idle_position_y", default=.0)
            idle_z_lift = config.getfloat("idle_position_z_lift", default=.4)
            idle_travel_s = min(60. * self.config_max_velocity,
                                config.getint("idle_position_travel_speed", default=6000))
            moves = []
            if idle_z_lift:
                moves.append("G91\nG1 Z%s F%s\nG90" % (idle_z_lift, idle_travel_s))
            moves.append("G1 X%s Y%s F%s" % (idle_x, idle_y, idle_travel_s))
            self.idle_position = "\n".join(moves)
            self.logger.info("Idle position: X:%s Y:%s Zlift:%s" % (
                idle_x, idle_y, idle_z_lift))
        self.logger.info("Idle position command: '%s'" %
            self.idle_position.replace("\n", ", "))
        # SET_VELOCITY_LIMIT command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('SET_VELOCITY_LIMIT', self.cmd_SET_VELOCITY_LIMIT,
                               desc=self.cmd_SET_VELOCITY_LIMIT_help)
        gcode.register_command('IDLE_POSITION', self.move_to_idle_pos,
                               desc="Move head to defined idle position")
        # gcode.register_command('M204', self.cmd_M204)
        # Register TURN_OFF_HEATERS command
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
        self.logger.info("Kinematic created: %s" % self.kin.name)
        self.logger.info("max_accel: %s" % (self.max_accel,))
        self.logger.info("max_accel_to_decel: %s" % (self.max_accel_to_decel,))
        self.logger.info("junction_deviation: %s" % (self.junction_deviation,))
    def move_to_idle_pos(self, *args):
        if self.idle_position:
            gcode = self.printer.lookup_object('gcode')
            self.wait_moves()
            orig = gcode.absolutecoord
            gcode.absolutecoord = True
            gcode.run_script_from_command(self.idle_position)
            gcode.absolutecoord = orig
    def get_estimated_print_time(self):
        return self.mcu.estimated_print_time(self.reactor.monotonic())
    # Print time tracking
    def update_move_time(self, movetime):
        self.print_time += movetime
        flush_to_time = self.print_time - self.move_flush_time
        for m in self.all_mcus:
            m.flush_moves(flush_to_time)
    def _calc_print_time(self):
        curtime = self.reactor.monotonic()
        est_print_time = self.mcu.estimated_print_time(curtime)
        if est_print_time + self.buffer_time_start > self.print_time:
            self.print_time = est_print_time + self.buffer_time_start
            self.last_print_start_time = self.print_time
            self.printer.send_event("toolhead:sync_print_time",
                                    curtime, est_print_time, self.print_time)
    def get_next_move_time(self):
        if self.sync_print_time:
            self.sync_print_time = False
            self.reactor.update_timer(self.flush_timer, self.reactor.NOW)
            self._calc_print_time()
        return self.print_time
    def _flush_lookahead(self, must_sync=False):
        sync_print_time = self.sync_print_time
        self.move_queue.flush()
        self.idle_flush_print_time = 0.
        if sync_print_time or must_sync:
            self.sync_print_time = True
            self.move_queue.set_flush_time(self.buffer_time_high)
            self.need_check_stall = -1.
            self.reactor.update_timer(self.flush_timer, self.reactor.NEVER)
            for m in self.all_mcus:
                m.flush_moves(self.print_time)
    def get_last_move_time(self):
        self._flush_lookahead()
        if self.sync_print_time:
            self._calc_print_time()
        return self.print_time
    def reset_print_time(self, min_print_time=0.):
        self._flush_lookahead(must_sync=True)
        est_print_time = self.mcu.estimated_print_time(self.reactor.monotonic())
        self.print_time = max(min_print_time, est_print_time)
    def _check_stall(self):
        eventtime = self.reactor.monotonic()
        if self.sync_print_time:
            # Building initial queue - make sure to flush on idle input
            if self.idle_flush_print_time:
                est_print_time = self.mcu.estimated_print_time(eventtime)
                if est_print_time < self.idle_flush_print_time:
                    self.print_stall += 1
                self.idle_flush_print_time = 0.
            self.reactor.update_timer(self.flush_timer, eventtime + 0.100)
            return
        # Check if there are lots of queued moves and stall if so
        while 1:
            est_print_time = self.mcu.estimated_print_time(eventtime)
            buffer_time = self.print_time - est_print_time
            stall_time = buffer_time - self.buffer_time_high
            if stall_time <= 0.:
                break
            if self.mcu.is_fileoutput():
                self.need_check_stall = self.reactor.NEVER
                return
            eventtime = self.reactor.pause(eventtime + min(1., stall_time))
        self.need_check_stall = est_print_time + self.buffer_time_high + 0.100
    def _flush_handler(self, eventtime):
        try:
            print_time = self.print_time
            buffer_time = print_time - self.mcu.estimated_print_time(eventtime)
            if buffer_time > self.buffer_time_low:
                # Running normally - reschedule check
                return eventtime + buffer_time - self.buffer_time_low
            # Under ran low buffer mark - flush lookahead queue
            self._flush_lookahead(must_sync=True)
            if print_time != self.print_time:
                self.idle_flush_print_time = self.print_time
        except:
            self.logger.exception("Exception in flush_handler")
            self.printer.invoke_shutdown("Exception in flush_handler")
        return self.reactor.NEVER
    # Movement commands
    def get_position(self):
        return list(self.commanded_pos)
    def set_position(self, newpos, homing_axes=()):
        self._flush_lookahead()
        self.commanded_pos[:] = newpos
        self.kin.set_position(newpos, homing_axes)
    def move(self, newpos, speed, check=True):
        move = Move(self, self.commanded_pos, newpos, speed)
        if not move.move_d:
            return
        if move.is_kinematic_move and check:
            self.kin.check_move(move)
        if move.axes_d[3]:
            self.extruder.check_move(move)
        self.commanded_pos[:] = move.end_pos
        self.move_queue.add_move(move)
        if self.print_time > self.need_check_stall:
            self._check_stall()
    def dwell(self, delay, check_stall=True):
        self.get_last_move_time()
        self.update_move_time(delay)
        if check_stall:
            self._check_stall()
    def motor_heater_off(self):
        self.motor_off()
        print_time = self.get_last_move_time()
        for n, h in self.printer.lookup_objects("heater"):
            h.set_temp(print_time, 0.0)
        for n, fan in self.printer.lookup_objects('fan'):
            fan.set_speed(print_time, 0.0)
    def motor_off(self):
        self.dwell(STALL_TIME)
        last_move_time = self.get_last_move_time()
        self.kin.motor_off(last_move_time)
        for key, ext in self.printer.extruder_get().items():
            ext.motor_off(last_move_time)
        self.printer.send_event("toolhead:motor_off", last_move_time)
        self.dwell(STALL_TIME)
        self.logger.debug('; Max time of %f', last_move_time)
    def wait_moves(self):
        self._flush_lookahead()
        if self.mcu.is_fileoutput():
            return
        eventtime = self.reactor.monotonic()
        while (not self.sync_print_time
               or self.print_time >= self.mcu.estimated_print_time(eventtime)):
            eventtime = self.reactor.pause(eventtime + 0.100)
    def set_extruder(self, extruder):
        last_move_time = self.get_last_move_time()
        self.extruder.set_active(last_move_time, False)
        extrude_pos = extruder.set_active(last_move_time, True)
        self.extruder = extruder
        self.extruder.setup_accel_order(self.accel_order)
        self.move_queue.set_extruder(extruder)
        self.commanded_pos[3] = extrude_pos
    def get_extruder(self):
        return self.extruder
    # Misc commands
    def stats(self, eventtime):
        for m in self.all_mcus:
            m.check_active(self.print_time, eventtime)
        buffer_time = self.print_time - self.mcu.estimated_print_time(eventtime)
        is_active = buffer_time > -60. or not self.sync_print_time
        return is_active, "print_time=%.3f buffer_time=%.3f print_stall=%d" % (
            self.print_time, max(buffer_time, 0.), self.print_stall)
    def check_busy(self, eventtime):
        est_print_time = self.mcu.estimated_print_time(eventtime)
        lookahead_empty = not self.move_queue.queue
        return self.print_time, est_print_time, lookahead_empty
    def get_status(self, eventtime):
        print_time = self.print_time
        estimated_print_time = self.mcu.estimated_print_time(eventtime)
        last_print_start_time = self.last_print_start_time
        buffer_time = print_time - estimated_print_time
        if buffer_time > -1. or not self.sync_print_time:
            status = "Printing"
        else:
            status = "Ready"
        return { 'status': status, 'print_time': print_time,
                 'estimated_print_time': estimated_print_time,
                 'printing_time': print_time - last_print_start_time }
    def get_print_time(self):
        return self.print_time - self.last_print_start_time
    def _handle_request_restart(self, print_time):
        self.motor_off()
    def _handle_shutdown(self):
        self.move_queue.reset()
        self.reset_print_time()
    def get_kinematics(self):
        return self.kin
    def get_max_velocity(self):
        return self.max_velocity, self.max_accel
    def get_max_axis_halt(self):
        # Determine the maximum velocity a cartesian axis could halt
        # at due to the junction_deviation setting.  The 8.0 was
        # determined experimentally.
        return min(self.max_velocity,
                   math.sqrt(8. * self.junction_deviation * self.max_accel))
    def _calc_junction_deviation(self):
        scv2 = self.square_corner_velocity**2
        self.junction_deviation = scv2 * (math.sqrt(2.) - 1.) / self.max_accel
        self.max_accel_to_decel = min(self.requested_accel_to_decel,
                                      self.max_accel)
    cmd_SET_VELOCITY_LIMIT_help = "Set printer velocity limits. " \
                                  "Args: [VELOCITY=] [ACCEL=] [SQUARE_CORNER_VELOCITY=]" \
                                  " [ACCEL_TO_DECEL=]"
    def cmd_SET_VELOCITY_LIMIT(self, params):
        print_time = self.get_last_move_time()
        gcode = self.printer.lookup_object('gcode')
        max_velocity = gcode.get_float('VELOCITY', params, self.max_velocity,
                                       above=0.)
        max_accel = gcode.get_float('ACCEL', params, self.max_accel, above=0.)
        square_corner_velocity = gcode.get_float(
            'SQUARE_CORNER_VELOCITY', params, self.square_corner_velocity,
            minval=0.)
        self.requested_accel_to_decel = gcode.get_float(
            'ACCEL_TO_DECEL', params, self.requested_accel_to_decel, above=0.)
        accel_order = gcode.get_int(
            'ACCEL_ORDER', params, self.accel_order, minval=2, maxval=6)
        if accel_order != self.accel_order:
            self.accel_order = accel_order
            self.ffi_lib.move_set_accel_order(self.cmove, accel_order)
            self.extruder.setup_accel_order(accel_order)
        self.max_velocity = min(max_velocity, self.config_max_velocity)
        self.max_accel = min(max_accel, self.config_max_accel)
        self.square_corner_velocity = min(square_corner_velocity,
                                          self.config_square_corner_velocity)
        self._calc_junction_deviation()
        msg = ("max_velocity: %.6f max_accel: %.6f accel_order: %d\n"
               "max_accel_to_decel: %.6f square_corner_velocity: %.6f" % (
                   max_velocity, max_accel, accel_order,
                   self.requested_accel_to_decel, square_corner_velocity))
        self.printer.set_rollover_info("toolhead", "toolhead: %s" % (msg,))
        gcode.respond_info(msg)
    def cmd_M204(self, params):
        gcode = self.printer.lookup_object('gcode')
        if 'P' in params and 'T' in params and 'S' not in params:
            # Use minimum of P and T for accel
            accel = min(gcode.get_float('P', params, above=0.),
                        gcode.get_float('T', params, above=0.))
        else:
            # Use S for accel
            accel = gcode.get_float('S', params, above=0.)
        self.max_accel = min(accel, self.config_max_accel)
        self._calc_junction_deviation()
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, params):
        print_time = self.get_last_move_time()
        for n, h in self.printer.lookup_objects("heater"):
            h.set_temp(print_time, 0.0)


def add_printer_objects(config):
    config.get_printer().add_object('toolhead', ToolHead(config))
    kinematics.extruder.add_printer_objects(config)
