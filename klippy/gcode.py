# Parse gcode commands
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, re, collections
import homing, extruder, heater

'''
TODO:
  * tool offset: G/Mxxx P2 X17.8 Y-19.3 Z0.0
  * FW retraction: G10, G11, M207, M208
  * Volymetric Extrution
  * M290: Babystepping
  * M600: Filament change pause

           =====================
           **** REPRAP stuff ***

M141 H<heater> S<temp>   : Set Chamber Temperature - IGNORE
M563                     : Define or remove a tool
M80                      : ATX Power On
M81                      : ATX Power Off
M144                     : Bed Standby (needs stanby temperature...), M140 -> back to active temp
M1 (partial?)

'''

'''
extruders: factor_extrude -> extrude_factor
self.extrude_factor -> self.extruder.extrude_factor!


base_position['E'] -> extruder class!

'''


tx_sequenceno = 0

class error(Exception):
    pass

# Parse and handle G-Code commands
class GCodeParser:
    error = error
    RETRY_TIME = 0.100
    def __init__(self, printer, fd_link):
        self.logger = printer.logger.getChild('gcode')
        self.printer = printer
        self.fd_r = fd_link.fd_r
        self.fd_w = fd_link.fd_w
        # Input handling
        self.reactor = printer.reactor
        self.is_processing_data = False
        self.is_fileinput = not not printer.get_start_args().get("debuginput")
        self.fd_handle = None
        if not self.is_fileinput:
            self.fd_handle = self.reactor.register_fd(self.fd_r, self.process_data)
        self.partial_input = ""
        self.bytes_read = 0
        self.input_log = collections.deque([], 50)
        # Command handling
        self.is_printer_ready = False
        self.base_gcode_handlers = self.gcode_handlers = {}
        self.ready_gcode_handlers = {}
        self.gcode_help = {}
        for cmd in self.all_handlers:
            func = getattr(self, 'cmd_' + cmd)
            wnr = getattr(self, 'cmd_' + cmd + '_when_not_ready', False)
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            self.register_command(cmd, func, wnr, desc)
            for a in getattr(self, 'cmd_' + cmd + '_aliases', []):
                self.register_command(a, func, wnr)
        # G-Code coordinate manipulation
        self.absolutecoord = self.absoluteextrude = True
        self.base_position = [0.0, 0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.homing_add = [0.0, 0.0, 0.0, 0.0]
        self.speed_factor = 1. / 60.
        # G-Code state
        self.need_ack = False
        self.toolhead = None
        self.extruder = None
        self.extruders = {}
        self.speed = 25.0
        self.axis2pos = {'X': 0, 'Y': 1, 'Z': 2, 'E': 3}
        #self.tx_sequenceno = 0
        self.simulate_print = False
        self.babysteps = 0.0 # Effect to Z only
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if not (len(cmd) >= 2 and not cmd[0].isupper() and cmd[1].isdigit()):
            origfunc = func
            func = lambda params: origfunc(self.get_extended_params(params))
        self.ready_gcode_handlers[cmd] = func
        if when_not_ready:
            self.base_gcode_handlers[cmd] = func
        if desc is not None:
            self.gcode_help[cmd] = desc
    def stats(self, eventtime):
        return "gcodein=%d" % (self.bytes_read,)
    def connect(self):
        self.is_printer_ready = True
        self.gcode_handlers = self.ready_gcode_handlers
        # Lookup printer components
        self.toolhead = self.printer.objects.get('toolhead')
        self.extruders = extruder.get_printer_extruders(self.printer)
        if 0 < len(self.extruders):
            self.extruder = self.extruders[0] # extruder0
            self.toolhead.set_extruder(self.extruder)
        if self.is_fileinput and self.fd_handle is None:
            self.fd_handle = self.reactor.register_fd(self.fd_r, self.process_data)
        global tx_sequenceno
        tx_sequenceno += 1
    def reset_last_position(self):
        if self.toolhead is not None:
            self.last_position = self.toolhead.get_position()
    def do_shutdown(self):
        if not self.is_printer_ready:
            return
        self.is_printer_ready = False
        self.gcode_handlers = self.base_gcode_handlers
        self.dump_debug()
        if self.is_fileinput:
            self.printer.request_exit()
        global tx_sequenceno
        tx_sequenceno += 1
    def motor_heater_off(self):
        if self.toolhead is None:
            return
        self.toolhead.motor_off()
        print_time = self.toolhead.get_last_move_time()
        for k,h in heater.get_printer_heaters(self.printer).items():
            h.set_temp(print_time, 0.0)
        # TODO FIXME : is it ok to switch off all fans???
        #              Should switch off only fan without temp ctrl
        for fan in self.printer.get_objects_with_prefix('fan'):
            fan.set_speed(print_time, 0.0)
    def dump_debug(self):
        out = []
        out.append("Dumping gcode input %d blocks" % (
            len(self.input_log),))
        for eventtime, data in self.input_log:
            out.append("Read %f: %s" % (eventtime, repr(data)))
        extrude_factor = 0.
        if self.extruder is not None:
            extrude_factor = self.extruder.extrude_factor
        out.append(
            "gcode state: absolutecoord=%s absoluteextrude=%s"
            " base_position=%s last_position=%s homing_add=%s"
            " speed_factor=%s extrude_factor=%s speed=%s" % (
                self.absolutecoord, self.absoluteextrude,
                self.base_position, self.last_position, self.homing_add,
                self.speed_factor, extrude_factor, self.speed))
        self.logger.info("\n".join(out))
    # Parse input into commands
    args_r = re.compile('([A-Z_]+|[A-Z*])')
    def process_commands(self, commands, need_ack=True):
        prev_need_ack = self.need_ack
        for line in commands:
            # Ignore comments and leading/trailing spaces
            line = origline = line.strip()
            cpos = line.find(';')
            if cpos >= 0:
                line = line[:cpos]
            # Break command into parts
            parts = self.args_r.split(line.upper())[1:]
            params = { parts[i]: parts[i+1].strip()
                       for i in range(0, len(parts), 2) }
            params['#original'] = origline
            if parts and parts[0] == 'N':
                # Skip line number at start of command
                del parts[:2]
            if not parts:
                self.cmd_default(params)
                continue
            params['#command'] = cmd = parts[0] + parts[1].strip()
            # Invoke handler for command
            self.need_ack = need_ack
            handler = self.gcode_handlers.get(cmd, self.cmd_default)
            try:
                handler(params)
                self.ack()
            except error as e:
                self.respond_error(str(e))
                self.reset_last_position()
            except:
                msg = 'Internal error on command:"%s"' % (cmd,)
                self.logger.exception(msg)
                self.printer.invoke_shutdown(msg)
                self.respond_error(msg)
            # self.ack()
        self.need_ack = prev_need_ack
    def split_string(text, splitlist):
        for sep in splitlist:
            text = text.replace(sep, splitlist[0])
        return filter(None, text.split(splitlist[0])) if splitlist else [text]
    def process_data(self, eventtime):
        data = os.read(self.fd_r, 4096)
        self.input_log.append((eventtime, data))
        self.bytes_read += len(data)
        lines = data.split('\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        if self.is_processing_data:
            if not self.is_fileinput and not lines:
                return
            self.reactor.unregister_fd(self.fd_handle)
            self.fd_handle = None
            if not self.is_fileinput and lines[0].strip().upper() == 'M112':
                self.cmd_M112({})
            while self.is_processing_data:
                eventtime = self.reactor.pause(eventtime + 0.100)
            self.fd_handle = self.reactor.register_fd(self.fd_r, self.process_data)
        self.is_processing_data = True
        self.process_commands(lines)
        if not data and self.is_fileinput:
            self.motor_heater_off()
            if self.toolhead is not None:
                self.toolhead.wait_moves()
            self.printer.request_exit()
        self.is_processing_data = False
    # Response handling
    def __write_resp(self, msg):
        global tx_sequenceno
        tx_sequenceno += 1
        os.write(self.fd_w, msg)
    def ack(self, msg=None):
        if not self.need_ack or self.is_fileinput:
            return
        if msg:
            self.__write_resp("ok %s\n" % (msg,))
        else:
            self.__write_resp("ok\n")
        self.need_ack = False
    def respond(self, msg):
        if self.is_fileinput:
            return
        self.__write_resp(msg+"\n")
    def respond_info(self, msg):
        self.logger.debug(msg)
        lines = [l.strip() for l in msg.strip().split('\n')]
        self.respond("// " + "\n// ".join(lines))
    def respond_error(self, msg):
        self.logger.warning(msg)
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.respond_info("\n".join(lines[:-1]))
        self.respond('!! %s' % (lines[-1].strip(),))
    # Parameter parsing helpers
    class sentinel: pass
    def get_str(self, name, params, default=sentinel, parser=str):
        if name in params:
            try:
                return parser(params[name])
            except:
                raise error("Error on '%s': unable to parse %s" % (
                    params['#original'], params[name]))
        if default is not self.sentinel:
            return default
        raise error("Error on '%s': missing %s" % (params['#original'], name))
    def get_int(self, name, params, default=sentinel):
        return self.get_str(name, params, default, parser=int)
    def get_float(self, name, params, default=sentinel):
        return self.get_str(name, params, default, parser=float)
    extended_r = re.compile(
        r'^\s*(?:N[0-9]+\s*)?'
        r'(?P<cmd>[a-zA-Z_][a-zA-Z_]+)(?:\s+|$)'
        r'(?P<args>[^#*;]*?)'
        r'\s*(?:[#*;].*)?$')
    def get_extended_params(self, params):
        m = self.extended_r.match(params['#original'])
        if m is None:
            # Not an "extended" command
            return params
        eargs = m.group('args')
        try:
            eparams = [earg.split('=', 1) for earg in eargs.split()]
            eparams = { k.upper(): v for k, v in eparams }
            eparams.update({k: params[k] for k in params if k.startswith('#')})
            return eparams
        except ValueError as e:
            raise error("Malformed command '%s'" % (params['#original'],))
    # Temperature wrappers
    def get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        # todo add PWM power?
        for key,e in self.extruders.items():
            heater = e.get_heater()
            if heater is not None:
                cur, target = heater.get_temp(eventtime)
                out.append("T%d:%.1f /%.1f" % (e.get_index(), cur, target))
        heater = self.printer.objects.get('heater_bed')
        if (heater is not None):
            cur, target = heater.get_temp(eventtime)
            out.append("B:%.1f /%.1f" % (cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    def bg_temp(self, heater):
        if self.is_fileinput:
            return
        eventtime = self.reactor.monotonic()
        while self.is_printer_ready and heater.check_busy(eventtime):
            print_time = self.toolhead.get_last_move_time()
            self.respond(self.get_temp(eventtime))
            eventtime = self.reactor.pause(eventtime + 1.)
    def set_temp(self, params, is_bed=False, wait=False):
        temp = self.get_float('S', params, 0.0) if (self.simulate_print is False) else 0.0
        heater = None
        if is_bed:
            heater = self.printer.objects.get('heater_bed')
        else:
            index = None
            if 'T' in params:
                index = self.get_int('T', params)
            elif 'P' in params:
                index = self.get_int('P', params)

            if index is not None:
                e = extruder.get_printer_extruder(self.printer, index)
                if e is not None:
                    heater = e.get_heater()
            elif self.extruder is not None:
                heater = self.extruder.get_heater()

        if heater is None:
            if temp > 0.:
                self.respond_error("Heater not configured")
            return
        print_time = self.toolhead.get_last_move_time()
        try:
            heater.set_temp(print_time, temp)
        except heater.error as e:
            raise error(str(e))
        if wait and temp and self.simulate_print is False:
            self.bg_temp(heater)
    def set_fan_speed(self, speed, index):
        fan = self.printer.objects.get('fan%d'%(index))
        if fan is None:
            if speed and not self.is_fileinput:
                self.respond_info("Fan not configured")
            return
        print_time = self.toolhead.get_last_move_time()
        fan.set_speed(print_time, speed)

    # G-Code special command handlers
    def cmd_default(self, params):
        if not self.is_printer_ready:
            self.respond_error(self.printer.get_state_message())
            return
        cmd = params.get('#command')
        if not cmd:
            self.logger.debug(params['#original'])
            return
        if cmd[0] == 'T' and len(cmd) > 1 and cmd[1].isdigit():
            # Tn command has to be handled specially
            self.cmd_Tn(params)
            return
        self.respond_info('Unknown command:"%s"' % (cmd,))

    def cmd_Tn(self, params):
        # Select Tool
        index = self.get_int('T', params)
        if index < 0:
            # Reprap WebGui uses T-1 in some cases, skip it
            return
        e = extruder.get_printer_extruder(self.printer, index)
        if e is None:
            self.respond_error("Extruder %d not configured" % (index,))
            return
        if self.extruder is e:
            return
        deactivate_gcode = self.extruder.get_activate_gcode(False)
        self.process_commands(deactivate_gcode.split('\n'), need_ack=False)
        try:
            self.toolhead.set_extruder(e)
        except homing.EndstopError as e:
            raise error(str(e))
        self.extruder = e
        self.reset_last_position()
        '''
        Reset extruder base position if G92 E0 is not called after tool change
        or some movement is done in activate code set by user.
        '''
        self.base_position[3] = self.last_position[3]
        activate_gcode = self.extruder.get_activate_gcode(True)
        self.process_commands(activate_gcode.split('\n'), need_ack=False)

    def _parse_movement(self, params, is_arch=False):
        # parse move command

        # Save coodinate system
        absolutecoord = self.absolutecoord
        if is_arch:
            self.absolutecoord = False; # change to relative

        try:
            for axis in 'XYZ':
                if axis in params:
                    v = float(params[axis])
                    pos = self.axis2pos[axis]
                    if not self.absolutecoord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]
            if 'E' in params:
                v = ( (float(params['E']) * self.extruder.extrude_factor) if (self.simulate_print is False) else 0.0 )
                if not self.absolutecoord or not self.absoluteextrude: # TODO: Only extruder????
                    # value relative to position of last move
                    self.last_position[3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3] = v + self.base_position[3]
            if 'F' in params:
                speed = float(params['F']) * self.speed_factor
                if speed <= 0.:
                    raise error("Invalid speed in '%s'" % (params['#original'],))
                self.speed = speed
            if is_arch:
                self.arch_I = 0.0
                self.arch_J = 0.0
                self.arch_R = None
                self.arch_circles = 0
                if 'R' in params:
                    self.arch_R = float(params['R'])
                if 'I' in params:
                    self.arch_I = float(params['I'])
                if 'J' in params:
                    self.arch_J = float(params['J'])
                if 'P' in params:
                    self.arch_circles = int(params['P'])
                    if self.arch_circles <= 0:
                        raise ValueError()
        except ValueError as e:
            raise error("Unable to parse move '%s'" % (params['#original'],))

        self.absolutecoord = absolutecoord

    def _apply_move(self, pos, speed):
        try:
            self.toolhead.move(pos, speed)
        except homing.EndstopError as e:
            raise error(str(e))

    def _plan_arc(self, position, arc_offset, clockwise):
        N_ARC_CORRECTION = 25

        p_axis = self.axis2pos['X']
        q_axis = self.axis2pos['Y']
        l_axis = self.axis2pos['Z']
        e_axis = self.axis2pos['E']

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        radius          = float(math.hypot(r_P, r_Q))
        center_P        = float(self.base_position[p_axis] - r_P)
        center_Q        = float(self.base_position[q_axis] - r_Q)
        rt_X            = float(position[p_axis] - center_P)
        rt_Y            = float(position[q_axis] - center_Q)
        linear_travel   = float(position[l_axis] - self.base_position[l_axis])
        extruder_travel = float(position[e_axis] - self.base_position[e_axis])

        # CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
        angular_travel = math.atan2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y)
        if (angular_travel < 0):
            angular_travel += math.radians(360)
        if (clockwise):
            angular_travel -= math.radians(360)

        # Make a circle if the angular rotation is 0 and the target is current position
        if (angular_travel == 0 and
            self.base_position[p_axis] == logical[p_axis] and
            self.base_position[q_axis] == logical[q_axis]):
            angular_travel = math.radians(360);

        mm_of_travel = math.hypot(angular_travel * radius, math.fabs(linear_travel))
        if (mm_of_travel < 0.001):
            return;

        segments = int(math.floor(mm_of_travel / (MM_PER_ARC_SEGMENT)))
        if (segments <= 0):
            segments = 1

        '''
        * Vector rotation by transformation matrix: r is the original vector, r_T is the rotated vector,
        * and phi is the angle of rotation. Based on the solution approach by Jens Geisler.
        *     r_T = [cos(phi) -sin(phi);
        *            sin(phi)  cos(phi)] * r ;
        *
        * For arc generation, the center of the circle is the axis of rotation and the radius vector is
        * defined from the circle center to the initial position. Each line segment is formed by successive
        * vector rotations. This requires only two cos() and sin() computations to form the rotation
        * matrix for the duration of the entire arc. Error may accumulate from numerical round-off, since
        * all double numbers are single precision on the Arduino. (True double precision will not have
        * round off issues for CNC applications.) Single precision error can accumulate to be greater than
        * tool precision in some cases. Therefore, arc path correction is implemented.
        *
        * Small angle approximation may be used to reduce computation overhead further. This approximation
        * holds for everything, but very small circles and large MM_PER_ARC_SEGMENT values. In other words,
        * theta_per_segment would need to be greater than 0.1 rad and N_ARC_CORRECTION would need to be large
        * to cause an appreciable drift error. N_ARC_CORRECTION~=25 is more than small enough to correct for
        * numerical drift error. N_ARC_CORRECTION may be on the order a hundred(s) before error becomes an
        * issue for CNC machines with the single precision Arduino calculations.
        *
        * This approximation also allows plan_arc to immediately insert a line segment into the planner
        * without the initial overhead of computing cos() or sin(). By the time the arc needs to be applied
        * a correction, the planner should have caught up to the lag caused by the initial plan_arc overhead.
        * This is important when there are successive arc motions.
        '''
        # Vector rotation matrix values
        arc_target = len(self.axis2pos) * [0]
        theta_per_segment    = float(angular_travel / segments)
        linear_per_segment   = float(linear_travel / segments)
        extruder_per_segment = float(extruder_travel / segments)
        sin_T                = theta_per_segment
        cos_T                = float(1 - 0.5 * SQ(theta_per_segment)) # Small angle approximation

        # Initialize the linear axis
        arc_target[l_axis] = self.base_position[l_axis];

        # Initialize the extruder axis
        arc_target[e_axis] = self.base_position[e_axis];

        fr_mm_s = self.speed

        if N_ARC_CORRECTION > 1:
            count = N_ARC_CORRECTION

        # Iterate (segments-1) times
        for i in range(1, segments):

            count = count - 1
            if (count and N_ARC_CORRECTION > 1):
                # Apply vector rotation matrix to previous r_P / 1
                r_new_Y = r_P * sin_T + r_Q * cos_T;
                r_P = r_P * cos_T - r_Q * sin_T;
                r_Q = r_new_Y;
            else:
                if N_ARC_CORRECTION > 1:
                    count = N_ARC_CORRECTION

                # Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments.
                # Compute exact location by applying transformation matrix from initial radius vector(=-offset).
                # To reduce stuttering, the sin and cos could be computed at different times.
                # For now, compute both at the same time.
                cos_Ti = cos(i * theta_per_segment)
                sin_Ti = sin(i * theta_per_segment)
                r_P    = -offset[0] * cos_Ti + offset[1] * sin_Ti
                r_Q    = -offset[0] * sin_Ti - offset[1] * cos_Ti

            # Update arc_target location
            arc_target[p_axis] = center_P + r_P
            arc_target[q_axis] = center_Q + r_Q
            arc_target[l_axis] = arc_target[l_axis] + linear_per_segment
            arc_target[e_axis] = arc_target[e_axis] + extruder_per_segment

            #clamp_to_software_endstops(arc_target);
            self._apply_move(arc_target, self.speed)

        # Ensure last segment arrives at target location.
        self._apply_move(position, self.speed)

        # As far as the parser is concerned, the position is now == target. In reality the
        # motion control system might still be processing the action and the real tool position
        # in any intermediate location.
        self.last_position = position

    def _calculate_arch(self, params, clockwise):
        self._parse_movement(params, True)
        arc_offset = [ self.arch_I, self.arch_J ]
        if self.arch_R is not None:
            r  = self.arch_R
            p1 = self.base_position[self.axis2pos['X']]
            q1 = self.base_position[self.axis2pos['Y']]
            p2 = self.last_position[self.axis2pos['X']]
            q2 = self.last_position[self.axis2pos['Y']]
            if (r and (p2 != p1 or q2 != q1)):
                # clockwise -1/1, counterclockwise 1/-1
                if clockwise ^ (r < 0):
                    e = -1
                else:
                    e = 1
                dx = p2 - p1                           # X differences
                dy = q2 - q1                           # Y differences
                d  = math.hypot(dx, dy)                # Linear distance between the points
                h  = math.sqrt(math.pow(r,2) -
                               math.pow((d * 0.5),2))  # Distance to the arc pivot-point
                mx = (p1 + p2) * 0.5                   # Point between the two points
                my = (q1 + q2) * 0.5                   # Point between the two points
                sx = -dy / d                           # Slope of the perpendicular bisector
                sy = dx / d                            # Slope of the perpendicular bisector
                cx = mx + e * h * sx                   # Pivot-point of the arc
                cy = my + e * h * sy                   # Pivot-point of the arc
                arc_offset[0] = cx - p1
                arc_offset[1] = cy - q1
        if arc_offset[0] == 0 or arc_offset[1] == 0:
            raise error("Unable to parse move '%s'" % (params['#original'],))

        # Number of circles to do
        if self.arch_circles is not None:
            for idx in range[0, self.arch_circles]:
                self._plan_arc(self.base_position, arc_offset, clockwise)
        # Arch itself
        self._plan_arc(self.last_position, arc_offset, clockwise)


    all_handlers = [
        'G1', 'G2', 'G3', 'G4', 'G10', 'G11', 'G20', 'G28', 'G29',
        'G90', 'G91', 'G92',
        'M0', 'M1', 'M18', 'M37', 'M82', 'M83',
        'M104', 'M105', 'M106', 'M107', 'M109',
        'M112', 'M114', 'M115', 'M118',
        'M140', 'M190',
        'M206', 'M220', 'M221', 'M290',
        'M302',
        'M400',
        'M550',
        'M851',
        'M900', 'M906',
        'IGNORE', 'QUERY_ENDSTOPS', 'PID_TUNE',
        'RESTART', 'FIRMWARE_RESTART', 'ECHO', 'STATUS', 'HELP']

    # Basic movement
    cmd_G1_aliases = ['G0']
    def cmd_G1(self, params):
        # Move
        self._parse_movement(params)
        self._apply_move(self.last_position, self.speed)

    # todo Arch support
    def cmd_G2(self, params):
        # G2 Xnnn Ynnn Innn Jnnn Ennn Fnnn (Clockwise Arc)
        self._calculate_arch(params, 1)
    def cmd_G3(self, params):
        # G3 Xnnn Ynnn Innn Jnnn Ennn Fnnn (Counter-Clockwise Arc)
        self._calculate_arch(params, 0)

    def cmd_G4(self, params):
        # Dwell
        if 'S' in params:
            delay = self.get_float('S', params)
        else:
            delay = self.get_float('P', params, 0.0) / 1000.0
        self.toolhead.dwell(delay)

    # todo firmware retraction support
    def cmd_G10(self, params):
        # G10: Retract
        short = 0
        if 'S' in params:
            short = self.get_int('S', params, 0)
    def cmd_G11(self, params):
        # G11: Unretract
        short = 0
        if 'S' in params:
            short = self.get_int('S', params, 0)

    def cmd_G20(self, params):
        # Set units to inches
        self.respond_error('Machine does not support G20 (inches) command')

    # Homing
    def cmd_G28(self, params):
        # Move to origin
        axes = []
        homing_order = self.toolhead.homing_order
        for axis in homing_order:
            if axis in params:
                axes.append(self.axis2pos[axis])
        if not axes:
            axes = [self.axis2pos[axis] for axis in homing_order]
        homing_state = homing.Homing(self.toolhead)
        if self.is_fileinput:
            homing_state.set_no_verify_retract()
        try:
            homing_state.home_axes(axes)
        except homing.EndstopError as e:
            raise error(str(e))
        '''
        newpos = self.toolhead.get_position()
        for axis in homing_state.get_axes():
            self.last_position[axis] = newpos[axis]
            self.base_position[axis] = -self.homing_add[axis]
        '''
        # Reset current position
        self.reset_last_position()
        for axis in homing_state.get_axes():
            self.base_position[axis] = -self.homing_add[axis]

    def cmd_G29(self, params):
        self.respond_info("Bed levelling is not supported yet!")
        #self.cmd_default(params)
        pass

    # G-Code coordinate manipulation
    def cmd_G90(self, params):
        # Use absolute coordinates
        self.absolutecoord = True
    def cmd_G91(self, params):
        # Use relative coordinates
        self.absolutecoord = False

    def cmd_G92(self, params):
        # Set position
        offsets = { p: self.get_float(a, params)
                    for a, p in self.axis2pos.items() if a in params }
        for p, offset in offsets.items():
            if p == 3:
                offset *= self.extruder.extrude_factor
            self.base_position[p] = self.last_position[p] - offset
        if not offsets:
            self.base_position = list(self.last_position)

    def cmd_M0(self, params):
        heaters_on = self.get_int('H', params, 0)
        if (heaters_on is 0):
            self.motor_heater_off()
        elif self.toolhead is not None:
            self.toolhead.motor_off()
            # self.respond("Printer reset invoked")
            # self.printer.request_exit('firmware_restart')
    def cmd_M1(self, params):
        # Wait for current moves to finish
        self.toolhead.wait_moves()
        self.motor_heater_off()
        # self.respond("Printer restart invoked")
        # self.printer.request_exit('restart')

    def cmd_M37(self, params):
        simulation_enabled = self.get_int('P', params, 0)
        if simulation_enabled is 1:
            self.simulate_print = True
        else:
            self.simulate_print = False

    def cmd_M82(self, params):
        # Use absolute distances for extrusion
        self.absoluteextrude = True
    def cmd_M83(self, params):
        # Use relative distances for extrusion
        self.absoluteextrude = False

    cmd_M18_aliases = ["M84"]
    def cmd_M18(self, params):
        # Turn off motors
        # Should wait queued moves to prevent unwanted error!
        self.toolhead.wait_moves()
        self.toolhead.motor_off()
    def cmd_M104(self, params):
        # Set Extruder Temperature
        self.set_temp(params)
    cmd_M105_when_not_ready = True
    def cmd_M105(self, params):
        # Get Extruder Temperature
        self.ack(self.get_temp(self.reactor.monotonic()))
    def cmd_M106(self, params):
        # Set fan speed
        self.set_fan_speed(self.get_float('S', params, 255.0) / 255.0,
                           self.get_int('P', params, 0))
    def cmd_M107(self, params):
        # Turn fan off
        self.set_fan_speed(0.0, self.get_int('P', params, 0))
    def cmd_M109(self, params):
        # Set Extruder Temperature and Wait
        self.set_temp(params, wait=True)
    # G-Code miscellaneous commands
    cmd_M112_when_not_ready = True
    def cmd_M112(self, params):
        # Emergency Stop
        self.printer.invoke_shutdown("Shutdown due to M112 command")
    cmd_M114_when_not_ready = True
    def cmd_M114(self, params):
        # Get Current Position
        if self.toolhead is None:
            self.cmd_default(params)
            return
        raw_pos = homing.query_position(self.toolhead)
        self.respond("X:%.3f Y:%.3f Z:%.3f E:%.3f Count %s" % (
            self.last_position[0], self.last_position[1],
            self.last_position[2], self.last_position[3],
            " ".join(["%s:%d" % (n.upper(), p) for n, p in raw_pos])))
    cmd_M115_when_not_ready = True
    def cmd_M115(self, params):
        # Get Firmware Version and Capabilities
        software_version = self.printer.get_start_args().get('software_version')
        kw = {"FIRMWARE_NAME": "Klipper", "FIRMWARE_VERSION": software_version}
        self.ack(" ".join(["%s:%s" % (k, v) for k, v in kw.items()]))

    def cmd_M118(self, params):
        self.respond_info(params['#original'].replace(params['#command'], ""))
    def cmd_M140(self, params):
        # Set Bed Temperature
        self.set_temp(params, is_bed=True)
    def cmd_M190(self, params):
        # Set Bed Temperature and Wait
        self.set_temp(params, is_bed=True, wait=True)
    def cmd_M206(self, params):
        # Set home offset
        offsets = { self.axis2pos[a]: self.get_float(a, params)
                    for a in 'XYZ' if a in params }
        for p, offset in offsets.items():
            self.base_position[p] += self.homing_add[p] - offset
            self.homing_add[p] = offset

    def cmd_M220(self, params):
        # M220: Set speed factor override percentage
        value = self.get_float('S', params, 100.) / (60. * 100.)
        if value <= 0.:
            raise error("Invalid factor in '%s'" % (params['#original'],))
        self.speed_factor = value
    def cmd_M221(self, params):
        # M221: Set extrude factor override percentage
        new_extrude_factor = self.get_float('S', params, 100.) / 100.
        if new_extrude_factor <= 0.:
            raise error("Invalid factor in '%s'" % (params['#original'],))
        index = extr = None
        # extruder number
        if 'D' in params:
            index = self.get_int('D', params)
        elif 'T' in params:
            index = self.get_int('T', params)
        if index is None:
            extr = self.extruder
        else:
            extr = extruder.get_printer_extruder(self.printer, index)
        if extr is not None:
            last_e_pos = self.last_position[3]
            e_value = (last_e_pos - self.base_position[3]) / extr.extrude_factor
            self.base_position[3] = last_e_pos - e_value * new_extrude_factor
            extr.extrude_factor = new_extrude_factor

    def cmd_M290(self, params):
        # Babystepping
        if 'S' in params:
            babysteps_to_apply = self.get_float('S', params)
            if (self.absolutecoord):
                self.base_position[self.axis2pos['Z']] += babysteps_to_apply
            else:
                self.last_position[self.axis2pos['Z']] += babysteps_to_apply
            self.babysteps += babysteps_to_apply

        elif 'R' in params: # Reset
            if (self.absolutecoord):
                self.base_position[self.axis2pos['Z']] -= self.babysteps
            else:
                self.last_position[self.axis2pos['Z']] -= self.babysteps
            self.babysteps = 0.0

        else:
            self.respond_info("Baby stepping offset is %.3fmm" % (self.babysteps,))

    def cmd_M302(self, params):
        # Allow cold extrusion
        #       M302         ; report current cold extrusion state
        #       M302 P0      ; enable cold extrusion checking
        #       M302 P1      ; disables cold extrusion checking
        #       M302 S0      ; always allow extrusion (disables checking)
        #       M302 S170    ; only allow extrusion above 170
        #       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
        disable = None
        temperature = None
        if 'P' in params:
            disable = self.get_int('P', params, 0) == 1
        if 'S' in params:
            temperature = self.get_int('S', params, -1)
        for k,h in heater.get_printer_heaters(self.printer).items():
            if not h.is_bed:
                h.set_min_extrude_temp(temperature, disable)
                status, temp = h.get_min_extrude_status()
                self.respond_info(
                    "Heater '{}' cold extrude: {}, min temp {}C".
                    format(h.name, status, temp))

    def cmd_M301(self, params):
        # TODO: M301: Set PID parameters
        pass
    def cmd_M304(self, params):
        # TODO: M304: Set PID parameters - Bed
        pass

    def cmd_M400(self, params):
        # Wait for current moves to finish
        self.toolhead.wait_moves()

    def cmd_M550(self, params):
        if 'P' in params:
            self.printer.name = params['P']
        self.logger.info("My name is now {}".format(self.printer.name))

    def cmd_M851(self, params):
        # Set X, Y, Z offsets
        offsets = { a.lower(): self.get_float(a, params)
                    for a, p in self.axis2pos.items() if a in params }
        if len(offsets) > 0:
            if self.toolhead is None:
                self.cmd_default(params)
                return
            self.toolhead.set_homing_offset(offsets)
        else:
            self.respond_info("Current offsets: X=%.2f Y=%.2f Z=%.2f" % \
                              (self.toolhead.kin.steppers[0].homing_offset,
                               self.toolhead.kin.steppers[1].homing_offset,
                               self.toolhead.kin.steppers[2].homing_offset))

    def cmd_M900(self, params):
        # driver status if exists
        for stepper in self.toolhead.kin.steppers:
            if hasattr(stepper.driver, 'print_status'):
                stepper.driver.print_status()
    def cmd_M906(self, params):
        # TMC current
        if 'X' in params:
            if hasattr(self.toolhead.kin.steppers[0].driver, 'set_current'):
                stepper.driver.set_current(self.get_float('X', params))
        if 'Y' in params:
            if hasattr(self.toolhead.kin.steppers[1].driver, 'set_current'):
                stepper.driver.set_current(self.get_float('Y', params))
        if 'Z' in params:
            if hasattr(self.toolhead.kin.steppers[2].driver, 'set_current'):
                stepper.driver.set_current(self.get_float('Z', params))



    cmd_IGNORE_when_not_ready = True
    cmd_IGNORE_aliases = [
        "G21",
        "M21",
        "M110", 'M120', 'M121', 'M122', "M141",
        'M291', 'M292',
        'M752', 'M753', 'M754', 'M755', 'M756',
        'M997'
    ]
    def cmd_IGNORE(self, params):
        # Commands that are just silently accepted
        pass
    cmd_QUERY_ENDSTOPS_help = "Report on the status of each endstop"
    cmd_QUERY_ENDSTOPS_aliases = ["M119"]
    def cmd_QUERY_ENDSTOPS(self, params):
        # Get Endstop Status
        res = homing.query_endstops(self.toolhead)
        self.respond(" ".join(["%s:%s" % (name, ["open", "TRIGGERED"][not not t])
                               for name, t in res]))
    cmd_PID_TUNE_help = "Run PID Tuning"
    cmd_PID_TUNE_aliases = ["M303"]
    def cmd_PID_TUNE(self, params):
        # Run PID tuning (M303 E<-1 or 0...> S<temp> C<count>)
        heater = None;
        heater_index = self.get_int('E', params, 0)
        if (heater_index == -1):
            heater = self.printer.objects.get('heater_bed')
        else:
            e = extruder.get_printer_extruder(self.printer, heater_index)
            if e is not None:
                heater = e.get_heater()
        if heater is None:
            self.respond_error("Heater is not configured")
        else:
            temp = self.get_float('S', params)
            #count = self.get_int('C', params, 12, 8)
            heater.start_auto_tune(temp)
            self.bg_temp(heater)
    def prep_restart(self):
        if self.is_printer_ready:
            self.respond_info("Preparing to restart...")
            self.motor_heater_off()
            self.toolhead.dwell(0.500)
            self.toolhead.wait_moves()
    cmd_RESTART_when_not_ready = True
    cmd_RESTART_help = "Reload config file and restart host software"
    def cmd_RESTART(self, params):
        self.prep_restart()
        self.printer.request_exit('restart')
    cmd_FIRMWARE_RESTART_when_not_ready = True
    cmd_FIRMWARE_RESTART_help = "Restart firmware, host, and reload config"
    cmd_FIRMWARE_RESTART_aliases = ["M999"]
    def cmd_FIRMWARE_RESTART(self, params):
        self.prep_restart()
        self.printer.request_exit('firmware_restart')
    cmd_ECHO_when_not_ready = True
    def cmd_ECHO(self, params):
        self.respond_info(params['#original'])
    cmd_STATUS_when_not_ready = True
    cmd_STATUS_help = "Report the printer status"
    def cmd_STATUS(self, params):
        msg = self.printer.get_state_message()
        if self.is_printer_ready:
            self.respond_info(msg)
        else:
            self.respond_error(msg)
    cmd_HELP_when_not_ready = True
    def cmd_HELP(self, params):
        cmdhelp = []
        if not self.is_printer_ready:
            cmdhelp.append("Printer is not ready - not all commands available.")
        cmdhelp.append("Available extended commands:")
        for cmd in sorted(self.gcode_handlers):
            if cmd in self.gcode_help:
                cmdhelp.append("%-10s: %s" % (cmd, self.gcode_help[cmd]))
        self.respond_info("\n".join(cmdhelp))
