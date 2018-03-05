import math
import gcode, homing

# These calculations are copied from Marlin FW (http://marlinfw.org/)

class GCodeArch(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.logger
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['G2', 'G3']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        self.axis2pos = self.gcode.axis2pos
        self.gcode.logger.info("GCodeArch support loaded")

    def cmd_G2(self, params):
        # G2 Xnnn Ynnn Innn Jnnn Ennn Fnnn (Clockwise Arc)
        self._calculate_arch(params, 1)

    def cmd_G3(self, params):
        # G3 Xnnn Ynnn Innn Jnnn Ennn Fnnn (Counter-Clockwise Arc)
        self._calculate_arch(params, 0)

    def _parse_movement(self, params):
        last_position = self.gcode.last_position

        try:
            for axis in 'XYZ':
                if axis in params:
                    v = float(params[axis])
                    pos = self.axis2pos[axis]
                    # value relative to position of last move
                    last_position[pos] += v
            if 'E' in params:
                v = ( (float(params['E']) * self.gcode.extruder.extrude_factor)
                      if (self.gcode.simulate_print is False) else 0. )
                # value relative to position of last move
                last_position[3] += v
            if 'F' in params:
                speed = float(params['F']) * self.gcode.speed_factor
                if speed <= 0.:
                    raise error("Invalid speed in '%s'" % (params['#original'],))
                self.speed = speed

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
            return last_position
        except ValueError as e:
            raise gcode.error("Unable to parse move '%s'" % (params['#original'],))

    def _apply_move(self, pos, speed):
        try:
            self.gcode.move_with_transform(pos, speed)
        except homing.EndstopError as e:
            raise gcode.error(str(e))

    def _plan_arc(self, position, arc_offset, clockwise):
        base_position = position
        N_ARC_CORRECTION = 25

        p_axis = self.axis2pos['X']
        q_axis = self.axis2pos['Y']
        l_axis = self.axis2pos['Z']
        e_axis = self.axis2pos['E']

        # Radius vector from center to current location
        r_P = -offset[0]
        r_Q = -offset[1]

        radius          = float(math.hypot(r_P, r_Q))
        center_P        = float(base_position[p_axis] - r_P)
        center_Q        = float(base_position[q_axis] - r_Q)
        rt_X            = float(position[p_axis] - center_P)
        rt_Y            = float(position[q_axis] - center_Q)
        linear_travel   = float(position[l_axis] - base_position[l_axis])
        extruder_travel = float(position[e_axis] - base_position[e_axis])

        # CCW angle of rotation between position and target from the circle center. Only one atan2() trig computation required.
        angular_travel = math.atan2(r_P * rt_Y - r_Q * rt_X, r_P * rt_X + r_Q * rt_Y)
        if (angular_travel < 0):
            angular_travel += math.radians(360)
        if (clockwise):
            angular_travel -= math.radians(360)

        # Make a circle if the angular rotation is 0 and the target is current position
        if (angular_travel == 0 and
            base_position[p_axis] == logical[p_axis] and
            base_position[q_axis] == logical[q_axis]):
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
        arc_target[l_axis] = base_position[l_axis];

        # Initialize the extruder axis
        arc_target[e_axis] = base_position[e_axis];

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
        self.gcode.last_position = position
        self.gcode.speed = self.speed

    def _calculate_arch(self, params, clockwise):
        last_position = self._parse_movement(params, True)
        base_position = self.gcode.base_position
        arc_offset = [ self.arch_I, self.arch_J ]
        if self.arch_R is not None:
            r  = self.arch_R
            p1 = base_position[self.axis2pos['X']]
            q1 = base_position[self.axis2pos['Y']]
            p2 = last_position[self.axis2pos['X']]
            q2 = last_position[self.axis2pos['Y']]
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
                self._plan_arc(base_position, arc_offset, clockwise)
        # Arch itself
        self._plan_arc(last_position, arc_offset, clockwise)

def load_config(config):
    GCodeArch(config)
