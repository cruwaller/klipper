
class BabySteps(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        for cmd in ['M290']:
            gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_%s_help' % cmd, None))
        self.z_axis_pos = gcode.axis2pos['Z']
        self.babysteps = 0.
        self.logger = gcode.logger
        self.logger.info("BabySteps initialized")
        printer.add_object("babysteps", self)

    cmd_M290_help = "Babystepping. Args: [S<offset>] | [R]"
    def cmd_M290(self, params):
        gcode = self.gcode
        absolute_coord = gcode.absolute_coord
        base_position = gcode.base_position
        last_position = gcode.last_position
        # Babystepping
        if 'S' in params:
            babysteps_to_apply = gcode.get_float('S', params)
            if absolute_coord:
                base_position[self.z_axis_pos] += babysteps_to_apply
            else:
                last_position[self.z_axis_pos] += babysteps_to_apply
            self.babysteps += babysteps_to_apply
        elif 'R' in params: # Reset
            if absolute_coord:
                base_position[self.z_axis_pos] -= self.babysteps
            else:
                last_position[self.z_axis_pos] -= self.babysteps
            self.babysteps = 0.0
        self.gcode.respond("Baby stepping offset is %.3fmm" % (
            self.babysteps,))
        gcode.base_position = base_position
        gcode.last_position = last_position
