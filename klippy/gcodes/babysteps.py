
class BabySteps(object):
    def __init__(self, printer):
        self.printer = printer
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['M290']:
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_%s_help' % cmd, None))
        self.z_axis_pos = self.gcode.axis2pos['Z']
        self.babysteps = 0.
        self.logger = self.gcode.logger
        self.logger.info("BabySteps initialized")
        printer.add_object("babysteps", self)

    cmd_M290_help = "Babystepping. Args: S to move or R to reset"
    def cmd_M290(self, params):
        absolutecoord = self.gcode.absolutecoord
        base_position = self.gcode.base_position
        last_position = self.gcode.last_position
        # Babystepping
        if 'S' in params:
            babysteps_to_apply = self.gcode.get_float('S', params)
            if absolutecoord:
                base_position[self.z_axis_pos] += babysteps_to_apply
            else:
                last_position[self.z_axis_pos] += babysteps_to_apply
            self.babysteps += babysteps_to_apply
        elif 'R' in params: # Reset
            if absolutecoord:
                base_position[self.z_axis_pos] -= self.babysteps
            else:
                last_position[self.z_axis_pos] -= self.babysteps
            self.babysteps = 0.0
        self.gcode.respond("Baby stepping offset is %.3fmm" % (self.babysteps,))
        self.gcode.base_position = base_position
        self.gcode.last_position = last_position

def load_gcode(printer):
    BabySteps(printer)
