# This file may be distributed under the terms of the GNU GPLv3 license.

class BabySteps(object):
    def __init__(self, config):
        self.babysteps = 0.
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M290", self.cmd_M290, desc=self.cmd_M290_help)

    cmd_M290_help = "Babystepping. Args: [S<offset>] | [R]"
    def cmd_M290(self, params):
        gcode = self.printer.lookup_object('gcode')
        z_axis_pos = gcode.axis2pos['Z']
        absolute_coord = gcode.absolute_coord
        base_position = gcode.base_position
        last_position = gcode.last_position
        # Babystepping
        if 'S' in params:
            babysteps_to_apply = gcode.get_float('S', params)
            if absolute_coord:
                base_position[z_axis_pos] += babysteps_to_apply
            else:
                last_position[z_axis_pos] += babysteps_to_apply
            self.babysteps += babysteps_to_apply
        elif 'R' in params: # Reset
            if absolute_coord:
                base_position[z_axis_pos] -= self.babysteps
            else:
                last_position[z_axis_pos] -= self.babysteps
            self.babysteps = 0.0
        gcode.base_position = base_position
        gcode.last_position = last_position
        gcode.respond("Baby stepping offset is %.3fmm" % (self.babysteps,))


def load_config(config):
    return BabySteps(config)
