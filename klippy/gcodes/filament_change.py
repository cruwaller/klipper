import re

class GCodeFilamentPause(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.logger = gcode.logger.getChild("filament_change")
        for cmd in ['M600', 'FILAMENT_CHANGE']:
            gcode.register_command(
                cmd, self.cmd_FILAMENT_CHANGE,
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        self.script_unload = config.get("commands_unload", default="")
        self.script_load = config.get("commands_load", default="")
        # Speeds
        self.travel_speed = config.getfloat(
            "travel_speed", 6000., above=0.) # 100mm/s
        self.retract_speed = config.getfloat(
            "retract_speed", 3000., above=0.) # 50mm/s
        self.load_speed = config.getfloat(
            "load_speed", 180., above=0.) # 3mm/s
        if not self.script_unload:
            # Positions
            self.pos_x = config.getfloat("x_position", .0)
            self.pos_y = config.getfloat("y_position", .0)
            self.z_lift = config.getfloat("z_lift", .0)
            self.retract_before_move = config.getboolean(
                "retract_before_move", default=False)
            self.retract_len = .0
            if self.retract_before_move:
                self.retract_len = config.getfloat("retract_len", above=0)
            self.len_load = config.getfloat("load_len")
            self.len_unload = config.getfloat("unload_len", default=self.len_load)
            self.logger.info("Position: X%s Y%s, Z lift: %s, retract %s, load_len: %s" % (
                self.pos_x, self.pos_y, self.z_lift, self.retract_len, self.len_load))
        else:
            self.logger.info("Unload: '%s'" % self.script_unload)
            self.logger.info("Load: '%s'" % self.script_load)
        self.resume_print_original = None
        self.last_position = gcode.last_position
        self.absolutecoord = gcode.absolutecoord

    cmd_M600_help = "Alias for FILAMENT_CHANGE"
    cmd_FILAMENT_CHANGE_help = "Args: [E<pos>], [L<pos>], [X<pos>], [Y<pos>], [Z<pos>]"
    def cmd_FILAMENT_CHANGE(self, params):
        """
        M600 : Filament Change

        [E<pos>] Retract before moving to change position
        [L<pos>] Load/unload length, longer for bowden
        [X<pos>] X position for filament change
        [Y<pos>] Y position for filament change
        [Z<pos>] Z relative lift for filament change position
        """
        gcode = self.printer.lookup_object('gcode')
        get_float = gcode.get_float
        run_script = gcode.run_script_from_command
        # Pause print first
        run_script('M25 P0\n')
        self.printer.lookup_object('toolhead').wait_moves()
        # Store coordinate system
        self.absolutecoord = gcode.absolutecoord
        # Store last head position
        self.last_position = gcode.last_position
        # Start unload procedure
        if self.script_unload:
            run_script(self.script_unload)
        else:
            gcode.absolutecoord = True
            retract_len = get_float('E', params, self.retract_len, minval=0.)
            self.len_unload = unload_len = get_float(
                'L', params, self.len_load, minval=0.)
            pos_x = get_float('X', params, self.pos_x, minval=0.)
            pos_y = get_float('Y', params, self.pos_y, minval=0.)
            z_lift = get_float('Z', params, self.z_lift, minval=0.)
            if retract_len:
                run_script('G92 E0\nG1 E-%s F%u' % (
                    retract_len, int(self.retract_speed)))
            if z_lift:
                gcode.absolutecoord = False
                run_script('G1 Z%s F600' % z_lift)
                gcode.absolutecoord = True
            move = " ".join(["G1", "F%u" % int(self.travel_speed),
                            'Y%f' % pos_y, 'X%f' % pos_x])
            run_script(move)
            run_script('G92 E0\nG1 E-%s F%u' % (
                unload_len, int(self.load_speed)))
        # Store original SD resume for continue
        self.resume_print_original = gcode.get_command_handler('M24')
        gcode.register_command('M24', self.cmd_FILAMENT_CHANGE_READY)
        gcode.respond_info("Please load new filament and resume")

    def cmd_FILAMENT_CHANGE_READY(self, params):
        gcode = self.printer.lookup_object('gcode')
        if self.script_load:
            gcode.run_script_from_command(self.script_load)
        else:
            # Load filament
            gcode.run_script_from_command('G92 E0\nG1 E%s F%u' % (
                self.len_unload, int(self.load_speed)))
            # Move head back to original position
            move = " ".join(["G1", "F%u" % int(self.travel_speed),
                            'Y%f' % self.last_position[1],
                             'X%f' % self.last_position[0]])
            gcode.run_script_from_command(move)
            gcode.run_script_from_command('G1 Z%f F400' % self.last_position[2])
        # restore coordinate system
        gcode.absolutecoord = self.absolutecoord
        # Restore original SD resume
        gcode.register_command('M24', self.resume_print_original)
        gcode.respond_info("Filament change over")

def load_config(config):
    GCodeFilamentPause(config)
