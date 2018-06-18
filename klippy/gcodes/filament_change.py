
class GCodeFilamentPause(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.logger = gcode.logger.getChild("filament_change")
        for cmd in ['M600', 'FILAMENT_CHANGE']:
            gcode.register_command(
                cmd, self.cmd_FILAMENT_CHANGE,
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        self.respond = self.gcode.respond_info
        self.script = config.get("commands")
        self.retract_speed = config.getfloat(
            "retract_speed", 1200., above=0.)
        self.load_speed = config.getfloat(
            "load_speed", 1200., above=0.)
        self.logger.info("Script: %s" % self.script)

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
        use_script = True
        get_float = self.gcode.get_float
        run_script = self.gcode.run_script
        move = {}
        retract_len = get_float('E', params, None, minval=0.)
        load_len = get_float('L', params, None, minval=0.)
        pos_x = get_float('X', params, None, minval=0.)
        pos_y = get_float('Y', params, None, minval=0.)
        pos_z = get_float('Z', params, None, minval=0.)
        # Pause print first
        run_script('M25 P0\n')
        if retract_len is not None:
            use_script = False
            run_script('G92 E0\nG1 E-%s F%u' % (
                retract_len, int(self.retract_speed)))
        if pos_z:
            use_script = False
            if self.gcode.absolutecoord:
                run_script('G91\nG1 Z%s F400\nG90' % pos_z)
            else:
                run_script('G1 Z%s F400' % pos_z)
        if pos_x is not None:
            move['X'] = pos_x
        if pos_y is not None:
            move['Y'] = pos_y
        if move:
            use_script = False
            self.gcode.cmd_G1(move)
        if load_len is not None:
            use_script = False
            run_script('G92 E0\nG1 E-%s F%u' % (
                load_len, int(self.load_speed)))
        if use_script:
            run_script(self.script)


def load_config(config):
    GCodeFilamentPause(config)
