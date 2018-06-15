
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
        self.script = filter(
            None, [ cmd.strip() for cmd in
                    config.get("commands").split('\n') ])
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
        process_commands = self.gcode.process_commands
        move = {}
        retract_len = get_float('E', params, None, minval=0.)
        load_len = get_float('L', params, None, minval=0.)
        pos_x = get_float('X', params, None, minval=0.)
        pos_y = get_float('Y', params, None, minval=0.)
        pos_z = get_float('Z', params, None, minval=0.)
        # Pause print first
        process_commands(['M25 P0'], need_ack=False)
        if retract_len is not None:
            use_script = False
            process_commands(
                ['G92 E0', 'G1 E-%s F1200' % retract_len, 'G92 E0'],
                need_ack=False)
        if pos_z:
            use_script = False
            process_commands(
                ['G91', 'G1 Z%s F400' % pos_z, 'G90'],
                need_ack=False)
        if pos_x is not None:
            move['X'] = pos_x
        if pos_y is not None:
            move['Y'] = pos_y
        if move:
            use_script = False
            self.gcode.cmd_G1(move)
        if load_len is not None:
            use_script = False
            process_commands(
                ['G92 E0', 'G1 E-%s F1200' % load_len, 'G92 E0'],
                need_ack=False)
        if use_script:
            process_commands(self.script, need_ack=False)


def load_config(config):
    GCodeFilamentPause(config)
