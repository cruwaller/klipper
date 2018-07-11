
# TODO FIXME : Change relative to last position for Z hop

class GCodeRetract(object):
    retract_s = retract_short_s = None
    recover_s = recover_short_s = None
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['G10', 'G11', 'M207', 'M208']:
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_' + cmd + '_help', None))
        # get configs
        self.z_hop = config.getfloat(
            'z_hop', default=0., minval=0.)
        # normal retract
        self.retract_dist = config.getfloat(
            'retract_dist', default=0., minval=0.)
        self.retract_speed = config.getfloat(
            'retract_speed', default=0., minval=0.)
        self.return_extra = config.getfloat(
            'recover_dist_addition', default=0.)
        self.return_speed = config.getfloat(
            'recover_speed', default=self.retract_speed, above=0.)
        # short retract
        self.retract_dist_short = config.getfloat(
            'retract_dist_short', default=0., minval=0.)
        self.return_extra_short = config.getfloat(
            'recover_dist_addition_short', default=0.)
        self._calc_commands()
        self.logger = self.gcode.logger
        self.logger.info("Firmware retract initialized")

    def _calc_commands(self):
        if self.retract_speed:
            if self.retract_dist:
                self.retract_s = "G92 E0\nG1 E-%f F%u" % (
                    self.retract_dist, self.retract_speed)
                self.recover_s = "G92 E0\nG1 E%f F%u" % (
                        (self.retract_dist + self.return_extra),
                        self.return_speed)
            if self.retract_dist_short:
                self.retract_short_s = "G92 E0\nG1 E-%f F%u" % (
                    self.retract_dist_short, self.retract_speed)
                self.recover_short_s = "G92 E0\nG1 E%f F%u" % (
                    (self.retract_dist_short + self.return_extra_short),
                    self.return_speed)

    def cmd_G10(self, params):
        # G10: Retract
        short = self.gcode.get_int('S', params, 0)
        script = self.retract_s
        if short:
            script = self.retract_short_s
        if script:
            self.gcode.run_script_from_command(script)

    def cmd_G11(self, params):
        # G11: Unretract
        short = self.gcode.get_int('S', params, 0)
        script = self.recover_s
        if short:
            script = self.recover_short_s
        if script:
            self.gcode.run_script_from_command(script)

    cmd_M207_help = "Set firmware retraction. Args: [F<feedrate>] [S<length>] [Z<hight>]"
    def cmd_M207(self, params):
        # M207 F<feedrate> S<length> Z<hight>
        self.retract_dist = self.gcode.get_float(
            'S', params, default=self.retract_dist)
        self.retract_speed = self.gcode.get_int(
            'F', params, default=self.retract_speed)
        self.z_hop = self.gcode.get_float(
            'Z', params, default=self.z_hop)
        # reprap support
        if "R" in params:
            self.return_extra = self.gcode.get_float(
                'R', params, default=0.)
        if "T" in params:
            self.return_speed = self.gcode.get_int(
                'T', params, default=self.retract_speed)
        self._calc_commands()
        params['#input'].respond_info("FW Retract: speed %s length %s z_hop %s" %
            (self.retract_speed, self.retract_dist, self.z_hop))

    cmd_M208_help = "Set fw retraction return. Args: [F<feedrate>] [S<length>]"
    def cmd_M208(self, params):
        # M208 F<feedrate> S<length>
        self.return_extra = self.gcode.get_float(
            'S', params, default=0.)
        self.return_speed = self.gcode.get_int(
            'F', params, default=self.retract_speed)
        self._calc_commands()
        params['#input'].respond_info("FW Retract recover: speed %s addition %s" %
            (self.return_speed, self.return_extra))

def load_config(config):
    GCodeRetract(config)
