
# TODO FIXME : Change relative to last position for Z hop

class GCodeRetract(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['G10', 'G11', 'M207', 'M208']:
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd),
                                        desc=desc)
        self.respond_dbg = self.gcode.respond_info
        self.respond_info = self.gcode.respond
        # get configs
        self.z_hop = config.getfloat(
            'z_hop', default=0., minval=0.)
        # normal retract
        self.retract_dist = config.getfloat(
            'retract_dist', default=0., minval=0.)
        self.retract_speed = config.getfloat(
            'retract_speed', default=0., minval=0.)
        self.return_dist = config.getfloat(
            'recover_dist_addition', default=0.)
        self.return_speed = config.getfloat(
            'recover_speed', default=self.retract_speed, minval=0.)
        # short retract
        self.retract_dist_short = config.getfloat(
            'retract_dist_short', default=0., minval=0.)
        self.return_dist_short = config.getfloat(
            'recover_dist_addition_short', default=0.)

        self.logger = self.gcode.logger
        self.logger.info("Firmware retract initialized")

    def cmd_G10(self, params):
        # G10: Retract
        if not self.retract_speed:
            return
        short = self.gcode.get_int('S', params, 0)
        if self.retract_dist and short == 0:
            self.gcode.run_script(
                "G1 E%f F%u" % (self.retract_dist, self.retract_speed))
            self.respond_dbg("retract")
        elif self.retract_dist_short:
            self.gcode.run_script(
                "G1 E%f F%u" % (self.retract_dist_short, self.retract_speed))
            self.respond_dbg("short retract")

    def cmd_G11(self, params):
        # G11: Unretract
        if not self.return_speed:
            return
        short = self.gcode.get_int('S', params, 0)
        if self.retract_dist and short == 0:
            self.gcode.run_script(
                "G1 E-%f F%u" % ((self.retract_dist + self.return_dist),
                                 self.return_speed))
            self.respond_dbg("retract recover")
        elif self.retract_dist_short:
            self.gcode.run_script(
                "G1 E-%f F%u" % ((self.retract_dist_short + self.return_dist_short),
                                 self.return_speed))
            self.respond_dbg("short retract recover")

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
            self.return_dist = self.gcode.get_float(
                'R', params, default=0.)
        if "T" in params:
            self.return_speed = self.gcode.get_int(
                'T', params, default=self.retract_speed)
        self.respond_info("FW Retract: speed %s length %s z_hop %s" %
                          (self.retract_speed, self.retract_dist, self.z_hop))

    def cmd_M208(self, params):
        # M208 F<feedrate> S<length>
        self.return_dist = self.gcode.get_float(
            'S', params, default=0.)
        self.return_speed = self.gcode.get_int(
            'F', params, default=self.retract_speed)
        self.respond_info("FW Retract recover: speed %s addition %s" %
                          (self.return_speed, self.return_dist))

def load_config(config):
    GCodeRetract(config)
