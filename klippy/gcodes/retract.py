

class GCodeRetract(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['G10', 'G11', 'M207', 'M208']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        self.retract_dist = self.retract_speed = 0.
        self.return_dist = self.return_speed = 0.
        self.logger = self.gcode.logger
        self.logger.info("Firmware retract initialized")

    def cmd_G10(self, params):
        # G10: Retract
        short = self.gcode.get_int('S', params, 0)
        if self.retract_dist and self.retract_speed:
            self.gcode.process_commands(
                "G1 E%f F%u" % (self.retract_dist, self.retract_speed),
                need_ack=False)
            self.gcode.respond_info("retract")

    def cmd_G11(self, params):
        # G11: Unretract
        short = self.gcode.get_int('S', params, 0)
        if self.return_dist and self.return_speed:
            self.gcode.process_commands(
                "G1 E%f F%u" % (self.return_dist, self.return_speed),
                need_ack=False)
            self.gcode.respond_info("retract return")

    def cmd_M207(self, params):
        pass
    def cmd_M208(self, params):
        pass

def load_config(config):
    GCodeRetract(config)
