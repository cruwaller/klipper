#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class AtxPower(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.state = False
        # Setup pin
        pin_params = printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True)
        self.pin = pin_params['chip'].setup_pin('digital_out', pin_params)
        # Register gcode commands
        self.gcode = gcode = printer.lookup_object('gcode')
        for cmd in ['ATX_ON', 'ATX_OFF']:
            func = getattr(self, 'cmd_' + cmd)
            desc = getattr(self, 'cmd_%s_help' % cmd, None)
            gcode.register_command(cmd, func, True, desc)
            for a in getattr(self, 'cmd_' + cmd + '_aliases', []):
                gcode.register_command(a, func, True)
    def get_state(self):
        return self.state
    def stats(self, eventtime):
        return False, "atx_state=%d" % (self.state,)
    cmd_ATX_ON_aliases = ['M80']
    cmd_ATX_ON_help = "ATX Power On"
    def cmd_ATX_ON(self, params):
        self.pin.set_digital(0, True)
        self.gcode.respond_info("ATX ON")
        self.state = True
    cmd_ATX_OFF_aliases = ['M81']
    cmd_ATX_OFF_help = "ATX Power Off"
    def cmd_ATX_OFF(self, params):
        self.pin.set_digital(0, False)
        self.gcode.respond_info("ATX OFF")
        self.state = False

def load_config(config):
    return AtxPower(config)
