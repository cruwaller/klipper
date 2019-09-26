#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class HostGpioEvent(object):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode_return = self.gcode_triggered = None
        self.change_state = lambda x: 0
        options = {'halt': 'halt', 'gcode': 'gcode',
            'kill': 'shutdown', 'restart': 'firmware_restart'}
        self.action = config.getchoice('action', options, 'gcode')
        options = {'falling' : 'falling', 'rising' : 'rising',
                   'both' : 'both'}
        edge = config.getchoice("edge", options)
        if self.action == 'gcode':
            gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
            self.gcode_triggered = gcode_macro.load_template(config, 'gcode')
            if edge == 'both':
                self.change_state = lambda x: x ^ 1
                self.gcode_return = gcode_macro.load_template(
                    config, 'gcode_return')
        self.state = 0
        # Setup pin
        pin_params = self.printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True, can_pullup=True)
        self.pin = pin_params['chip'].setup_pin("digital_event", pin_params)
        self.pin.set_event(self._event_callback, edge)
    def _event_callback(self, _):
        state = self.change_state(self.state)
        self.printer.get_logger().warning(
            'Host event happened! Run action: %s' % self.action)
        if self.action == 'gcode':
            self._run_gcode(state)
        elif self.action == 'halt':
            self.printer.invoke_shutdown("Shutdown due to host gpio event")
        else:
            self.printer.request_exit(self.action)
        self.state = state
    def _run_gcode(self, state):
        gcode = self.printer.lookup_object('gcode')
        if state:
            gcode.run_script_from_command(self.gcode_triggered.render())
        elif self.gcode_return:
            gcode.run_script_from_command(self.gcode_return.render())


def load_config_prefix(config):
    return HostGpioEvent(config)
