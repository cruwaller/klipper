#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class HostGpioEvent(object):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.v_sd = self.gcode_cmd = None
        options = {'halt': 'halt', 'gcode': 'gcode',
            'kill': 'shutdown', 'restart': 'firmware_restart'}
        self.action = config.getchoice('action', options, 'gcode')
        if self.action == 'gcode':
            gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
            self.gcode_cmd = gcode_macro.load_template(config, 'gcode')
        # Setup pin
        pin_params = self.printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True, can_pullup=True)
        self.pin = pin_params['chip'].setup_pin(
            "digital_event", pin_params)
        options = {'falling' : 'falling', 'rising' : 'rising',
                   'both' : 'both'}
        self.pin.set_event(self._event_callback,
                           config.getchoice("edge", options))
    def _event_callback(self, channel):
        self.printer.get_logger().exception(
            'Host event happened! Request exit: %s' % self.action)
        if self.action == 'gcode':
            self._run_gcode()
            return
        if self.action == 'halt':
            self.printer.invoke_shutdown("Shutdown due to host gpio event")
            return
        self.printer.request_exit(self.action)
    def _handle_ready(self):
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
    def _run_gcode(self):
        if self.v_sd is not None and self.v_sd.is_active():
            # Printing from virtual sd, run pause command
            self.v_sd.do_pause()
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command(self.gcode_cmd.render())


def load_config_prefix(config):
    return HostGpioEvent(config)
