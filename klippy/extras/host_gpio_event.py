
class HostGpioEvent(object):
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        # Setup pin
        pin_params = printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True, can_pullup=True)
        self.pin = pin_params['chip'].setup_pin(
            "digital_event", pin_params)
        options = {'falling' : 'falling', 'rising' : 'rising',
                   'both' : 'both'}
        self.pin.set_event(self._event_callback,
                           config.getchoice("edge", options))
        options = {'kill' : 'shutdown', 'reboot' : 'firmware_restart'}
        self.action = config.getchoice('action', options)
    def _event_callback(self, channel):
        self.printer.request_exit(self.action)


def load_config_prefix(config):
    return HostGpioEvent(config)
