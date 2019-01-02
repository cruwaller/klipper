
class HostGpioEvent(object):
    def __init__(self, config):
        name = config.get_name().split()[1].strip().replace(" ", "_")
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        hostpins = printer.try_load_module(config, 'hostpins')
        self.logger = hostpins.get_logger(name)
        # Setup pin
        self.pin = hostpins.setup_pin("gpio_event", config.get("pin"))
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
