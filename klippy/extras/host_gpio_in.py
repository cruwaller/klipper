
class HostGpioIn(object):
    def __init__(self, config):
        self.name = name = config.get_name().split()[1]
        printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        # Setup pin
        pin_params = printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True, can_pullup=True)
        self.pin = pin_params['chip'].setup_pin(
            "digital_in", pin_params)
        # Register gcode command
        gcode.register_mux_command(
            "HOST_GPIO_READ", "NAME", name,
            self.cmd_read,
            desc="Read defined pin value. Args NAME=",
            when_not_ready=True)
    def cmd_read(self, params):
        self.gcode.respond_info("Pin %s value %s" % (
            self.name, self.pin.get_digital()))


def load_config_prefix(config):
    return HostGpioIn(config)
