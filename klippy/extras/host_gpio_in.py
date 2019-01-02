
class HostGpioIn(object):
    def __init__(self, config):
        name = config.get_name().split()[1].strip().replace(" ", "_")
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        hostpins = printer.try_load_module(config, 'hostpins')
        self.logger = hostpins.get_logger(name)
        # Setup pin
        self.pin = hostpins.setup_pin("gpio_in", config.get("pin"))
        gcode.register_mux_command(
            "HOST_GPIO_READ", "NAME", name,
            self.cmd_read,
            desc="Read defined pin value. Args NAME=",
            when_not_ready=True)
    def cmd_read(self, params):
        self.gcode.respond_info(
            "Current pin value %s" % self.pin.read())


def load_config_prefix(config):
    return HostGpioIn(config)
