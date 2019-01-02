
class HostGpioOut(object):
    def __init__(self, config):
        name = config.get_name().split()[1].strip().replace(" ", "_")
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        hostpins = printer.try_load_module(config, 'hostpins')
        self.logger = hostpins.get_logger(name)
        # Setup pin
        self.pin = hostpins.setup_pin("gpio_out", config.get("pin"))
        gcode.register_mux_command(
            "HOST_GPIO_WRITE", "NAME", name,
            self.cmd_write,
            desc="Set pin value. args: NAME= VALUE=",
            when_not_ready=True)
    def cmd_write(self, params):
        value = self.gcode.get_float('VALUE', params)
        self.pin.write(value)
        self.gcode.respond_info("Pin set ok")


def load_config_prefix(config):
    return HostGpioOut(config)
