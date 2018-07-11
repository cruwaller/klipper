
class HostGpioOut(object):
    def __init__(self, config):
        name = config.get_name().split()[1].strip().replace(" ", "_")
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        hostcpu = printer.lookup_object('hostcpu')
        self.logger = hostcpu.get_logger(name)
        # Setup pin
        self.pin = hostcpu.setup_pin("gpio_out", config.get("pin"))
        gcode.register_mux_command(
            "HOST_GPIO_WRITE", "NAME", name,
            self.cmd_write,
            desc="Set pin value. args: NAME= VALUE=",
            when_not_ready=True)
        self.logger.info("%s loaded" % name)
    def cmd_write(self, params):
        value = self.gcode.get_float('VALUE', params)
        self.pin.write(value)
        params['#input'].respond_info("Pin set ok")


def load_config_prefix(config):
    return HostGpioOut(config)
