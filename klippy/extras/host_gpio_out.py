
class HostGpioOut(object):
    def __init__(self, config):
        self.name = name = config.get_name().split()[1].strip().replace(" ", "_")
        printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        # Setup pin
        pin_params = printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True)
        self.pin = pin_params['chip'].setup_pin(
            "digital_out", pin_params)
        # Register gcode command
        gcode.register_mux_command(
            "HOST_GPIO_WRITE", "NAME", name,
            self.cmd_WRITE,
            desc="Set pin value. args: NAME= VALUE=",
            when_not_ready=True)
    def cmd_WRITE(self, params):
        value = bool(self.gcode.get_int(
            'VALUE', params, minval=0, maxval=1))
        self.pin.set_digital(0, value)
        self.gcode.respond_info("Pin %s set to %s" % (
            self.name, value))


def load_config_prefix(config):
    return HostGpioOut(config)
