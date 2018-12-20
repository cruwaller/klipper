
class HostGpioPwm(object):
    def __init__(self, config):
        name = config.get_name().split()[1].strip().replace(" ", "_")
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        hostcpu = printer.lookup_object('hostcpu')
        self.logger = hostcpu.get_logger(name)
        # Setup pin
        self.pin = hostcpu.setup_pin("pwm", config.get("pin"))
        self.max_power = config.getfloat(
            "max_power", default=1., minval=.0, maxval=1.)
        self.min_power = config.getfloat(
            "min_power", default=.0, minval=.0, maxval=self.max_power)
        initial_duty = config.getfloat(
            'duty', default=.0, minval=self.min_power,
            maxval=self.max_power)
        self.pin.write(initial_duty)
        initial_freq = config.getfloat('freq', default=1, minval=1)
        self.pin.set_freq(initial_freq)
        gcode.register_mux_command(
            "HOST_PWM", "NAME", name,
            self.cmd_read,
            desc="Set PWM duty. Args NAME= [DUTY=] [FREQ=]",
            when_not_ready=True)
        self.logger.info("%s : duty %s, freq %s, min %s, max %s" % (
            name, initial_duty, initial_freq,
            self.min_power, self.max_power))
    def cmd_read(self, params):
        duty = self.gcode.get_float('DUTY', params,
            default=None, minval=.0, maxval=1.0)
        freq = self.gcode.get_float('FREQ', params,
            default=None, minval=1)
        if freq is not None:
            self.pin.set_freq(freq)
        if duty is not None:
            self.pin.write(max(self.min_power, min(duty, self.max_power)))
        self.gcode.respond_info("Duty: %s, freq: %s" % (
            self.pin.get_duty(), self.pin.get_freq()))


def load_config_prefix(config):
    return HostGpioPwm(config)
