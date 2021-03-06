#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class HostGpioPwm(object):
    def __init__(self, config):
        self.name = name = config.get_name().split()[1]
        printer = config.get_printer()
        # Setup pin
        pin_params = printer.lookup_object('pins').lookup_pin(
            config.get('pin'), can_invert=True)
        self.pin = pin_params['chip'].setup_pin("pwm", pin_params)
        initial_freq = config.getfloat('freq', default=1, minval=1)
        self.pin.set_freq(initial_freq)
        self.min_power = config.getfloat(
            "min_power", default=.0, minval=.0, maxval=1.)
        self.max_power = config.getfloat(
            "max_power", default=1., minval=self.min_power, maxval=1.)
        initial_duty = config.getfloat(
            'duty', default=self.min_power, minval=self.min_power,
            maxval=self.max_power)
        self.pin.set_pwm(initial_duty)
        # Register gcode command
        self.gcode = gcode = printer.lookup_object('gcode')
        gcode.register_mux_command(
            "HOST_PWM", "NAME", name, self.cmd_PWM,
            desc="Set PWM duty. Args NAME= [DUTY=] [FREQ=]",
            when_not_ready=True)
    def cmd_PWM(self, params):
        duty = self.gcode.get_float('DUTY', params,
            default=None, minval=.0, maxval=1.0)
        freq = self.gcode.get_float('FREQ', params,
            default=None, minval=1)
        if freq is not None:
            self.pin.set_freq(freq)
        if duty is not None:
            self.pin.set_pwm(0, max(self.min_power, min(duty, self.max_power)))
        self.gcode.respond_info("PWM '%s': duty: %s, freq: %s" % (
            self.name, self.pin.get_duty(), self.pin.get_freq()))


def load_config_prefix(config):
    return HostGpioPwm(config)
