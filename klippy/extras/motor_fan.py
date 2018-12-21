import fan

PIN_MIN_TIME = 0.100

class MotorFan:
    _power = .0
    def __init__(self, config):
        self.printer = config.get_printer()
        self.fan = fan.PrinterFan(config, default_shutdown_speed=1.)
        self.mcu = self.fan.mcu_fan.get_mcu()
        self.fan_speed = config.getfloat("fan_speed", 1., minval=0., maxval=1.)
        self.reactor = self.printer.get_reactor()
        self.set_timer = self.reactor.register_timer(
            self.reactor_callback)
        self.logger = self.fan.logger = \
            self.printer.logger.getChild(
                self.fan.name.replace(" ", "_"))
        self.printer.register_event_handler('motor_state', self.event_handler)
    def event_handler(self, state):
        if state == 'off':
            self._power = 0.
        else:
            self._power = self.fan_speed
        self.reactor.update_timer(self.set_timer,
                                  self.reactor.NOW)
    def reactor_callback(self, eventtime):
        print_time = self.mcu.estimated_print_time(eventtime) + PIN_MIN_TIME
        self.fan.set_speed(print_time, self._power)
        return self.reactor.NEVER

def load_config(config):
    return MotorFan(config)
