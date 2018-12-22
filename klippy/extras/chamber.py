
import temperature_fan

class Chamber:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.fan = temperature_fan.TemperatureFan(
            config, chamber=True)
        self.logger = self.fan.logger
        self.gcode.register_command(
            "M141", self.cmd_M141)
    def cmd_M141(self, params):
        if self.fan.fan is None:
            # No fan is configured, cannot change target
            return
        target_temp = self.gcode.get_float(
            "S", params, default=self.fan.target_temp,
            minval=self.fan.min_temp, maxval=self.fan.max_temp)
        self.fan.target_temp = target_temp
        self.logger.debug("New chamber target temperature is %s" %
                          target_temp)
    def get_temp(self, *args):
        return self.fan.last_temp, self.fan.target_temp
    def is_fan_active(self):
        return 0. < self.fan.last_speed_value
    def stats(self, eventtime):
        fan = self.fan
        return False, '%s: temp=%.1f fan_speed=%.3f' % (
            fan.name, fan.last_temp, fan.last_speed_value)


def load_config(config):
    return Chamber(config)
