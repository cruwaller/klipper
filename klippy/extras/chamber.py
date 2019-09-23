# This file may be distributed under the terms of the GNU GPLv3 license.

"""
[chamber]
target_temp:    20.
pin:            ar11
control:        watermark
# sensor params
sensor_type:    NTC 100K beta 3950
sensor_pin:     analog10
min_temp:       0
max_temp:       120
# heater params
heater_pin:     ar6
index: 1

# heater and fan by names
[chamber]
target_temp:    150.
pin:            ar11
# chamber with heater by name
heater:         heater_name

# no heater, just fan
[chamber]
target_temp:    20.
pin:            ar11
sensor:         sensor_name
"""

import temperature_fan

class Chamber:
    def __init__(self, config):
        # Support chamber with heater and/or fan
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.logger = self.printer.get_logger(self.name)
        self.fan = self.heater = None
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        pheater = self.printer.lookup_object('heater')
        heater = config.get('heater', None)
        if heater is not None:
            # setup chamber heater if configured
            self.heater = pheater.setup_heater(config.getsection(heater))
        elif config.get('heater_pin', None) is not None:
            self.heater = pheater.setup_heater(config)
        self.sensor = self.heater.sensor if self.heater else None
        if config.get('pin', None) is not None:
            self.fan = temperature_fan.TemperatureFan(config, self.sensor)
            self.sensor = self.fan.sensor
        if self.sensor is None:
            self.sensor = pheater.setup_sensor(config)
        self.sensor.setup_callback(self.temperature_callback)
        self.min_temp, self.max_temp = self.sensor.get_min_max_temp()
        self.target_temp = config.getfloat('target_temp', 0.,
            minval=self.min_temp, maxval=self.max_temp)
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("M141", self.cmd_M141)
        self.logger.debug("Chamber created")
    def temperature_callback(self, read_time, temp):
        if self.fan:
            self.fan.temperature_callback(read_time, temp)
        if self.heater:
            self.heater.temperature_callback(read_time, temp)
    def _heater_set_temp(self, target_temp):
        if not self.heater or not target_temp:
            return
        print_time = self.printer.lookup_object(
            'toolhead').get_last_move_time()
        self.heater.set_temp(print_time, target_temp)
    def _fan_set_temp(self, target_temp):
        if self.fan:
            self.fan.set_temp(target_temp)
    def _handle_ready(self):
        if self.heater:
            self._heater_set_temp(self.target_temp)
        self._fan_set_temp(self.target_temp)
    def cmd_M141(self, params):
        gcode = self.printer.lookup_object('gcode')
        target_temp = gcode.get_float("S", params, default=self.target_temp)
        self._heater_set_temp(target_temp)
        self._fan_set_temp(target_temp)
        self.target_temp = target_temp
    def get_temp(self, *args):
        eventtime = self.printer.get_reactor().monotonic()
        if self.heater:
            return self.heater.get_temp(eventtime)
        return self.fan.get_temp(eventtime)
    def is_fan_active(self):
        if self.fan is None:
            return False
        return 0. < self.fan.last_speed_value
    def stats(self, eventtime):
        out = []
        if self.fan is not None:
            out.append(self.fan.stats(eventtime)[1])
        if self.heater:
            out.append(self.heater.stats(eventtime)[1])
        return False, " ".join(out)


def load_config(config):
    return Chamber(config)
