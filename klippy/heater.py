# Printer heater support
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, threading


######################################################################
# Heater
######################################################################

KELVIN_TO_CELCIUS = -273.15
MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.

DEBUG_TARGET = 25.

class error(Exception):
    pass

class HeaterProtect:
    def __init__(self, config, heater):
        self.printer = printer = heater.printer
        self.heater = heater
        self.gcode = gcode = heater.printer.lookup_object('gcode')
        #register events
        printer.register_event_handler("klippy:shutdown",
                                       self._handle_shutdown)
        printer.register_event_handler("klippy:disconnect",
                                       self._handle_shutdown)
        printer.register_event_handler("klippy:ready",
                                       self._handle_ready)
        # heat check timer
        self.protection_period_heat = \
            config.getfloat('protect_period_heat', 10.0, above=0.0, maxval=120.0)
        self.protection_hysteresis_heat = \
            config.getfloat('protect_hysteresis_heat', 4.0, above=0.50)
        self.protection_period = \
            config.getfloat('protect_period', 10.0, above=0.0, maxval=120.0)
        self.protect_hyst_runaway = \
            config.getfloat('protect_hysteresis_runaway', 4.0, above=0.0)
        self.protect_hyst_cooling = \
            config.getfloat('protect_hysteresis_cooling', 1.0, above=0.0)
        self.protect_hyst_idle = \
            config.getfloat('protect_hysteresis_idle', 1.0, above=0.0, maxval=5.0)
        self.reactor = printer.reactor
        self.protection_timer = self.reactor.register_timer(self._check_heating)
        self.protection_last_temp = 9999.
        self.protect_runaway_disabled = False
        for _name in [heater.get_name(), str(heater.get_index())]:
            gcode.register_mux_command("TEMP_PROTECTION", "HEATER", _name,
                                       self.cmd_TEMP_PROTECTION,
                                       desc=self.cmd_TEMP_PROTECTION_help)
    def set_runaway(self, disabled=False):
        self.protect_runaway_disabled = disabled
    def start(self):
        self.protect_state = None
        self.reactor.update_timer(self.protection_timer,
                                  self.reactor.NOW)
    def stop(self):
        self.reactor.update_timer(self.protection_timer,
                                  self.reactor.NEVER)
    cmd_TEMP_PROTECTION_help = "agrs: HEATER= [HYSTERESIS_IDLE=] [HYSTERESIS_COOLING=] " \
                               "[PERIOD_HEAT=] [HYSTERESIS_HEAT=] " \
                               "[PERIOD_RUNAWAY=] [HYSTERESIS_RUNAWAY=]"
    def cmd_TEMP_PROTECTION(self, params):
        # Stop timer during the param update
        self.reactor.update_timer(self.protection_timer,
                                  self.reactor.NEVER)
        # Heating
        self.protection_period_heat = self.gcode.get_float(
            'PERIOD_HEAT', params, self.protection_period_heat,
            above=0.0, maxval=120.0)
        self.protection_hysteresis_heat = self.gcode.get_float(
            'HYSTERESIS_HEAT', params, self.protection_hysteresis_heat,
            above=0.5)
        # Maintain
        self.protection_period = self.gcode.get_float(
            'PERIOD_RUNAWAY', params, self.protection_period,
            above=0.0, maxval=120.0)
        self.protect_hyst_runaway = self.gcode.get_float(
            'HYSTERESIS_RUNAWAY', params, self.protect_hyst_runaway,
            above=0.0)
        # Cooling
        self.protect_hyst_cooling = self.gcode.get_float(
            'HYSTERESIS_COOLING', params, self.protect_hyst_cooling,
            above=0.0)
        # Idle
        self.protect_hyst_idle = self.gcode.get_float(
            'HYSTERESIS_IDLE', params, self.protect_hyst_idle,
            above=0.0, maxval=60.)
        # Star timer again
        self.start()
        self.gcode.respond_info(
            "Idle hysteresis %.2fC, Cooling hysteresis %.2fC\n"
            "Heating: period %.2fs, hysteresis %.2fC\n"
            "Runaway: period %.2fs, hysteresis %.2fC" % (
                self.protect_hyst_idle, self.protect_hyst_cooling,
                self.protection_period_heat, self.protection_hysteresis_heat,
                self.protection_period, self.protect_hyst_runaway))
    def _handle_shutdown(self):
        self.reactor.update_timer(self.protection_timer,
                                  self.reactor.NEVER)
    def _handle_ready(self):
        if not self.heater.sensor.get_mcu().is_shutdown() and \
           not self.heater.mcu_pwm.get_mcu().is_shutdown():
            self.start()
    protect_state = None
    def _check_heating(self, eventtime):
        next_time = 15.0  # next 15sec from now for idle
        with self.heater.lock:
            current_temp = self.heater.last_temp
            target_temp = self.heater.target_temp
        # initiate protection
        if self.protect_state is None or self.protection_last_temp == 0:
            if target_temp == 0:
                self.protect_state = 'idle'
            elif current_temp <= (target_temp - self.protect_hyst_runaway):
                next_time = self.protection_period_heat
                self.protect_state = 'heating'
            elif current_temp > target_temp > 0:
                next_time = self.protection_period
                self.protect_state = 'cooling'
            else:
                next_time = self.protection_period
                self.protect_state = 'maintain'
        elif self.protect_state == 'maintain':
            # Check hysteresis during maintain
            if self.protect_hyst_runaway < abs(current_temp - target_temp) and \
                    not self.protect_runaway_disabled:
                self.__protect_error(
                    "Thermal runaway! current temp %s, last %s" %
                    (current_temp, self.protection_last_temp))
            next_time = self.protection_period
        elif self.protect_state == 'idle':
            #  check that temp is not increased over the limit
            if target_temp:
                if current_temp > target_temp:
                    self.protect_state = "cooling"
                else:
                    self.protect_state = "heating"
                next_time = .5
            elif (self.protection_last_temp + self.protect_hyst_idle) < \
                    current_temp:
                self.__protect_error(
                    "Idle error! temperatures: current %s last %s - idle hysteresis %s" %
                    (current_temp, self.protection_last_temp, self.protect_hyst_idle))
        elif self.protect_state == 'cooling':
            next_time = self.protection_period_heat
            # Heater off
            if target_temp == 0:
                self.protect_state = 'idle'
            # Check if cooling period is over
            elif (current_temp - self.protect_hyst_runaway) < target_temp:
                self.protect_state = 'heating'
            # Check that heater is cooling
            elif (self.protection_last_temp -
                  self.protect_hyst_cooling) < current_temp:
                self.__protect_error(
                    "Cooling error! current temp %s, last %s" %
                    (current_temp, self.protection_last_temp))
        elif self.protect_state == 'heating':
            next_time = self.protection_period_heat
            # Check hysteresis during the heating
            if ((target_temp - self.protect_hyst_runaway)
                    <= current_temp <=
                    (target_temp + self.protect_hyst_runaway)):
                # Heating over, start maintaining protect
                self.protect_state = 'maintain'
            elif current_temp < target_temp:
                if abs(current_temp - self.protection_last_temp) < self.protection_hysteresis_heat:
                    self.__protect_error(
                        "Heating error! current temp %s, last %s" %
                        (current_temp, self.protection_last_temp))
        self.protection_last_temp = current_temp
        #self.logger.debug("Protection at %s: state %s, temperatures %.2f / %.2f" %
        #                  (eventtime, self.protect_state, current_temp, target_temp))
        return eventtime + next_time
    def __protect_error(self, errorstr):
        self.heater.logger.error(errorstr)
        self.heater.set_temp(0, 0)
        self.printer.request_exit('shutdown')
        # self.printer.invoke_shutdown(msg + HINT_THERMAL)

class Heater:
    error = error
    def get_name(self):
        return self.name
    def get_index(self):
        return self.index
    def __init__(self, config, sensor, index=None):
        self.printer = printer = config.get_printer()
        printer.register_event_handler("gcode:request_restart", self._turn_off)
        self.gcode = printer.lookup_object('gcode')
        name = config.get_name()
        self.name = name.replace(' ', '_')
        self.index = index = config.getint("index", index)
        if "bed" in name:
            self.index = -1 # bed is always -1
        elif index is None:
            try:
                if 'extruder' in name:
                    self.index = int(name.replace('extruder', '').strip())
                else:
                    self.index = int(name.replace('heater', '').strip())
            except ValueError:
                if config.has_section('reprapgui'):
                    self.index = config.getint("index")
        self.logger = printer.get_logger(self.name)
        self.heating_start_time = 0.
        self.heating_end_time = None
        self.temp_debug = 0.
        self.sensor = sensor
        min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELCIUS, default=None)
        max_temp = config.getfloat('max_temp', above=min_temp, default=None)
        sensor.setup_minmax(min_temp, max_temp)
        self.min_temp, self.max_temp = sensor.get_min_max_temp()
        self.logger.debug(
            "Heater index: %s, temperature range: min %.2f, max %.2f" % (
                self.index, self.min_temp, self.max_temp))
        sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = sensor.get_report_time_delta()
        # Setup temperature checks
        self.set_min_extrude_temp(170., False, True)
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.smooth_time = config.getfloat('smooth_time', 2., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.last_temp_time = 0.
        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        # Setup output heater pin
        heater_pin = config.get('heater_pin')
        ppins = self.printer.lookup_object('pins')
        if algo is ControlBangBang and self.max_power == 1.:
            self.mcu_pwm = ppins.setup_pin('digital_out', heater_pin)
        else:
            self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
            pwm_cycle_time = config.getfloat(
                'pwm_cycle_time', 0.100, above=0., maxval=self.pwm_delay)
            self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        # self.printer.try_load_module(config, "verify_heater %s" % (name,))
        self.printer.try_load_module(config, "pid_calibrate")
        self.protect = HeaterProtect(config, self)
        # register gcode commands
        for key in [self.get_name(), str(self.get_index())]:
            self.gcode.register_mux_command(
                "SET_HEATER_TEMPERATURE", "HEATER", key,
                self.cmd_SET_HEATER_TEMPERATURE,
                desc=self.cmd_SET_HEATER_TEMPERATURE_help)
    def _turn_off(self, print_time):
        self.set_temp(print_time, 0)
    def get_min_extrude_status(self):
        stat = "prevented"
        if self.min_extrude_temp_disabled:
            stat = "allowed"
        return stat, self.min_extrude_temp
    def set_min_extrude_temp(self, temp, disable=None, init=False):
        if disable is not None:
            self.min_extrude_temp_disabled = disable
        if temp is None:
            return
        if self.max_temp < temp and not init:
            raise self.error("min_extrude_temp {} is not between min_temp {} and max_temp {}!"
                             .format(temp, self.min_temp, self.max_temp))
        self.min_extrude_temp = temp
        is_fileoutput = (self.printer.get_start_args().get('debugoutput')
                         is not None)
        self.can_extrude = ((self.min_extrude_temp <= self.min_temp) or
                            self.min_extrude_temp_disabled or
                            is_fileoutput)
    def get_heating_time(self):
        if self.heating_end_time is None:
            return 0.
        return self.heating_end_time - self.heating_start_time
    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.:
            value = 0.
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
            and abs(value - self.last_pwm_value) < 0.05):
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value
        self.logger.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
                          self.name, value, pwm_time,
                          self.last_temp, self.last_temp_time, self.target_temp)
        self.mcu_pwm.set_pwm(pwm_time, value)
    def temperature_callback(self, read_time, temp):
        '''
        # >>>>> DEBUG DEBUG DEBUG >>>>>
        if self.target_temp:
            if self.last_pwm_value:
                self.temp_debug += 1.5
            else:
                self.temp_debug -= .5
            temp = self.temp_debug
        else:
            if self.temp_debug > DEBUG_TARGET:
                self.temp_debug -= .05
            else:
                self.temp_debug = DEBUG_TARGET
            #self.temp_debug = temp
            temp = self.temp_debug
        # <<<<< DEBUG DEBUG DEBUG <<<<<
        #'''
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            self.control.temperature_update(read_time, temp, self.target_temp)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp or
                                self.min_extrude_temp_disabled)
        if self.heating_end_time is None:
            if self.heating_start_time is None:
                self.heating_start_time = read_time
            elif temp >= self.target_temp:
                self.heating_end_time = read_time
        # self.logger.debug("read_time=%.3f temperature=%f", read_time, temp)
    # External commands
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, print_time, degrees, auto_tune=False):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise error("Requested temperature (%.1f) out of range (%.1f:%.1f)"
                        % (degrees, self.min_temp, self.max_temp))
        self.protect.set_runaway(auto_tune)
        with self.lock:
            self.target_temp = degrees
        self.protect.start()
        if degrees:
            self.heating_end_time = self.heating_start_time = None
    def get_temp(self, eventtime):
        print_time = self.sensor.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def check_busy(self, eventtime):
        if self.target_temp <= 0.:
            # Heating stopped
            return False
        with self.lock:
            return self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp)
    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control
    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp
    def stats(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.name, target_temp, last_temp, last_pwm_value)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
        return {'temperature': smoothed_temp, 'target': target_temp}
    cmd_SET_HEATER_TEMPERATURE_help = "Sets a heater temperature"
    def cmd_SET_HEATER_TEMPERATURE(self, params):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        temp = self.gcode.get_float('TARGET', params, 0.)
        self.set_temp(print_time, temp)


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.logger = heater.logger.getChild('bangbang')
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_update(self, read_time, temp, target_temp):
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater_max_power)
        else:
            self.heater.set_pwm(read_time, 0.)
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return (smoothed_temp < (target_temp - self.max_delta)) or \
               ((target_temp + self.max_delta) < smoothed_temp)


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1


class ControlPID:
    def __init__(self, heater, config):
        self.logger = heater.logger.getChild('pid')
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.logger.debug("PID: Kp %s, Ki %s, Kd %s", self.Kp, self.Ki, self.Kd)
        self.min_deriv_time = heater.get_smooth_time()
        self.imax = config.getfloat('pid_integral_max', self.heater_max_power,
                                    minval=0.)
        self.temp_integ_max = self.imax / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
        self.gcode = gcode = config.get_printer().lookup_object('gcode')
        for name in [heater.get_name(), str(heater.get_index())]:
            gcode.register_mux_command("SET_PID_PARAMS", "HEATER", name,
                                       self.cmd_SET_PID_PARAMS,
                                       desc=self.cmd_SET_PID_PARAMS_help)
    cmd_SET_PID_PARAMS_help = "Args: [HEATER=] [P=] [I=] [D=] " \
                              "[DERIV_TIME=] [INTEGRAL_MAX=]"
    def cmd_SET_PID_PARAMS(self, params):
        self.Kp = self.gcode.get_float(
            'P', params, self.Kp*PID_PARAM_BASE, minval=0.) / PID_PARAM_BASE
        self.Ki = self.gcode.get_float(
            'I', params, self.Ki*PID_PARAM_BASE, minval=0.) / PID_PARAM_BASE
        self.Kd = self.gcode.get_float(
            'D', params, self.Kd*PID_PARAM_BASE, minval=0.) / PID_PARAM_BASE
        self.min_deriv_time = self.gcode.get_float(
            'DERIV_TIME', params, self.min_deriv_time, above=0.)
        self.imax = self.gcode.get_float(
            'INTEGRAL_MAX', params, self.imax, minval=0.)
        self.temp_integ_max = self.imax / self.Ki
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
        self.gcode.respond_info(
            "PID params: P=%.2f I=%.2f D=%.2f TIME=%.2f MAX=%.2f" %
            (self.Kp*PID_PARAM_BASE, self.Ki*PID_PARAM_BASE, self.Kd*PID_PARAM_BASE,
             self.min_deriv_time, self.imax))
    def temperature_update(self, read_time, temp, target_temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        # self.logger.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(self.heater_max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


######################################################################
# Sensor and heater lookup
######################################################################

class sentinel:
    pass

class PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.sensors = {}
        self.gcode_id_to_sensor = {}
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_heaters)
        # Register TURN_OFF_HEATERS command
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name == 'extruder':
            heater_name = 'extruder0'
        elif heater_name == 'heater_bed':
            heater_name = 'heater bed'
        elif 'extruder' not in heater_name:
            # config [heater 0] case
            heater_name = 'heater ' + heater_name
        # Check if heater already exist
        if heater_name in self.heaters:
            heater = self.heaters[heater_name]
            self.register_sensor(config, heater, gcode_id)
            return heater
            # raise config.error("Heater %s already registered" % (heater_name,))
        # Setup sensor
        sensor = config.get("sensor", None)
        if sensor is not None:
            sensor = self.setup_sensor(config.getsection('sensor %s' % sensor))
        else:
            sensor = self.setup_sensor(config)
        # Create heater
        heater = Heater(config, sensor)
        self.heaters[heater_name] = heater
        self.register_sensor(config, heater, gcode_id)
        return heater
    def lookup_heater(self, heater_name, default=sentinel):
        heater_name = heater_name.split()[-1]
        if heater_name == 'extruder':
            heater_name = 'extruder0'
        elif heater_name == 'heater_bed':
            heater_name = 'heater bed'
        elif 'extruder' not in heater_name:
            # config [heater 0] case
            heater_name = 'heater ' + heater_name
        if heater_name not in self.heaters:
            if default is not sentinel:
                return default
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    def setup_sensor(self, config):
        self.printer.try_load_module(config, "thermistor")
        self.printer.try_load_module(config, "adc_temperature")
        self.printer.try_load_module(config, "spi_temperature")
        name = config.get_name().replace('sensor', '').strip()
        if name in self.sensors:
            return self.sensors[name]
        sensor_type = config.get('sensor_type')
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        self.sensors[name] = self.sensor_factories[sensor_type](config)
        return self.sensors[name]
    def get_gcode_sensors(self):
        return self.gcode_id_to_sensor.items()
    def register_sensor(self, config, psensor, gcode_id=None):
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def turn_off_all_heaters(self, print_time):
        for heater in self.heaters.values():
            heater.set_temp(print_time, 0.)
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, params):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.turn_off_all_heaters(print_time)
    def get_heaters(self):
        return self.heaters

def add_printer_objects(config):
    config.get_printer().add_object('heater', PrinterHeaters(config))

def load_config_prefix(config):
    if 'heater bed' == config.get_name():
        pheater = config.get_printer().lookup_object('heater')
        pheater.setup_heater(config, 'B')
