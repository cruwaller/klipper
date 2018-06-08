# Printer heater support
#
# Copyright (C) 2016,2017  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import threading
import extras.sensors as sensors

######################################################################
# Heater
######################################################################

MAX_HEAT_TIME = 5.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.


class error(Exception):
    pass


class PrinterHeater:
    error = error

    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = printer.lookup_object('gcode')
        self.name = config.get_name()
        try:
            self.index = int(self.name[7:])
        except ValueError:
            self.index = -1  # Mark to bed
        self.logger = printer.logger.getChild(self.name.replace(" ", "_"))
        sensor_name = config.get('sensor')
        self.logger.debug("Add heater '{}', index {}, sensor {}".
                          format(self.name, self.index, sensor_name))
        self.sensor = sensors.load_sensor(
            config.getsection('sensor %s' % sensor_name))
        self.sensor.setup_callback(self.temperature_callback)
        self.report_delta = self.sensor.get_report_delta()
        self.mcu_sensor = self.sensor.get_mcu()
        self.min_temp, self.max_temp = self.sensor.get_min_max_temp()
        self.min_extrude_temp = 170.  # Set by the extruder
        self.min_extrude_temp_disabled = False
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)
        self.lock = threading.Lock()
        self.last_temp = 0.
        self.last_temp_time = 0.
        self.target_temp = 0.
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        heater_pin = config.get('heater_pin')
        ppins = printer.lookup_object('pins')
        if algo is ControlBangBang and self.max_power == 1.:
            self.mcu_pwm = ppins.setup_pin('digital_out', heater_pin)
        else:
            self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
            pwm_cycle_time = config.getfloat(
                'pwm_cycle_time', 0.100, above=0., maxval=self.report_delta)
            self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        self.is_fileoutput = self.mcu_pwm.get_mcu().is_fileoutput()
        self.can_extrude = (self.min_extrude_temp <= self.min_temp or
                            self.is_fileoutput)
        self.control = algo(self, config)
        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.
        # Load additional modules
        printer.try_load_module(config, "pid_calibrate")
        # heat check timer
        self.protection_period_heat = \
            config.getfloat('protect_period_heat', 10.0, above=0.0, maxval=120.0)
        self.protection_hysteresis_heat = \
            config.getfloat('protect_hysteresis_heat', 4.0, above=0.50)
        self.protection_period = \
            config.getfloat('protect_period', 10.0, above=0.0, maxval=120.0)
        self.protect_hyst_runaway = \
            config.getfloat('protect_hysteresis_runaway', 4.0, above=0.0)
        self.reactor = printer.reactor
        self.protection_timer = self.reactor.register_timer(self._check_heating)
        self.protection_last_temp = None
        self.protect_runaway_disabled = False
    def _check_heating(self, eventtime):
        next_time = 10.0  # next 10sec from now
        with self.lock:
            current_temp = self.last_temp
            target_temp = self.target_temp
        if self.protection_last_temp is None:
            self.is_heating = False
            self.is_runaway = False
            self.is_cooling = False
            # Set init value
            self.protection_last_temp = current_temp
            if current_temp <= (target_temp - self.protect_hyst_runaway):
                self.is_heating = True
                next_time = self.protection_period_heat
            elif current_temp > target_temp > 0:
                self.is_cooling = True
                next_time = self.protection_period
            else:
                self.is_runaway = True
                next_time = self.protection_period
        elif self.is_runaway:
            # Check hysteresis during maintain
            if self.protect_hyst_runaway < abs(current_temp - target_temp) and \
                    not self.protect_runaway_disabled:
                self.__protect_error(
                    "Thermal runaway! current temp %s, last %s" %
                    (current_temp, self.protection_last_temp))
            self.protection_last_temp = current_temp
            next_time = self.protection_period
        elif self.is_cooling:
            next_time = self.protection_period_heat
            if (current_temp - self.protect_hyst_runaway) < target_temp:
                self.is_cooling = False
                self.is_heating = True
        elif self.is_heating:
            # Check hysteresis during the preheating
            if ((target_temp - self.protect_hyst_runaway)
                    <= current_temp <=
                    (target_temp + self.protect_hyst_runaway)):
                self.is_runaway = True
            elif current_temp < target_temp:
                if abs(current_temp - self.protection_last_temp) < self.protection_hysteresis_heat:
                    self.__protect_error(
                        "Heating error! current temp %s, last %s" %
                        (current_temp, self.protection_last_temp))
            self.protection_last_temp = current_temp
            next_time = self.protection_period_heat
        self.logger.debug("check_heating(eventtime {}, next {}) {} / {}".
                          format(eventtime, (eventtime + next_time),
                                 current_temp, target_temp))
        return eventtime + next_time
    def __protect_error(self, errorstr):
        self.set_temp(0, 0)
        self.gcode.respond_stop(errorstr)
        self.printer.request_exit('shutdown')
    def get_min_extrude_status(self):
        stat = "prevented"
        if self.min_extrude_temp_disabled:
            stat = "allowed"
        return stat, self.min_extrude_temp
    def set_min_extrude_temp(self, temp, disable=None):
        if disable is not None:
            self.min_extrude_temp_disabled = disable
        if temp is None:
            return
        if self.max_temp < temp:
            raise self.error("min_extrude_temp {} is not between min_temp {} and max_temp {}!"
                             .format(temp, self.min_temp, self.max_temp))
        self.min_extrude_temp = temp
        self.can_extrude = ((self.min_extrude_temp <= self.min_temp) or
                            self.min_extrude_temp_disabled or
                            self.is_fileoutput)
    def set_pwm(self, read_time, value):
        if self.target_temp <= 0.:
            value = 0.
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
                and abs(value - self.last_pwm_value) < 0.05):
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.report_delta
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value
        self.logger.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
                          self.name, value, pwm_time,
                          self.last_temp, self.last_temp_time, self.target_temp)
        self.mcu_pwm.set_pwm(pwm_time, value)
    temp_debug = 0.
    def temperature_callback(self, read_time, read_value):
        temp = self.sensor.calc_temp(read_value)
        '''
        # >>>>> DEBUG DEBUG DEBUG >>>>>
        if self.target_temp:
            if self.last_pwm_value:
                self.temp_debug += 1.
            else:
                self.temp_debug -= 1.
            temp = self.temp_debug
        else:
            self.temp_debug = temp
        # <<<<< DEBUG DEBUG DEBUG <<<<<
        '''
        with self.lock:
            self.last_temp = temp
            self.last_temp_time = read_time
            self.can_extrude = (self.min_extrude_temp_disabled or
                                temp >= self.min_extrude_temp)
            self.control.temperature_callback(read_time, temp)
        # self.logger.debug("read_time=%.3f read_value=%f temperature=%f",
        #                  read_time, read_value, temp)
    # External commands
    def set_temp(self, print_time, degrees, auto_tune=False):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise error("Requested temperature (%.1f) out of range (%.1f:%.1f)"
                        % (degrees, self.min_temp, self.max_temp))
        self.protect_runaway_disabled = auto_tune
        with self.lock:
            self.target_temp = degrees
        if degrees:
            # Start checking
            self.protection_last_temp = None
            self.reactor.update_timer(self.protection_timer,
                                      self.reactor.NOW)
            self.logger.debug("Temperature protection timer started")
        else:
            # stop checking
            self.reactor.update_timer(self.protection_timer,
                                      self.reactor.NEVER)
            self.logger.debug("Temperature protection timer stopped")
    def get_temp(self, eventtime):
        print_time = self.mcu_sensor.estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.last_temp, self.target_temp
    def check_busy(self, eventtime):
        if self.target_temp <= 0.:
            # Heating stopped
            return False
        with self.lock:
            return self.control.check_busy(eventtime)
    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control
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
            last_temp = self.last_temp
        return {'temperature': last_temp, 'target': target_temp}


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.logger = heater.logger.getChild('bangbang')
        self.heater = heater
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_callback(self, read_time, temp):
        if self.heating and temp >= self.heater.target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= self.heater.target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater.max_power)
        else:
            self.heater.set_pwm(read_time, 0.)
    def check_busy(self, eventtime):
        return (self.heater.last_temp < (self.heater.target_temp - self.max_delta)) or \
               ((self.heater.target_temp + self.max_delta) < self.heater.last_temp)


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1


class ControlPID:
    def __init__(self, heater, config):
        self.logger = heater.logger.getChild('pid')
        self.heater = heater
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = config.getfloat('pid_deriv_time', 2., above=0.)
        self.imax = config.getfloat('pid_integral_max', heater.max_power, minval=0.)
        self.temp_integ_max = self.imax / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
        self.gcode = gcode = config.get_printer().lookup_object('gcode')
        for name in [heater.name.replace(" ", "_").upper(), str(heater.index)]:
            gcode.register_mux_command("SET_PID_PARAMS", "HEATER", name,
                                       self.cmd_SET_PID_PARAMS,
                                       desc=self.cmd_SET_PID_PARAMS_help)
    cmd_SET_PID_PARAMS_help = "HEATER, P, I, D, DERIV_TIME, INTEGRAL_MAX"
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
    def temperature_callback(self, read_time, temp):
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        # Calculate accumulated temperature "error"
        temp_err = self.heater.target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        # self.logger.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(self.heater.max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
    def check_busy(self, eventtime):
        temp_diff = self.heater.target_temp - self.heater.last_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


def load_config(config):
    raise config.get_printer().config_error(
        "Naming without index (bed or [0-9]+) is not allowed")


def load_config_prefix(config):
    return PrinterHeater(config)
