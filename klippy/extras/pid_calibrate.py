# Calibration of heater PID settings
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import extruder, heater

class PIDCalibrate:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command(
            'PID_CALIBRATE', self.cmd_PID_CALIBRATE,
            desc=self.cmd_PID_CALIBRATE_help)
        self.gcode.register_command('M303', self.cmd_M303)
        self.logger = self.printer.logger.getChild('PIDCalibrate')
    cmd_PID_CALIBRATE_help = "Run PID calibration test"
    def cmd_PID_CALIBRATE(self, params):
        heater_name = self.gcode.get_str('HEATER', params).lower()
        target = self.gcode.get_float('TARGET', params)
        count = self.gcode.get_int('TARGET', params, 12)
        write_file = self.gcode.get_int('WRITE_FILE', params, 0)
        try:
            if 'extruder' in heater_name:
                tgt_heater = extruder.get_printer_extruder(self.printer,
                    int(heater_name[8:])).get_heater()
            elif 'heater_bed' == heater_name:
                tgt_heater = self.printer.lookup_object('heater bed')
            else:
                tgt_heater = self.printer.lookup_object(
                    'heater %s' % heater_name)
        except ValueError:
            raise self.gcode.error("Error: extruder index is missing!")
        except (AttributeError, self.printer.config_error) as e:
            raise self.gcode.error("Error: Heater not found! Check heater name and try again")
        self.__start(tgt_heater, target, write_file, count)
    def __start(self, tgt_heater, target, write_file=False, count=12):
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        calibrate = ControlAutoTune(tgt_heater, self.logger, count)
        old_control = tgt_heater.set_control(calibrate)
        try:
            tgt_heater.set_temp(print_time, target, auto_tune=True)
        except tgt_heater.error as e:
            tgt_heater.set_control(old_control)
            raise self.gcode.error(str(e))
        self.gcode.bg_temp(tgt_heater)
        tgt_heater.set_control(old_control)
        if write_file:
            calibrate.write_file('/tmp/heattest.txt')
        try:
            Kp, Ki, Kd = calibrate.calc_final_pid()
        except Exception:
            raise self.gcode.error("Error during calibrarion.")
        self.logger.info("Autotune: final: Kp=%f Ki=%f Kd=%f", Kp, Ki, Kd)
        self.gcode.respond_info(
            "PID parameters: pid_Kp=%.3f pid_Ki=%.3f pid_Kd=%.3f\n"
            "To use these parameters, update the printer config file with\n"
            "the above and then issue a RESTART command" % (Kp, Ki, Kd))
    def cmd_M303(self, params):
        # Run PID tuning (M303 E<-1 or 0...> S<temp> C<count> W<write_file>)
        tgt_heater = None
        heater_index = self.gcode.get_int('E', params, 0)
        if heater_index == -1:
            tgt_heater = self.printer.lookup_object('heater bed', None)
        else:
            e = extruder.get_printer_extruder(self.printer, heater_index)
            if e is not None:
                tgt_heater = e.get_heater()
        if tgt_heater is None:
            self.gcode.error("Heater is not configured")
        else:
            temp = self.gcode.get_float('S', params)
            count = self.gcode.get_int('C', params, 12)
            write = self.gcode.get_int('W', params, 0)
            self.__start(tgt_heater, temp, write, count)


TUNE_PID_DELTA = 5.0

class ControlAutoTune:
    def __init__(self, tgt_heater, logger, count):
        self.heater = tgt_heater
        self.logger = logger
        self.count = count
        # Heating control
        self.heating = False
        self.peak = 0.
        self.peak_time = 0.
        # Peak recording
        self.peaks = []
        # Sample recording
        self.last_pwm = 0.
        self.pwm_samples = []
        self.temp_samples = []
    # Heater control
    def set_pwm(self, read_time, value):
        if value != self.last_pwm:
            self.pwm_samples.append((read_time + self.heater.report_delta, value))
            self.last_pwm = value
        self.heater.set_pwm(read_time, value)
    def temperature_callback(self, read_time, temp):
        self.temp_samples.append((read_time, temp))
        if self.heating and temp >= self.heater.target_temp:
            self.heating = False
            self.check_peaks()
        elif (not self.heating
              and temp <= self.heater.target_temp - TUNE_PID_DELTA):
            self.heating = True
            self.check_peaks()
        if self.heating:
            self.set_pwm(read_time, self.heater.max_power)
            if temp < self.peak:
                self.peak = temp
                self.peak_time = read_time
        else:
            self.set_pwm(read_time, 0.)
            if temp > self.peak:
                self.peak = temp
                self.peak_time = read_time
    def check_busy(self, eventtime):
        if self.heating or len(self.peaks) < self.count:
            return True
        return False
    # Analysis
    def check_peaks(self):
        self.peaks.append((self.peak, self.peak_time))
        if self.heating:
            self.peak = 9999999.
        else:
            self.peak = -9999999.
        if len(self.peaks) < 4:
            return
        self.calc_pid(len(self.peaks)-1)
    def calc_pid(self, pos):
        temp_diff = self.peaks[pos][0] - self.peaks[pos-1][0]
        time_diff = self.peaks[pos][1] - self.peaks[pos-2][1]
        max_power = self.heater.max_power
        Ku = 4. * (2. * max_power) / (abs(temp_diff) * math.pi)
        Tu = time_diff

        Ti = 0.5 * Tu
        Td = 0.125 * Tu
        Kp = 0.6 * Ku * heater.PID_PARAM_BASE
        Ki = Kp / Ti
        Kd = Kp * Td
        self.logger.info("Autotune: raw=%f/%f Ku=%f Tu=%f  Kp=%f Ki=%f Kd=%f",
                         temp_diff, max_power, Ku, Tu, Kp, Ki, Kd)
        return Kp, Ki, Kd
    def calc_final_pid(self):
        if len(self.peaks) == 0:
            raise Exception("Internal error with peaks!")
        cycle_times = [(self.peaks[pos][1] - self.peaks[pos-2][1], pos)
                       for pos in range(4, len(self.peaks))]
        if len(cycle_times) == 0:
            raise Exception("Internal error with cycle_timers!")
        midpoint_pos = sorted(cycle_times)[len(cycle_times)/2][1]
        return self.calc_pid(midpoint_pos)
    # Offline analysis helper
    def write_file(self, filename):
        pwm = ["pwm: %.3f %.3f" % (time, value)
               for time, value in self.pwm_samples]
        out = ["%.3f %.3f" % (time, temp) for time, temp in self.temp_samples]
        f = open(filename, "wb")
        f.write('\n'.join(pwm + out))
        f.close()

def load_config(config):
    return PIDCalibrate(config)
