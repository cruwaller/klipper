import os, logging
import pins

(sysname, hostname, release, version, machine) = os.uname()

GPIO = None

if machine in ["x86_64", "x86"]:
    class GpioTemp:
        logger = logging.getLogger('Linux.GpioTemp')
        BOARD = 'BOARD'
        BCM = 'BCM'
        RISING = 'RISING'
        FALLING = 'FALLING'
        BOTH = 'BOTH'
        PUD_UP = 'PUD_UP'
        PUD_DOWN = 'PUD_DOWN'
        PUD_OFF = 'PUD_OFF'
        IN = 'IN'
        OUT = 'OUT'
        HIGH = 'HIGH'
        LOW = 'LOW'
        def __init__(self, *args, **kwargs):
            pass
        def setmode(self, mode):
            self.logger.debug("setmode(%s)" % str(mode))
        def setup(self, channel, mode, pull_up_down=PUD_OFF, initial=None):
            self.logger.debug("setup(channel=%s, mode=%s, pull_up_down=%s, initial=%s)" % (
                channel, mode, pull_up_down, initial))
        def cleanup(self):
            self.logger.debug("cleanup()")
        def input(self, channel):
            self.logger.debug("input(channel=%s)" % channel)
            return channel & 1
        def output(self, channel, state):
            self.logger.debug("output(channel=%s, state=%s)" % (channel, state))
        def add_event_detect(self, channel, event, callback=None, bouncetime=None):
            self.logger.debug("add_event_detect(channel=%s, event=%s, callback=%s, bouncetime=%s)" % (
                channel, event, callback, bouncetime))
        def remove_event_detect(self, channel):
            self.logger.debug("remove_event_detect(channel=%s)" % channel)
        class PWM:
            logger = logging.getLogger('Linux.GpioTemp.PWM')
            def __init__(self, ch, f):
                self.logger.debug("__init__(channel=%s, freq=%s)" % (ch, f))
            def start(self, duty):
                self.logger.debug("start(duty=%s)" % duty)
            def stop(self):
                self.logger.debug("stop()")
            def ChangeFrequency(self, f):
                self.logger.debug("ChangeFrequency(%s)" % f)
            def ChangeDutyCycle(self, d):
                self.logger.debug("ChangeDutyCycle(%s)" % d)
    GPIO = GpioTemp()
elif sysname == "Linux":
    if machine in ['armv6l', 'armv7l']:
        try:
            import RPi.GPIO as GPIO
        except RuntimeError:
            raise Exception("Error importing RPi.GPIO! "
                            "This is probably because you need superuser privileges. "
                            "You can achieve this by using 'sudo' to run your script")
        except ImportError:
            raise Exception("Error importing RPi.GPIO!"
                            "Try to install it first: ~/klippy-env/bin/pip install RPi.GPIO")
    elif machine == 'aarch64':
        try:
            import OPi.GPIO as GPIO
        except ImportError:
            raise Exception("Error importing OPi.GPIO!"
                            "Try to install it first: ~/klippy-env/bin/pip install OPi.GPIO")

try:
    PWM = GPIO.PWM
except AttributeError:
    PWM = None


class HostGpioIn:
    def __init__(self, pin_params):
        self.invert = pin_params['invert']
        self.channel = channel = pin_params['pin_number']
        pull_up_down = GPIO.PUD_UP if pin_params['pullup'] else GPIO.PUD_DOWN
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
    def get_digital(self, *args):
        return GPIO.input(self.channel) ^ self.invert


class HostGpioEvent:
    channel = None
    def __init__(self, pin_params):
        self.invert = pin_params['invert']
        self.channel = channel = pin_params['pin_number']
        pull_up_down = [GPIO.PUD_DOWN, GPIO.PUD_UP][pin_params['pullup']]
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
    def __del__(self):
        if self.channel is not None:
            GPIO.remove_event_detect(self.channel)
    def get_digital(self, *args):
        return GPIO.input(self.channel) ^ self.invert
    def set_event(self, cb, edge, bounce=10):
        edge = edge.upper()
        eventc = {"RISING": GPIO.RISING,
                  "FALLING": GPIO.FALLING,
                  "BOTH": GPIO.BOTH}
        if self.invert:
            eventc["RISING"] = GPIO.FALLING
            eventc["FALLING"] = GPIO.RISING
        GPIO.add_event_detect(self.channel, eventc[edge],
                              callback=cb, bouncetime=bounce)


class HostGpioOut:
    def __init__(self, pin_params):
        self.invert = invert = pin_params['invert']
        self._static = self.shutdown_value = False
        state = GPIO.HIGH if invert else GPIO.LOW
        self.channel = channel = pin_params['pin_number']
        GPIO.setup(channel, GPIO.OUT,
                   pull_up_down=GPIO.PUD_OFF, initial=state)
        self.__write(False)
    def __write(self, state, force=False):
        if self._static and force is False:
            return
        GPIO.output(self.channel, bool(state) ^ self.invert)
    def printer_state(self, state):
        if state == 'shutdown':
            self.__write(self.shutdown_value, force=True)
    def set_digital(self, print_time, value):
        self.__write(value)
    def set_pwm(self, print_time, value):
        self.__write(value >= 0.5)
    def setup_max_duration(self, max_duration):
        pass
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        self.shutdown_value = shutdown_value
        self.set_pwm(0, start_value)
        self._static = is_static


class HostPwm:
    def __init__(self, pin_params):
        if PWM is None:
            raise pin_params['chip'].config_error("PWM is not supported!")
        self.invert = invert = pin_params['invert']
        self.freq = 10 # default cycle_time 0.100s
        self.duty = None
        self.shutdown_value = 100 * invert
        channel = pin_params['pin_number']
        GPIO.setup(channel, GPIO.OUT)
        self.pwm = GPIO.PWM(channel, self.freq)
        self.pwm.start(self.shutdown_value)
        self.__calc_duty(.0)
        self._static = False
    def __del__(self):
        self.pwm.stop()
    def __set_duty(self, duty):
        self.pwm.ChangeDutyCycle(duty * 100.)
    def __set_freq(self, freq):
        self.freq = freq
        self.pwm.ChangeFrequency(freq)
    def __calc_duty(self, duty):
        """ Duty can be 0.0 ... 1.0 """
        if 0. <= duty <= 1.:
            if self.invert:
                self.duty = 1. - duty
            else:
                self.duty = duty
            return self.duty
        raise Exception("PWM duty '%s' is not valid!" % duty)
    def __write(self, duty, force=False):
        if self._static and force is False:
            # Cannot change duty value if static!
            return
        self.__set_duty(self.__calc_duty(duty))
    def get_duty(self):
        if self.invert:
            return 1. - self.duty
        return self.duty
    def get_freq(self):
        return self.freq
    def set_freq(self, freq):
        self.__set_freq(freq)
    def printer_state(self, state):
        if state == 'shutdown':
            self.__write(self.shutdown_value, force=True)
    def setup_max_duration(self, max_duration):
        pass
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self.__set_freq(1. / cycle_time)
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        self.shutdown_value = shutdown_value
        self.__write(start_value)
        self._static = is_static
    def set_pwm(self, print_time, value):
        self.__write(value)


class HostSpi:
    # GPIO pins:
    #   MOSI GPIO10/P19
    #   MISO GPIO09/P21
    #   SCLK  GPIO11/P23
    #   CE0  GPIO08/P24
    #   CE1  GPIO07/P26
    def __init__(self, pin_params):
        raise pin_params['chip'].config_error(
            "Host SPI is not implemented yet")


class HostI2C:
    # GPIO pins:
    #   SDA1 P2
    #   SCL1 P3
    def __init__(self, pin_params):
        raise pin_params['chip'].config_error(
            "Host I2C is not implemented yet")


class HostPins(object):
    pin_mode = None
    def __init__(self, config):
        printer = config.get_printer()
        printer.lookup_object('pins').register_chip('host', self)
        self.config_error = printer.config_error
        self.logger = printer.logger.getChild("hostpins")
        # BOARD = physical pin number in pinstripe
        # BCM   = Broadcom's chip pin number
        GPIO.setmode(GPIO.BOARD)
        pinmap = {}
        if machine in ["armv7l", 'aarch64']:
            # Raspberry Pi3 or Orange Pi PC2
            pinmap = self._rpi_v2_pins()
        elif machine == "armv6l":
            # Raspberry Pi1
            pinmap = self._rpi_v1_pins()
        elif machine in ["x86_64", "x86"]:
            # debug linux!
            pinmap = self._rpi_v2_pins()
        self.available_pins = pinmap
    def __del__(self):
        GPIO.cleanup()
    def __str__(self):
        return "HostPins"
    def _rpi_v1_pins(self):
        # Add BOARD mapping
        gpios = {"P%d" % i: i for i in [
            # V1 pin header is 26 pins
            3, 5, 7, 8, 10, 11, 12, 13, 15, 16, 18, 19,
            21, 22, 23, 24, 26,
        ]}
        # Add BCM to BOARD mapping
        gpios.update({"GPIO%d" % i[0]: i[1] for i in [
            # All models:
            (4, 7), (14, 8), (15, 10), (17, 11), (18, 12),
            (22, 15), (23, 16), (24, 18), (10, 19), (9, 21), (25, 22),
            (11, 23), (8, 24), (7, 26),
            # Pi 1 Model B Revision 2.0 or later:
            (2, 3), (3, 5), (27, 13),
            # Pi 1 Model B Revision 1.0:
            (0, 3), (1, 5), (21, 13),
        ]})
        return gpios
    def _rpi_v2_pins(self):
        # V2 pin header is 40 pins
        gpios = self._rpi_v1_pins()
        gpios.update({"P%d" % i: i for i in [
            29, 31, 32, 33, 35, 36, 37, 38, 40
        ]})
        gpios.update({"GPIO%d" % i[0]: i[1] for i in [
            (5, 29), (6, 31), (12, 32), (13, 33), (19, 35),
            (16, 36), (26, 37), (20, 38), (21, 40)
        ]})
        return gpios
    def get_logger(self, child=None):
        if child is not None:
            return self.logger.getChild(child)
        return self.logger
    def get_pin_number(self, pin):
        if pin not in self.available_pins:
            raise self.config_error(
                "Requested pin '%s' is not available!" % pin)
        return self.available_pins[pin]
    def setup_pin(self, pin_type, pin_params):
        pcs = {'digital_in': HostGpioIn,
               'digital_out': HostGpioOut,
               'digital_event': HostGpioEvent,
               'spi': HostSpi, 'i2c': HostI2C, 'pwm': HostPwm}
        if pin_type not in pcs:
            raise self.config_error(
                "pin type %s not supported on mcu" % (pin_type,))
        pin_params['pin_number'] = self.get_pin_number(pin_params['pin'])
        co = pcs[pin_type](pin_params)
        return co

def load_config(config):
    return HostPins(config)
