import os, logging
import pins

(sysname, hostname, release, version, machine) = os.uname()

GPIO = None

if machine in ["x86_64", "x86"]:
    class GpioTemp:
        logger = logging.getLogger('GPIO_DBG')
        BOARD = 0
        BCM = 1

        RISING = FALLING = BOTH = 0
        PUD_UP = 2
        PUD_DOWN = 1
        PUD_OFF = 0
        IN = 1
        OUT = 0
        HIGH = 1
        LOW = 0
        def __init__(self, *args, **kwargs):
            pass
        def setmode(self, mode):
            pass
        def setup(self, *args, **kwargs):
            pass
        def cleanup(self, *args, **kwargs):
            pass
        def input(self, *args, **kwargs):
            self.logger.debug("Read %s" % str(args))
            return 0
        def output(self, *args, **kwargs):
            self.logger.debug("Write %s" % str(args))
            pass
        def add_event_detect(self, *args, **kwargs):
            pass
        def remove_event_detect(self, *args, **kwargs):
            pass
        class PWM:
            logger = logging.getLogger('PWM_DBG')
            def __init__(self, ch, f):
                pass
            def start(self, duty):
                pass
            def stop(self):
                pass
            def ChangeFrequency(self, f):
                self.logger.debug("Freq %s" % f)
                pass
            def ChangeDutyCycle(self, d):
                self.logger.debug("Duty %s" % d)
                pass
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
    def __init__(self, pin_params, logger):
        self.logger = logger
        self.invert = pin_params['invert']
        self.channel = channel = pin_params['pin_number']
        pull_up_down = GPIO.PUD_UP if pin_params['pullup'] else GPIO.PUD_DOWN
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
        logger.debug("GPIO IN initialized (pull up: %s)" % pin_params['pullup'])
    def __read(self):
        return GPIO.input(self.channel) ^ self.invert
    def get_digital(self, *args):
        return self.__read()


class HostGpioEvent:
    channel = None
    def __init__(self, pin_params, logger):
        self.logger = logger
        self.invert = pin_params['invert']
        self.channel = channel = pin_params['pin_number']
        pull_up_down = [GPIO.PUD_DOWN, GPIO.PUD_UP][pin_params['pullup']]
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
        logger.debug("GPIO IN event initialized (pull up: %s)" % pin_params['pullup'])
    def __del__(self):
        if self.channel is not None:
            GPIO.remove_event_detect(self.channel)
    def read(self):
        return GPIO.input(self.channel)
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
        self.logger.debug("Event '%s' set. Bounce %s" % (edge, bounce,))


class HostGpioOut:
    def __init__(self, pin_params, logger):
        self.logger = logger
        self.invert = invert = pin_params['invert']
        self._static = self.shutdown_value = False
        state = GPIO.HIGH if invert else GPIO.LOW
        self.channel = channel = pin_params['pin_number']
        GPIO.setup(channel, GPIO.OUT,
                   pull_up_down=GPIO.PUD_OFF, initial=state)
        self.__write(False)
        logger.debug("GPIO OUT initialized to state: %s" % state)
    def __write(self, state):
        if self._static:
            return
        GPIO.output(self.channel, bool(state) ^ self.invert)
    def printer_state(self, state):
        if state == 'shutdown':
            self.__write(self.shutdown_value)
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
    def __init__(self, pin_params, logger):
        self.logger = logger
        if PWM is None:
            raise pin_params['chip'].config_error("PWM is not supported!")
        self.invert = invert = pin_params['invert']
        self.freq = 10 # cycle_time 0.100
        self.duty = None
        self.shutdown_value = self.duty_off = 100 * invert
        channel = pin_params['pin_number']
        GPIO.setup(channel, GPIO.OUT)
        self.pwm = GPIO.PWM(channel, 1) # 1Hz
        self.pwm.start(self.duty_off)
        self.set_duty(.0)
        self._static = False
        # logger.debug("GPIO PWM initialized (duty init: %s)" % (self.duty_off,))
    def __del__(self):
        self.pwm.stop()
    def __set_duty(self, duty):
        self.pwm.ChangeDutyCycle(duty * 100.)
        self.logger.debug("PWM duty to %s" % duty)
    def __set_freq(self, freq):
        self.freq = freq
        self.pwm.ChangeFrequency(freq)
        self.logger.debug("PWM freq to %s Hz" % freq)
    def set_duty(self, duty):
        """ Duty can be 0.0 ... 1.0 """
        if 0. <= duty <= 1.:
            if self.invert:
                self.duty = 1. - duty
            else:
                self.duty = duty
            return self.duty
        raise Exception("PWM duty '%s' is not valid!" % duty)
    def write(self, duty, freq=None):
        if self._static:
            # Cannot change duty value if static!
            return
        if freq is not None:
            self.__set_freq(freq)
        self.__set_duty(self.set_duty(duty))
    def get_duty(self):
        if self.invert:
            return 1. - self.duty
        return self.duty
    def get_freq(self):
        return self.freq
    def start(self):
        self.__set_duty(self.duty)
    def stop(self):
        self.__set_duty(self.duty_off)
    def printer_state(self, state):
        if state == 'shutdown':
            self.write(self.shutdown_value)
    def setup_max_duration(self, max_duration):
        # self.logger.debug("max_duration: %s", max_duration)
        pass
    def setup_cycle_time(self, cycle_time, hardware_pwm=False):
        self.__set_freq(1. / cycle_time)
    def setup_start_value(self, start_value, shutdown_value, is_static=False):
        if is_static and start_value != shutdown_value:
            raise pins.error("Static pin can not have shutdown value")
        self.shutdown_value = shutdown_value
        self.write(start_value)
        self._static = is_static
    def set_pwm(self, print_time, value):
        self.write(value)


class HostSpi:
    def __init__(self, pin_params, logger):
        self.logger = logger
        raise pin_params['chip'].config_error(
            "Host SPI is not implemented yet")


class HostI2C:
    def __init__(self, pin_params, logger):
        self.logger = logger
        raise pin_params['chip'].config_error(
            "Host I2C is not implemented yet")


class HostPins(object):
    pin_mode = None
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.printer.lookup_object('pins').register_chip('host', self)
        self.config_error = self.printer.config_error
        self.logger = logger = printer.logger.getChild("hostpins")
        self.available_pins = {}
        self._setmode(config.get('mode', default='BOARD'))
        logger.info("Initialized")
    def __del__(self):
        GPIO.cleanup()
        self.logger.warning("GPIO ports reset done!")
    def __str__(self):
        return "HostPins"
    def _setmode(self, mode='BOARD'):
        if mode is None:
            return
        # BOARD = physical pin number in pinstripe
        # BCM   = Broadcom's chip pin number
        self.pin_mode = mode
        if mode == 'BCM':
            mode = GPIO.BCM
        else:
            mode = GPIO.BOARD
        GPIO.setmode(mode)
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
        self.logger.debug("GPIO pin mode set to '%s'" % self.pin_mode)
    def _rpi_v1_pins(self):
        # V1 pin header is 26 pins
        if self.pin_mode == "BOARD":
            return {"P%d" % i: i for i in [
                3, 5, 7, 8, 10, 11, 12, 13, 15, 16, 18, 19,
                21, 22, 23, 24, 26
            ]}
        # All models in BCM mode:
        gpios = {"GPIO%d" % i: i for i in [
            4, 14, 15, 17, 18, 22, 23, 24, 10, 9, 25, 11, 8, 7
        ]}
        # Pi 1 Model B Revision 2.0 or later:
        gpios.update({"GPIO%d" % i: i for i in [2, 3, 27]})
        # Pi 1 Model B Revision 1.0:
        gpios.update({"GPIO%d" % i: i for i in [0, 1, 21]})
        return gpios
    def _rpi_v2_pins(self):
        # V2 pin header is 40 pins
        gpios = self._rpi_v1_pins()
        if self.pin_mode == "BOARD":
            gpios.update({"P%d" % i: i for i in [
                29, 31, 32, 33, 35, 36, 37, 38, 40
            ]})
        else:
            gpios.update({"GPIO%d" % i: i for i in [
                5, 6, 12, 13, 19, 16, 26, 20, 21
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
        co = pcs[pin_type](pin_params, logger=self.get_logger(pin_params['pin']))
        return co

def load_config(config):
    return HostPins(config)
