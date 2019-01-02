import os, re

(sysname, hostname, release, version, machine) = os.uname()

GPIO = None

if machine in ["x86_64", "x86"]:
    class GpioTemp:
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
            return 0
        def output(self, *args, **kwargs):
            pass
        def add_event_detect(self, *args, **kwargs):
            pass
        def remove_event_detect(self, *args, **kwargs):
            pass
        class PWM:
            def __init__(self, ch, f):
                pass
            def start(self, duty):
                pass
            def stop(self):
                pass
            def ChangeFrequency(self, f):
                pass
            def ChangeDutyCycle(self, d):
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
        self.channel = channel = pin_params['pin_number'] \
            #pin_params['chip'].get_pin_number(pin_params['pin'])
        pull_up_down = GPIO.PUD_UP if pin_params['pullup'] else GPIO.PUD_DOWN
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
        logger.debug("GPIO IN initialized (pull up: %s)" % pin_params['pullup'])
    def read(self):
        return GPIO.input(self.channel) ^ self.invert


class HostGpioEvent:
    channel = None
    def __init__(self, pin_params, logger):
        self.logger = logger
        self.invert = pin_params['invert']
        self.channel = channel = pin_params['pin_number'] \
            #pin_params['chip'].get_pin_number(pin_params['pin'])
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
        state = GPIO.HIGH if invert else GPIO.LOW
        self.channel = channel = pin_params['pin_number'] \
            #pin_params['chip'].get_pin_number(pin_params['pin'])
        GPIO.setup(channel, GPIO.OUT,
                   pull_up_down=GPIO.PUD_OFF, initial=state)
        self.write(False)
        logger.debug("GPIO OUT initialized to state: %s" % state)
    def write(self, state):
        GPIO.output(self.channel, bool(state) ^ self.invert)


class HostPwm:
    def __init__(self, pin_params, logger):
        self.logger = logger
        if PWM is None:
            raise pin_params['chip'].config_error("PWM is not supported!")
        self.invert = invert = pin_params['invert']
        self.freq = 1
        self.duty = None
        self.duty_off = 100 * invert
        #channel = pin_params['chip'].get_pin_number(pin_params['pin'])
        channel = pin_params['pin_number']
        GPIO.setup(channel, GPIO.OUT)
        self.pwm = GPIO.PWM(channel, 1) # 1Hz
        self.pwm.start(self.duty_off)
        self.set_duty(.0)
        logger.debug("GPIO PWM initialized (duty init: %s)" % (self.duty_off,))
    def __del__(self):
        self.pwm.stop()
    def __set_duty(self, duty):
        self.pwm.ChangeDutyCycle(duty * 100.)
        self.logger.debug("PWM duty to %s" % duty)
    def set_freq(self, freq):
        self.freq = freq
        self.pwm.ChangeFrequency(freq)
        self.logger.debug("PWM freq to %s" % freq)
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
        if freq is not None:
            self.set_freq(freq)
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
        self.config_error = self.printer.config_error
        self.logger = logger = printer.logger.getChild("hostpins")
        self.available_pins = self.active_pins = {}
        self.setmode(config.get('mode', default='BOARD'))
        logger.info("Initialized")
    def __del__(self):
        self.reset()
    def __str__(self):
        return "HostPins"
    def reset(self):
        GPIO.cleanup()
        self.active_pins = {}
        self.logger.warning("GPIO ports reset done!")
    def setmode(self, mode='BOARD'):
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
        self.logger.debug("GPIO pin mode is now '%s'" % self.pin_mode)
    @staticmethod
    def getmode():
        return GPIO.getmode()
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
        return self.available_pins[pin]
    def setup_pin(self, pin_type, pin_desc):
        pin_params = self.lookup_pin(pin_type, pin_desc)
        pcs = {'gpio_in': HostGpioIn, 'gpio_out': HostGpioOut,
               'spi': HostSpi, 'i2c': HostI2C, 'pwm': HostPwm,
               'gpio_event': HostGpioEvent}
        pin_type = pin_params['type']
        if pin_type not in pcs:
            raise self.config_error(
                "pin type %s not supported on mcu" % (pin_type,))
        co = pcs[pin_type](pin_params, logger=self.logger.getChild(pin_params['pin']))
        return co
    def lookup_pin(self, pin_type, pin_desc, share_type=None):
        if pin_type not in ['gpio_in', 'gpio_out', 'gpio_event',
                            'spi', 'i2c', 'pwm']:
            raise self.config_error("Invalid pin type %s" % pin_type)
        can_invert = pin_type in ['gpio_in', 'gpio_out', 'gpio_event', 'pwm']
        can_pullup = pin_type in ['gpio_in', 'gpio_event']
        desc = pin_desc
        pullup = invert = 0
        if can_pullup and '^' in desc:
            pullup = 1
        if can_invert and '!' in desc:
            invert = 1
        pin = re.sub("[\^!]", "", desc).strip()
        if [c for c in '^!: ' if c in pin]:
            format = ""
            if can_pullup:
                format += "[^] "
            if can_invert:
                format += "[!] "
            raise self.config_error(
                "Invalid pin description '%s'\n"
                "Format is: %s pin_name" % (pin_desc, format))
        if pin not in self.available_pins:
            raise self.config_error(
                "Requested pin '%s' is not available!" % pin)
        if pin in self.active_pins:
            pin_params = self.active_pins[pin]
            if (share_type is None and pin_params['class'] is not None) or \
                    share_type != pin_params['share_type']:
                raise self.config_error(
                    "pin %s used multiple times in config" % (pin,))
            if invert != pin_params['invert'] or pullup != pin_params['pullup']:
                raise self.config_error(
                    "Shared pin %s must have same polarity" % (pin,))
            return pin_params
        pin_params = {'chip': self,
                      'type': pin_type, 'share_type': share_type,
                      'pin': pin, 'pin_number': self.get_pin_number(pin),
                      'invert': invert, 'pullup': pullup,
                      'class': None}
        self.active_pins[pin] = pin_params
        #self.logger.debug("PIN:\n%s" % "\n".join(
        #    ["    %s: %s" % (k, v,) for k, v in pin_params.items()]))
        return pin_params


def load_config(config):
    return HostPins(config.getsection('hostpins'))
