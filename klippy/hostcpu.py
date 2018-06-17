import os, re
import pins

(sysname, nodename, release, version, machine) = os.uname()

GPIO = None

if machine in ["x86_64", "x86"]:
    class GpioTemp:
        PUD_UP = PUD_DOWN = 0
        IN = OUT = 0
        RISING = FALLING = BOTH = 0
        def __init__(self, *args, **kwargs):
            pass
        def setup(self, *args, **kwargs):
            pass
        def cleanup(self, *args, **kwargs):
            pass
        def input(self, *args, **kwargs):
            return 0
        def add_event_detect(self, *args, **kwargs):
            pass
        def remove_event_detect(self, *args, **kwargs):
            pass
    GPIO = GpioTemp()
else:
    if nodename == "raspberrypi":
        try:
            import RPi.GPIO as GPIO
        except RuntimeError:
            raise Exception("Error importing RPi.GPIO! "
                            "This is probably because you need superuser privileges. "
                            "You can achieve this by using 'sudo' to run your script")
        except ImportError:
            raise Exception("Error importing OPi.GPIO!"
                            "Try to install it first: pip install OPi.GPIO")
    elif "orangepi" in nodename:
        try:
            import OPi.GPIO as GPIO
        except ImportError:
            raise Exception("Error importing OPi.GPIO!"
                            "Try to install it first: pip install OPi.GPIO")

if GPIO is not None:
    try:
        PWM = GPIO.PWM
    except AttributeError:
        PWM = None

class HostGpioIn:
    def __init__(self, pin_params):
        self.invert = pin_params['invert']
        self.channel = channel = \
            pin_params['chip'].get_pin_number(pin_params['pin'])
        pull_up_down = GPIO.PUD_UP if pin_params['pullup'] else GPIO.PUD_DOWN
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
    def read(self):
        if self.invert:
            return not GPIO.input(self.channel)
        return GPIO.input(self.channel)

class HostGpioEvent:
    def __init__(self, pin_params):
        self.channel = channel = \
            pin_params['chip'].get_pin_number(pin_params['pin'])
        pull_up_down = GPIO.PUD_UP if pin_params['pullup'] else GPIO.PUD_DOWN
        GPIO.setup(channel, GPIO.IN, pull_up_down=pull_up_down)
    def __del__(self):
        GPIO.remove_event_detect(self.channel)
    def read(self):
        return GPIO.input(self.channel)
    def set_event(self, cb, edge, bounce=10):
        eventc = {"rising": GPIO.RISING,
                  "falling": GPIO.FALLING,
                  "both": GPIO.BOTH}
        GPIO.add_event_detect(self.channel, eventc[edge],
                              callback=cb, bouncetime=bounce)

class HostGpioOut:
    def __init__(self, pin_params):
        self.invert = invert = pin_params['invert']
        state = GPIO.HIGH if invert else GPIO.LOW
        self.channel = channel = \
            pin_params['chip'].get_pin_number(pin_params['pin'])
        GPIO.setup(channel, GPIO.OUT,
                   pull_up_down=GPIO.PUD_OFF, initial=state)
    def write(self, state):
        state = not state if self.invert else state
        GPIO.output(self.channel, state)

class HostPwm:
    def __init__(self, pin_params):
        if PWM is None:
            raise Exception("PWM is not supported!")
        self.freq = 1.
        channel = pin_params['chip'].get_pin_number(pin_params['pin'])
        GPIO.setup(channel, GPIO.OUT)
        self.pwm = GPIO.PWM(channel, self.freq)
    def set_freq(self, freq):
        self.freq = freq
        self.pwm.ChangeFrequency(freq)
    def write(self, duty, freq=None):
        if freq is not None:
            self.pwm.ChangeFrequency(freq)
        self.pwm.start(duty)
    def stop(self):
        self.pwm.stop()

class HostSpi:
    def __init__(self):
        raise Exception("Host SPI is not implemented yet")

class HostI2C:
    def __init__(self):
        raise Exception("Host I2C is not implemented yet")

class HostCpu(object):
    def __init__(self, printer):
        try:
            GPIO.setmode(GPIO.BOARD)
        except AttributeError:
            if machine not in ["x86_64", "x86"]:
                raise printer.config_error("GPIO is not defined")
        self.printer = printer
        self.logger = printer.logger.getChild("hostcpu")
        self.active_pins = {}
        pinmap = {}
        if nodename == "raspberrypi":
            if machine == "armv7l":
                pinmap = self._rpi_v2_pins()
            elif machine == "armv6l":
                pinmap = self._rpi_v1_pins()
        elif "orangepi" in nodename:
            pinmap = self._rpi_v2_pins()
        self.available_pins = pinmap
        # printer.add_object("hostcpu", self)
    def __del__(self):
        if GPIO is not None:
            GPIO.cleanup()
    def _rpi_v1_pins(self):
        return {"P%d" % i: i for i in [2, 3, 4, 17, 27, 22, 10, 9, 11,
                                       14, 15, 18, 23, 24, 25, 8, 7]}
    def _rpi_v2_pins(self):
        gpios = self._rpi_v1_pins()
        gpios.update({"P%d" % i: i for i in [5, 6, 13, 19, 26,
                                            12, 16, 20, 21]})
        return gpios
    def _orangepi_pins(self):
        return self._rpi_v2_pins()
    def get_pin_number(self, pin):
        return self.available_pins.get(pin, None)
    def setup_pin(self, pin_type, pin_desc):
        pin_params = self.lookup_pin(pin_type, pin_desc)
        pcs = {'gpio_in': HostGpioIn, 'gpio_out': HostGpioOut,
               'spi': HostSpi, 'i2c': HostI2C, 'pwm': HostPwm,
               'event': HostGpioEvent}
        pin_type = pin_params['type']
        if pin_type not in pcs:
            raise pins.error("pin type %s not supported on mcu" % (pin_type,))
        co = pcs[pin_type](self, pin_params)
        return co
    def lookup_pin(self, pin_type, pin_desc, share_type=None):
        if pin_type not in ['gpio_in', 'gpio_out', 'spi', 'i2c', 'pwm', 'event']:
            raise pins.error("Invalid pin type %s" % pin_type)
        can_invert = pin_type in ['gpio_in', 'gpio_out']
        can_pullup = pin_type == 'gpio_in'
        desc = pin_desc
        pullup = invert = 0
        if can_pullup and '^' in desc:
            pullup = 1
        if can_invert and '!' in desc:
            invert = 1
        pin = re.sub('[\^!]', '', desc).strip()
        if [c for c in '^!: ' if c in pin]:
            format = ""
            if can_pullup:
                format += "[^] "
            if can_invert:
                format += "[!] "
            raise pins.error(
                "Invalid pin description '%s'\n"

                "Format is: %s pin_name" % (pin_desc, format))
        if pin in self.active_pins:
            pin_params = self.active_pins[pin]
            if (share_type is None and pin_params['class'] is not None) or \
                    share_type != pin_params['share_type']:
                raise pins.error(
                    "pin %s used multiple times in config" % (pin,))
            if invert != pin_params['invert'] or pullup != pin_params['pullup']:
                raise pins.error(
                    "Shared pin %s must have same polarity" % (pin,))
            return pin_params
        pin_params = {'chip': self, 'chip_name': "hostcpu",
                      'type': pin_type, 'share_type': share_type,
                      'pin': pin, 'invert': invert, 'pullup': pullup,
                      'class': None}
        self.active_pins[pin] = pin_params
        return pin_params


def add_printer_objects(printer, config):
    printer.add_object('hostcpu', HostCpu(printer))
