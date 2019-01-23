# Accelerometer support
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import extras.bus as bus
import pins


class Accelerometer:
    def __init__(self, config, spi_mode):
        self.name = config.get_name()[13:].strip()
        self.printer = config.get_printer()
        self.logger = self.printer.get_logger('accelerometer.' + self.name)
        self.gcode = self.printer.lookup_object("gcode")
        self.spi = bus.MCU_SPI_from_config(
            config, spi_mode, default_speed=2000000)
        # Allow virtual endstop to be created
        self.pin = config.get('pin', None)
        self.ppins = ppins = self.printer.lookup_object("pins")
        ppins.register_chip("accelerometer_" + self.name, self)
        self.printer.register_event_handler("klippy:ready", self.ready_handler)
        # Register gcode commands
        self.gcode.register_mux_command(
            "ACCELEROMETER_POS", "NAME", self.name,
            self.cmd_POSITIONS, desc=self.cmd_POSITIONS_help)
        self.gcode.register_mux_command(
            "ACCELEROMETERL_STATUS", "NAME", self.name,
            self.cmd_STATUS, desc=self.cmd_STATUS_help)
    # Endstop support
    def setup_pin(self, pin_type, pin_params):
        if pin_type != 'endstop' and pin_params['pin'] != 'virtual_endstop':
            raise self.ppins.error("virtual endstop is only supported")
        self.prepare_pin(pin_params)
        return VirtualEndstop(self)
    def status(self):
        return "None"
    def get_position_str(self):
        return "None"
    def send(self, cmd):
        self.spi.spi_send(cmd)
    def transfer(self, cmd, skip=0, convert=True, reverse=False):
        params = self.spi.spi_transfer(cmd)
        res = list(bytearray(params['response']))[skip:]
        if not convert:
            return res
        if reverse:
            res.reverse()
        value = 0
        for b in res:
            value <<= 8
            value |= int(b)
        return value
    def prepare_pin(self, pin_params):
        pass
    def ready_handler(self):
        pass
    def home_prepare(self, *args):
        pass
    def home_finalize(self):
        pass
    cmd_POSITIONS_help = "ACCELEROMETER_POS NAME=name"
    def cmd_POSITIONS(self, params):
        self.gcode.respond(self.get_position_str())
    cmd_STATUS_help = "ACCELEROMETER_STATUS NAME=name"
    def cmd_STATUS(self, params):
        self.gcode.respond(self.status())


# Endstop wrapper
class VirtualEndstop:
    def __init__(self, accelerometer):
        self.prepare_done = False
        self.logger = accelerometer.logger
        self.accelerometer = accelerometer
        if accelerometer.pin is None:
            raise pins.error("Accelerometer virtual endstop requires pin")
        ppins = accelerometer.printer.lookup_object('pins')
        self.mcu_endstop = mcu_endstop = ppins.setup_pin(
            'endstop', accelerometer.pin)
        if mcu_endstop.get_mcu() is not accelerometer.spi.get_mcu():
            raise pins.error("Accelerometer virtual endstop must be on same mcu")
        # Wrappers to MCU_endstop class
        self.get_mcu = mcu_endstop.get_mcu
        self.add_stepper = mcu_endstop.add_stepper
        self.get_steppers = mcu_endstop.get_steppers
        self.home_start = mcu_endstop.home_start
        self.home_wait = mcu_endstop.home_wait
        self.query_endstop = mcu_endstop.query_endstop
        self.query_endstop_wait = mcu_endstop.query_endstop_wait
        self.TimeoutError = mcu_endstop.TimeoutError
    def home_prepare(self, *args):
        if self.prepare_done:
            return
        self.accelerometer.home_prepare(*args)
        self.mcu_endstop.home_prepare()
        self.prepare_done = True
    def home_finalize(self):
        if not self.prepare_done:
            return
        self.accelerometer.home_finalize()
        self.mcu_endstop.home_finalize()
        self.prepare_done = False


class ADXL345(Accelerometer):
    REG_DEVID          = 0x00    # R,     11100101,   Device ID
    REG_THRESH_TAP     = 0x1D    # R/W,   00000000,   Tap threshold
    REG_OFSX           = 0x1E    # R/W,   00000000,   X-axis offset
    REG_OFSY           = 0x1F    # R/W,   00000000,   Y-axis offset
    REG_OFSZ           = 0x20    # R/W,   00000000,   Z-axis offset
    REG_DUR            = 0x21    # R/W,   00000000,   Tap duration
    REG_LATENT         = 0x22    # R/W,   00000000,   Tap latency
    REG_WINDOW         = 0x23    # R/W,   00000000,   Tap window
    REG_THRESH_ACT     = 0x24    # R/W,   00000000,   Activity threshold
    REG_THRESH_INACT   = 0x25    # R/W,   00000000,   Inactivity threshold
    REG_TIME_INACT     = 0x26    # R/W,   00000000,   Inactivity time
    REG_ACT_INACT_CTL  = 0x27    # R/W,   00000000,   Axis enable control for activity and inactiv ity detection
    REG_THRESH_FF      = 0x28    # R/W,   00000000,   Free-fall threshold
    REG_TIME_FF        = 0x29    # R/W,   00000000,   Free-fall time
    REG_TAP_AXES       = 0x2A    # R/W,   00000000,   Axis control for single tap/double tap
    REG_ACT_TAP_STATUS = 0x2B    # R,     00000000,   Source of single tap/double tap
    REG_BW_RATE        = 0x2C    # R/W,   00001010,   Data rate and power mode control
    REG_POWER_CTL      = 0x2D    # R/W,   00000000,   Power-saving features control
    REG_INT_ENABLE     = 0x2E    # R/W,   00000000,   Interrupt enable control
    REG_INT_MAP        = 0x2F    # R/W,   00000000,   Interrupt mapping control
    REG_INT_SOUCE      = 0x30    # R,     00000010,   Source of interrupts
    REG_DATA_FORMAT    = 0x31    # R/W,   00000000,   Data format control
    REG_DATAX0         = 0x32    # R,     00000000,   X-Axis Data 0
    REG_DATAX1         = 0x33    # R,     00000000,   X-Axis Data 1
    REG_DATAY0         = 0x34    # R,     00000000,   Y-Axis Data 0
    REG_DATAY1         = 0x35    # R,     00000000,   Y-Axis Data 1
    REG_DATAZ0         = 0x36    # R,     00000000,   Z-Axis Data 0
    REG_DATAZ1         = 0x37    # R,     00000000,   Z-Axis Data 1
    REG_FIFO_CTL       = 0x38    # R/W,   00000000,   FIFO control
    REG_FIFO_STATUS    = 0x39    # R,     00000000,   FIFO status

    def __convert_value(self, value, raw=False):
        if raw:
            return value
        bits = self.resolution_in_bits
        # check the sign bit
        if (value & (1 << (bits - 1))) != 0:
            # compute negative value
            value = value - (1 << bits)
        return value * self.filter_ratio

    def __init__(self, config):
        Accelerometer.__init__(self, config, spi_mode=3)
        # Full resolution mode changes threshold values to 4mg/LSB
        #   instaed of 62.5mg/LSB
        self.full_res = config.getboolean('full_res', True)
        # DC = absolute threshold value, AC = referenced threshold
        self.isr_act_absolute = config.getboolean(
            'isr_active_absolute', default=True)
        self.isr_inact_absolute = config.getboolean(
            'isr_inactive_absolute', default=True)
        self.act_threshold(config.getint(
            'isr_active_threshold', default=1, above=0, maxval=0xff))
        self.inact_threshold(config.getint(
            'isr_inactive_threshold', default=1, above=0, maxval=0xff))
        self.inact_timer(config.getint(
            'isr_inactive_timer', default=0, minval=0, maxval=0xffff))
        self.isr_type = config.getchoice(
            'isr_type', ['both', 'active', 'inactive'], default='active')
        self.filter_range = config.getchoice(
            'filter_range', ['2g', '4g', '8g', '16g'], default='2g')
        filter_odr = config.getchoice(
            'filter_ord', ['12.5Hz', '25Hz', '50Hz',
                           '100Hz', '200Hz', '400Hz'], default='400Hz')
        self.filter_odr = 1. / float(filter_odr[:-2])
        # Fixed 10bit resolution in normal mode
        self.resolution_in_bits = 10
        if self.full_res:
            # up to 13bits in full resolution mode (4mg/LSB)
            self.resolution_in_bits = 13
        self.filter_ratio = (2 * float(self.filter_range[:-1])
                             / float(1 << self.resolution_in_bits))
        # Configure:
        self.int_map(self.isr_type)
        self.int_source(self.isr_type)
        self.bw_rate(filter_odr)
        if self.pin is None:
            self.data_format(range=self.filter_range, full_res=self.full_res)

    def prepare_pin(self, pin_params):
        self.data_format(invert=pin_params['invert'],
                         range=self.filter_range,
                         full_res=self.full_res)
    def ready_handler(self):
        pass
    def home_prepare(self, *args):
        self.int_enable(self.isr_type)
        self.power_control(measure='measure')
    def home_finalize(self):
        self.int_enable()
        self.power_control()
    def get_position_str(self):
        x, y, z = self.read_all()
        return "Current measurement results:" \
               "\nX=%.4fmg\nY=%.4fmg\nZ=%.4fmg" % (x, y, z)

    def start(self):
        self.power_control(measure='measure')
    def read_all(self, raw=False):
        result = self.transfer([
            self.REG_DATAX0,
            0x00, 0x00, # X
            0x00, 0x00, # Y
            0x00, 0x00, # Z
        ], skip=1, convert=False)
        x = self.__convert_value((int(result[1]) << 8) + int(result[0]), raw)
        y = self.__convert_value((int(result[3]) << 8) + int(result[2]), raw)
        z = self.__convert_value((int(result[5]) << 8) + int(result[4]), raw)
        return x, y, z

    def get_x(self, raw=False):
        value = self.transfer(
            [self.REG_DATAX0, 0x00, 0x00], skip=1, reverse=True)
        return self.__convert_value(value, raw)
    def get_y(self, raw=False):
        value = self.transfer(
            [self.REG_DATAY0, 0x00, 0x00], skip=1, reverse=True)
        return self.__convert_value(value, raw)
    def get_z(self, raw=False):
        value = self.transfer(
            [self.REG_DATAZ0, 0x00, 0x00], skip=1, reverse=True)
        return self.__convert_value(value, raw)

    def act_threshold(self, threshold):
        # The scale factor is 62.5 mg/LSB
        self.send([self.REG_THRESH_ACT, threshold & 0xff])
    def inact_threshold(self, threshold):
        # The scale factor is 62.5 mg/LSB
        self.send([self.REG_THRESH_INACT, threshold & 0xff])
    def inact_timer(self, time):
        # The scale factor is 1 sec/LSB
        self.send([self.REG_TIME_INACT, time & 0xff])
    def act_inact_control(self, type='disabled',
                          absolute_overthreshold=True, absolute_underthreshold=True):
        regorig = None
        reg = 0  # set to default
        if type not in ['init', 'disabled']:
            reg = regorig = self.transfer([self.REG_ACT_INACT_CTL, 0x00], skip=1)
            if 'overthreshold' in type or type == 'both':
                reg &= 0b00001111
                reg |= (not absolute_overthreshold) << 7
                reg |= 0b111 << 4 # enabled XYZ
            if 'underthreshold' in type or type == 'both':
                reg &= 0b11110000
                reg |= (not absolute_underthreshold) << 3
                reg |= 0b111 # enabled XYZ
        if regorig != reg:
            self.send([self.REG_ACT_INACT_CTL, reg])
    def bw_rate(self, rate, low_power=False):
        reg = {'12.5Hz': 0b0111, '25Hz': 0b1000, '50Hz': 0b1001,
               '100Hz': 0b1010, '200Hz': 0b1011, '400Hz': 0b1100}[rate]
        reg |= (low_power << 4)
        self.send([self.REG_BW_RATE, reg])
    def power_control(self, measure='standby', link=True,
                      wakeup=False, autosleep=False):
        reg  = link << 5
        reg |= autosleep << 4
        reg |= {'standby': 0b0, 'measure': 0b1}[measure] << 3
        #reg |= sleep << 2
        #reg |= wakeup # limited bw activity detection in standby
        self.send([self.REG_POWER_CTL, reg])

    @staticmethod
    def __int_handle(reg, type):
        if type == 'disable':
            reg = (reg & ~(0b11 << 3))
        else:
            if type in ['overthreshold', 'active', 'both']:
                reg = (reg & ~(1 << 4)) | 1 << 4
            if type == ['underthreshold', 'inactive', 'both']:
                reg = (reg & ~(1 << 3)) | 1 << 3
        return reg
    reg_int_enable = 0
    def int_enable(self, type='disable'):
        reg = self.__int_handle(self.reg_int_enable, type)
        self.send([self.REG_INT_ENABLE, reg])
        self.reg_int_enable = reg
    reg_int_map = 0
    def int_map(self, type='disable'):
        reg = self.__int_handle(self.reg_int_map, type)
        self.send([self.REG_INT_MAP, reg])
        self.reg_int_map = reg
    reg_int_source = 0
    def int_source(self, type='disable'):
        reg = self.__int_handle(self.reg_int_source, type)
        self.send([self.REG_INT_SOUCE, reg])
        self.reg_int_source = reg
    reg_data_format = 0
    def data_format(self, invert=False, full_res=True,
                    justify=False, range='2g'):
        reg = self.reg_data_format
        reg = (reg & ~(1 << 5)) | invert << 5
        reg = (reg & ~(1 << 3)) | full_res << 3
        reg = (reg & ~(1 << 2)) | justify << 2
        reg = (reg & ~0b11) | {'2g': 0b00, '4g': 0b01, '8g': 0b10, '16g': 0b11}[range]
        self.send([self.REG_DATA_FORMAT, reg])
        self.reg_data_format = reg


class ADXL362(Accelerometer):
    # Commands
    WRITE = 0xA
    READ  = 0xB
    READ_FIFO = 0xD
    # Registers
    DEVID_AD = 0x0
    DEVID_MST = 0x1
    PARTID = 0x2
    REVID = 0x3
    XDATA = 0x8
    YDATA = 0x9
    ZDATA = 0xA
    STATUS = 0xB
    FIFO_ENTRIES_L = 0x0C
    FIFO_ENTRIES_H = 0x0D
    XDATA_L = 0x0E
    XDATA_H = 0x0F
    YDATA_L = 0x10
    YDATA_H = 0x11
    ZDATA_L = 0x12
    ZDATA_H = 0x13
    TEMP_L = 0x14
    TEMP_H = 0x15
    SOFT_RESET = 0x1F
    THRESH_ACT_L = 0x20
    THRESH_ACT_H = 0x21
    TIME_ACT = 0x22
    THRESH_INACT_L = 0x23
    THRESH_INACT_H = 0x24
    TIME_INACT_L = 0x25
    TIME_INACT_H = 0x26
    ACT_INACT_CTL = 0x27
    FIFO_CONTROL = 0x28
    FIFO_SAMPLES = 0x29
    INTMAP1 = 0x2A
    INTMAP2 = 0x2B
    FILTER_CTL = 0x2C
    POWER_CTL = 0x2D

    def __init__(self, config):
        Accelerometer.__init__(self, config, spi_mode=0)
        # Init local variables
        self.odr = 1. / 100.
        # Config commands
        self.reset()
        self.act_threshold(config.getint(
            'isr_active_threshold', default=0, minval=0, maxval=0x7ff))
        self.act_timer(config.getint(
            'isr_active_timer', default=0, minval=0, maxval=0xff))
        self.isr_act_absolute = config.getboolean(
            'isr_active_absolute', default=True)
        self.inact_threshold(config.getint(
            'isr_inactive_threshold', default=0, minval=0, maxval=0x7ff))
        self.inact_timer(config.getint(
            'isr_inactive_timer', default=0, minval=0, maxval=0xffff))
        self.isr_inact_absolute = config.getboolean(
            'isr_inactive_absolute', default=True)
        self.isr_type = config.getchoice(
            'isr_type', ['both', 'active', 'inactive'], default='active')
        filter_range = config.getchoice(
            'filter_range', ['2g', '4g', '8g'], default='2g')
        filter_odr = config.getchoice(
            'filter_ord', ['12.5Hz', '25Hz', '50Hz',
                           '100Hz', '200Hz', '400Hz'], default='400Hz')
        self.filter_control(range=filter_range, odr=filter_odr)
        self.filter_ratio = 2 * float(filter_range[:-1]) / float(1 << 12)
    # Internals
    def __convert_value(self, value, raw=False):
        if raw:
            return value
        # check the sign bit (bit11)
        if (value & (1 << 11)) != 0:
            # compute negative value
            value = value - (1 << 12)
        return value * self.filter_ratio
    def prepare_pin(self, pin_params):
        act = self.isr_type in ['active', 'both']
        inact = self.isr_type in ['inactive', 'both']
        self.interrupt_map(bank=0, invert=pin_params['invert'],
                           inact=inact, act=act)
    def ready_handler(self):
        pass
    def home_prepare(self, *args):
        self.act_inact_control(type=self.isr_type,
            absolute_overthreshold=self.isr_act_absolute,
            absolute_underthreshold=self.isr_inact_absolute)
        self.power_control(measure='measure')
    def home_finalize(self):
        self.act_inact_control()
        self.power_control(measure='standby')
    # Configurations
    def start(self):
        self.power_control(measure='measure')
    def read_all(self, raw=False):
        result = self.transfer([
            self.READ, self.XDATA_L,
            0x00, 0x00, # X
            0x00, 0x00, # Y
            0x00, 0x00, # Z
            0x00, 0x00  # Temperature
        ], skip=2, convert=False)
        x = self.__convert_value((int(result[1]) << 8) + int(result[0]), raw=raw)
        y = self.__convert_value((int(result[3]) << 8) + int(result[2]), raw=raw)
        z = self.__convert_value((int(result[5]) << 8) + int(result[4]), raw=raw)
        t = self.__convert_value((int(result[7]) << 8) + int(result[6]), raw=raw)
        return x, y, z, t

    def get_position_str(self):
        x, y, z, t = self.read_all()
        return "Current measurement results:" \
               "\nX=%.4fmg\nY=%.4fmg\nZ=%.4fmg" \
               "\nTemperature=%s" % (x, y, z, t)
    def status(self):
        """
            STATUS_ERR = 1 << 7
            STATUS_AWAKE = 1 << 6
            STATUS_INACT = 1 << 5
            STATUS_ACT = 1 << 4
            STATUS_FIFO_OVER_RUN = 1 << 3
            STATUS_FIFO_WATER_MARK = 1 << 2
            STATUS_FIFO_READY = 1 << 1
            STATUS_DATA_READY = 1 << 0
        """
        resp = ['Status:']
        status = self.transfer(
            [self.READ, self.STATUS, 0x00], skip=2)
        if status & 0x1:
            resp.append("Data is ready")
        if status & 0x2:
            resp.append("FIFO is ready")
        if status & 0x4:
            resp.append("FIFO watermark")
        if status & 0x8:
            resp.append("FIFO overrun")
        if status & 0x10:
            resp.append("Activity detected")
        if status & 0x20:
            resp.append("Inactivitiy detected")
        # Active (1) or inactive (0) state
        resp.append("State: %s" % ['Inactive', 'Active'][bool(status & 0x40)])
        if status & 0x80:
            resp.append("Error state")
        return "\n".join(resp)
    def fifo_entries(self):
        samples = self.transfer(
            [self.READ, self.FIFO_ENTRIES_L, 0x00, 0x00], skip=2, reverse=True)
        return samples & 0x3ff
    def get_x(self, raw=False):
        value = self.transfer(
            [self.READ, self.XDATA_L, 0x00, 0x00], skip=2, reverse=True)
        return self.__convert_value(value, raw=raw)
    def get_y(self, raw=False):
        value = self.transfer(
            [self.READ, self.YDATA_L, 0x00, 0x00], skip=2, reverse=True)
        return self.__convert_value(value, raw=raw)
    def get_z(self, raw=False):
        value = self.transfer(
            [self.READ, self.ZDATA_L, 0x00, 0x00], skip=2, reverse=True)
        return self.__convert_value(value, raw=raw)
    def get_temperature(self, raw=False):
        value = self.transfer(
            [self.READ, self.TEMP_L, 0x00, 0x00], skip=2, reverse=True)
        return self.__convert_value(value, raw=raw)
    def reset(self):
        self.send([self.WRITE, self.SOFT_RESET, ord('R')])
    def act_threshold(self, threshold):
        # THRESH_ACT[g] = THRESH_ACT[codes] / Sensitivity[codes per g]
        self.send([self.WRITE, self.THRESH_ACT_L,
                   threshold & 0xFF, (threshold >> 8) & 0x7])
    def act_timer(self, time, raw=False):
        # time in s = TIME_ACT / ODR
        if not raw:
            time = int(time / self.odr)
        if time > 0xff:
            raise self.printer.config_error("activity timer is too long!")
        self.send([self.WRITE, self.TIME_ACT, time])
    def inact_threshold(self, threshold):
        # THRESH_INACT[g] = THRESH_INACT[codes] / Sensitivity[codes per g]
        self.send([self.WRITE, self.THRESH_INACT_L,
                   threshold & 0xff, (threshold >> 8) & 0x7])
    def inact_timer(self, time, raw=False):
        # time in s = TIME_INACT / ODR
        if not raw:
            time = int(time / self.odr)
        if time > 0xffff:
            raise self.printer.config_error("inactivity timer is too long!")
        self.send([self.WRITE, self.TIME_INACT_L, time & 0xff, time >> 8])
    def act_inact_control(self, type='disabled', mode=None,
                          absolute_overthreshold=True, absolute_underthreshold=True):
        regorig = None
        if mode == 'init':
            reg = 0 # set to default
        else:
            reg = regorig = self.transfer([self.READ, self.ACT_INACT_CTL, 0x00], skip=2)
            if mode is not None:
                mode = {'default': 0b00, 'linked': 0b01, 'loop': 0b11}[mode]
                reg = (reg & ~(0b11 << 4)) | mode << 4
            if type == 'disabled':
                reg &= 0b11110000
            else:
                if type in ['overthreshold', 'active', 'both']:
                    reg &= 0b11111100
                    reg |= (not absolute_overthreshold) << 1
                    reg |= 1  # enabled
                if type in ['underthreshold', 'inactive', 'both']:
                    reg &= 0b11110011
                    reg |= (not absolute_underthreshold) << 3
                    reg |= 1 << 2 # enabled
        if regorig != reg:
            self.send([self.WRITE, self.ACT_INACT_CTL, reg])
    def fifo_control(self, above_half=False, temperature=False, mode='disabled'):
        reg = (above_half << 3)
        reg |= (temperature << 2)
        reg |= {'disabled': 0b00, 'oldest': 0b01,
                'stream': 0b10, 'triggered': 0b11}[mode]
        self.send([self.WRITE, self.FIFO_CONTROL, reg])
    def fifo_samples(self, samples=0x80):
        # range 0 ... 511
        # above_half in fifo control is used as a MSB
        self.send([self.WRITE, self.FIFO_SAMPLES, samples])
    def interrupt_map(self, bank, invert=False, awake=False, inact=False,
                      act=False, fifo_over_run=False, fifo_water_mark=False,
                      fifo_ready=False, data_ready=False):
        addr = [self.INTMAP1, self.INTMAP2][bank - 1]
        reg  = invert << 7
        reg |= awake << 6
        reg |= inact << 5
        reg |= act << 4
        reg |= fifo_over_run << 3
        reg |= fifo_water_mark << 2
        reg |= fifo_ready << 1
        reg |= data_ready
        self.send([self.WRITE, addr, reg])
    def filter_control(self, range='2g', half_bw=True,
                       ext_sample=False, odr='100Hz'):
        reg = ({'2g': 0b00, '4g': 0b01, '8g': 0b10}[range]) << 6
        reg |= (half_bw << 4)
        reg |= (ext_sample << 3)
        reg |= {'12.5Hz': 0b000, '25Hz': 0b001, '50Hz': 0b010,
                '100Hz': 0b011, '200Hz': 0b100, '400Hz': 0b101}[odr]
        self.send([self.WRITE, self.FILTER_CTL, reg])
        # calculate converter
        self.odr = 1. / float(odr[:-2])
    def power_control(self, measure='standby', noise='normal',
                      wakeup=False, autosleep=False):
        reg = {'normal': 0b00, 'low': 0b01, 'ultra_low': 0b10}[noise] << 4
        reg |= (wakeup << 3) # limited bw activity detection, standby
        reg |= (autosleep << 2)
        reg |= {'standby': 0b00, 'measure': 0b10}[measure]
        self.send([self.WRITE, self.POWER_CTL, reg])


def load_config_prefix(config):
    _type = config.get('type')
    if _type.upper() == 'ADXL362':
        return ADXL362(config)
    return ADXL345(config)
