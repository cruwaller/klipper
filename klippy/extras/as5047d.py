# AS5047D Magnetic Rotary Position Sensor Control
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import struct, types, collections
import bus
import driver.field_helpers as field_helpers

AS5047D_READ  = 0x4000
AS5047D_WRITE = 0x0000

# Masks
AS5047D_PARITY_BIT = 0b1000000000000000
AS5047D_ERROR_BIT  = 0b0100000000000000
AS5047D_DATA_BITS  = 0b0011111111111111

Registers = {
    # Volatile Registers
    "NOP"        : 0x0000,
    "ERRFL"      : 0x0001,
    "PROG"       : 0x0003,
    "DIAAGC"     : 0x3FFC,
    "MAG"        : 0x3FFD,
    "ANGLE"      : 0x3FFE, # Angle without compensation
    "ANGLECOM"   : 0x3FFF, # Compensated Angle
    # Non-Volatile Registers
    "ZPOSM"      : 0x0016, # MSB
    "ZPOSL"      : 0x0017, # LSB
    "SETTINGS1"  : 0x0018,
    "SETTINGS2"  : 0x0019,
    # Extras
    "ANGLEUNC_READ": 0x7FFE, # = cacl_parity(READ | ANGLEUNC)
    "ANGLECOM_READ": 0xFFFF, # = cacl_parity(READ | ANGLECOM)
}

Fields = {
    # Volatile Registers
    'ERRFL': {
        "FRERR"   : 0b0000000000000001,
        "INVCOMM" : 0b0000000000000010,
        "PARERR"  : 0b0000000000000100,
    },
    'DIAAGC': {
        "AGC"  : 0b0000000011111111,
        "LF"   : 0b0000000100000000,
        "COF"  : 0b0000001000000000,
        "MAGH" : 0b0000010000000000,
        "MAGL" : 0b0000100000000000,
    },
    # Non-Volatile Registers
    'ZPOSM': {
        'POSITION 8MSBs':  0b0000000011111111 # 8 MSBs
    },
    'ZPOSL': {
        'POSITION 6LSBs':  0b0000000000111111, # 6 LSBs
        'COMP_L_ERROR_EN': 0b0000000001000000,
        'COMP_H_ERROR_EN': 0b0000000010000000,
    },
    'ZERO_POS': {
        'ZPOSITION':       0b0011111111111111,
        'COMP_L_ERROR_EN': 0b0100000000000000,
        'COMP_H_ERROR_EN': 0b1000000000000000,
    },
    'SETTINGS1': {
        'DIR'     : 0b00000100, # Rotation direction
        'UVW_ABI' : 0b00001000, # Defines the PWM Output
        'DAECDIS' : 0b00010000, # Disable Dynamic Angle Error Compensation
        'ABIBIN'  : 0b00100000, # ABI decimal or binary selection
        'DS'      : 0b01000000, # ANGLECOM reg data; 0->DAECANG, 1->CORDICANG
        'PWM_ON'  : 0b10000000, # Enables PWM
    },
    'SETTINGS2': {
        'UVWPP'  : 0b00000111,
        'HYS'    : 0b00011000,
        'ABIRES' : 0b11100000,
    },
}

FieldFormatters = {
    # ERRFL register
    'FRERR':   (lambda v: '1(Framing error)' if v else ""),
    'INVCOMM': (lambda v: '1(Invalid register)' if v else ""),
    'PARERR':  (lambda v: '1(Parity error)' if v else ""),
    # DIAAGC register
    'LF':      (lambda v: '1(internal offset loop finished)' if v else ""),
    'COF':     (lambda v: '1(CORDIC overflow)' if v else ""),
    'MAGH':    (lambda v: '1(Magnetic field strength too high)' if v else ""),
    'MAGL':    (lambda v: '1(Magnetic field strength too low)' if v else ""),
    # ZPOSL register
    'ZPOSITION':       (lambda v: '%s' % int(((v & 0xff) << 6) + ((v >> 8) & 0x3f))),
    # 'ZPOSITION':       (lambda v: '%s' % v),
    'COMP_L_ERROR_EN': (lambda v: '1(MAGH enabled)' if v else "0(MAGH disabled)"),
    'COMP_H_ERROR_EN': (lambda v: '1(MAGL enabled)' if v else "0(MAGL disabled)"),
    # SETTINGS1 register
    'DIR':     (lambda v: '1(CCW)' if v else "0(CW)"),
    'UVW_ABI': (lambda v: '1(UVW)' if v else "0(ABI)"),
    'DAECDIS': (lambda v: '1(DAE OFF)' if v else "0(DAE ON)"),
    'ABIBIN':  (lambda v: '1(ABI binary)' if v else "0(ABI decimal)"),
    'DS':      (lambda v: '1(CORDICANG)' if v else "0(DAECANG)"),
    'PWM_ON':  (lambda v: '1(PWM enabled)' if v else "0(PWM disabled)"),
    # SETTINGS2 register
    'UVWPP':   (lambda v: '%s(Pole pairs)' % {
        0b000: 1, 0b001: 2, 0b010: 3, 0b011: 4,
        0b100: 5, 0b101: 6, 0b110: 7, 0b111: 7}[v]),
    'HYS':     (lambda v: '%s(Hysteresis idx)' % v),
    'ABIRES':  (lambda v: '%s(ABI resolution idx)' % v),
}


def cacl_parity(data):
    data_temp = data
    par = 0
    while data_temp:
        par ^= data_temp & 0x1
        data_temp >>= 0x1
    return (par << 15) | data


'''
for k, v in sorted(Registers.items()):
    print "R:  %-15s : cacl_parity(0x%04X) = 0x%04X" % (
        k, (AS5047D_READ | v), cacl_parity(AS5047D_READ | v))
    print "W:  %-15s : cacl_parity(0x%04X) = 0x%04X" % (
        k, (AS5047D_WRITE | v), cacl_parity(AS5047D_WRITE | v))
'''

class AS5047D:
    settings1 = 0
    settings2 = 0

    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = name = config.get_name().split()[-1]
        self.logger = self.printer.get_logger('encoder.' + name)
        # Registers
        self.regs = collections.OrderedDict()
        self.fields = field_helpers.FieldHelper(
            Fields, FieldFormatters, self.regs)
        set_field = self.fields.set_field
        # settings1
        mode = config.getchoice('mode', {'abi': 0, 'uvw': 1}, default='abi')
        set_field("UVW_ABI", mode)
        set_field("DAECDIS", config.getboolean('dae_disabled', default=False))
        if mode == 0:
            abi_mode = config.getchoice(
                'abi_mode', {'decimal': 0, 'binary': 1}, default='decimal')
            set_field("ABIBIN", abi_mode)
            if abi_mode:
                abi_res = {2048: 0, 1024: 1}
            else:
                abi_res = {2000: 0, 1600: 1, 1200: 2, 800: 3, 400:
                    4, 200: 5, 100: 6, 32: 7}
            abi_res_idx = config.getchoice(
                'abi_resolution', abi_res, default=abi_res.keys()[0])
            set_field("ABIRES", abi_res_idx)
            abi_pulses = {i: v for v, i in abi_res.items()}[abi_res_idx]
            # Set correct field formatter
            FieldFormatters['ABIRES'] = \
                (lambda v: '%s(ABI resolution)' % {
                    i: v for v, i in abi_res.items()}[v])
            if abi_pulses >= 1600:
                FieldFormatters['HYS'] = \
                    (lambda v: '%s(Hysteresis)' % {
                        0: '0.53', 1: '0.35', 2: '0.175', 3: 'NA'}[v])
            elif abi_pulses <= 1024:
                FieldFormatters['HYS'] = \
                    (lambda v: '%s(Hysteresis)' % {
                        0: '0.7', 1: '0.35', 2: 'NA', 3: '1.05'}[v])
        set_field("DS", config.getchoice('dataselect',
            {'daecang': 0, 'cordicang': 1}, default='daecang'))
        set_field("PWM_ON", config.getboolean('pwm_enabled', default=False))
        # settings2
        set_field("UVWPP", config.getint('uvw_pole_pairs',
            default=1, minval=1, maxval=7) - 1)
        set_field("HYS", config.getint('hysteresis',
            default=0, minval=0, maxval=3))
        # SPI config
        spi_mode = config.get('spi_mode', 1)
        self.spi = spi = bus.MCU_SPI_from_config(
            config, spi_mode, default_speed=8000000)
        self.mcu = spi.get_mcu()
        self._oid = spi.get_oid()
        self.mcu.register_config_callback(self._build_config_cb)
        self.printer.register_event_handler("klippy:ready", self.ready_handler)
        # Register gcode commands
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command(
            "ENCODER_DUMP", "NAME", self.name.upper(),
            self.cmd_DUMP, desc=self.cmd_DUMP_help)
        self.gcode.register_mux_command(
            "ENCODER_ERROR", "NAME", self.name.upper(),
            self.cmd_ERROR, desc=self.cmd_ERROR_help)
        self.gcode.register_mux_command(
            "ENCODER_DIAG", "NAME", self.name.upper(),
            self.cmd_DIAG, desc=self.cmd_DIAG_help)
        self.gcode.register_mux_command(
            "ENCODER_POS", "NAME", self.name.upper(),
            self.cmd_POS, desc=self.cmd_POS_help)
    # ============ GCODE   ===============
    cmd_DUMP_help = "ENCODER_DUMP NAME="
    def cmd_DUMP(self, params):
        msg = self.diagnostic()
        self.gcode.respond_info(msg)
    cmd_ERROR_help = "ENCODER_ERROR NAME="
    def cmd_ERROR(self, params):
        self.gcode.respond_info(self.get_error())
    cmd_DIAG_help = "ENCODER_DIAG NAME="
    def cmd_DIAG(self, params):
        self.gcode.respond_info(self.get_diag_and_agc())
    cmd_POS_help = "ENCODER_POS NAME= [COMP=]"
    def cmd_POS(self, params):
        _t = self.gcode.get_int('COMP', params, default=1, minval=0, maxval=1)
        func = [self.get_position, self.get_position_compensated][_t]
        self.gcode.respond_info("Current position: %s" % func())
    # ============ CONTROL ===============
    @staticmethod
    def __get_parity(val):
        return int(bool(val & AS5047D_PARITY_BIT))
    def __check_error(self, val):
        if val & AS5047D_ERROR_BIT:
            self.logger.error("Error bit set!")
            return self.get_error()
        return ""
    def get_error(self, val=None):
        # ERRFL register
        if val is None:
            val = self.read("ERRFL")
        msg = self.fields.pretty_format('ERRFL', val)
        if val:
            self.logger.error(msg)
        return msg
    def get_diag_and_agc(self, val=None):
        # DIAAGC register
        resp = []
        if val is None:
            val = self.read("DIAAGC")
            err = self.__check_error(val)
            if err:
                resp.append(err)
        resp.append(self.fields.pretty_format('DIAAGC', val))
        msg = "\n".join(resp)
        self.logger.info(msg)
        return msg
    def diagnostic(self):
        resp = [
            '--------------------------------------------------',
            '--- AS5047 diagnostic']
        self.__transfer('DIAAGC')
        diaagc = self.__transfer('ERRFL')
        resp.append(self.get_diag_and_agc(diaagc))
        errfl = self.__transfer('MAG')
        resp.append(self.get_error(errfl))
        mag = self.__transfer('ANGLECOM')
        resp.append("%-11s %08x" % ("MAG", mag))
        anglecom = self.__transfer('ANGLE')
        resp.append("%-11s %08x" % ("ANGLECOM", anglecom))
        angle = self.__transfer('SETTINGS1')
        resp.append("%-11s %08x" % ("ANGLE", angle))
        settings1 = self.__transfer('SETTINGS2')
        resp.append(self.fields.pretty_format('SETTINGS1', settings1))
        settings2 = self.__transfer('ZPOSM')
        resp.append(self.fields.pretty_format('SETTINGS2', settings2))
        zposm = self.__transfer('ZPOSL')
        zposl = self.__transfer('ANGLECOM_READ', parity=False)
        resp.append(self.fields.pretty_format(
            'ZERO_POS', ((zposl << 8) + zposm)))
        resp.append('--------------------------------------------------')
        msg = "\n".join(resp)
        self.logger.info(msg)
        return msg
    def get_position(self):
        return self.read('ANGLE')
    def get_position_compensated(self):
        return self.read('ANGLECOM')
    # ============ SETUP ===============
    def _build_config_cb(self):
        pass
    def ready_handler(self):
        if not self.mcu.is_shutdown():
            # configure encoder
            for reg_name, val in self.regs.items():
                self.write(reg_name, val)
    # ============ SPI ===============
    conv32 = struct.Struct('>I').pack # uint32
    conv16 = struct.Struct('>H').pack # uint16
    def write(self, cmd, val):
        cmd = Registers.get(cmd, cmd)
        cmd |= AS5047D_WRITE
        _cmd = list(self.conv16(cacl_parity(cmd)))
        _val = list(self.conv16(val))
        self.logger.debug("==>> WRITE 0x%04X : 0x%04X" % (cmd, val))
        self.spi.spi_send(_cmd)
        self.spi.spi_send(_val)
    def read(self, cmd, parity=True):
        cmd = Registers.get(cmd, cmd)
        _cmd = cmd | AS5047D_READ
        if parity:
            _cmd = cacl_parity(_cmd)
        _cmd = list(self.conv16(_cmd))
        self.spi.spi_send(_cmd)
        val = self.__transfer([0, 0])
        self.logger.debug("<<== READ 0x%04X : 0x%04X" % (cmd, val))
        return val
    def __transfer(self, cmd, parity=True, read=True):
        if not isinstance(cmd, types.ListType):
            cmd = Registers.get(cmd, cmd)
            if parity:
                cmd |= AS5047D_READ if read else AS5047D_WRITE
                cmd = cacl_parity(cmd)
            cmd = list(self.conv16(cmd))
        params = self.spi.spi_transfer(cmd)
        res = list(bytearray(params['response']))
        self.logger.debug("<<== TRANSFER %s : %s" % (cmd, res))
        value = 0
        for b in res:
            value <<= 8
            value |= int(b)
        return value & AS5047D_DATA_BITS


def load_config_prefix(config):
    return AS5047D(config)
