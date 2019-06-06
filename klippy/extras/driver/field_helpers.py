# TMC2208 UART communication and configuration
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math, logging, collections

######################################################################
# Field helpers
######################################################################

# Return the position of the first bit set in a mask
def ffs(mask):
    return (mask & -mask).bit_length() - 1

class FieldHelper:
    def __init__(self, all_fields, signed_fields=[], field_formatters={},
                 registers=None):
        self.all_fields = all_fields
        self.signed_fields = {sf: 1 for sf in signed_fields}
        self.field_formatters = field_formatters
        self.registers = registers
        if self.registers is None:
            self.registers = collections.OrderedDict()
        self.field_to_register = { f: r for r, fields in self.all_fields.items()
                                   for f in fields }
    def set_reg_value(self, reg_name, new_value):
        self.registers[reg_name] = new_value
    def get_reg_value(self, reg_name):
        return self.registers.get(reg_name, 0)
    def get_field(self, field_name, reg_value=None, reg_name=None):
        # Returns value of the register field
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers[reg_name]
        mask = self.all_fields[reg_name][field_name]
        field_value = (reg_value & mask) >> ffs(mask)
        if field_name in self.signed_fields and ((reg_value & mask) << 1) > mask:
            field_value -= (1 << field_value.bit_length())
        return field_value
    def set_field(self, field_name, field_value, reg_value=None, reg_name=None):
        # Returns register value with field bits filled with supplied value
        if reg_name is None:
            reg_name = self.field_to_register[field_name]
        if reg_value is None:
            reg_value = self.registers.get(reg_name, 0)
        mask = self.all_fields[reg_name][field_name]
        new_value = (reg_value & ~mask) | ((field_value << ffs(mask)) & mask)
        self.registers[reg_name] = new_value
        return new_value
    def set_config_field(self, config, field_name, default, config_name=None):
        # Allow a field to be set from the config file
        if config_name is None:
            config_name = "driver_" + field_name.upper()
        reg_name = self.field_to_register[field_name]
        mask = self.all_fields[reg_name][field_name]
        maxval = mask >> ffs(mask)
        if maxval == 1:
            val = config.getboolean(config_name, default)
        elif field_name in self.signed_fields:
            val = config.getint(config_name, default,
                minval=-(maxval // 2 + 1), maxval=maxval // 2)
        else:
            val = config.getint(config_name, default, minval=0, maxval=maxval)
        return self.set_field(field_name, val)
    def pretty_format(self, reg_name, reg_value):
        # Provide a string description of a register
        reg_fields = self.all_fields.get(reg_name, {})
        reg_fields = sorted([(mask, name) for name, mask in reg_fields.items()])
        fields = []
        for mask, field_name in reg_fields:
            field_value = self.get_field(field_name, reg_value, reg_name)
            sval = self.field_formatters.get(field_name, str)(field_value)
            if sval and sval != "0":
                fields.append(" %s=%s" % (field_name, sval))
        return "%-11s %08x%s" % (reg_name + ":", reg_value, "".join(fields))


######################################################################
# G-Code command helpers
######################################################################

class TMCCommandHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        self.query_registers = None
        self.gcode = self.printer.lookup_object("gcode")
        self.gcode.register_mux_command(
            "SET_TMC_FIELD", "STEPPER", self.name.upper(),
            self.cmd_SET_TMC_FIELD, desc=self.cmd_SET_TMC_FIELD_help)
        self.gcode.register_mux_command(
            "INIT_TMC", "STEPPER", self.name.upper(),
            self.cmd_INIT_TMC, desc=self.cmd_INIT_TMC_help)
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
    def _init_registers(self, print_time):
        # Send registers
        for reg_name, val in self.fields.registers.items():
            self.mcu_tmc.set_register(reg_name, val, print_time)
    def _handle_connect(self):
        try:
            self._init_registers(0.)
        except self.printer.command_error as e:
            raise self.printer.config_error(str(e))
    cmd_INIT_TMC_help = "Initialize TMC stepper driver registers"
    def cmd_INIT_TMC(self, params):
        logging.info("INIT_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self._init_registers(print_time)
    cmd_SET_TMC_FIELD_help = "Set a register field of a TMC driver"
    def cmd_SET_TMC_FIELD(self, params):
        if 'FIELD' not in params or 'VALUE' not in params:
            raise self.gcode.error("Invalid command format")
        field_name = self.gcode.get_str('FIELD', params)
        reg_name = self.fields.lookup_register(field_name, None)
        if reg_name is None:
            raise self.gcode.error("Unknown field name '%s'" % (field_name,))
        value = self.gcode.get_int('VALUE', params)
        reg_val = self.fields.set_field(field_name, value)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        self.mcu_tmc.set_register(reg_name, reg_val, print_time)
    # DUMP_TMC support
    def setup_register_dump(self, query_registers):
        self.query_registers = query_registers
        self.gcode.register_mux_command(
            "DUMP_TMC", "STEPPER", self.name.upper(),
            self.cmd_DUMP_TMC, desc=self.cmd_DUMP_TMC_help)
    cmd_DUMP_TMC_help = "Read and display TMC stepper driver registers"
    def cmd_DUMP_TMC(self, params):
        logging.info("DUMP_TMC %s", self.name)
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        read_regs = self.query_registers(print_time)
        read_regs_by_name = { reg_name: val for reg_name, val in read_regs }
        self.gcode.respond_info("========== Write-only registers ==========")
        for reg_name, val in self.fields.registers.items():
            if reg_name not in read_regs_by_name:
                self.gcode.respond_info(
                    self.fields.pretty_format(reg_name, val))
        self.gcode.respond_info("========== Queried registers ==========")
        for reg_name, val in read_regs:
            self.gcode.respond_info(self.fields.pretty_format(reg_name, val))


######################################################################
# TMC stepper current config helper
######################################################################

MAX_CURRENT = 2.000

class TMCCurrentHelper:
    def __init__(self, config, mcu_tmc):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        run_current = config.getfloat('run_current',
                                      above=0., maxval=MAX_CURRENT)
        hold_current = config.getfloat('hold_current', run_current,
                                       above=0., maxval=MAX_CURRENT)
        self.sense_resistor = config.getfloat('sense_resistor', 0.110, above=0.)
        vsense, irun, ihold = self._calc_current(run_current, hold_current)
        self.fields.set_field("vsense", vsense)
        self.fields.set_field("IHOLD", ihold)
        self.fields.set_field("IRUN", irun)
        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "SET_TMC_CURRENT", "STEPPER", self.name.upper(),
            self.cmd_SET_TMC_CURRENT, desc=self.cmd_SET_TMC_CURRENT_help)
    def _calc_current_bits(self, current, vsense):
        sense_resistor = self.sense_resistor + 0.020
        vref = 0.32
        if vsense:
            vref = 0.18
        cs = int(32. * current * sense_resistor * math.sqrt(2.) / vref
                 - 1. + .5)
        return max(0, min(31, cs))
    def _calc_current(self, run_current, hold_current):
        vsense = False
        irun = self._calc_current_bits(run_current, vsense)
        ihold = self._calc_current_bits(hold_current, vsense)
        if irun < 16 and ihold < 16:
            vsense = True
            irun = self._calc_current_bits(run_current, vsense)
            ihold = self._calc_current_bits(hold_current, vsense)
        return vsense, irun, ihold
    def _calc_current_from_field(self, field_name):
        bits = self.fields.get_field(field_name)
        sense_resistor = self.sense_resistor + 0.020
        vref = 0.32
        if self.fields.get_field("vsense"):
            vref = 0.18
        current = (bits + 1) * vref / (32 * sense_resistor * math.sqrt(2.))
        return round(current, 2)
    cmd_SET_TMC_CURRENT_help = "Set the current of a TMC driver"
    def cmd_SET_TMC_CURRENT(self, params):
        gcode = self.printer.lookup_object('gcode')
        if 'HOLDCURRENT' in params:
            hold_current = gcode.get_float(
                'HOLDCURRENT', params, above=0., maxval=MAX_CURRENT)
        else:
            hold_current = self._calc_current_from_field("IHOLD")
        if 'CURRENT' in params:
            run_current = gcode.get_float(
                'CURRENT', params, minval=hold_current, maxval=MAX_CURRENT)
        else:
            run_current = self._calc_current_from_field("IRUN")
        if 'HOLDCURRENT' not in params and 'CURRENT' not in params:
            # Query only
            gcode.respond_info("Run Current: %0.2fA Hold Current: %0.2fA"
                               % (run_current, hold_current))
            return
        print_time = self.printer.lookup_object('toolhead').get_last_move_time()
        vsense, irun, ihold = self._calc_current(run_current, hold_current)
        if vsense != self.fields.get_field("vsense"):
            val = self.fields.set_field("vsense", vsense)
            self.mcu_tmc.set_register("CHOPCONF", val, print_time)
        self.fields.set_field("IHOLD", ihold)
        val = self.fields.set_field("IRUN", irun)
        self.mcu_tmc.set_register("IHOLD_IRUN", val, print_time)


######################################################################
# Config reading helpers
######################################################################

# Helper to configure and query the microstep settings
class TMCMicrostepHelper:
    def __init__(self, config, mcu_tmc):
        self.mcu_tmc = mcu_tmc
        self.fields = mcu_tmc.get_fields()
        steps = {'256': 0, '128': 1, '64': 2, '32': 3, '16': 4,
                 '8': 5, '4': 6, '2': 7, '1': 8}
        mres = config.getchoice('microsteps', steps)
        self.fields.set_field("MRES", mres)
    def get_microsteps(self):
        return 256 >> self.fields.get_field("MRES")
    def get_phase(self):
        field_name = "MSCNT"
        if self.fields.lookup_register(field_name, None) is None:
            # TMC2660 uses MSTEP
            field_name = "MSTEP"
        reg = self.mcu_tmc.get_register(self.fields.lookup_register(field_name))
        mscnt = self.fields.get_field(field_name, reg)
        return mscnt >> self.fields.get_field("MRES")

# Helper to configure "stealthchop" mode
def TMCStealthchopHelper(config, mcu_tmc, tmc_freq):
    fields = mcu_tmc.get_fields()
    en_pwm_mode = False
    velocity = config.getfloat('stealthchop_threshold', 0., minval=0.)
    if velocity:
        stepper_name = " ".join(config.get_name().split()[1:])
        stepper_config = config.getsection(stepper_name)
        step_dist = stepper_config.getfloat('step_distance')
        step_dist_256 = step_dist / (1 << fields.get_field("MRES"))
        threshold = int(tmc_freq * step_dist_256 / velocity + .5)
        fields.set_field("TPWMTHRS", max(0, min(0xfffff, threshold)))
        en_pwm_mode = True
    reg = fields.lookup_register("en_pwm_mode", None)
    if reg is not None:
        fields.set_field("en_pwm_mode", en_pwm_mode)
    else:
        # TMC2208 uses en_spreadCycle
        fields.set_field("en_spreadCycle", not en_pwm_mode)
