# Support for Marlin/Smoothie/Reprap style firmware retraction via G10/G11
#
# Copyright (C) 2019  Len Trigg <lenbok@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class FirmwareRetraction:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.retract_length = config.getfloat('retract_length', 0., minval=0.)
        self.retract_speed = config.getfloat('retract_speed', 20., minval=1)
        self.unretract_extra_length = config.getfloat(
            'unretract_extra_length', 0., minval=0.)
        self.unretract_speed = config.getfloat('unretract_speed', 10., minval=1)
        self.unretract_length = (self.retract_length
                                 + self.unretract_extra_length)
        self.z_hop = config.getfloat('z_hop', 0., minval=0.)
        self.is_retracted = False
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('SET_RETRACTION', self.cmd_SET_RETRACTION)
        self.gcode.register_command('GET_RETRACTION', self.cmd_GET_RETRACTION)
        self.gcode.register_command('G10', self.cmd_G10)
        self.gcode.register_command('G11', self.cmd_G11)
        self.gcode.register_command('M207', self.cmd_M207,
                                    desc=self.cmd_M207_help)
        self.gcode.register_command('M208', self.cmd_M208,
                                    desc=self.cmd_M208_help)

    def get_status(self, eventtime):
        return {
            "retract_length": self.retract_length,
            "retract_speed": self.retract_speed,
            "unretract_extra_length": self.unretract_extra_length,
            "unretract_speed": self.unretract_speed,
            "z_hop": self.z_hop,
        }

    def cmd_SET_RETRACTION(self, params):
        self.retract_length = self.gcode.get_float(
            'RETRACT_LENGTH',
            params, self.retract_length, minval=0.)
        self.retract_speed = self.gcode.get_float(
            'RETRACT_SPEED',
            params, self.retract_speed, minval=1)
        self.unretract_extra_length = self.gcode.get_float(
            'UNRETRACT_EXTRA_LENGTH',
            params, self.unretract_extra_length, minval=0.)
        self.unretract_speed = self.gcode.get_float(
            'UNRETRACT_SPEED',
            params, self.unretract_speed, minval=1)
        self.unretract_length = (self.retract_length
                                 + self.unretract_extra_length)
        self.z_hop = self.gcode.get_float(
            'Z_HOP',
            params, self.z_hop, minval=0.)
        self.is_retracted = False

    def cmd_GET_RETRACTION(self, params):
        msg = ("RETRACT_LENGTH=%.5f RETRACT_SPEED=%.5f "
               "UNRETRACT_EXTRA_LENGTH=%.5f UNRETRACT_SPEED=%.5f Z_HOP=%.5f"
               % (self.retract_length, self.retract_speed,
                  self.unretract_extra_length, self.unretract_speed, self.z_hop))
        self.gcode.respond_info(msg)

    def cmd_G10(self, params):
        if not self.is_retracted:
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=_retract_state\n"
                "G91\n"
                "G1 E-%.5f F%d\n"
                "G1 Z%.5f F6000\n"
                "RESTORE_GCODE_STATE NAME=_retract_state"
                % (self.retract_length, self.retract_speed*60, self.z_hop))
            self.is_retracted = True

    def cmd_G11(self, params):
        if self.is_retracted:
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE NAME=_retract_state\n"
                "G91\n"
                "G1 E%.5f F%d\n"
                "G1 Z-%.2f F6000\n"
                "RESTORE_GCODE_STATE NAME=_retract_state"
                % (self.unretract_length, self.unretract_speed*60, self.z_hop))
            self.is_retracted = False

    cmd_M207_help = "Set fw retraction; [F<feedrate>] [S<length>] [Z<hight>]"
    def cmd_M207(self, params):
        # M207 F<feedrate> S<length> Z<hight>
        self.retract_length = self.gcode.get_float(
            'S', params, default=self.retract_length)
        self.retract_speed = self.gcode.get_int(
            'F', params, default=self.retract_speed)
        self.z_hop = self.gcode.get_float('Z', params, default=self.z_hop)
        # reprap support
        self.unretract_extra_length = self.gcode.get_float(
            'R', params, default=self.unretract_extra_length)
        self.unretract_speed = self.gcode.get_int(
            'T', params, default=self.unretract_speed)
        self.gcode.respond_info("FW Retract: speed %s length %s z_hop %s" %
            (self.retract_speed, self.retract_length, self.z_hop))

    cmd_M208_help = "Set fw unretraction; [F<feedrate>] [S<length>]"
    def cmd_M208(self, params):
        # M208 F<feedrate> S<length>
        self.unretract_extra_length = self.gcode.get_float(
            'S', params, default=self.unretract_extra_length)
        self.unretract_speed = self.gcode.get_int(
            'F', params, default=self.unretract_speed)
        self.gcode.respond_info("FW Retract recover: speed %s addition %s" %
            (self.unretract_speed, self.unretract_extra_length))

def load_config(config):
    return FirmwareRetraction(config)
