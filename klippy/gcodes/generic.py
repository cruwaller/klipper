
class GenericGcode:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.gcode = gcode = printer.lookup_object('gcode')
        for cmd in ['M1', 'M118',
            'M301', 'M302', 'M304',
            'M851', 'M900']:
            gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_%s_help' % cmd, None))
        # just discard
        for cmd in ['M120', 'M121', 'M122',
            'M291', 'M292',
            'M752', 'M753', 'M754', 'M755', 'M756', 'M997']:
            gcode.register_command(cmd, gcode.cmd_IGNORE)
        # M999 to reset
        gcode.register_command('M999', gcode.cmd_FIRMWARE_RESTART,
                               when_not_ready=True,
                               desc="Alias to FIRMWARE_RESTART")

    def cmd_ignore(self, params):
        pass

    def cmd_M1(self, params):
        toolhead = self.printer.lookup_object('toolhead')
        # Wait for current moves to finish
        toolhead.wait_moves()
        toolhead.motor_heater_off()

    def cmd_M118(self, params):
        self.gcode.respond(params['#original'].replace(params['#command'], ""))

    cmd_M302_help = "Cold Extrude. Args [P<bool>] [S<temp>]"
    def cmd_M302(self, params):
        # Cold extrusion
        #       M302         ; report current cold extrusion state
        #       M302 P0      ; enable cold extrusion checking
        #       M302 P1      ; disables cold extrusion checking
        #       M302 S0      ; always allow extrusion (disables checking)
        #       M302 S170    ; only allow extrusion above 170
        #       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
        disable = None
        temperature = None
        if 'P' in params:
            disable = self.gcode.get_int('P', params, 0) == 1
        if 'S' in params:
            temperature = self.gcode.get_int('S', params, -1)
        resp = []
        for extruder in self.printer.extruder_get().values():
            heater = extruder.get_heater()
            heater.set_min_extrude_temp(temperature, disable)
            status, temp = heater.get_min_extrude_status()
            resp.append("Heater '%s' cold extrude: %s, min temp %.2fC"
                        % (heater.name, status, temp))
        self.gcode.respond("\n".join(resp))

    def cmd_M301(self, params):
        # M301: Set PID parameters
        self.gcode.respond("Obsolete, use SET_PID_PARAMS")

    def cmd_M304(self, params):
        # M304: Set PID parameters - Bed
        self.gcode.respond("Obsolete, use SET_PID_PARAMS")

    cmd_M851_help = "Set axis offset. Args [X<offset] [Y<offset>] [Z<offset>]"
    def cmd_M851(self, params):
        toolhead = self.printer.lookup_object('toolhead')
        axis2pos = self.gcode.axis2pos
        # Set X, Y, Z offsets
        rails = toolhead.get_kinematics().get_rails()
        offsets = {axis2pos[a]: self.gcode.get_float(a, params)
            for a, p in axis2pos.items() if a in params}
        for p, offset in offsets.items():
            rails[p].set_homing_offset(offset)
        self.gcode.respond("Current offsets: X=%.2f Y=%.2f Z=%.2f" %
                           (rails[0].homing_offset,
                           rails[1].homing_offset,
                           rails[2].homing_offset))

    def cmd_M900(self, params):
        # Pressure Advance configuration
        self.gcode.respond_error("Obsolete, use SET_PRESSURE_ADVANCE")
