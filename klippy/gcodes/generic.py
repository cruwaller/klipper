import extruder

class GenericGcode(object):
    def __init__(self, printer):
        self.printer = printer
        self.gcode = printer.lookup_object('gcode')
        self.toolhead = printer.lookup_object('toolhead')
        for cmd in ['M0', 'M1', 'M37', 'M118', 'M205',
                    'M301', 'M302', 'M304',
                    'M851', 'M900']:
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                desc=getattr(self, 'cmd_%s_help' % cmd, None))
        # just discard
        for cmd in ['M120', 'M121', 'M122', "M141",
                    'M291', 'M292',
                    'M752', 'M753', 'M754', 'M755', 'M756', 'M997']:
            self.gcode.register_command(cmd, self.gcode.cmd_IGNORE)
        # M999 to reset
        self.gcode.register_command('M999',
                                    self.gcode.cmd_FIRMWARE_RESTART,
                                    when_not_ready=True,
                                    desc="Alias to FIRMWARE_RESTART")
        self.respond_info = self.gcode.respond # self.gcode.respond_info
        self.axis2pos = self.gcode.axis2pos
        self.logger = self.gcode.logger
        self.logger.info("Generic GCode extension initialized")

    def cmd_ignore(self, params):
        pass

    def motor_heater_off(self):
        self.gcode.toolhead.motor_off()
        print_time = self.gcode.toolhead.get_last_move_time()
        for h in self.printer.lookup_module_objects("heater"):
            h.set_temp(print_time, 0.0)
        for fan in self.printer.lookup_module_objects('fan'):
            fan.set_speed(print_time, 0.0)

    def cmd_M0(self, params):
        heaters_on = self.gcode.get_int('H', params, 0)
        if heaters_on is 0:
            self.motor_heater_off()
        elif self.gcode.toolhead is not None:
            self.gcode.toolhead.motor_off()

    def cmd_M1(self, params):
        # Wait for current moves to finish
        self.gcode.toolhead.wait_moves()
        self.motor_heater_off()

    cmd_M37_help = "Enable P1 or disable P0 print simulation"
    def cmd_M37(self, params):
        simulation_enabled = self.gcode.get_int('P', params, 0)
        if simulation_enabled is 1:
            self.gcode.simulate_print = True
        else:
            self.gcode.simulate_print = False

    def cmd_M118(self, params):
        self.respond_info(params['#original'].replace(params['#command'], ""))

    def cmd_M205(self, params):
        # Set advanced settings
        value = self.gcode.get_float('X', params, None)
        if value is not None and 0. < value:
            self.gcode.toolhead.junction_deviation = value
            self.gcode.toolhead.get_kinematics().update_velocities()
        self.respond_info("Junction deviation %.2f" % (self.gcode.toolhead.junction_deviation,))

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
        for h in self.printer.lookup_module_objects("heater"):
            h.set_min_extrude_temp(temperature, disable)
            status, temp = h.get_min_extrude_status()
            if "bed" not in h.name:
                self.respond_info(
                    "Heater '{}' cold extrude: {}, min temp {}C".
                    format(h.name, status, temp))

    def cmd_M301(self, params):
        # M301: Set PID parameters
        self.gcode.respond_info("Obsolete, use SET_PID_PARAMS")

    def cmd_M304(self, params):
        # M304: Set PID parameters - Bed
        self.gcode.respond_info("Obsolete, use SET_PID_PARAMS")

    cmd_M851_help = "Set axis offset. Args [X<offset] [Y<offset>] [Z<offset>]"
    def cmd_M851(self, params):
        # Set X, Y, Z offsets
        steppers = self.gcode.toolhead.get_kinematics().get_steppers()
        offsets = { self.axis2pos[a]: self.gcode.get_float(a, params)
                    for a, p in self.axis2pos.items() if a in params }
        for p, offset in offsets.items():
            steppers[p].set_homing_offset(offset)
        self.respond_info("Current offsets: X=%.2f Y=%.2f Z=%.2f" % \
                          (steppers[0].homing_offset,
                           steppers[1].homing_offset,
                           steppers[2].homing_offset))

    def cmd_M900(self, params):
        # Pressure Advance configuration
        self.gcode.respond_info("Obsolete, use SET_PRESSURE_ADVANCE")


def load_gcode(printer, config):
    GenericGcode(printer)
