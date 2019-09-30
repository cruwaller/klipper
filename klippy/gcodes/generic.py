#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class GenericGcode:
    def __init__(self, config):
        self.printer = config.get_printer()
        gcode = self.printer.lookup_object('gcode')
        for cmd in ['G29', 'G32', 'M1', 'M302', 'M561', 'M851',
                    'QUERY_COMPENSATION']:
            wnr = getattr(self, 'cmd_' + cmd + '_when_not_ready', False)
            gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd),
                when_not_ready=wnr,
                desc=getattr(self, 'cmd_%s_help' % cmd, None))
        # Discard not used gcodes:
        for cmd in ['M120', 'M121', 'M122',
            'M291', 'M292',
            'M752', 'M753', 'M754', 'M755', 'M756', 'M997']:
            gcode.register_command(cmd, gcode.cmd_IGNORE, when_not_ready=True)
        # M118 to ECHO
        gcode.register_command('M118', gcode.cmd_ECHO, when_not_ready = True,
                               desc = "Alias to ECHO")
        # M999 to reset
        gcode.register_command('M999', gcode.cmd_FIRMWARE_RESTART,
                               when_not_ready = True,
                               desc = "Alias to FIRMWARE_RESTART")
    def cmd_G29(self, params):
        # wrap probe commands
        gcode = self.printer.lookup_object('gcode')
        type = gcode.get_int('S', params, 0)
        if type == 1:
            # load compensation bitmap from SD
            gcode.run_script_from_command("BED_MESH_PROFILE LOAD=default\n")
            return
        elif type == 2:
            # clean compensation bitmap
            gcode.run_script_from_command("BED_MESH_CLEAR\n")
            return
        # do probing
        gcode.run_script_from_command("BED_MESH_CALIBRATE\n")
    def cmd_G32(self, params):
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command("BED_MESH_CALIBRATE\n")
    def cmd_M561(self, params):
        # M561: Disable bed compensation
        gcode = self.printer.lookup_object('gcode')
        gcode.run_script_from_command("BED_MESH_CLEAR\n")
    def cmd_QUERY_COMPENSATION(self, params):
        out = []
        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        calibrate = getattr(bed_mesh, "calibrate", None)
        if calibrate is not None:
            table = calibrate.get_probed_z_table()
            if table is not None:
                out.append("Bed equation fits points")
                for pos, z in table:
                    # TODO: change z output to .6 while default is .3 ??
                    out.append("[%.1f, %.1f, %.6f]" % (pos[0], pos[1], z))
        gcode = self.printer.lookup_object('gcode')
        gcode.respond(" ".join(out))

    def cmd_M1(self, params):
        toolhead = self.printer.lookup_object('toolhead')
        # Wait for current moves to finish
        toolhead.wait_moves()
        toolhead.motor_heater_off()

    cmd_M302_help = "Cold Extrude. Args [P<bool>] [S<temp>]"
    cmd_M302_when_not_ready = True
    def cmd_M302(self, params):
        # Cold extrusion
        #       M302         ; report current cold extrusion state
        #       M302 P0      ; enable cold extrusion checking
        #       M302 P1      ; disables cold extrusion checking
        #       M302 S0      ; always allow extrusion (disables checking)
        #       M302 S170    ; only allow extrusion above 170
        #       M302 S170 P1 ; set min extrude temp to 170 but leave disabled
        gcode = self.printer.lookup_object('gcode')
        disable = None
        temperature = None
        if 'P' in params:
            disable = gcode.get_int('P', params, 0) == 1
        if 'S' in params:
            temperature = gcode.get_int('S', params, -1)
        resp = []
        for extruder in self.printer.extruder_get().values():
            heater = extruder.get_heater()
            heater.set_min_extrude_temp(temperature, disable)
            status, temp = heater.get_min_extrude_status()
            resp.append("Heater '%s' cold extrude: %s, min temp %.2fC"
                        % (heater.name, status, temp))
        gcode.respond("\n".join(resp))

    cmd_M851_help = "Set axis offset. Args [X<offset] [Y<offset>] [Z<offset>]"
    cmd_M851_when_not_ready = True
    def cmd_M851(self, params):
        gcode = self.printer.lookup_object('gcode')
        toolhead = self.printer.lookup_object('toolhead')
        axis2pos = gcode.axis2pos
        # Set X, Y, Z offsets
        rails = toolhead.get_kinematics().get_rails()
        offsets = {axis2pos[a]: gcode.get_float(a, params)
            for a, p in axis2pos.items() if a in params}
        for p, offset in offsets.items():
            rails[p].set_homing_offset(offset)
        gcode.respond("Current offsets: X=%.2f Y=%.2f Z=%.2f" %
                      (rails[0].homing_offset,
                       rails[1].homing_offset,
                       rails[2].homing_offset))
    def cmd_M851_new_way(self, params):
        gcode = self.printer.lookup_object('gcode')
        cmd = []
        for axis in "XYZ":
            if axis in params:
                cmd.append("%s_ADJUST=%f" % (
                    axis, gcode.get_float(axis, params)))
        if cmd:
            gcode.run_script_from_command(
                "SET_GCODE_OFFSET %s\n" % " ".join(cmd))
        # print out the current position + offsets
        gcode.run_script_from_command("GET_POSITION\n")


#def load_config(config):
#    return GenericGcode(config)
