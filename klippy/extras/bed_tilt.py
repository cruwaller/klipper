# Bed tilt compensation
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import probe, mathutil

class BedTilt:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.x_adjust = config.getfloat('x_adjust', 0.)
        self.y_adjust = config.getfloat('y_adjust', 0.)
        self.z_adjust = 0.
        self.disabled = False
        if config.get('points', None) is not None:
            BedTiltCalibrate(config, self)
        self.toolhead = None
        self.gcode = gcode = self.printer.lookup_object('gcode')
        gcode.set_move_transform(self)
        for cmd in ['BED_TILT_DISABLE', 'M561']:
            gcode.register_command(
                cmd, self.cmd_BED_TILT_DISABLE,
                desc=self.cmd_BED_TILT_DISABLE_help)
        for cmd in ['BED_TILT_ENABLE']:
            gcode.register_command(
                cmd, self.cmd_BED_TILT_ENABLE,
                desc=self.cmd_BED_TILT_ENABLE_help)
        gcode.register_command("G29", self.cmd_G29)
    cmd_BED_TILT_DISABLE_help = "Disable bed tilt compensation"
    def cmd_BED_TILT_DISABLE(self, params):
        # This cancels any bed-plane fitting as the result of probing (or anything else)
        # and returns the machine to moving in the user's coordinate system.
        self.disabled = True
    cmd_BED_TILT_ENABLE_help = "Enable bed tilt compensation"
    def cmd_BED_TILT_ENABLE(self, params):
        self.disabled = False
    def cmd_G29(self, params):
        s = self.gcode.get_int("S", params, default=0)
        if s == 0:
            self.disabled = False
            self.gcode.run_script_from_command("BED_TILT_CALIBRATE")
        elif s == 1:
            # Load map...
            pass
        elif s == 2:
            self.disabled = True
    def printer_state(self, state):
        if state == 'connect':
            self.toolhead = self.printer.lookup_object('toolhead')
    def get_position(self):
        if self.toolhead is None:
            return [0., 0., 0., 0.]
        x, y, z, e = self.toolhead.get_position()
        return [x, y, z - x*self.x_adjust - y*self.y_adjust - self.z_adjust, e]
    def move(self, newpos, speed):
        if self.disabled:
            self.toolhead.move(newpos, speed)
            return
        x, y, z, e = newpos
        self.toolhead.move([x, y, z + x*self.x_adjust + y*self.y_adjust
                            + self.z_adjust, e], speed)
    def get_adjust(self):
        if self.disabled:
            return .0, .0, .0
        return self.x_adjust, self.y_adjust, self.z_adjust

# Helper script to calibrate the bed tilt
class BedTiltCalibrate:
    sender = None
    def __init__(self, config, bedtilt):
        self.printer = config.get_printer()
        self.bedtilt = bedtilt
        self.probe_helper = probe.ProbePointsHelper(config, self)
        # Automatic probe:z_virtual_endstop XY detection
        self.z_position_endstop = None
        if config.has_section('stepper_z'):
            zconfig = config.getsection('stepper_z')
            self.z_position_endstop = zconfig.getfloat('position_endstop', None)
        # Register BED_TILT_CALIBRATE command
        self.gcode = self.printer.lookup_object('gcode')
        for cmd in ['BED_TILT_CALIBRATE', 'G32']:
            self.gcode.register_command(
                cmd, self.cmd_BED_TILT_CALIBRATE,
                desc=self.cmd_BED_TILT_CALIBRATE_help)
    cmd_BED_TILT_CALIBRATE_help = "Bed tilt calibration script"
    def cmd_BED_TILT_CALIBRATE(self, params):
        self.bedtilt.disabled = False
        self.sender = params["#input"].respond_info
        self.gcode.run_script_from_command("G28")
        self.probe_helper.start_probe()
    def get_probed_position(self):
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        return kin.calc_position()
    def finalize(self, z_offset, positions):
        bed_tilt = self.bedtilt
        logging.info("Calculating bed_tilt with: %s", positions)
        params = { 'x_adjust': bed_tilt.x_adjust,
                   'y_adjust': bed_tilt.y_adjust,
                   'z_adjust': z_offset }
        logging.info("Initial bed_tilt parameters: %s", params)
        def adjusted_height(pos, params):
            x, y, z = pos
            return (z - x*params['x_adjust'] - y*params['y_adjust']
                    - params['z_adjust'])
        def errorfunc(params):
            total_error = 0.
            for pos in positions:
                total_error += adjusted_height(pos, params)**2
            return total_error
        new_params = mathutil.coordinate_descent(
            params.keys(), params, errorfunc)
        logging.info("Calculated bed_tilt parameters: %s", new_params)
        for pos in positions:
            logging.info("orig: %s new: %s", adjusted_height(pos, params),
                         adjusted_height(pos, new_params))
        # Update current bed_tilt calculations
        # bed_tilt = self.printer.lookup_object('bed_tilt')
        bed_tilt.x_adjust = new_params['x_adjust']
        bed_tilt.y_adjust = new_params['y_adjust']
        z_diff = new_params['z_adjust'] - z_offset
        bed_tilt.z_adjust = z_diff
        self.gcode.reset_last_position()
        # Report results back to user
        if self.z_position_endstop is not None:
            # Cartesian style robot
            z_extra = ""
            probe = self.printer.lookup_object('probe', None)
            if probe is not None:
                last_home_position = probe.last_home_position()
                if last_home_position is not None:
                    # Using z_virtual_endstop
                    home_x, home_y = last_home_position[:2]
                    z_diff -= home_x * new_params['x_adjust']
                    z_diff -= home_y * new_params['y_adjust']
                    z_extra = " (when Z homing at %.3f,%.3f)" % (home_x, home_y)
            z_adjust = "stepper_z position_endstop: %.6f%s\n" % (
                self.z_position_endstop - z_diff, z_extra)
        else:
            # Delta (or other) style robot
            z_adjust = "Add %.6f to endstop position\n" % (-z_diff,)
        msg = "%sx_adjust: %.6f y_adjust: %.6f" % (
            z_adjust, new_params['x_adjust'], new_params['y_adjust'])
        self.printer.set_rollover_info("bed_tilt", "bed_tilt: %s" % (msg,))
        self.sender.respond_info(
            "%s\nThe above parameters have been applied to the current\n"
            "session. Update the printer config file with the above to\n"
            "use these settings in future sessions." % (msg,))

def load_config(config):
    return BedTilt(config)
