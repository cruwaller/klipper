#
# This file may be distributed under the terms of the GNU GPLv3 license.
#

class GCodeFilamentPause(object):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.gcode = self.printer.lookup_object('gcode')
        self.v_sd = None
        self.is_paused = False
        self.resume_print_original = None
        self.start_time = self.reactor.NEVER
        self.last_button_state = False
        self.last_cb_event_time = 0.
        self.event_delay = config.getfloat('event_delay', 3., above=0.)
        gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
        self.cmd_unload = gcode_macro.load_template(config, 'unload_gcode')
        self.cmd_load = gcode_macro.load_template(config, 'load_gcode')
        self.recover_velocity = config.getfloat('recover_velocity', 40.) * 60.
        self.gcode.register_command('FILAMENT_CHANGE', self.cmd_FILAMENT_CHANGE,
                                    desc="Initiate filament change procedure")
        switch_pin = config.get("pin", None)
        if switch_pin is not None:
            # setup gpio pin to trigger filament change
            self.buttons = self.printer.try_load_module(config, 'buttons')
            self.buttons.register_buttons([switch_pin], self._button_handler)
    def _handle_ready(self):
        self.v_sd = self.printer.lookup_object('virtual_sdcard', None)
        self.start_time = self.reactor.monotonic() + 2.
    def _button_handler(self, eventtime, state):
        if eventtime < self.start_time or state == self.last_button_state:
            self.last_button_state = state
            return
        if (eventtime - self.last_cb_event_time) > self.event_delay:
            self.last_cb_event_time = eventtime
            if state:
                # runout detected
                self.reactor.register_callback(self._ready_event_handler)
            #else:
            #    pass
        self.last_button_state = state
    def _ready_event_handler(self, _):
        self.cmd_FILAMENT_CHANGE_READY({})
    def _swap_start_gcode(self):
        if self.resume_print_original is None:
            self.resume_print_original = self.gcode.get_command_handler('M24')
            self.gcode.register_command('M24', None)
            self.gcode.register_command('M24', self.cmd_FILAMENT_CHANGE_READY)
            return
        self.gcode.register_command('M24', None)
        self.gcode.register_command('M24', self.resume_print_original)
        self.resume_print_original = None
    def _add_gcodes(self):
        self.gcode.register_command('FILAMENT_CHANGE_READY',
                                    self.cmd_FILAMENT_CHANGE_READY,
                                    desc="finalize filament change and continue")
        self.gcode.register_command('FILAMENT_CHANGE_CANCEL',
                                    self.cmd_FILAMENT_CHANGE_CANCEL,
                                    desc="End filament change")
    def _remove_gcodes(self):
        self.gcode.register_command('FILAMENT_CHANGE_READY', None)
        self.gcode.register_command('FILAMENT_CHANGE_CANCEL', None)
    def cmd_FILAMENT_CHANGE(self, _):
        if self.is_paused:
            raise self.gcode.error("Filament change already ongoing")
        self.is_paused = True
        if self.v_sd is not None and self.v_sd.is_active():
            # Printing from virtual sd, pause print
            self.v_sd.do_pause()
            self._swap_start_gcode()
        self.gcode.run_script_from_command(
            "SAVE_GCODE_STATE STATE=FILAMENT_CHANGE_STATE")
        self.gcode.run_script_from_command(self.cmd_unload.render())
        self._add_gcodes()
        self.gcode.respond_info("Please load new filament and resume with "
                                "M24 or FILAMENT_CHANGE_READY")
    def cmd_FILAMENT_CHANGE_READY(self, params):
        if not self.is_paused:
            raise self.gcode.error("Filament change in not ongoing")
        velocity = self.gcode.get_float('VELOCITY', params,
                                        self.recover_velocity)
        self.gcode.run_script_from_command(self.cmd_load.render())
        self.gcode.run_script_from_command(
            "RESTORE_GCODE_STATE STATE=FILAMENT_CHANGE_STATE "
            "MOVE=1 MOVE_SPEED=%.4f" % (velocity,))
        self._swap_start_gcode()
        if self.v_sd is not None and not self.v_sd.is_active():
            # Printing from virtual sd, continue print
            try:
                self.v_sd.cmd_M24({})
            except self.gcode.error:
                pass
        self._remove_gcodes()
        self.is_paused = False
        self.gcode.respond_info("Filament changed")
    def cmd_FILAMENT_CHANGE_CANCEL(self, params):
        move = ""
        if self.gcode.get_int('MOVE', params, 0):
            velocity = self.gcode.get_float('VELOCITY', params,
                                            self.recover_velocity)
            move = " MOVE=1 MOVE_SPEED=%.4f" % velocity
        self.is_paused = False
        self._swap_start_gcode()
        self._remove_gcodes()
        # just restore state
        self.gcode.run_script_from_command(
            "RESTORE_GCODE_STATE STATE=FILAMENT_CHANGE_STATE%s" % move)


def load_config(config):
    GCodeFilamentPause(config)
