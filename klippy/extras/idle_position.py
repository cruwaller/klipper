# This file may be distributed under the terms of the GNU GPLv3 license.

class IdlePosition:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = self.printer.logger.getChild('idle_position')
        self.in_idle_pos = False
        self.saved = False
        # register events
        self.printer.register_event_handler('vsd:status', self._sd_status)
        # load configs
        gcode_macro = self.printer.try_load_module(config, 'gcode_macro')
        self.command = gcode_macro.load_template(config, 'gcode')
        self.recover_velocity = config.getfloat('recover_velocity', 40.,
                                                above=0.)
        # register gcode command
        self.gcode = self.printer.lookup_object('gcode')
        self.gcode.register_command('IDLE_POSITION', self.cmd_IDLE_POSITION,
                                    desc=self.cmd_IDLE_POSITION_help)
        self.gcode.register_command('IDLE_RESUME', self.cmd_IDLE_RESUME,
                                    desc=self.cmd_IDLE_RESUME_help)
    def _sd_status(self, status):
        if status in ['pause', 'error', 'stop', 'done']:
            self.move()
        elif status == 'start':
            self.resume()
    def move(self, store=True):
        if self.in_idle_pos:
            return
        if store:
            self.saved = True
            self.gcode.run_script_from_command(
                "SAVE_GCODE_STATE STATE=IDLE_POS")
        self.in_idle_pos = True
        self.gcode.run_script_from_command(self.command.render())
    def resume(self):
        if not self.in_idle_pos:
            return
        if self.saved:
            self.gcode.run_script_from_command(
                "RESTORE_GCODE_STATE STATE=IDLE_POS MOVE=1 MOVE_SPEED=%.1f" % (
                    self.recover_velocity))
        self.saved = False
        self.in_idle_pos = False
    cmd_IDLE_POSITION_help = "Move head to defined idle position"
    def cmd_IDLE_POSITION(self, params):
        store = bool(self.gcode.get_int('STORE', params, 0))
        self.move(store)
    cmd_IDLE_RESUME_help = "Move head to back to original position"
    def cmd_IDLE_RESUME(self, params):
        self.resume()


def load_config(config):
    return IdlePosition(config)
