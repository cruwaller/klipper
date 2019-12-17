# Macro gcode file execution
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, shlex

class DWCMacro:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.get_logger('dwc.macro')
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        # sdcard state
        self.vsd = printer.try_load_module(config, "virtual_sdcard")
        self.macro_dirname = self.vsd.sdcard_dirname
        self.current_file = None
        self.file_position = 0
        # Work timer
        self.reactor = printer.get_reactor()
        self.must_pause_work = self.cmd_from_sd = False
        self.work_timer = None
        # Register commands
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['M98']:
            wnr = getattr(self, 'cmd_' + cmd + '_when_not_ready', False)
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd), wnr)
    def _handle_shutdown(self):
        if self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except AttributeError:
                self.logger.exception("shutdown read")
                return
            self.logger.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                             readpos, repr(data[:readcount]),
                             self.file_position, repr(data[readcount:]))
    def _load_file(self, filename):
        # Select file
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        if self.current_file is not None:
            self.current_file.close()
            self.current_file = None
            self.file_position = 0
        try:
            if '*' in filename:
                filename = filename[:filename.find('*')].strip()
        except:
            raise self.gcode.error("Unable to extract filename")
        if filename.startswith('/'):
            filename = filename[1:]
        try:
            fname = os.path.join(self.macro_dirname, filename)
            f = open(fname, 'rb')
            f.seek(0)
        except:
            self.logger.exception("virtual_sdcard file open")
            raise self.gcode.error("Unable to open file")
        self.current_file = f
        self.file_position = 0
    def _start_exec(self):
        # Start/resume
        if self.current_file is None:
            raise self.gcode.error("file is not loaded")
        if self.work_timer is not None:
            raise self.gcode.error("busy")
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
    # G-Code commands
    cmd_M98_when_not_ready = True
    def cmd_M98(self, params):
        if self.vsd.is_active():
            raise self.gcode.error("virtual sdcard busy")
        # Run macro
        orig = params['#original']
        macro_f = shlex.split(orig[orig.find("P") + 1:])[0].strip()
        self.logger.info("Executing macro %s" % macro_f)
        self._load_file(macro_f)
        self._start_exec()
        self.gcode.respond("Macro started")
    # Background work timer
    def work_handler(self, eventtime):
        self.logger.info("Starting macro (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            self.logger.exception("seek")
            self.gcode.respond_error("Unable to seek file")
            self.work_timer = None
            return self.reactor.NEVER
        gcode_mutex = self.gcode.get_mutex()
        partial_input = ""
        lines = []
        while not self.must_pause_work:
            if not lines:
                # Read more data
                try:
                    data = self.current_file.read(8192)
                except:
                    self.logger.exception("read")
                    self.gcode.respond_error("Error on dwc_macro read")
                    break
                if not data:
                    # End of file
                    self.current_file.close()
                    self.current_file = None
                    self.logger.info("Finished macro")
                    self.gcode.respond("DWC macro done ")
                    break
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                self.reactor.pause(self.reactor.NOW)
                continue
            # Pause if any other request is pending in the gcode class
            if gcode_mutex.test():
                self.reactor.pause(self.reactor.monotonic() + 0.100)
                continue
            # Dispatch command
            self.cmd_from_sd = True
            try:
                self.gcode.run_script(lines[-1])
            except self.gcode.error as e:
                self.logger.error("Error in g-code handling: %s" % e)
                self.gcode.respond_error("Error: dwc_macro stop ok")
                break
            except:
                self.logger.exception("dispatch")
                break
            self.cmd_from_sd = False
            self.file_position += len(lines.pop()) + 1
        self.logger.info("Exiting (position %d)", self.file_position)
        self.work_timer = None
        self.cmd_from_sd = False
        return self.reactor.NEVER

def load_config(config):
    return DWCMacro(config)
