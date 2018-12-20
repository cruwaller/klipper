# Virtual sdcard support (print files directly from a host g-code file)
#
# Copyright (C) 2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, errno

class VirtualSD:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.logger.getChild('VirtualSD')
        self.simulate_print = False
        self.toolhead = None
        # sdcard state
        sd = config.get('path')
        self.sdcard_dirname = os.path.normpath(os.path.expanduser(sd))
        self.current_file = None
        self.file_position = self.file_size = 0
        # Work timer
        self.reactor = printer.get_reactor()
        self.must_pause_work = False
        self.work_timer = None
        # Register commands
        self.gcode = printer.lookup_object('gcode')
        self.gcode.register_command('M21', None)
        for cmd in ['M0', 'M20', 'M21', 'M23', 'M24', 'M25', 'M26', 'M27',
                    'M32', 'M37', 'M98']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))
        for cmd in ['M28', 'M29', 'M30']:
            self.gcode.register_command(cmd, self.cmd_error)
        self.gcode.register_command('SD_SET_PATH',
            self.cmd_SD_SET_PATH)
        self.done_cb = []
        self.last_gco_sender = None
        self.logger.debug("SD card path: %s", self.sdcard_dirname)
    def cmd_SD_SET_PATH(self, params):
        self.sdcard_dirname = self.gcode.get_str("TYPE", params,
            default=self.sdcard_dirname)
        self.logger.debug("SD card path: %s", self.sdcard_dirname)
    def get_current_file_name(self):
        try:
            return self.current_file.name
        except AttributeError:
            return None
    def printer_state(self, state):
        if state == 'shutdown' and self.work_timer is not None:
            self.must_pause_work = True
            try:
                readpos = max(self.file_position - 1024, 0)
                readcount = self.file_position - readpos
                self.current_file.seek(readpos)
                data = self.current_file.read(readcount + 128)
            except:
                self.logger.exception("virtual_sdcard shutdown read")
                return
            self.logger.info("Virtual sdcard (%d): %s\nUpcoming (%d): %s",
                             readpos, repr(data[:readcount]),
                             self.file_position, repr(data[readcount:]))
        elif state == "ready":
            self.toolhead = self.printer.lookup_object('toolhead')
    def stats(self, eventtime):
        if self.work_timer is None:
            return False, ""
        return True, "sd_pos=%d" % (self.file_position,)
    def get_file_list(self):
        dname = self.sdcard_dirname
        try:
            filenames = os.listdir(self.sdcard_dirname)
            return [(fname, os.path.getsize(os.path.join(dname, fname)))
                    for fname in filenames]
        except:
            self.logger.exception("virtual_sdcard get_file_list")
            raise self.gcode.error("Unable to get file list")
    def get_status(self, eventtime, extended=False):
        progress = 0.
        if self.work_timer is not None and self.file_size:
            progress = float(self.file_position) / self.file_size
        stat = {'progress': progress}
        if extended:
            try:
                stat["file"] = self.current_file.name
            except AttributeError:
                stat["file"] = 'N/A'
            stat['simulation'] = int(self.simulate_print)
            stat['printing'] = int(self.work_timer is not None)
        return stat
    def register_done_cb(self, cb):
        if cb not in self.done_cb:
            self.done_cb.append(cb)
    def get_progress(self):
        if self.current_file is None or not self.file_size:
            return .0
        return float(self.file_position) / self.file_size
    # G-Code commands
    def cmd_error(self, params):
        raise self.gcode.error("SD write not supported")
    def cmd_M0(self, params):
        heaters_on = self.gcode.get_int('H', params, 0)
        if heaters_on is 0:
            self.toolhead.motor_heater_off()
        elif self.toolhead is not None:
            self.toolhead.motor_off()
        for cb in self.done_cb:
            cb('stop')
    def cmd_M20(self, params):
        # List SD card
        files = self.get_file_list()
        self.gcode.respond("Begin file list")
        for fname, fsize in files:
            self.gcode.respond("%s %d" % (fname, fsize))
        self.gcode.respond("End file list")
    def cmd_M21(self, params):
        # Initialize SD card
        self.gcode.respond("SD card ok")
    def cmd_M23(self, params):
        # Select SD file
        is_abs_path = bool(self.gcode.get_int(
            'IS_ABS', params, default=0))
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        if self.current_file is not None:
            self.current_file.close()
            self.current_file = None
            self.file_position = self.file_size = 0
        try:
            orig = params['#original']
            filename = orig[orig.find("M23") + 3:].split()[0].strip()
            if '*' in filename:
                filename = filename[:filename.find('*')].strip()
        except:
            raise self.gcode.error("Unable to extract filename")
        if not is_abs_path and filename.startswith('/'):
            filename = filename[1:]
        try:
            fname = filename
            if not is_abs_path:
                fname = os.path.join(self.sdcard_dirname, filename)
            f = open(fname, 'rb')
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
        except IOError:
            self.logger.exception("virtual_sdcard file open")
            raise self.gcode.error("Unable to open file")
        self.gcode.respond("File opened:%s Size:%d" % (filename, fsize))
        self.gcode.respond("File selected")
        self.current_file = f
        self.file_position = 0
        self.file_size = fsize
        # Reset extruders filament counters
        for i, e in self.printer.extruder_get().items():
            e.raw_filament = 0.
        for cb in self.done_cb:
            cb('loaded')
        self.simulate_print = False
    def cmd_M24(self, params):
        # Start/resume SD print
        if self.current_file is None:
            raise self.gcode.error("SD file is not loaded")
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        for cb in self.done_cb:
            cb('start')
        self.must_pause_work = False
        self.work_timer = self.reactor.register_timer(
            self.work_handler, self.reactor.NOW)
    def cmd_M25(self, params):
        # Pause SD print
        if self.work_timer is not None:
            self.must_pause_work = True
            for cb in self.done_cb:
                cb('pause')
        # Move head to parking position before pause
        pause = self.gcode.get_int(
            'P', params, default=1, minval=0, maxval=1)
        if pause:
            self.printer.lookup_object('toolhead').move_to_idle_pos()
    def cmd_M26(self, params):
        # Set SD position
        if self.work_timer is not None:
            raise self.gcode.error("SD busy")
        pos = self.gcode.get_int('S', params, minval=0)
        self.file_position = pos
    def cmd_M27(self, params):
        # Report SD print status
        if self.current_file is None or self.work_timer is None:
            self.gcode.respond("Not SD printing.")
            return
        self.gcode.respond("SD printing byte %d/%d" % (
            self.file_position, self.file_size))
    def cmd_M32(self, params):
        # Select and start gcode file
        orig = params['#original']
        gco_f = orig[orig.find("M32") + 3:].split()[0].strip()
        gco_f = gco_f.replace('"', '')
        params['#original'] = 'M23 %s' % gco_f
        self.cmd_M23(params)
        self.simulate_print = False
        self.cmd_M24(params)
    def cmd_M37(self, params):
        orig = params['#original']
        gco_f = orig[orig.find("P") + 1:].split()[0].strip()
        gco_f = gco_f.replace('"', '')
        params['#original'] = 'M23 %s' % gco_f
        self.logger.info("Simulate: %s" % gco_f)
        self.cmd_M23(params)
        self.simulate_print = True
        self.logger.info("Simulated print starting")
        self.cmd_M24(params)
    def cmd_M98(self, params):
        # Run macro
        orig = params['#original']
        macro_f = orig[orig.find("P") + 1:].split()[0].strip()
        macro_f = macro_f.replace('"', '')
        params['#original'] = 'M23 %s' % macro_f
        self.cmd_M23(params)
        self.simulate_print = False
        self.logger.info("Executing macro")
        self.cmd_M24(params)
    # Background work timer
    def work_handler(self, eventtime):
        self.gcode.simulate_print = self.simulate_print
        self.logger.info("Starting SD card print (position %d)", self.file_position)
        self.reactor.unregister_timer(self.work_timer)
        try:
            self.current_file.seek(self.file_position)
        except:
            self.logger.exception("virtual_sdcard seek")
            self.gcode.respond_error("Unable to seek file")
            self.work_timer = None
            self.gcode.simulate_print = False
            return self.reactor.NEVER
        partial_input = ""
        lines = []
        while not self.must_pause_work:
            if not lines:
                # Read more data
                try:
                    data = self.current_file.read(8192)
                except:
                    self.logger.exception("virtual_sdcard read")
                    self.gcode.respond_error("Error on virtual sdcard read")
                    for cb in self.done_cb:
                        cb('error')
                    break
                if not data:
                    # End of file
                    self.current_file.close()
                    self.current_file = None
                    self.logger.info("Finished SD card print")
                    self.gcode.respond("Done printing file")
                    for cb in self.done_cb:
                        cb('done')
                    break
                lines = data.split('\n')
                lines[0] = partial_input + lines[0]
                partial_input = lines.pop()
                lines.reverse()
                self.reactor.pause(self.reactor.NOW)
                continue
            # Dispatch command
            try:
                res = self.gcode.process_batch(lines[-1])
                if not res:
                    self.reactor.pause(self.reactor.monotonic() + 0.100)
                    continue
            except self.gcode.error as e:
                self.logger.error("GCode error: %s" % e)
                self.gcode.respond_error("VSD GCode error: %s" % e)
                for cb in self.done_cb:
                    cb('error')
                break
            except:
                self.logger.exception("virtual_sdcard dispatch")
                break
            self.file_position += len(lines.pop()) + 1
        self.logger.info("Exiting SD card print (position %d)",
                         self.file_position)
        self.work_timer = None
        return self.reactor.NEVER

def load_config(config):
    return VirtualSD(config)
