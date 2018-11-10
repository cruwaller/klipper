# Parse gcode commands
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, re, collections, Queue
import homing, extruder, heater, util

DEFAULT_PRIORITY = 0


class error(Exception):
    pass

class InputGcode:
    def __init__(self, gcode, fd_r, fd_func=None, need_ack=True, prio=0):
        self.prio = prio
        self.fd = fd_r
        self.fd_func = fd_func
        self.gcode = gcode
        self.need_ack = need_ack
    def __cmp__(self, other):
        return cmp(self.prio, other.prio)
    def _write(self, msg):
        if self.fd_func is not None:
            self.fd_func(msg)
        elif self.fd is not None:
            os.write(self.fd, msg)
    # Response handling
    def ack(self, msg=None):
        if not self.need_ack:
            return
        if msg:
            self._write("ok %s\n" % (msg,))
        else:
            self._write("ok\n")
    def respond(self, msg):
        self._write(msg+"\n")
    def respond_info(self, msg):
        lines = [l.strip() for l in msg.strip().split('\n')]
        self.respond("// " + "\n// ".join(lines))
    def respond_error(self, msg):
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.respond("Error: %s" % "\n".join(lines[:-1]))
        self.respond_info('%s' % (lines[-1].strip(),))
    def respond_stop(self, msg):
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.respond_info("\n".join(lines))
        self.respond('!! %s' % (lines[0].strip(),))


# Parse and handle G-Code commands
class GCodeParser:
    error = error
    RETRY_TIME = 0.100
    def __init__(self, printer, fd):
        self.logger = printer.logger.getChild('gcode')
        self.printer = printer
        self.fds = {}
        # Input handling
        self.reactor = printer.get_reactor()
        self.is_processing_data = False
        self.is_fileinput = not not printer.get_start_args().get("debuginput")
        if not self.is_fileinput:
            fd_handle = self.__start_reader(
                fd, self.process_data_fd, DEFAULT_PRIORITY)
            self.fds['default'] = fd_handle
            self.cmd_CREATE_PIPE(
                {"PATH": "/tmp/reprapgui", "PRIO": 15})
        self.bytes_read = 0
        self.input_log = collections.deque([], 50)
        # Command handling
        self.is_printer_ready = False
        self.base_gcode_handlers = self.gcode_handlers = {}
        self.ready_gcode_handlers = {}
        self.mux_commands = {}
        self.gcode_help = {}
        for cmd in self.all_handlers:
            func = getattr(self, 'cmd_' + cmd)
            wnr = getattr(self, 'cmd_' + cmd + '_when_not_ready', False)
            desc = getattr(self, 'cmd_' + cmd + '_help', None)
            self.register_command(cmd, func, wnr, desc)
            for a in getattr(self, 'cmd_' + cmd + '_aliases', []):
                self.register_command(a, func, wnr)
        # G-Code coordinate manipulation
        self.absolutecoord = self.absoluteextrude = True
        self.base_position = [0.0, 0.0, 0.0, 0.0]
        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.homing_position = [0.0, 0.0, 0.0, 0.0]
        self.speed_factor = 1. / 60.
        self.move_transform = self.move_with_transform = None
        self.position_with_transform = (lambda: [0., 0., 0., 0.])
        # G-Code state
        self.need_ack = False
        self.toolhead = self.extruder = None
        self.speed = 25.0
        self.axis2pos = {'X': 0, 'Y': 1, 'Z': 2, 'E': 3}
        self.simulate_print = False
        self.auto_temp_report = True
        # GCode input Queue
        self.process_queue = Queue.PriorityQueue(maxsize=20)  # TODO maxsize?
        self.logger.info("queue fileno %s" % self.process_queue)
        self.q_timer = self.reactor.register_timer(
            self.__process_queue, self.reactor.NOW)
        # Register control commands
        self.register_command("CREATE_PTY", self.cmd_CREATE_PIPE, when_not_ready=True)
        self.register_command("AUTO_TEMP_REPORT", self.cmd_AUTO_TEMP_REPORT,
                              when_not_ready=True,
                              desc="Disable/Enable auto temperature reports. [AUTO=0|1]")
    def cmd_CREATE_PIPE(self, params):
        if self.is_fileinput:
            return
        path = self.get_str("PATH", params)
        prio = self.get_int("PRIO", params, default=DEFAULT_PRIORITY,
                            minval=0, maxval=15)
        if path == self.printer.get_start_arg('inputtty'):
            msg = "Cannot override main input: %s" % (path,)
            self.logger.error(msg)
            params['#input'].respond_error(msg)
            return
        if path not in self.fds:
            fd = util.create_pty(path)
            fd_handle = self.__start_reader(fd, self.process_data_fd, prio)
            self.fds[path] = fd_handle
            self.logger.info("PTYs: %s created. fd = %s" % (path, fd,))
            if '#input' in params:
                params['#input'].respond("ok - input is %s" % (path,))
    def cmd_AUTO_TEMP_REPORT(self, params):
        self.auto_temp_report = self.get_int("AUTO", params,
            default=self.auto_temp_report, minval=0, maxval=1)
        params['#input'].respond_info("Auto temperature reporting %s" %
                                      bool(self.auto_temp_report))
    def __start_reader(self, fd, func, prio=0):
        handle = self.reactor.register_fd_thread(fd, func)
        handle.partial_input = ""
        handle.priority = prio
        return handle
    def register_fd(self, name, fd, prio=5):
        if name not in self.fds:
            fd_handle = self.__start_reader(fd, self.process_data_fd, prio)
            self.fds[name] = fd_handle
    def temperature_auto_report(self, val=True):
        self.auto_temp_report = val
    def register_command(self, cmd, func, when_not_ready=False, desc=None):
        if func is None:
            if cmd in self.ready_gcode_handlers:
                del self.ready_gcode_handlers[cmd]
            if cmd in self.base_gcode_handlers:
                del self.base_gcode_handlers[cmd]
            return
        #if cmd in self.ready_gcode_handlers:
        #    raise error("gcode command %s already registered" % (cmd,))
        if not (len(cmd) >= 2 and not cmd[0].isupper() and cmd[1].isdigit()):
            origfunc = func
            func = lambda params: origfunc(self.get_extended_params(params))
        else:
            raise self.printer.config_error("GCode must be is capitals! cmd: %s" % cmd)
        self.ready_gcode_handlers[cmd] = func
        if when_not_ready:
            self.base_gcode_handlers[cmd] = func
        if desc is not None:
            self.gcode_help[cmd] = desc
    def get_command_handler(self, cmd):
        return self.ready_gcode_handlers.get(cmd, None)
    def register_mux_command(self, cmd, key, value, func, desc=None, when_not_ready=False):
        prev = self.mux_commands.get(cmd)
        if prev is None:
            self.register_command(cmd, self.cmd_mux, desc=desc, when_not_ready=when_not_ready)
            self.mux_commands[cmd] = prev = (key, {})
        prev_key, prev_values = prev
        if prev_key != key:
            raise error("mux command %s %s %s may have only one key (%s)" % (
                cmd, key, value, prev_key))
        if value in prev_values:
            raise error("mux command %s %s %s already registered (%s)" % (
                cmd, key, value, prev_values))
        prev_values[value] = func
    def set_move_transform(self, transform):
        if transform is None:
            # Remove transform
            self.move_transform = None
            self.move_with_transform = self.toolhead.move
            self.position_with_transform = self.toolhead.get_position
            return
        if self.move_transform is not None:
            raise self.printer.config_error(
                "G-Code move transform already specified")
        self.move_transform = transform
        self.move_with_transform = transform.move
        self.position_with_transform = transform.get_position
    def stats(self, eventtime):
        return False, "gcodein=%d" % (self.bytes_read,)
    def get_status(self, eventtime):
        busy = self.is_processing_data
        return {'speed_factor': self.speed_factor * 60., 'busy': busy}
    def printer_state(self, state):
        if state == 'shutdown':
            if not self.is_printer_ready:
                return
            self.is_printer_ready = False
            self.gcode_handlers = self.base_gcode_handlers
            self.dump_debug()
            if self.is_fileinput:
                self.printer.request_exit('error_exit')
            return
        elif state == 'connect':
            pass
        if state != 'ready':
            return
        self.is_printer_ready = True
        self.gcode_handlers = self.ready_gcode_handlers
        # Lookup printer components
        self.toolhead = self.printer.lookup_object('toolhead')
        if self.move_transform is None:
            self.move_with_transform = self.toolhead.move
            self.position_with_transform = self.toolhead.get_position
        self.extruder = self.printer.extruder_get(0)
        if self.extruder is not None:
            self.toolhead.set_extruder(self.extruder)
    def reset_last_position(self):
        self.last_position = self.position_with_transform()
    def motor_heater_off(self):
        self.toolhead.motor_off()
        print_time = self.toolhead.get_last_move_time()
        for h in self.printer.lookup_module_objects("heater"):
            h.set_temp(print_time, 0.0)
        for fan in self.printer.lookup_module_objects('fan'):
            fan.set_speed(print_time, 0.0)
    def dump_debug(self):
        out = ["Dumping gcode input %d blocks" % (
            len(self.input_log),)]
        for eventtime, data in self.input_log:
            out.append("Read %f: %s" % (eventtime, repr(data)))
        try:
            extrude_factor = self.extruder.extrude_factor
        except AttributeError:
            extrude_factor = 0.
        out.append(
            "gcode state: absolutecoord=%s absoluteextrude=%s"
            " base_position=%s last_position=%s homing_position=%s"
            " speed_factor=%s extrude_factor=%s speed=%s" % (
                self.absolutecoord, self.absoluteextrude,
                self.base_position, self.last_position, self.homing_position,
                self.speed_factor, extrude_factor, self.speed))
        self.logger.info("\n".join(out))
    # Parse input into commands
    args_r = re.compile('([A-Z_]+|[A-Z*/])')
    def process_command(self, gco_in):
        # Ignore comments and leading/trailing spaces
        line = origline = gco_in.gcode.strip()
        cpos = line.find(';')
        if cpos >= 0:
            line = line[:cpos]
        # Break command into parts
        parts = self.args_r.split(line.upper())[1:]
        params = { parts[i]: parts[i+1].strip()
                   for i in range(0, len(parts), 2) }
        params['#original'] = origline
        if parts and parts[0] == 'N':
            # Skip line number at start of command
            del parts[:2]
        if not parts:
            # Treat empty line as empty command
            parts = ['', '']
        params['#command'] = cmd = parts[0] + parts[1].strip()
        params['#input'] = gco_in
        params['#gcode'] = self
        # Invoke handler for command
        handler = self.gcode_handlers.get(cmd, self.cmd_default)
        try:
            handler(params)
        except error as e:
            gco_in.respond_error(str(e))
            self.reset_last_position()
            if not gco_in.need_ack:
                raise
        except:
            msg = 'Internal error on command:"%s"' % (cmd,)
            self.logger.exception(msg)
            self.printer.invoke_shutdown(msg)
            gco_in.respond_stop(msg)
            if not gco_in.need_ack:
                raise
        gco_in.ack()

    def process_commands(self, commands, need_ack=True):
        for line in commands:
            self.process_command(
                InputGcode(line, None, need_ack=need_ack))

    m112_r = re.compile('^(?:[nN][0-9]+)?\s*[mM]112(?:\s|$)')
    def process_data_fd(self, eventtime, handler):
        prio = handler.priority
        fd_r = handler.fileno()
        # Read input, separate by newline, and add to queue
        try:
            data = os.read(fd_r, 4096)
        except OSError:
            return False
        # self.logger.info("input: %s" % data)
        self.input_log.append((eventtime, data))
        self.bytes_read += len(data)
        lines = data.split('\n')
        lines[0] = handler.partial_input + lines[0]
        handler.partial_input = lines.pop()
        if "M112" in data:
            # Emergency stop, kill immediately
            self.process_queue.put(
                InputGcode("M112", fd_r, prio=99),
                block=True)
            return False
        for line in lines:
            #if self.m112_r.match(line) is not None:
            #    self.process_queue.put(
            #        InputGcode("M112", fd_r, prio=99),
            #        block=True)
            #    return True
            if len(line) <= 1:
                continue
            self.process_queue.put(
                InputGcode(line, fd_r, prio=prio),
                block=True)
        return False # do not stop processing

    def __process_queue(self, eventtime):
        self.reactor.unregister_timer(self.q_timer)
        while True:
            try:
                _gco = self.process_queue.get_nowait()
                self.is_processing_data = True
                try:
                    self.process_command(_gco)
                finally:
                    self.is_processing_data = False
            except Queue.Empty:
                self.reactor.pause(self.reactor.monotonic() + 0.100)
    def push_command_to_queue(self, cmd, resp_func=None, prio=5):
        lines = cmd.split('\n')
        for line in lines:
            self.process_queue.put(
                InputGcode(line, None, fd_func=resp_func, prio=prio),
                block=True)

    def process_batch(self, command):
        if self.is_processing_data:
            return False
        self.is_processing_data = True
        try:
            self.process_commands([command], need_ack=False)
        finally:
            self.is_processing_data = False
        return True
    def run_script_from_command(self, script):
        prev_need_ack = self.need_ack
        try:
            self.process_commands(script.split('\n'), need_ack=False)
        finally:
            self.need_ack = prev_need_ack
    def respond(self, fd, msg):
        if self.is_fileinput:
            return
        if fd is not None:
            os.write(fd, msg+"\n")
        else:
            for path, handler in self.fds.items():
                os.write(handler.fd, msg + "\n")
    def respond_info(self, fd, msg):
        self.logger.debug(msg)
        lines = [l.strip() for l in msg.strip().split('\n')]
        self.respond(fd, "// " + "\n// ".join(lines))
    def respond_error(self, fd, msg):
        self.logger.warning(msg.replace('\n', ". "))
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.respond(fd, "Error: %s" % "\n".join(lines[:-1]))
        self.respond_info(fd, '%s' % (lines[-1].strip(),))
    def respond_stop(self, fd, msg):
        self.logger.error(msg.replace('\n', ". "))
        lines = msg.strip().split('\n')
        if len(lines) > 1:
            self.respond_info(fd, "\n".join(lines))
        self.respond(fd, '!! %s' % (lines[0].strip(),))
        if self.is_fileinput:
            self.printer.request_exit('error_exit')
    # Parameter parsing helpers
    class sentinel: pass
    def get_str(self, name, params, default=sentinel, parser=str,
                minval=None, maxval=None, above=None, below=None):
        if name not in params:
            if default is self.sentinel:
                raise error("Error on '%s': missing %s" % (
                    params['#original'], name))
            return default
        try:
            value = parser(params[name])
        except:
            raise error("Error on '%s': unable to parse %s" % (
                params['#original'], params[name]))
        if minval is not None and value < minval:
            raise self.error("Error on '%s': %s must have minimum of %s" % (
                params['#original'], name, minval))
        if maxval is not None and value > maxval:
            raise self.error("Error on '%s': %s must have maximum of %s" % (
                params['#original'], name, maxval))
        if above is not None and value <= above:
            raise self.error("Error on '%s': %s must be above %s" % (
                params['#original'], name, above))
        if below is not None and value >= below:
            raise self.error("Error on '%s': %s must be below %s" % (
                params['#original'], name, below))
        return value
    def get_int(self, name, params, default=sentinel, minval=None, maxval=None):
        return self.get_str(name, params, default, parser=int,
                            minval=minval, maxval=maxval)
    def get_float(self, name, params, default=sentinel,
                  minval=None, maxval=None, above=None, below=None):
        return self.get_str(name, params, default, parser=float, minval=minval,
                            maxval=maxval, above=above, below=below)
    extended_r = re.compile(
        r'^\s*(?:N[0-9]+\s*)?'
        r'(?P<cmd>[a-zA-Z_][a-zA-Z_]+)(?:\s+|$)'
        r'(?P<args>[^#*;]*?)'
        r'\s*(?:[#*;].*)?$')
    def get_extended_params(self, params):
        m = self.extended_r.match(params['#original'])
        if m is None:
            # Not an "extended" command
            return params
        eargs = m.group('args')
        try:
            eparams = [earg.split('=', 1) for earg in eargs.split()]
            eparams = { k.upper(): v for k, v in eparams }
            eparams.update({k: params[k] for k in params if k.startswith('#')})
            return eparams
        except ValueError as e:
            raise error("Malformed command '%s'" % (params['#original'],))
    # Temperature wrappers
    def get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        for key, e in self.printer.extruder_get().items():
            heater = e.get_heater()
            if heater is not None:
                cur, target = heater.get_temp(eventtime)
                out.append("T%d:%.1f /%.1f" % (e.get_index(), cur, target))
        heater = self.printer.lookup_object('heater bed', None)
        if heater is not None:
            cur, target = heater.get_temp(eventtime)
            out.append("B:%.1f /%.1f" % (cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    def bg_temp(self, heater, resp_fd=None):
        if self.is_fileinput:
            return
        eventtime = self.reactor.monotonic()
        while self.is_printer_ready and heater.check_busy(eventtime):
            self.toolhead.get_last_move_time()
            tempstr = self.get_temp(eventtime)
            if self.auto_temp_report:
                self.respond(resp_fd, tempstr) # auto report to all clients
            else:
                self.logger.debug(tempstr)
            eventtime = self.reactor.pause(eventtime + 1.)
    def set_temp(self, params, is_bed=False, wait=False):
        heater = None
        temp = 0.
        if self.simulate_print is False:
            if "S" in params:
                temp = self.get_float('S', params, 0.)
            elif "R" in params:
                temp = self.get_float('R', params, 0.)
            if temp < 0:
                temp = 0.
        if is_bed:
            heater = self.printer.lookup_object('heater bed', None)
        else:
            index = None
            if 'T' in params:
                index = self.get_int('T', params, None)
            elif 'P' in params:
                index = self.get_int('P', params, None)

            if index is not None:
                e = self.printer.extruder_get(index)
                if e is not None:
                    heater = e.get_heater()
            elif self.extruder is not None:
                heater = self.extruder.get_heater()

        if heater is None:
            if temp > 0.:
                params['#input'].respond_error("Heater not configured")
            return
        print_time = self.toolhead.get_last_move_time()
        try:
            heater.set_temp(print_time, temp)
        except heater.error as e:
            raise error(str(e))
        if wait and temp and self.simulate_print is False:
            self.bg_temp(heater, params['#input'].fd)
    def set_fan_speed(self, speed, index, handler=None):
        fan = self.printer.lookup_object('fan %d' % (index,), None)
        if fan is None:
            if speed and not self.is_fileinput and handler is not None:
                handler.respond_info("Fan not configured")
            return
        print_time = self.toolhead.get_last_move_time()
        fan.set_speed(print_time, speed)
    # G-Code special command handlers
    def cmd_default(self, params):
        if not self.is_printer_ready:
            params['#input'].respond_error(self.printer.get_state_message())
            return
        cmd = params.get('#command')
        if not cmd:
            self.logger.debug(params['#original'])
            return
        if cmd[0] == 'T' and len(cmd) > 1 and cmd[1].isdigit():
            # Tn command has to be handled specially
            self.cmd_Tn(params)
            return
        params['#input'].respond_info('Unknown command:"%s"' % (cmd,))
    def cmd_Tn(self, params):
        # Select Tool
        index = self.get_int('T', params)
        if index < 0:
            # Reprap WebGui uses T-1 to deselect filament, skip it
            return
        e = self.printer.extruder_get(index)
        if e is None:
            params['#input'].respond_error(
                "Extruder %d not configured" % (index,))
            return
        if self.extruder is e:
            return
        if self.extruder is not None:
            self.run_script_from_command(self.extruder.get_activate_gcode(False))
        try:
            self.toolhead.set_extruder(e)
        except homing.EndstopError as e:
            raise error(str(e))
        self.extruder = e
        self.reset_last_position()
        self.base_position[3] = self.last_position[3]
        self.run_script_from_command(self.extruder.get_activate_gcode(True))
    def cmd_mux(self, params):
        key, values = self.mux_commands[params['#command']]
        if None in values:
            key_param = self.get_str(key, params, None)
        else:
            key_param = self.get_str(key, params)
        if key_param not in values:
            raise error("The value '%s' is not valid for %s" % (key_param, key))
        values[key_param](params)
    all_handlers = [
        'G1', 'G4', 'G28', 'M18', 'M400',
        'G20', 'M82', 'M83', 'G90', 'G91', 'G92', 'M114', 'M220', 'M221',
        'SET_GCODE_OFFSET', 'M206',
        'M105', 'M104', 'M109', 'M140', 'M190', 'M106', 'M107',
        'M112', 'M115', 'IGNORE', 'QUERY_ENDSTOPS', 'GET_POSITION',
        'RESTART', 'FIRMWARE_RESTART', 'ECHO', 'STATUS', 'HELP']
    # G-Code movement commands
    cmd_G1_aliases = ['G0']
    def cmd_G1(self, params):
        # Move
        try:
            for axis in 'XYZ':
                if axis in params:
                    v = float(params[axis])
                    pos = self.axis2pos[axis]
                    if not self.absolutecoord:
                        # value relative to position of last move
                        self.last_position[pos] += v
                    else:
                        # value relative to base coordinate position
                        self.last_position[pos] = v + self.base_position[pos]
            if 'E' in params:
                try:
                    v = ( (float(params['E']) * self.extruder.extrude_factor)
                          if (self.simulate_print is False) else 0.0 )
                except AttributeError:
                    v = 0.
                if not self.absolutecoord or not self.absoluteextrude:
                    # value relative to position of last move
                    self.last_position[3] += v
                else:
                    # value relative to base coordinate position
                    self.last_position[3] = v + self.base_position[3]
            if 'F' in params:
                speed = float(params['F']) * self.speed_factor
                if speed <= 0.:
                    raise error("Invalid speed in '%s'" % (params['#original'],))
                self.speed = speed
        except ValueError as e:
            raise error("Unable to parse move '%s'" % (params['#original'],))
        try:
            self.move_with_transform(self.last_position, self.speed)
        except homing.EndstopError as e:
            raise error(str(e))
    cmd_G4_aliases = ['G04']
    def cmd_G4(self, params):
        # Dwell
        if 'S' in params:
            delay = self.get_float('S', params, minval=0.)
        else:
            delay = self.get_float('P', params, 0., minval=0.) / 1000.
        self.toolhead.dwell(delay)
    def cmd_G28(self, params):
        # Move to origin
        axes = []
        homing_order = self.toolhead.homing_order
        for axis in homing_order:
            if axis in params:
                axes.append(self.axis2pos[axis])
        if not axes:
            axes = [self.axis2pos[axis] for axis in homing_order]
        homing_state = homing.Homing(self.toolhead)
        if self.is_fileinput:
            homing_state.set_no_verify_retract()
        try:
            homing_state.home_axes(axes)
        except homing.EndstopError as e:
            raise error(str(e))
        for axis in homing_state.get_axes():
            self.base_position[axis] = self.homing_position[axis]
        self.reset_last_position()
    cmd_M18_aliases = ["M84"]
    def cmd_M18(self, params):
        # Turn off motors
        # Should wait queued moves to prevent unwanted error!
        self.toolhead.wait_moves()
        self.toolhead.motor_off()
    def cmd_M400(self, params):
        # Wait for current moves to finish
        self.toolhead.wait_moves()
    # G-Code coordinate manipulation
    def cmd_G20(self, params):
        # Set units to inches
        params['#input'].respond_error(
            'Machine does not support G20 (inches) command')
    def cmd_M82(self, params):
        # Use absolute distances for extrusion
        self.absoluteextrude = True
    def cmd_M83(self, params):
        # Use relative distances for extrusion
        self.absoluteextrude = False
    def cmd_G90(self, params):
        # Use absolute coordinates
        self.absolutecoord = True
    def cmd_G91(self, params):
        # Use relative coordinates
        self.absolutecoord = False
    def cmd_G92(self, params):
        # Set position
        offsets = { p: self.get_float(a, params)
                    for a, p in self.axis2pos.items() if a in params }
        for p, offset in offsets.items():
            if p == 3:
                try:
                    offset *= self.extruder.extrude_factor
                except AttributeError:
                    pass
            self.base_position[p] = self.last_position[p] - offset
        if not offsets:
            self.base_position = list(self.last_position)
    cmd_M114_when_not_ready = True
    def cmd_M114(self, params):
        # Get Current Position
        p = [lp - bp for lp, bp in zip(self.last_position, self.base_position)]
        try:
            p[3] /= self.extruder.extrude_factor
        except AttributeError:
            pass
        params['#input'].respond("X:%.3f Y:%.3f Z:%.3f E:%.3f" % tuple(p))
    def cmd_M220(self, params):
        # Set speed factor override percentage
        value = self.get_float('S', params, 100., above=0.) / (60. * 100.)
        self.speed_factor = value
    def cmd_M221(self, params):
        # Set extrude factor override percentage
        new_extrude_factor = self.get_float('S', params, 100., above=0.) / 100.
        index = None
        # extruder number
        if 'D' in params:
            index = self.get_int('D', params, None)
        elif 'T' in params:
            index = self.get_int('T', params, None)
        if index is None:
            extr = self.extruder
        else:
            extr = self.printer.extruder_get(index)
        if extr is not None:
            last_e_pos = self.last_position[3]
            e_value = (last_e_pos - self.base_position[3]) / extr.extrude_factor
            self.base_position[3] = last_e_pos - e_value * new_extrude_factor
            extr.extrude_factor = new_extrude_factor
    cmd_SET_GCODE_OFFSET_help = \
        "Set a virtual offset to positions. " \
        "args: [X|Y|Z_ADJUST=offset]"
    def cmd_SET_GCODE_OFFSET(self, params):
        for axis, pos in self.axis2pos.items():
            if axis in params:
                offset = self.get_float(axis, params)
            elif axis + '_ADJUST' in params:
                offset = self.homing_position[pos]
                offset += self.get_float(axis + '_ADJUST', params)
            else:
                continue
            delta = offset - self.homing_position[pos]
            self.last_position[pos] += delta
            self.base_position[pos] += delta
            self.homing_position[pos] = offset
    def cmd_M206(self, params):
        # Offset axes
        offsets = { self.axis2pos[a]: self.get_float(a, params)
                    for a in 'XYZ' if a in params }
        for p, offset in offsets.items():
            self.base_position[p] -= self.homing_position[p] + offset
            self.homing_position[p] = -offset
    # G-Code temperature and fan commands
    cmd_M105_when_not_ready = True
    def cmd_M105(self, params):
        # Get Extruder Temperature
        params['#input'].ack(self.get_temp(self.reactor.monotonic()))
    def cmd_M104(self, params):
        # Set Extruder Temperature
        self.set_temp(params)
    def cmd_M109(self, params):
        # Set Extruder Temperature and Wait
        self.set_temp(params, wait=True)
    def cmd_M140(self, params):
        # Set Bed Temperature
        self.set_temp(params, is_bed=True)
    def cmd_M190(self, params):
        # Set Bed Temperature and Wait
        self.set_temp(params, is_bed=True, wait=True)
    def cmd_M106(self, params):
        # Set fan speed
        self.set_fan_speed(self.get_float('S', params, 255., minval=0.) / 255.,
                           self.get_int('P', params, 0),
                           params['#input'])
    def cmd_M107(self, params):
        # Turn fan off
        self.set_fan_speed(0., self.get_int('P', params, 0),
                           params['#input'])
    # G-Code miscellaneous commands
    cmd_M112_help = "Emergency shutdown"
    cmd_M112_when_not_ready = True
    def cmd_M112(self, params):
        # Emergency Stop
        self.printer.invoke_shutdown("Shutdown due to M112 command")
    cmd_M115_help = "Get firmware version and capabilities"
    cmd_M115_when_not_ready = True
    def cmd_M115(self, params):
        # Get Firmware Version and Capabilities
        software_version = self.printer.get_start_args().get('software_version')
        kw = {"FIRMWARE_NAME": "Klipper", "FIRMWARE_VERSION": software_version}
        params['#input'].ack(" ".join(["%s:%s" % (k, v) for k, v in kw.items()]))
    cmd_IGNORE_when_not_ready = True
    cmd_IGNORE_aliases = ["G21", "M110", "M21"]
    def cmd_IGNORE(self, params):
        # Commands that are just silently accepted
        pass
    cmd_QUERY_ENDSTOPS_help = "Report on the status of each endstop"
    cmd_QUERY_ENDSTOPS_aliases = ["M119"]
    def cmd_QUERY_ENDSTOPS(self, params):
        # Get Endstop Status
        res = homing.query_endstops(self.toolhead)
        params['#input'].respond(" ".join(["%s:%s" % (name, ["open", "TRIGGERED"][not not t])
                                 for name, t in res]))
    cmd_GET_POSITION_help = "Get current axes positions"
    cmd_GET_POSITION_when_not_ready = True
    def cmd_GET_POSITION(self, params):
        if self.toolhead is None:
            self.cmd_default(params)
            return
        kin = self.toolhead.get_kinematics()
        steppers = []
        rails = kin.get_rails()
        for rail in rails:
            steppers += rail.get_steppers()
        mcu_pos = " ".join(["%s:%d" % (s.get_name(), s.get_mcu_position())
                            for s in steppers])
        stepper_pos = " ".join(
            ["%s:%.6f" % (s.get_name(), s.mcu_stepper.get_commanded_position())
             for s in steppers])
        kinematic_pos = " ".join(["%s:%.6f"  % (a, v)
                                  for a, v in zip("XYZE", kin.calc_position())])
        toolhead_pos = " ".join(["%s:%.6f" % (a, v) for a, v in zip(
            "XYZE", self.toolhead.get_position())])
        gcode_pos = " ".join(["%s:%.6f"  % (a, v)
                              for a, v in zip("XYZE", self.last_position)])
        base_pos = " ".join(["%s:%.6f"  % (a, v)
                             for a, v in zip("XYZE", self.base_position)])
        homing_pos = " ".join(["%s:%.6f"  % (a, v)
                               for a, v in zip("XYZ", self.homing_position)])
        params['#input'].respond_info(
            "mcu: %s\n"
            "stepper: %s\n"
            "kinematic: %s\n"
            "toolhead: %s\n"
            "gcode: %s\n"
            "gcode base: %s\n"
            "gcode homing: %s" % (
                mcu_pos, stepper_pos, kinematic_pos, toolhead_pos,
                gcode_pos, base_pos, homing_pos))
    def request_restart(self, result, handler):
        if self.is_printer_ready:
            handler.respond_info("Preparing to restart...")
            self.motor_heater_off()
            self.toolhead.dwell(0.500)
            self.toolhead.wait_moves()
        self.printer.request_exit(result)
    cmd_RESTART_when_not_ready = True
    cmd_RESTART_help = "Reload config file and restart host software"
    def cmd_RESTART(self, params):
        self.request_restart('restart', params['#input'])
    cmd_FIRMWARE_RESTART_when_not_ready = True
    cmd_FIRMWARE_RESTART_help = "Restart firmware, host, and reload config"
    def cmd_FIRMWARE_RESTART(self, params):
        self.request_restart('firmware_restart', params['#input'])
    cmd_ECHO_help = "Repond same command back to sender"
    cmd_ECHO_when_not_ready = True
    def cmd_ECHO(self, params):
        params['#input'].respond_info(params['#original'])
    cmd_STATUS_when_not_ready = True
    cmd_STATUS_help = "Report the printer status"
    def cmd_STATUS(self, params):
        msg = self.printer.get_state_message()
        if self.is_printer_ready:
            params['#input'].respond_info(msg)
        else:
            params['#input'].respond_error(msg)
    cmd_HELP_when_not_ready = True
    def cmd_HELP(self, params):
        cmdhelp = []
        if not self.is_printer_ready:
            cmdhelp.append("Printer is not ready - not all commands available.")
        cmdhelp.append("Available extended commands:")
        for cmd in sorted(self.gcode_handlers):
            if cmd in self.gcode_help:
                cmdhelp.append("%-10s: %s" % (cmd, self.gcode_help[cmd]))
        params['#input'].respond_info("\n".join(cmdhelp))
