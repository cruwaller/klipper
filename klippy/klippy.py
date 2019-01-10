#!/usr/bin/env python2
# Main code for host side printer firmware
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, optparse, logging, time, collections, importlib
import util, reactor, queuelogger, msgproto
import gcode, configfile, pins, mcu, toolhead
import gcodes

# Include extras path to search dir
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), "extras")))

message_ready = "Printer is ready"

message_startup = """
The klippy host software is attempting to connect.
Please retry in a few moments.
Printer is not ready!
"""

message_restart = """
Once the underlying issue is corrected, use the "RESTART"
command to reload the config and restart the host software.
Printer is halted
"""

message_protocol_error = """
This type of error is frequently caused by running an older
version of the firmware on the micro-controller (fix by
recompiling and flashing the firmware).
Once the underlying issue is corrected, use the "RESTART"
command to reload the config and restart the host software.
Protocol error connecting to printer
"""

message_mcu_connect_error = """
Once the underlying issue is corrected, use the
"FIRMWARE_RESTART" command to reset the firmware, reload the
config, and restart the host software.
Error configuring printer
"""

message_shutdown = """
Once the underlying issue is corrected, use the
"FIRMWARE_RESTART" command to reset the firmware, reload the
config, and restart the host software.
Printer is shutdown
"""

class Printer:
    config_error = configfile.error
    def __init__(self, input_fd, bglogger, start_args):
        self.logger = logging.getLogger('printer')
        self.bglogger = bglogger
        self.start_args = start_args
        self.reactor = reactor.Reactor()
        self.reactor.register_callback(self._connect)
        self.state_message = message_startup
        self.is_shutdown = False
        self.run_result = None
        self.event_handlers = {}
        self._extruders = {}
        gc = gcode.GCodeParser(self, input_fd)
        self.objects = collections.OrderedDict({'gcode': gc})
        self.state_cb = [gc.printer_state]
    class sentinel:
        pass
    def get_start_arg(self, name, default=sentinel):
        if default is not self.sentinel:
            return self.start_args.get(name, default)
        return self.start_args[name]
    def get_start_args(self):
        return self.start_args
    def get_logger(self, name=None):
        if name is not None:
            return self.logger.getChild(name.replace(" ", "_"))
        return self.logger
    def get_reactor(self):
        return self.reactor
    def get_state_message(self):
        return self.state_message
    def extruder_add(self, extr):
        extruders = self._extruders
        if extr.index in extruders:
            raise self.config_error("Extruders cannot have same index!")
        extruders[extr.index] = extr
    def extruder_get(self, index=None, default=sentinel):
        extruders = self._extruders
        if index is None:
            return dict(extruders)
        if default is self.sentinel:
            return extruders.get(index)
        return extruders.get(index, default)
    def _set_state(self, msg):
        self.state_message = msg
        if msg != message_ready:
            if self.start_args.get('debuginput') is not None:
                self.request_exit('error_exit')
            else:
                for cb in self.state_cb:
                    cb('halt')
    def add_object(self, name, obj):
        if obj in self.objects:
            raise self.config_error(
                "Printer object '%s' already created" % (name,))
        self.objects[name] = obj
    def lookup_object(self, name, default=configfile.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is configfile.sentinel:
            raise self.config_error("Unknown config object '%s'" % (name,))
        return default
    def lookup_objects(self, module=None):
        if module is None:
            return list(self.objects.items())
        prefix = module + ' '
        objs = [(n, self.objects[n])
                for n in self.objects if n.startswith(prefix)]
        if module in self.objects:
            return [(module, self.objects[module])] + objs
        return objs
    def set_rollover_info(self, name, info, log=True):
        if log:
            self.logger.info(info)
        if self.bglogger is not None:
            self.bglogger.set_rollover_info(name, info)
    def try_load_module(self, config, section, folder=None):
        if section in self.objects:
            return self.objects[section]
        module_parts = section.split()
        module_name = module_parts[0]
        try:
            if folder is not None:
                module_name = ".".join([folder, module_name])
            mod = importlib.import_module(module_name)
        except ImportError:
            return None
        init_func = 'load_config'
        if len(module_parts) > 1:
            init_func = 'load_config_prefix'
        init_func = getattr(mod, init_func, None)
        if init_func is not None:
            self.objects[section] = init_func(config.getsection(section))
        return self.objects.get(section, None)
    def _try_load_extensions(self, folder, func, config):
        files = os.listdir(os.path.join(os.path.dirname(__file__), folder))
        for module in files:
            if module == '__init__.py' or module[-3:] != '.py':
                continue
            try:
                mod_name = module[:-3]
                mod_path = ".".join([folder, mod_name])
                mod = importlib.import_module(mod_path)
            except ImportError:
                continue
            init_func = getattr(mod, func, None)
            if init_func is not None:
                init_func(self, config)
    def _read_config(self):
        self.objects['configfile'] = pconfig = configfile.PrinterConfig(self)
        config = pconfig.read_main_config()
        if self.bglogger is not None:
            pconfig.log_config(config)
        self._extruders = {}
        all_sections = config.get_prefix_sections('')
        # Create printer components
        for m in [pins, mcu]:
            m.add_printer_objects(config)
        for section_config in all_sections:
            self.try_load_module(config, section_config.get_name())
        for m in [toolhead]:
            m.add_printer_objects(config)

        # Load generic gcode extensions
        gcodes.load_gcodes(config)
        # Load 'auto' modules
        self._try_load_extensions('modules', 'load_module', config)
        for section in all_sections:
            self.try_load_module(config, section.get_name(), folder="modules")
        for section in all_sections:
            self.try_load_module(config, section.get_name(), folder="modules_host")
        # Validate that there are no undefined parameters in the config file
        # pconfig.check_unused_options(config)
        # Determine which printer objects have state callbacks
        self.state_cb = [o.printer_state for o in self.objects.values()
                         if hasattr(o, 'printer_state')]
    def _connect(self, eventtime):
        try:
            self._read_config()
            for cb in self.state_cb:
                if self.state_message is not message_startup:
                    return
                cb('connect')
        except (self.config_error, pins.error) as e:
            self.logger.exception("Config error")
            self._set_state("%s%s" % (str(e), message_restart))
        except msgproto.error as e:
            self.logger.exception("Protocol error")
            self._set_state("%s%s" % (str(e), message_protocol_error))
        except mcu.error as e:
            self.logger.exception("MCU error during connect")
            self._set_state("%s%s" % (str(e), message_mcu_connect_error))
        except:
            self.logger.exception("Unhandled exception during connect")
            self._set_state("Internal error during connect.%s" % (
                message_restart,))
        try:
            self._set_state(message_ready)
            for cb in self.event_handlers.get("klippy:ready", []):
                if self.state_message is not message_ready:
                    return
                cb()
        except:
            logging.exception("Unhandled exception during ready callback")
            self.invoke_shutdown("Internal error during ready callback")
    def run(self):
        systime = time.time()
        monotime = self.reactor.monotonic()
        self.logger.info("Start printer at %s (%.1f %.1f)",
                     time.asctime(time.localtime(systime)), systime, monotime)
        # Enter main reactor loop
        try:
            self.reactor.run()
        except:
            self.logger.exception("Unhandled exception during run")
            return "error_exit"
        # Check restart flags
        run_result = self.run_result
        try:
            if run_result == 'firmware_restart':
                for n, m in self.lookup_objects(module='mcu'):
                    m.microcontroller_restart()
            self.send_event("klippy:disconnect")
        except:
            self.logger.exception("Unhandled exception during post run")
        return run_result
    def invoke_shutdown(self, msg):
        if self.is_shutdown:
            return
        self.is_shutdown = True
        self._set_state("%s%s" % (msg, message_shutdown))
        for cb in self.event_handlers.get("klippy:shutdown", []):
            try:
                cb()
            except:
                logging.exception("Exception during shutdown handler")
    def invoke_async_shutdown(self, msg):
        self.reactor.register_async_callback(
            (lambda e: self.invoke_shutdown(msg)))
    def register_event_handler(self, event, callback):
        self.event_handlers.setdefault(event, []).append(callback)
    def send_event(self, event, *params):
        return [cb(*params) for cb in self.event_handlers.get(event, [])]
    def request_exit(self, result):
        self.run_result = result
        self.reactor.end()


######################################################################
# Startup
######################################################################

def arg_dictionary(option, opt_str, value, parser):
    key, fname = "dictionary", value
    if '=' in value:
        mcu_name, fname = value.split('=', 1)
        key = "dictionary_" + mcu_name
    if parser.values.dictionary is None:
        parser.values.dictionary = {}
    parser.values.dictionary[key] = fname

def main():
    usage = "%prog [options] <config file>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-i", "--debuginput", dest="debuginput",
                    help="read commands from file instead of from tty port")
    opts.add_option("-I", "--input-tty", dest="inputtty", default='/tmp/printer',
                    help="input tty name (default is /tmp/printer)")
    opts.add_option("-l", "--logfile", dest="logfile",
                    help="write log to file instead of stderr")
    opts.add_option("-v", action="store_true", dest="verbose",
                    help="enable debug messages")
    opts.add_option("-o", "--debugoutput", dest="debugoutput",
                    help="write output to file instead of to serial port")
    opts.add_option("-d", "--dictionary", dest="dictionary", type="string",
                    action="callback", callback=arg_dictionary,
                    help="file to read for mcu protocol dictionary")
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")
    start_args = {
        'config_file': os.path.abspath(os.path.normpath(os.path.expanduser(args[0]))),
        'start_reason': 'startup'}

    input_fd = bglogger = None

    debuglevel = logging.INFO
    if options.verbose:
        debuglevel = logging.DEBUG
    if options.debuginput:
        start_args['debuginput'] = options.debuginput
        debuginput = open(options.debuginput, 'rb')
        input_fd = debuginput.fileno()
    else:
        start_args['inputtty'] = options.inputtty
        input_fd = util.create_pty(options.inputtty)
    if options.debugoutput:
        start_args['debugoutput'] = options.debugoutput
        start_args.update(options.dictionary)
    if options.logfile:
        start_args['logfile'] = options.logfile
        bglogger = queuelogger.setup_bg_logging(options.logfile, debuglevel)
    else:
        logging.basicConfig(level=debuglevel, format=queuelogger.LOGFORMAT)
    logging.getLogger().setLevel(debuglevel)
    logging.info("Starting Klippy...")
    start_args['software_version'] = util.get_git_version()
    versions = "\n".join([
        "Args: %s" % (sys.argv,),
        "Git version: %s" % (repr(start_args['software_version']),),
        "CPU: %s" % (util.get_cpu_info(),),
        "Python: %s" % (repr(sys.version),)])
    if bglogger is not None:
        logging.info(versions)

    # Start Printer() class
    while 1:
        if bglogger is not None:
            bglogger.clear_rollover_info()
            bglogger.set_rollover_info('versions', versions)
        printer = Printer(input_fd, bglogger, start_args)
        res = printer.run()
        if res in ['exit', 'error_exit']:
            break
        time.sleep(1.)
        logging.info("Restarting printer")
        start_args['start_reason'] = res

    if bglogger is not None:
        bglogger.stop()

    if res == 'error_exit':
        sys.exit(-1)


if __name__ == '__main__':
    util.fix_sigint()
    main()
