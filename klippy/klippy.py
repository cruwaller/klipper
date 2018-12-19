#!/usr/bin/env python2
# Main code for host side printer firmware
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, optparse, logging, time
import collections, ConfigParser, importlib
import util, reactor, queuelogger, msgproto
import gcode, pins, mcu, toolhead
import hostcpu, gcodes

# Include extras path to search dir
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), "extras")))

status_delay = 1.0

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

class ConfigWrapper:
    error = ConfigParser.Error
    class sentinel:
        pass
    def __init__(self, printer, fileconfig, access_tracking, section):
        self.printer = printer
        self.fileconfig = fileconfig
        self.access_tracking = access_tracking
        self.section = section
    def get_printer(self):
        return self.printer
    def get_name(self):
        return self.section
    def _get_wrapper(self, parser, option, default,
                     minval=None, maxval=None, above=None, below=None):
        if (default is not self.sentinel
            and not self.fileconfig.has_option(self.section, option)):
            return default
        self.access_tracking[(self.section.lower(), option.lower())] = 1
        try:
            v = parser(self.section, option)
            if type(v) == str:
                v = v.strip('"|\'')
        except self.error as e:
            raise
        except:
            raise self.error("Unable to parse option '%s' in section '%s'" % (
                option, self.section))
        if minval is not None and v < minval:
            raise self.error(
                "Option '%s' in section '%s' must have minimum of %s" % (
                    option, self.section, minval))
        if maxval is not None and v > maxval:
            raise self.error(
                "Option '%s' in section '%s' must have maximum of %s" % (
                    option, self.section, maxval))
        if above is not None and v <= above:
            raise self.error(
                "Option '%s' in section '%s' must be above %s" % (
                    option, self.section, above))
        if below is not None and v >= below:
            raise self.error(
                "Option '%s' in section '%s' must be below %s" % (
                    option, self.section, below))
        return v
    def get(self, option, default=sentinel):
        return self._get_wrapper(self.fileconfig.get, option, default)
    def getint(self, option, default=sentinel,
               minval=None, maxval=None, above=None, below=None):
        return self._get_wrapper(self.fileconfig.getint, option, default,
                                 minval, maxval, above, below)
    def getfloat(self, option, default=sentinel,
                 minval=None, maxval=None, above=None, below=None):
        return self._get_wrapper(self.fileconfig.getfloat, option, default,
                                 minval, maxval, above, below)
    def getboolean(self, option, default=sentinel):
        return self._get_wrapper(self.fileconfig.getboolean, option, default)
    def getchoice(self, option, choices, default=sentinel):
        c = self.get(option, default)
        if c not in choices:
            raise self.error(
                "Choice '%s' for option '%s' in section '%s'"
                " is not a valid choice" % (c, option, self.section))
        if type(choices) == list:
            return c
        return choices[c] # dict
    def getsection(self, section):
        return ConfigWrapper(self.printer, self.fileconfig,
                             self.access_tracking, section)
    def has_section(self, section):
        return self.fileconfig.has_section(section)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in self.fileconfig.sections()
                if s.startswith(prefix)]

class ConfigLogger():
    def __init__(self, cfg, bglogger):
        self.logger = logging.getLogger('config')
        self.lines = ["===== Config file ====="]
        cfg.write(self)
        self.lines.append("=======================")
        data = "\n".join(self.lines)
        self.logger.info(data)
        bglogger.set_rollover_info("config", data)
    def write(self, data):
        self.lines.append(data.strip())

class Printer:
    config_error = ConfigParser.Error
    def __init__(self, input_fd, bglogger, start_args):
        self.logger = logging.getLogger('printer')
        self.bglogger = bglogger
        self.start_args = start_args
        self.reactor = reactor.Reactor()
        gc = gcode.GCodeParser(self, input_fd)
        self.objects = collections.OrderedDict({'gcode': gc})
        self.reactor.register_callback(self._connect)
        self.state_message = message_startup
        self.is_shutdown = False
        self.run_result = None
        self.state_cb = []
        self._extruders = {}
    class sentinel:
        pass
    def get_start_arg(self, name, default=sentinel):
        if default is not self.sentinel:
            return self.start_args.get(name, default)
        return self.start_args[name]
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def get_state_message(self):
        return self.state_message
    def extruder_add(self, extr):
        extruders = self._extruders
        if extr.index in extruders:
            raise self.config_error("Extruders cannot have same index!")
        extruders[extr.index] = extr
    def extruder_get(self, index=None,
                     default=ConfigWrapper.sentinel):
        extruders = self._extruders
        if index is None:
            return dict(extruders)
        if default is ConfigWrapper.sentinel:
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
    def lookup_object(self, name, default=ConfigWrapper.sentinel):
        if name in self.objects:
            return self.objects[name]
        if default is ConfigWrapper.sentinel:
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
        fileconfig = ConfigParser.RawConfigParser()
        config_file = self.start_args['config_file']
        res = fileconfig.read(config_file)
        if not res:
            raise self.config_error("Unable to open config file %s" % (
                config_file,))
        if self.bglogger is not None:
            ConfigLogger(fileconfig, self.bglogger)
        # Create printer components
        access_tracking = {}
        config = ConfigWrapper(self, fileconfig, access_tracking, 'printer')
        # Read config
        for m in [pins, mcu, hostcpu]:
            m.add_printer_objects(config)
        for section in fileconfig.sections():
            self.try_load_module(config, section)
        self._extruders = {}
        for m in [toolhead]:
            m.add_printer_objects(config)

        # Load generic gcode extensions
        gcodes.load_gcodes(config)
        # Load 'auto' modules
        self._try_load_extensions('modules', 'load_module', config)
        for section in fileconfig.sections():
            self.try_load_module(config, section, folder="modules")
        for section in fileconfig.sections():
            self.try_load_module(config, section, folder="modules_host")
        '''
        # Validate that there are no undefined parameters in the config file
        valid_sections = { s: 1 for s, o in access_tracking }
        for section_name in fileconfig.sections():
            section = section_name.lower()
            if section not in valid_sections and section not in self.objects:
                raise self.config_error(
                    "Section '%s' is not a valid config section" % (section,))
            for option in fileconfig.options(section_name):
                option = option.lower()
                if (section, option) not in access_tracking:
                    raise self.config_error(
                        "Option '%s' is not valid in section '%s'" % (
                            option, section))
        '''
        # Determine which printer objects have state callbacks
        self.state_cb = [o.printer_state for o in self.objects.values()
                         if hasattr(o, 'printer_state')]
    def _connect(self, eventtime):
        try:
            self._read_config()
            for cb in self.state_cb:
                if self.state_message is not message_startup:
                    return self.reactor.NEVER
                cb('connect')
            self._set_state(message_ready)
            for cb in self.state_cb:
                if self.state_message is not message_ready:
                    return self.reactor.NEVER
                cb('ready')
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
        return self.reactor.NEVER
    def run(self):
        systime = time.time()
        monotime = self.reactor.monotonic()
        self.logger.info("Start printer at %s (%.1f %.1f)",
                     time.asctime(time.localtime(systime)), systime, monotime)
        while 1:
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
                for cb in self.state_cb:
                    cb('disconnect')
            except:
                self.logger.exception("Unhandled exception during post run")
            return run_result
    def invoke_shutdown(self, msg):
        if self.is_shutdown:
            return
        self.is_shutdown = True
        self._set_state("%s%s" % (msg, message_shutdown))
        for cb in self.state_cb:
            cb('shutdown')
    def invoke_async_shutdown(self, msg):
        self.reactor.register_async_callback(
            (lambda e: self.invoke_shutdown(msg)))
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
    global status_delay
    usage = "%prog [options] <config file>"
    opts = optparse.OptionParser(usage)
    opts.add_option("-i", "--debuginput", dest="debuginput",
                    help="read commands from file instead of from tty port")
    opts.add_option("-I", "--input-tty", dest="inputtty", default='/tmp/printer',
                    help="Default input tty name (default is /tmp/printer)")
    opts.add_option("-l", "--logfile", dest="logfile",
                    help="write log to file instead of stderr")
    opts.add_option("-v", action="store_true", dest="verbose",
                    help="enable debug messages")
    opts.add_option("-o", "--debugoutput", dest="debugoutput",
                    help="write output to file instead of to serial port")
    opts.add_option("-d", "--dictionary", dest="dictionary", type="string",
                    action="callback", callback=arg_dictionary,
                    help="file to read for mcu protocol dictionary")
    opts.add_option("-s", "--status", dest="status_delay", type="int",
                    help="Status report interval")
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")
    start_args = {
        'config_file': os.path.abspath(os.path.normpath(os.path.expanduser(args[0]))),
        'start_reason': 'startup'}

    if options.status_delay:
        status_delay = options.status_delay

    bglogger = None
    input_fd = {}

    debuglevel = logging.INFO
    if options.verbose:
        debuglevel = logging.DEBUG
    if options.debuginput:
        start_args['debuginput'] = options.debuginput
        debuginput = open(options.debuginput, 'rb')
        input_fd['default'] = {'fd': debuginput.fileno(), 'prio': 0 }
    else:
        # Default input tty
        start_args['inputtty'] = options.inputtty
        input_fd['default'] = {'fd': util.create_pty(options.inputtty), 'prio': 0}
        # Reprap gui specific input
        if options.inputtty != '/tmp/reprapgui':
            input_fd['reprapgui'] = {'fd': util.create_pty('/tmp/reprapgui'), 'prio': 15}
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
