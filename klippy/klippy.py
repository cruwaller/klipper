#!/usr/bin/env python2
# Main code for host side printer firmware
#
# Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import sys, os, optparse, logging, time, threading
import collections, ConfigParser, importlib

# Include extras path to search dir
sys.path.append(os.path.join(os.path.dirname(__file__), "extras"))

import util, reactor, queuelogger, msgproto
import gcode, pins, mcu, chipmisc, toolhead, extruder, heater


status_delay = 1.0

message_ready = "Printer is ready"

message_startup = """
The klippy host software is attempting to connect.  Please
retry in a few moments.
Printer is not ready
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

class InputLink:
    def __init__(self, tty=None, dbgin=None):
        if dbgin is not None:
            self.fd_w = self.fd_r = open(dbgin, 'rb').fileno()
        elif tty is not None:
            self.fd_w = self.fd_r = util.create_pty(tty)
        else:
            # Create a pipes
            self.fd_rx_r, self.fd_rx_w = os.pipe() # Klippy RX
            self.fd_tx_r, self.fd_tx_w = os.pipe() # Klippy TX

            self.fd_r = self.fd_rx_r
            self.fd_w = self.fd_tx_w

class ConfigWrapper:
    error = ConfigParser.Error
    class sentinel:
        pass
    def __init__(self, printer, fileconfig, section):
        self.printer = printer
        self.fileconfig = fileconfig
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
        self.printer.all_config_options[
            (self.section.lower(), option.lower())] = 1
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
    def getint(self, option, default=sentinel, minval=None, maxval=None):
        return self._get_wrapper(
            self.fileconfig.getint, option, default, minval, maxval)
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
                "Option '%s' in section '%s' is not a valid choice" % (
                    option, self.section))
        return choices[c]
    def getsection(self, section):
        return ConfigWrapper(self.printer, self.fileconfig, section)
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
        self.name = "Klipper printer"
        self.logger = logging.getLogger('printer')
        self.bglogger = bglogger
        self.start_args = start_args
        if bglogger is not None:
            bglogger.set_rollover_info("config", None)
        self.reactor = reactor.Reactor()
        gc = gcode.GCodeParser(self, input_fd)
        self.objects = collections.OrderedDict({'gcode': gc})
        self.stats_timer = self.reactor.register_timer(self._stats)
        self.connect_timer = self.reactor.register_timer(
            self._connect, self.reactor.NOW)
        self.all_config_options = {}
        self.state_message = message_startup
        self.is_shutdown = False
        self.async_shutdown_msg = ""
        self.run_result = None
        self.stats_cb = []
        self.state_cb = []
        self.starttime = time.time() # RepRap WebGUI
    def get_start_args(self):
        return self.start_args
    def get_reactor(self):
        return self.reactor
    def get_state_message(self):
        return self.state_message
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
    def lookup_module_objects(self, module_name):
        prefix = module_name + ' '
        objs = [self.objects[n] for n in self.objects if n.startswith(prefix)]
        if module_name in self.objects:
            return [self.objects[module_name]] + objs
        return objs
    def set_rollover_info(self, name, info):
        if self.bglogger is not None:
            self.bglogger.set_rollover_info(name, info)
    def _stats(self, eventtime, force_output=False):
        stats = [cb(eventtime) for cb in self.stats_cb]
        if max([s[0] for s in stats] + [force_output]):
            self.logger.info("Stats %.1f: %s", eventtime,
                             ' '.join([s[1] for s in stats]))
        return eventtime + status_delay
    def _try_load_module(self, config, section):
        if section in self.objects:
            return
        module_parts = section.split()
        module_name = module_parts[0]
        try:
            mod = importlib.import_module(module_name)
        except ImportError:
            return
        init_func = 'load_config'
        if len(module_parts) > 1:
            init_func = 'load_config_prefix'
        init_func = getattr(mod, init_func, None)
        if init_func is not None:
            self.objects[section] = init_func(config.getsection(section))
    def _try_load_extensions(self, folder, func):
        files = os.listdir(os.path.join(os.path.dirname(__file__), folder))
        for module in files:
            if module == '__init__.py' or module[-3:] != '.py':
                continue
            try:
                mod_name = ".".join([folder, module[:-3]])
                mod = importlib.import_module(mod_name)
            except ImportError as e:
                continue
            init_func = getattr(mod, func, None)
            if init_func is not None:
                init_func(self)
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
        config = ConfigWrapper(self, fileconfig, 'printer')
        # Read my name
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        # Read config
        for m in [pins, mcu]:
            m.add_printer_objects(self, config)
        self.logger.info("========================================")
        for section in fileconfig.sections():
            self._try_load_module(config, section)
        self.logger.info("========================================")
        for m in [chipmisc, toolhead, extruder]:
            m.add_printer_objects(self, config)

        # Load gcode extensions
        self._try_load_extensions('gcodes', 'load_gcode')
        # Load modules
        self._try_load_extensions('modules', 'load_module')

        '''
        # Validate that there are no undefined parameters in the config file
        valid_sections = { s: 1 for s, o in self.all_config_options }
        for section in fileconfig.sections():
            section = section.lower()
            if section not in valid_sections and section not in self.objects:
                raise self.config_error("Unknown config file section '%s'" % (
                    section,))
            for option in fileconfig.options(section):
                option = option.lower()
                if (section, option) not in self.all_config_options:
                    raise self.config_error(
                        "Unknown option '%s' in section '%s'" % (
                            option, section))
        '''
        # Determine which printer objects have stats/state callbacks
        self.stats_cb = [o.stats for o in self.objects.values()
                         if hasattr(o, 'stats')]
        self.state_cb = [o.printer_state for o in self.objects.values()
                         if hasattr(o, 'printer_state')]
    def _connect(self, eventtime):
        self.reactor.unregister_timer(self.connect_timer)
        try:
            self._read_config()
            for cb in self.state_cb:
                if self.state_message is not message_startup:
                    return self.reactor.NEVER
                cb('connect')
            self.state_message = message_ready
            for cb in self.state_cb:
                if self.state_message is not message_ready:
                    return self.reactor.NEVER
                cb('ready')
            if self.start_args.get('debugoutput') is None:
                self.reactor.update_timer(self.stats_timer, self.reactor.NOW)
        except (self.config_error, pins.error) as e:
            self.logger.exception("Config error")
            self.state_message = "%s%s" % (str(e), message_restart)
        except msgproto.error as e:
            self.logger.exception("Protocol error")
            self.state_message = "%s%s" % (str(e), message_protocol_error)
        except mcu.error as e:
            self.logger.exception("MCU error during connect")
            self.state_message = "%s%s" % (str(e), message_mcu_connect_error)
        except:
            self.logger.exception("Unhandled exception during connect")
            self.state_message = "Internal error during connect.%s" % (
                message_restart,)
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
                return "exit"
            # Check restart flags
            run_result = self.run_result
            try:
                if run_result == 'shutdown':
                    self.invoke_shutdown(self.async_shutdown_msg)
                    continue
                self._stats(self.reactor.monotonic(), force_output=True)
                if run_result == 'firmware_restart':
                    for m in self.lookup_module_objects('mcu'):
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
        self.state_message = "%s%s" % (msg, message_shutdown)
        for cb in self.state_cb:
            cb('shutdown')
    def invoke_async_shutdown(self, msg):
        self.async_shutdown_msg = msg
        self.request_exit("shutdown")
    def request_exit(self, result="exit"):
        self.run_result = result
        self.reactor.end()

    def web_getconfig(self):
        # self.logger.info("****** KLIPPER: web_getconfig() *******")
        toolhead = self.objects.get('toolhead')

        _extrs   = extruder.get_printer_extruders(self)
        num_extruders = len(_extrs)

        toolhead.kin.steppers[0].position_min
        toolhead.kin.steppers[0].position_max
        toolhead.max_accel
        return {
            "axisMins"            : [
                toolhead.kin.steppers[0].position_min,
                toolhead.kin.steppers[1].position_min,
                toolhead.kin.steppers[2].position_min,
            ],
            "axisMaxes"           : [
                toolhead.kin.steppers[0].position_max,
                toolhead.kin.steppers[1].position_max,
                toolhead.kin.steppers[2].position_max,
            ],
            "accelerations"       : [toolhead.max_accel] * (3 + num_extruders),
            "currents"            : [1.00] * (3 + num_extruders),
            "firmwareElectronics" : util.get_cpu_info(),
            "firmwareName"        : "KLIPPER",
            "firmwareVersion"     : self.get_start_args().get('software_version'),
            "firmwareDate"        : "2017-12-01",
            "idleCurrentFactor"   : 0.0,
            "idleTimeout"         : toolhead.motor_off_time,
            "minFeedrates"        : [0.00] * (3 + num_extruders),
            "maxFeedrates"        : [toolhead.max_velocity] * (3 + num_extruders)
            }
    def web_getcurrentstate(self):
        if self.is_shutdown:
            return "H"
        elif self.gcode.is_printer_ready == False:
            return 'C'
        return None
    def web_getstatus(self, _type=1):
        states = {
            False : 0,
            True  : 2
        }

        toolhead = self.objects.get('toolhead')
        if toolhead is not None:
            curr_pos = toolhead.get_position()
        else:
            curr_pos = [0] * 4
        fans     = [ fan.last_fan_value * 100.0 for fan in self.lookup_module_objects("fan") ]
        heatbed  = self.lookup_object('heater bed')
        _heaters = self.printer.lookup_module_objects("heater")
        _extrs   = extruder.get_printer_extruders(self)
        num_extruders = len(_extrs)

        coords_extr = [0] * num_extruders
        coords_extr[toolhead.extruder.index] = curr_pos[3]

        # _type == 1 is always included
        status_block = {
            "seq"   : gcode.tx_sequenceno,
            "coords": {
                "axesHomed" : toolhead.kin.is_homed(),
                "extr"      : coords_extr,
                "xyz"       : curr_pos[:3],
            },
            "currentTool": toolhead.extruder.index,        # -1 means none
            "params": {
                "atxPower"    : 0,
                "fanPercent"  : fans,
                "speedFactor" : self.gcode.speed_factor * 60. * 100.0,
                "extrFactors" : [ e.extrude_factor * 100.0 for i,e in _extrs.items() ],
                "babystep"    : float("%.3f" % self.gcode.babysteps),
            },
            # This must be included....
            "sensors": {
                "probeValue"     : -1, # 0
                "probeSecondary" : [0,0],  # Hidden for unmodulated probes, otherwise its array size depends on the probe type (usually 1 or 2)
                "fanRPM"         : 0,
            },
            "time" : (time.time() - self.starttime)  # time since last reset
        }

        #status_block["status"] = '';
        status_block["temps"] = {}

        if (heatbed is not None):
            status_block["temps"].update( {
                "bed": {
                    "current" : float("%.2f" % heatbed.last_temp),
                    "active"  : float("%.2f" % heatbed.target_temp),
                    # state = HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4
                    "state"   : states[True if heatbed.last_pwm_value > 0.0 else False],
                    "heater"  : heatbed.index+1,
                },
            } )

        htr_current = [0.0] * num_extruders
        htr_active  = [0.0] * num_extruders
        htr_state   = [  3] * num_extruders
        htr_heater  = [ -1] * num_extruders
        htr_standby = [0.0] * num_extruders
        for idx,htr in _heaters.items():
            if htr == heatbed:
                continue
            index = htr.index
            htr_current[index] = float("%.2f" % htr.last_temp)
            htr_active[index]  = float("%.2f" % htr.target_temp)
            htr_state[index]   = states[True if htr.last_pwm_value > 0.0 else False]
            htr_heater[index]  = htr.index+1
        status_block["temps"].update( {
            "heads": {
                "current" : htr_current,
                "active"  : htr_active,
                "standby" : htr_standby,
                "state"   : htr_state, # 0: off, 1: standby, 2: active, 3: fault (same for bed)
                "heater"  : htr_heater,
            },
        } )

        if (_type == 2):
            try:
                max_temp  = self.gcode.extruder.heater.max_temp
                cold_temp = self.gcode.extruder.heater.min_extrude_temp
                if self.gcode.extruder.heater.min_extrude_temp_disabled:
                    cold_temp = 0.0
            except AttributeError:
                max_temp  = 0.0
                cold_temp = 0.0
            status_block.update( {
                "coldExtrudeTemp" : cold_temp,
                "coldRetractTemp" : cold_temp,
                "tempLimit"       : max_temp,
                "endstops_IGN"        : 7,                # NEW: As of 1.09n-ch, this field provides a bitmap of all stopped drive endstops
                "firmwareName"    : "Klipper",
                "geometry"        : toolhead.kin.name,      # cartesian, coreXY, delta
                "axes"            : 3,                # Subject to deprecation - may be dropped in RRF 1.20
                "volumes"         : 1,                # Num of SD cards
                "mountedVolumes"  : 1,                # Bitmap of all mounted volumes
                "name"            : self.name,
                #"probe": {
                #    "threshold" : 500,
                #    "height"    : 2.6,
                #    "type"      : 1
                #},
                #"mcutemp": { # Not available on RADDS
                #    "min": 26.4,
                #    "cur": 30.5,
                #    "max": 43.4
                #},
                #"vin": { # Only DuetNG (Duet Ethernet + WiFi)
                #    "min": 10.4,
                #    "cur": 12.3,
                #    "max": 12.5
                #},
            } )

            tools = []
            for extr_key in _extrs:
                extr = _extrs[extr_key]
                values = {
                    "number"  : extr.index,
                    "name"    : extr.name,
                    "heaters" : [ (extr.heater.index + 1) ],
                    "drives"  : [],
                    #"axisMap" : [
                    #    [1,0,0,0,0,0], # X
                    #    [0,1,0,0,0,0]  # Y
                    #],
                    #"filament" : "N/A",
                }
                tools.append(values)
            status_block["tools"] = tools

        elif (_type == 3):
            status_block.update( {
                "currentLayer"       : 0,
                "currentLayerTime"   : 0.0,
                "extrRaw"            : [0.0] * num_extruders,  # How much filament would have been printed without extrusion factors applied
                "fractionPrinted"    : 0.0,         # one decimal place

                "firstLayerDuration" : 0.0,
                "firstLayerHeight"   : 0.0,
                "printDuration"      : 0.0,
                "warmUpDuration"     : 0.0,

                "timesLeft": {
                    "file"     : 0.0,
                    "filament" : 0.0,
                    "layer"    : 0.0
                }
            } )
        return status_block


######################################################################
# Startup
######################################################################

def start_helper(cfg_file,
                 loglevel=logging.INFO,
                 logfile=None,
                 debuginput=None,
                 debugoutput=None,
                 inputtty=None,
                 stat_interval=None,
                 startreason="unknown",
                 **kwargs):
    global status_delay
    start_args = {'config_file': cfg_file,
                  'start_reason': startreason}

    input_fd = bglogger = None
    if stat_interval is not None:
        status_delay = stat_interval

    if inputtty.__class__.__name__ is not "InputLink":
        if debuginput:
            start_args['debuginput'] = debuginput
            #input_fd = open(debuginput, 'rb').fileno()
            input_fd = InputLink(dbgin=debuginput)
        else:
            #input_fd = util.create_pty(inputtty)
            input_fd = InputLink(tty=inputtty)
    else:
        input_fd = inputtty

    if debugoutput:
        start_args['debugoutput'] = debugoutput
        start_args.update(kwargs)

    if logfile:
        bglogger = queuelogger.setup_bg_logging(logfile,
                                                loglevel)
    else:
        logging.basicConfig(level=loglevel,
                            format=queuelogger.LOGFORMAT)
    logging.getLogger().setLevel(loglevel)

    logging.info("Starting Klippy...")
    start_args['software_version'] = util.get_git_version()
    if bglogger is not None:
        lines = ["Args: %s" % (sys.argv,),
                 "Git version: %s" % (repr(start_args['software_version']),),
                 "CPU: %s" % (util.get_cpu_info(),),
                 "Python: %s" % (repr(sys.version),)]
        lines = "\n".join(lines)
        logging.info(lines)
        bglogger.set_rollover_info('versions', lines)

    return Printer(input_fd, bglogger, start_args)

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
    opts.add_option("-s", "--status", dest="status_delay", type="int",
                    help="Status report interval")
    options, args = opts.parse_args()
    if len(args) != 1:
        opts.error("Incorrect number of arguments")

    debuglevel = logging.INFO
    if options.verbose:
        debuglevel = logging.DEBUG

    if options.dictionary is None:
        options.dictionary = {}

    start_reason = "startup"

    # Start Printer() class
    while 1:
        printer = start_helper(cfg_file=args[0],
                               loglevel=debuglevel,
                               logfile=options.logfile,
                               stat_interval=options.status_delay,
                               debuginput=options.debuginput,
                               inputtty=options.inputtty,
                               debugoutput=options.debugoutput,
                               startreason=start_reason,
                               **options.dictionary
        );
        res = printer.run()
        if res == 'exit':
            break
        time.sleep(1.)
        logging.info("Restarting printer")
        start_reason = res

        if printer.bglogger is not None:
            printer.bglogger.stop()

if __name__ == '__main__':
    util.fix_sigint()
    main()
