# This file may be distributed under the terms of the GNU GPLv3 license.

import time, util, json, math

class GuiStats:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.get_logger("gui_stats")
        # required modules
        self.reactor = printer.get_reactor()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.toolhead = printer.lookup_object('toolhead')
        self.babysteps = printer.try_load_module(config, 'babysteps')
        self.sd = printer.try_load_module(config, "virtual_sdcard")
        # variables
        self.starttime = time.time()
        self.curr_state = 'PNR'
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        self.cpu_info = util.get_cpu_info()
        self.sw_version = printer.get_start_arg('software_version', 'Unknown')
        self.auto_report = False
        self.auto_report_timer = None
        # Print statistics
        self.layer_stats = []
        self.warmup_time = None
        self.print_time = self.last_time = .0
        self.first_layer_start = None
        self.firstLayerHeight = .0
        # register callbacks
        printer.register_event_handler('vsd:status', self.sd_status)
        printer.register_event_handler('gcode:layer_changed', self.layer_changed)
        printer.register_event_handler("klippy:ready", self.handle_ready)
        printer.register_event_handler("klippy:connect", self._handle_connect)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("klippy:halt", self._handle_shutdown)
        printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        # register control commands
        for cmd in ["GUISTATS_GET_ARGS",
                    "GUISTATS_GET_CONFIG", "GUISTATS_GET_STATUS",
                    "GUISTATS_GET_SD_INFO",
                    "GUISTATS_AUTO_REPORT"]:
            gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd), when_not_ready=True)
        printer.add_object("gui_stats", self)
        self.logger.info("GUI STATS LOADED!")

    def get_current_state(self):
        return self.curr_state

    # ================================================================================
    # Commands
    def cmd_GUISTATS_GET_ARGS(self, params):
        dump = json.dumps(self.printer.get_start_args())
        self.gcode.respond(dump)

    def cmd_GUISTATS_GET_CONFIG(self, params):
        dump = json.dumps(self.get_config_stats())
        self.gcode.respond(dump)

    def cmd_GUISTATS_GET_STATUS(self, params):
        _type = self.gcode.get_int("TYPE", params,
            default=1, minval=1, maxval=3)
        stats = self.get_status_stats(_type)
        dump = json.dumps(stats)
        self.gcode.respond(dump)

    def cmd_GUISTATS_GET_SD_INFO(self, params):
        dump = "virtual sd is not available"
        if self.sd is not None:
            dump = json.dumps(self.sd.get_status(0, True))
        self.gcode.respond(dump)

    def cmd_GUISTATS_AUTO_REPORT(self, params):
        self.auto_report = self.gcode.get_int("ENABLE", params,
            default=self.auto_report, minval=0, maxval=1)
        if self.auto_report and self.auto_report_timer is None:
            self.auto_report_timer = self.reactor.register_timer(
                self._auto_temp_report_cb, self.reactor.NOW)
        elif not self.auto_report and self.auto_report_timer is not None:
            self.reactor.unregister_timer(self.auto_report_timer)
            self.auto_report_timer = None
        self.gcode.respond("Auto reporting %s ok" %
                           ['diabled', 'enabled'][self.auto_report])

    # ================================================================================
    # Callbacks
    def _auto_temp_report_cb(self, eventtime):
        #self.logger.debug("AUTO Report @ %s" % eventtime)
        stats = self.get_status_stats(3)
        dump = json.dumps(stats)
        self.gcode.respond('GUISTATS_REPORT='+dump)
        return eventtime + .250

    def _handle_shutdown(self):
        self.curr_state = "H"
    def _handle_disconnect(self):
        self.curr_state = "C"
    def handle_ready(self):
        self.curr_state = "I"
        if self.auto_report and self.auto_report_timer is None:
            self.auto_report_timer = self.reactor.register_timer(
                self._auto_temp_report_cb, self.reactor.NOW)
        elif not self.auto_report and self.auto_report_timer is not None:
            self.reactor.unregister_timer(self.auto_report_timer)
            self.auto_report_timer = None
    def _handle_connect(self):
        self.curr_state = "B"

    def sd_status(self, status):
        if status == 'pause':
            self.curr_state = "S"
        elif status == 'start':
            self.curr_state = "P"
            self.last_time = self.toolhead.get_estimated_print_time()
        elif status == 'error' or status == "stop":
            self.curr_state = "I"
        elif status == 'done':
            toolhead = self.toolhead
            toolhead.wait_moves() # TODO: remove?
            self.curr_state = "I"
        elif status == 'loaded':
            self.layer_stats = []
            self.warmup_time = None
            self.print_time = .0

    def layer_changed(self, change_time, layer, height, *args):
        # 1st call is "heating ready"
        self.logger.debug("Layer changed cb: time %s, layer %s, h=%s" % (
            change_time, layer, height))
        try:
            start_time = self.layer_stats[-1]['end time']
        except IndexError:
            # 1st layer change
            start_time = change_time
            self.warmup_time = self.print_time # warmup ready
        self.layer_stats.append(
            {'start time': start_time,
             'layer time': (change_time - start_time),
             'end time': change_time})

    # ================================================================================
    # Statistics
    def get_config_stats(self):
        printer = self.printer
        _extrs = printer.extruder_get()
        kinematic = self.toolhead.get_kinematics()
        motor_off_time = printer.lookup_object('idle_timeout').idle_timeout
        currents = []
        max_feedrates = []
        accelerations = []
        axisMins = []
        axisMaxes = []
        axisName = []
        # read rails
        for limit in kinematic.get_max_limits():
            rail = limit['rail']
            accel = limit['acc']
            velocity = limit['velocity']
            _min, _max = rail.get_range()
            steppers = rail.get_steppers()
            for stp in steppers:
                max_feedrates.append(int(velocity))
                accelerations.append(int(accel))
                axisMins.append(_min)
                axisMaxes.append(_max)
                axisName.append(stp.get_name(short=True))
                get_current = getattr(stp.get_driver(), "get_current", None)
                if get_current is not None:
                    currents.append(int(get_current()))
                else:
                    currents.append(-1)
        # read extrudersl
        for e in _extrs.values():
            limits = e.get_max_e_limits()
            max_feedrates.append(int(limits['velocity']))
            accelerations.append(int(limits['acc']))
            axisMins.append(0)
            axisMaxes.append(limits['max_e_dist'])
            axisName.append("e%s" % e.get_index())
            get_current = getattr(limits['stepper'].get_driver(),
                                  "get_current", None)
            if get_current is not None:
                currents.append(int(get_current()))
            else:
                currents.append(-1)
        config = {
            "err"                 : 0,
            "axisNames"           : axisName,
            "axisMins"            : axisMins,
            "axisMaxes"           : axisMaxes,
            "accelerations"       : accelerations,
            "currents"            : currents,
            "firmwareElectronics" : self.cpu_info,
            "firmwareName"        : "Klipper",
            "firmwareVersion"     : self.sw_version,
            "idleCurrentFactor"   : 0.0,
            "idleTimeout"         : motor_off_time,
            "minFeedrates"        : [0.00] * len(max_feedrates),
            "maxFeedrates"        : max_feedrates
            }
        return config

    def get_status_stats(self, _type=1):
        pheater = self.printer.lookup_object('heater')
        toolhead = self.toolhead
        # STATES = 0: off, 1: standby, 2: active, 3: fault (same for bed)
        states = {False : 0, True  : 2}
        curr_extruder = toolhead.get_extruder()
        curr_pos = toolhead.get_position()
        fans     = [ fan.last_fan_value * 100.0 for n, fan in
                     self.printer.lookup_objects("fan") ]
        heatbed  = pheater.lookup_heater('heater bed', None)
        _extrs   = self.printer.extruder_get()
        kinematic = toolhead.get_kinematics()
        homed_axes = [0] * 3
        if getattr(kinematic, "is_homed", None) is not None:
            homed_axes = kinematic.is_homed()

        atx_pwr = pheater.lookup_heater('atx_power', None)
        atx_state = atx_pwr.get_state() if atx_pwr else 0

        babysteps = self.babysteps.babysteps if self.babysteps else 0.

        # _type == 1 is always included
        status_block = {
            "status": self.curr_state,
            "seq": 0,
            "coords": {
                "axesHomed": homed_axes,
                "extr": [e.extrude_pos for i, e in _extrs.items()],
                "xyz": curr_pos[:3],
            },
            "currentTool": curr_extruder.get_index(),
            "params": {
                "atxPower": atx_state,
                "fanPercent": fans,
                "speedFactor": self.gcode.speed_factor * 60. * 100.0,
                "extrFactors": [e.get_extrude_factor(procent=True)
                                for i, e in _extrs.items()],
                "babystep": float("%.3f" % babysteps),
            },
            "sensors": {
                # "fanRPM": 0,
            },
            "time": (time.time() - self.starttime),
            "temps": {}
        }

        #bed_tilt = self.printer.lookup_object('bed_tilt', default=None)
        #if bed_tilt:
        #    probe_x, probe_y, probeValue = bed_tilt.get_adjust()
        #    status_block['sensors']['probeValue'] = probeValue
        #    status_block['sensors']['probeSecondary'] = [probe_x, probe_y]

        heatbed_add = (heatbed is not None)
        num_extruders = len(_extrs)
        total_heaters = num_extruders + heatbed_add
        htr_current = [.0] * total_heaters
        # HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4
        htr_state   = [3] * total_heaters
        extr_states = {
            "active"  : [],
            "standby" : [[ .0 ]] * num_extruders
        }
        for extr in _extrs.values():
            htr = extr.get_heater()
            temp, target = htr.get_temp(0)
            index = extr.get_index() + heatbed_add
            htr_current[index] = float("%.2f" % temp)
            htr_state[index] = states[(target > 0.0)]
            extr_states['active'].append([float("%.2f" % target)])
        # Tools target temps
        status_block["temps"].update({'tools': extr_states})

        if heatbed is not None:
            temp, target = heatbed.get_temp(0)
            htr_current[0] = float("%.2f" % temp)
            htr_state[0] = states[(target > 0.0)]
            # Heatbed target temp
            status_block["temps"].update( {
                "bed": {
                    "active"  : float("%.2f" % target),
                    "heater"  : 0,
                },
            } )

        chamber = self.printer.lookup_object('chamber', default=None)
        if chamber is not None:
            current, target = chamber.get_temp(0)
            status_block["temps"].update( {
                "chamber": {
                    "active"  : float("%.2f" % target),
                    "heater"  : len(htr_current),
                },
            } )
            htr_current.append(float("%.2f" % current))
            htr_state.append(states[chamber.is_fan_active()])

        cabinet = self.printer.lookup_object('cabinet', default=None)
        if cabinet is not None:
            current, target = cabinet.get_temp(0)
            status_block["temps"].update( {
                "cabinet": {
                    "active"  : float("%.2f" % target),
                    "heater"  : len(htr_current),
                },
            } )
            htr_current.append(current)
            htr_state.append(states[target > 0.0])

        status_block["temps"].update( {
            "current" : htr_current,
            "state"   : htr_state,
        } )

        if _type >= 2:
            max_temp  = 0.0
            cold_temp = 0.0
            if hasattr(curr_extruder, "get_heater"):
                heater = curr_extruder.get_heater()
                max_temp  = heater.max_temp
                cold_temp = heater.min_extrude_temp
                if heater.min_extrude_temp_disabled:
                    cold_temp = 0.0

            # endstop states
            endstops_hit = 0
            if any(homed_axes):
                index = 0
                for home_state, rail in zip(homed_axes, kinematic.get_rails()):
                    num_steppers = len(rail.get_steppers())
                    if home_state and num_steppers:
                        endstops_hit |= (
                                (int(math.pow(2, num_steppers)) - 1) << index)
                    index += num_steppers

            status_block.update( {
                "coldExtrudeTemp" : cold_temp,
                "coldRetractTemp" : cold_temp,
                "tempLimit"       : max_temp,
                "endstops"        : endstops_hit,
                "firmwareName"    : "Klipper",
                "geometry"        : kinematic.name,   # cartesian, coreXY, delta
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
            for key, extr in _extrs.items():
                values = {
                    "number"   : extr.get_index(),
                    "name"     : extr.name,
                    "heaters"  : [ extr.heater.get_index() + 1 ],
                    "drives"   : [ 3+extr.get_index() ],
                    #"filament" : "N/A",
                }
                tools.append(values)
            status_block["tools"] = tools

        if _type >= 3:
            lstat = self.layer_stats
            current_time = toolhead.get_estimated_print_time()
            printing_time = self.print_time
            if self.curr_state == "P":
                # Update time while printing
                printing_time += current_time - self.last_time
                self.last_time = current_time
                self.print_time = printing_time
            curr_layer = len(lstat)
            try:
                layer_time_curr = current_time - lstat[-1]['end time']
            except IndexError:
                layer_time_curr = printing_time
            try:
                first_layer_time = lstat[1]['layer time']
            except IndexError:
                first_layer_time = layer_time_curr

            warmup_time = self.warmup_time
            if warmup_time is None:
                # Update warmup time
                warmup_time = printing_time

            # Print time estimations
            progress = 0.
            if self.sd is not None:
                progress = self.sd.get_progress()
            remaining_time_file = 0.
            if progress > 0:
                remaining_time_file = (printing_time / progress) - printing_time

            # Used filament amount
            remaining_time_fila = 0.
            '''
            fila_total = sum(e for e in info['filament'])
            if fila_total > 0:
                fila_used = sum(e.raw_filament for i, e in _extrs.items())
                fila_perc = (fila_used / fila_total)
                remaining_time_fila = (printing_time / fila_perc) - printing_time
            '''

            # Layer statistics
            remaining_time_layer = 0.
            '''
            num_layers = 0
            layerHeight = info['layerHeight']
            firstLayerHeight = info['firstLayerHeight']
            if layerHeight > 0:
                num_layers = int( (info["height"] - firstLayerHeight +
                                  layerHeight) / layerHeight )
            if num_layers:
                proc = curr_layer / num_layers
                if proc > 0:
                    remaining_time_layer = (printing_time / proc) - printing_time
            '''

            '''
            self.logger.debug(
                "TYPE3: layer %s, time: %s, 1st time: %s, warmup: %.2f, progress: %.2f, "
                "file_time: %.2f, printing_time: %f" % (
                curr_layer, layer_time_curr, first_layer_time, warmup_time, progress,
                remaining_time_file, printing_time))
            #'''

            # Fill status block
            status_block.update( {
                "progressType"       : 0, # 1 = layer, else file progress
                "currentLayer"       : curr_layer,
                "currentLayerTime"   : layer_time_curr,
                # How much filament would have been printed without extrusion factors applied
                "extrRaw"            : [ float("%0.1f" % e.raw_filament)
                                         for i, e in _extrs.items() ],
                "fractionPrinted"    : float("%.1f" % (progress * 100.)),

                "firstLayerDuration" : first_layer_time,
                "SKIP_ firstLayerHeight"   : float("%.1f" % self.firstLayerHeight),
                "printDuration"      : printing_time,
                "warmUpDuration"     : float("%.1f" % warmup_time),

                "timesLeft": {
                    "file"     : float("%.1f" % remaining_time_file),
                    "filament" : [ float("%.1f" % remaining_time_fila) ],
                    "layer"    : float("%.1f" % remaining_time_layer),
                }
            } )
        # self.logger.debug("%s", json.dumps(status_block, indent=4))
        return status_block
