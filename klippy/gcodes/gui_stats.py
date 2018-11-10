import time, util, json, os, re


class ParseError(Exception):
    pass


class GuiStats:
    def __init__(self, printer, config):
        self.printer = printer
        self.logger = printer.logger.getChild("gui_stats")
        # required modules
        self.gcode = gcode = printer.lookup_object('gcode')
        self.toolhead = printer.lookup_object('toolhead')
        self.toolhead.register_cb('layer', self.layer_changed)
        self.babysteps = printer.lookup_object('babysteps')
        self.sd = printer.try_load_module(config, "virtual_sdcard")
        # variables
        self.starttime = time.time()
        self.curr_state = 'C'
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        self.cpu_info = util.get_cpu_info()
        self.sw_version = printer.get_start_arg('software_version', 'Unknown')
        # Print statistics
        self.layer_stats = []
        self.warmup_time = .1
        self.first_layer_start = None
        self.last_used_file = None
        self.firstLayerHeight = .0
        # register callbacks
        self.sd.register_done_cb(self.sd_print_done)
        # register control commands
        for cmd in ["GUISTATS_GET_CONFIG"]:
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd), when_not_ready=True)
        for cmd in ["GUISTATS_GET_STATUS"]:
            self.gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd), when_not_ready=True)
        printer.add_object("gui_stats", self)

    def get_current_state(self):
        return self.curr_state

    # ================================================================================
    # Callbacks
    def printer_state(self, state):
        if state == "connect":
            self.curr_state = "B"
        elif state == "ready":

            self.curr_state = "I"
        elif state == "disconnect":
            self.curr_state = "C"
        elif state == "shutdown" or state == "halt":
            self.curr_state = "H"

    def sd_print_done(self, status, *args, **kwargs):
        if status == 'pause':
            self.curr_state = "S"
        elif status == 'start':
            self.curr_state = "P"
        elif status == 'error' or status == "stop":
            self.curr_state = "I"
        elif status == 'done':
            toolhead = self.toolhead
            toolhead.wait_moves()
            self.layer_changed(toolhead.get_print_time())
            self.curr_state = "I"
        elif status == 'loaded':
            self.layer_stats = []
            self.warmup_time = .1

    def layer_changed(self, change_time):
        if len(self.layer_stats) == 0 and self.first_layer_start is None:
            # First layer started
            self.logger.info(" ====== 1st LAYER ===== %s " % (change_time,))
            self.first_layer_start = change_time
        else:
            # last end is start time
            if self.first_layer_start is not None:
                start_time = self.first_layer_start
                self.first_layer_start = None
            else:
                start_time = self.layer_stats[-1]['end time']
            self.logger.info(" ====== LAYER CHANGED ===== %s - %s" % (start_time, change_time))
            self.layer_stats.append(
                {'start time': start_time,
                 'layer time': (change_time - start_time),
                 'end time': change_time} )

    # ================================================================================
    # Commands
    def cmd_GUISTATS_GET_CONFIG(self, params):
        dump = json.dumps(self.get_config_stats())
        params["#input"].respond(dump)

    def cmd_GUISTATS_GET_STATUS(self, params):
        _type = self.gcode.get_int("TYPE", params,
            default=1, minval=1, maxval=3)
        stats = self.get_status_stats(_type)
        dump = json.dumps(stats)
        params["#input"].respond(dump)

    # ================================================================================
    # Statistics
    def get_config_stats(self):
        printer = self.printer
        _extrs = printer.extruder_get()
        toolhead = self.toolhead
        kinematic = toolhead.get_kinematics()
        rails = kinematic.get_rails()
        currents = []
        max_feedrates = []
        accelerations = []
        axisMins = []
        axisMaxes = []
        for rail in rails:
            _min, _max = rail.get_range()
            steppers = rail.get_steppers()
            for idx, stp in enumerate(steppers):
                axisMins.append(_min)
                axisMaxes.append(_max)
                get_current = getattr(stp.driver, "get_current", None)
                if get_current is not None:
                    currents.append(int(get_current()))
                else:
                    currents.append(-1)
                _vel, _accel = stp.get_max_velocity()
                max_feedrates.append(int(_vel))
                accelerations.append(int(_accel))
        for idx, e in _extrs.items():
            _vel, _accel = e.get_max_velocity()
            max_feedrates.append(int(_vel))
            accelerations.append(int(_accel))
            get_current = getattr(e.stepper.driver, "get_current", None)
            if get_current is not None:
                currents.append(int(get_current()))
            else:
                currents.append(-1)
        return {
            "axisMins"            : axisMins,
            "axisMaxes"           : axisMaxes,
            "accelerations"       : accelerations,
            "currents"            : currents,
            "firmwareElectronics" : self.cpu_info,
            "firmwareName"        : "Klipper",
            "firmwareVersion"     : self.sw_version,
            "firmwareDate"        : "2017-12-01",
            "idleCurrentFactor"   : 0.0,
            "idleTimeout"         : toolhead.motor_off_time,
            "minFeedrates"        : [0.00] * (len(max_feedrates) + len(_extrs)),
            "maxFeedrates"        : max_feedrates
            }

    def get_status_stats(self, _type=1):
        toolhead = self.toolhead
        states = {
            False : 0,
            True  : 2
        }
        curr_extruder = toolhead.get_extruder()
        curr_pos = toolhead.get_position()
        fans     = [ fan.last_fan_value * 100.0 for fan in
                     self.printer.lookup_module_objects("fan") ]
        heatbed  = self.printer.lookup_object('heater bed', None)
        _heaters = self.printer.lookup_module_objects("heater")
        total_htrs = len(_heaters)
        _extrs   = self.printer.extruder_get()

        # _type == 1 is always included
        status_block = {
            "status": self.curr_state,
            "seq": 0,
            "coords": {
                "axesHomed": toolhead.kin.is_homed(),
                "extr": [e.extrude_pos
                         for i, e in _extrs.items()],
                "xyz": curr_pos[:3],
            },
            "currentTool": curr_extruder.index,
            "params": {
                "atxPower": 0,
                "fanPercent": fans,
                "speedFactor": self.gcode.speed_factor * 60. * 100.0,
                "extrFactors": [e.get_extrude_factor(procent=True) for i, e in _extrs.items()],
                "babystep": float("%.3f" % self.babysteps.babysteps),
            },
            "sensors": {
                # "fanRPM": 0,
            },
            "time": (time.time() - self.starttime),
            "temps": {}
        }

        bed_tilt = self.printer.lookup_object('bed_tilt', default=None)
        if bed_tilt:
            probe_x, probe_y, probeValue = bed_tilt.get_adjust()
            status_block['sensors']['probeValue'] = probeValue
            status_block['sensors']['probeSecondary'] = [probe_x, probe_y]

        if heatbed is not None:
            heatbed_status = heatbed.get_status(0)
            status_block["temps"].update( {
                "bed": {
                    "active"  : float("%.2f" % heatbed_status['target']),
                    "heater"  : 0,
                },
            } )

        htr_current = [0.0] * total_htrs
        # HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4
        htr_state   = [  3] * total_htrs
        for htr in _heaters:
            status = htr.get_status(0)
            index = htr.index + 1
            if htr == heatbed:
                index = 0
            htr_current[index] = float("%.2f" % status['temperature'])
            # htr_state[index]   = states[True if htr.last_pwm_value > 0.0 else False]
            htr_state[index] = states[(status['target'] > 0.0)]

        chamber = self.printer.lookup_object('chamber', default=None)
        if chamber is not None:
            current, target = chamber.get_temp()
            status_block["temps"].update( {
                "chamber": {
                    "active"  : float("%.2f" % target),
                    "heater"  : len(htr_current),
                },
            } )
            htr_current.append(float("%.2f" % current))
            htr_state.append(states[chamber.is_fan_active()])
        '''
        cabinet = self.printer.lookup_object('cabinet', default=None)
        if cabinet is not None:
            current, target = cabinet.get_temp()
            status_block["temps"].update( {
                "cabinet": {
                    "active"  : float("%.2f" % target),
                    "heater"  : len(htr_current),
                },
            } )
            htr_current.append(current)
            htr_state.append(states[target > 0.0])
        '''

        status_block["temps"].update( {
            "current" : htr_current,
            "state"   : htr_state, # 0: off, 1: standby, 2: active, 3: fault (same for bed)
            "heads": {
                "state": htr_state[1:],
            },
        } )

        # Tools target temps
        status_block["temps"].update( {
            'tools': {
                "active"  : [ [float("%.2f" % e.get_heater().target_temp)]
                              for i, e in _extrs.items() ],
                "standby" : [ [ .0 ] for i, e in _extrs.items() ],
            },
        } )

        if _type == 2:
            max_temp  = 0.0
            cold_temp = 0.0
            if hasattr(curr_extruder, "get_heater"):
                heater = curr_extruder.get_heater()
                max_temp  = heater.max_temp
                cold_temp = heater.min_extrude_temp
                if heater.min_extrude_temp_disabled:
                    cold_temp = 0.0
            endstops_hit = 0
            for idx, state in enumerate(toolhead.get_kinematics().is_homed()):
                endstops_hit |= (state << idx)
            status_block.update( {
                "coldExtrudeTemp" : cold_temp,
                "coldRetractTemp" : cold_temp,
                "tempLimit"       : max_temp,
                "endstops"        : endstops_hit,
                "firmwareName"    : "Klipper",
                "geometry"        : toolhead.kin.name, # cartesian, coreXY, delta
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
                    "number"   : extr.index,
                    "name"     : extr.name,
                    "heaters"  : [ extr.heater.index + 1 ],
                    "drives"   : [ 3+extr.index ],
                    #"filament" : "N/A",
                }
                tools.append(values)
            status_block["tools"] = tools

        elif _type == 3:
            fname = self.sd.get_current_file_name()
            if fname is None:
                fname = self.last_used_file
            self.last_used_file = fname

            printing_time = toolhead.get_print_time()
            curr_layer = len(self.layer_stats)

            first_layer_time = 0.
            layer_time_curr = .0
            if curr_layer > 0 or self.first_layer_start is not None:
                lstat = self.layer_stats
                try:
                    first_layer_time = lstat[0]['layer time']
                except IndexError:
                    first_layer_time = printing_time
                try:
                    layer_time_curr = lstat[-1]['layer time']
                except IndexError:
                    layer_time_curr = printing_time
            else:
                self.warmup_time += printing_time - self.warmup_time

            # Print time estimations
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

            # Fill status block
            status_block.update( {
                "currentLayer"       : curr_layer,
                "currentLayerTime"   : layer_time_curr,
                # How much filament would have been printed without extrusion factors applied
                "extrRaw"            : [ float("%0.1f" % e.raw_filament)
                                         for i, e in _extrs.items() ],
                "fractionPrinted"    : float("%.1f" % (progress * 100.)),

                "firstLayerDuration" : first_layer_time,
                "SKIP_ firstLayerHeight"   : self.firstLayerHeight,
                "printDuration"      : printing_time,
                "warmUpDuration"     : float("%.1f" % self.warmup_time),

                "timesLeft": {
                    "file"     : float("%.1f" % remaining_time_file),
                    "filament" : [ float("%.1f" % remaining_time_fila) ],
                    "layer"    : float("%.1f" % remaining_time_layer),
                }
            } )
        return status_block


def load_gcode(printer, config):
    if config.has_section("reprapgui") or \
            config.has_section("reprapgui_process"):
        GuiStats(printer, config)
