# This file may be distributed under the terms of the GNU GPLv3 license.

import time, util, json, math

class GuiStats:
    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.logger = printer.get_logger("gui_stats")
        # required modules
        self.reactor = printer.get_reactor()
        self.gcode = gcode = printer.lookup_object('gcode')
        self.babysteps = printer.try_load_module(config, 'babysteps')
        self.sd = printer.try_load_module(config, "virtual_sdcard")
        printer.try_load_module(config, "analyse_gcode", folder="modules")
        # variables
        self.toolhead = None
        self.starttime = time.time()
        self.curr_state = 'PNR'
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        self.auto_report = False
        self.auto_report_timer = None
        # Print statistics
        self._stats_type_1 = {}
        self._stats_type_2 = {}
        self._stats_type_3 = {}
        self.print_start_time = self.starttime
        self.last_print_layer_change = .0
        # register callbacks
        printer.register_event_handler('vsd:status', self._sd_status)
        printer.register_event_handler('vsd:file_loaded', self._sd_file_loaded)
        printer.register_event_handler('gcode:layer_changed', self._layer_changed)
        printer.register_event_handler('klippy:config_ready', self._config_ready)
        printer.register_event_handler("klippy:ready", self.handle_ready)
        printer.register_event_handler("klippy:connect", self._handle_connect)
        printer.register_event_handler("klippy:shutdown", self._handle_shutdown)
        printer.register_event_handler("klippy:halt", self._handle_shutdown)
        printer.register_event_handler("klippy:disconnect", self._handle_disconnect)
        printer.register_event_handler("homing:homed_rails", self._homing_ready)
        # register control commands
        for cmd in ["GUISTATS_GET_ARGS",
                    "GUISTATS_GET_CONFIG", "GUISTATS_GET_STATUS",
                    "GUISTATS_GET_SD_INFO",
                    "GUISTATS_AUTO_REPORT"]:
            gcode.register_command(
                cmd, getattr(self, 'cmd_' + cmd), when_not_ready=True)

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

    def _config_ready(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        self._stats_type_1_reset()
        self._stats_type_2_reset()
        self._stats_type_3_reset()
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

    def _homing_ready(self, homing_state, rails):
        # self._parse_homed_states()
        for axis in homing_state.get_axes():
            self.logger.info("Homed axis '%s'", axis)
            self._stats_type_1["coords"]["axesHomed"][axis] = 1

    def _sd_status(self, status):
        if status == 'pause':
            self.curr_state = "S"
        elif status == 'start':
            self.curr_state = "P"
            # reset time counters
            self.print_start_time = self.last_print_layer_change = time.time()
        elif status == 'error' or status == "stop":
            self.curr_state = "I"
        elif status == 'done':
            toolhead = self.toolhead
            toolhead.wait_moves()
            self._stats_type_3['lastLayerTime'] = \
                time.time() - self.last_print_layer_change
            self.curr_state = "I"
        elif status == 'loaded':
            pass
    def _sd_file_loaded(self, fname):
        handler = self.printer.lookup_object("analyse_gcode")
        file_info = handler.get_file_info(fname)
        firstLayerHeight = file_info.get('firstLayerHeight', 0.)
        # clear stats when new file is loaded
        self._stats_type_3_reset(firstLayerHeight)

    def _layer_changed(self, change_time, layer, height, *args):
        #self.logger.debug("Layer changed cb: time %s, layer %s, h=%s" % (
        #    change_time, layer, height))
        type_3 = self._stats_type_3
        current_time = time.time()
        print_time = current_time - self.last_print_layer_change
        current_layer = type_3['currentLayer'] + 1
        type_3['currentLayer'] = current_layer
        if current_layer == 1:
            # heating / preparation is done when 1st call happens
            type_3['warmUpDuration'] = int(self.toolhead.get_print_time())
        elif current_layer == 2:
            type_3['firstLayerDuration'] = int(print_time + .5)
        else:
            type_3['previousLayerTime'] = type_3['currentLayerTime']
        self.last_print_layer_change = current_time

    # ================================================================================
    def _parse_homed_states(self):
        homed_axes = [0] * 3
        kinematic = self.toolhead.get_kinematics()
        if hasattr(kinematic, "is_homed"):
            homed_axes = kinematic.is_homed()
            self._stats_type_1["coords"]["axesHomed"] = homed_axes
        if any(homed_axes):
            endstops_hit = 0
            index = 0
            for home_state, rail in zip(homed_axes, kinematic.get_rails()):
                num_steppers = len(rail.get_steppers())
                if home_state and num_steppers:
                    endstops_hit |= (
                            (int(math.pow(2, num_steppers)) - 1) << index)
                index += num_steppers
            self._stats_type_2["endstops"] = endstops_hit

    def _stats_type_1_reset(self):
        self._stats_type_1 = {
            "status": self.curr_state,
            "seq": 0,
            "coords": {
                "axesHomed": [0] * 3,
                "extr": [0],
                "xyz": [0] * 3,
            },
            "currentTool": 0,
            "params": {
                # "atxPower": 0,
                "fanPercent": [0],
                "speedFactor": 100.,
                "extrFactors": [100.],
                "babystep": .0,
                "bed_mesh_ok": 0,
            },
            "sensors": {},
            "time": (time.time() - self.starttime),
            "temps": {},
        }

    def _stats_type_2_reset(self):
        # Fill stats type 2
        kinematic = self.toolhead.get_kinematics()
        _extrs = self.printer.extruder_get()
        heaters = self.printer.lookup_object('heater')
        max_temp = 0.0
        for _heater in heaters.get_heaters().values():
            if _heater.max_temp > max_temp:
                max_temp = _heater.max_temp
        tools = []
        for extr in _extrs.values():
            index = extr.get_index()
            values = {
                "number"   : index,
                "name"     : extr.name,
                "heaters"  : [ extr.heater.get_index() + 1 ],
                "drives"   : [ 3 + index ],
                #"filament" : "N/A",
            }
            tools.append(values)
        self._stats_type_2 = {
            "coldExtrudeTemp": 170.,
            "coldRetractTemp": 170.,
            "tempLimit":       max_temp,
            "endstops":        0,
            "firmwareName":    "Klipper",
            "geometry":        kinematic.name,  # cartesian, coreXY, delta
            "axes":            3,  # Subject to deprecation - may be dropped in RRF 1.20
            "volumes":         1,  # Num of SD cards
            "mountedVolumes":  1,  # Bitmap of all mounted volumes
            "name":            self.name,
            # "probe": {
            #    "threshold" : 500,
            #    "height"    : 2.6,
            #    "type"      : 1
            # },
            # "mcutemp": { # Not available on RADDS
            #    "min": 26.4,
            #    "cur": 30.5,
            #    "max": 43.4
            # },
            # "vin": { # Only DuetNG (Duet Ethernet + WiFi)
            #    "min": 10.4,
            #    "cur": 12.3,
            #    "max": 12.5
            # },
            "tools": tools,
        }

    def _stats_type_3_reset(self, first_layer_height=0.):
        self._stats_type_3 = {
            "progressType": 0, # 1 = layer, else file progress
            "previousLayerTime": 0,
            "lastLayerTime": 0,
            "currentLayer": 0,
            "currentLayerTime": 0,
            "extrRaw": [.0] * 9,
            "fractionPrinted": .0,
            "firstLayerDuration": 0,
            "firstLayerHeight": first_layer_height,
            "printDuration": 0.,
            "warmUpDuration": 0.,
            "timesLeft": {
                "file": .0,
                # "filament": [.0] * 9,
                # "layer": .0,
            },
        }

    # Statistics
    def get_config_stats(self):
        if self.toolhead is None:
            return {"err": 1}
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
            "firmwareElectronics" : util.get_cpu_info(),
            "firmwareName"        : "Klipper",
            "firmwareVersion"     : printer.get_start_arg('software_version',
                                                          'Unknown'),
            "idleCurrentFactor"   : 0.0,
            "idleTimeout"         : motor_off_time,
            "minFeedrates"        : [0.00] * len(max_feedrates),
            "maxFeedrates"        : max_feedrates
            }
        return config

    def get_status_stats(self, _type=1):
        if self.toolhead is None:
            return {"err": 1, "seq": 0}
        pheater = self.printer.lookup_object('heater')
        # STATES = 0: off, 1: standby, 2: active, 3: fault (same for bed)
        states = {False : 0, True  : 2}
        curr_extruder = self.toolhead.get_extruder()
        curr_pos = self.toolhead.get_position()
        fans = [fan.last_fan_value * 100.0 for n, fan in
                self.printer.lookup_objects("fan")]
        heatbed = pheater.lookup_heater('heater bed', None)
        _extrs = self.printer.extruder_get()

        babysteps = self.babysteps.babysteps if self.babysteps else 0.

        # _type == 1 is always included
        status_block = self._stats_type_1
        status_block["status"] = self.curr_state
        status_block["currentTool"] = curr_extruder.get_index()
        status_block["time"] = time.time() - self.starttime

        # update coordinates
        status_block['coords']["extr"] = [
            e.extrude_pos for i, e in _extrs.items()]
        status_block['coords']["xyz"] = curr_pos[:3]

        # update ATX status
        atx_pwr = self.printer.lookup_object('atx_power', None)
        # atx_state = atx_pwr.get_state() if atx_pwr else 0
        if atx_pwr is not None:
            status_block["params"]["atxPower"] = atx_pwr.get_state()

        # update params
        status_block["params"].update({
            # "atxPower":    atx_state,
            "fanPercent":  fans,
            "speedFactor": self.gcode.speed_factor * 60. * 100.0,
            "extrFactors": [e.get_extrude_factor(procent=True)
                for i, e in _extrs.items()],
            "babystep":    float("%.3f" % babysteps),
        })

        probe = self.printer.lookup_object('probe', None)
        if probe:
            x_offset, y_offset, z_offset = probe.get_offsets()
            status_block['sensors']['probe'] = {
                'probeValue': z_offset,
                'probeSecondary': [x_offset, y_offset]}

        bed_mesh = self.printer.lookup_object('bed_mesh', None)
        if bed_mesh:
            # Check whether bed mesh is loaded or not
            status_block['params']['bed_mesh_ok'] = int(
                bed_mesh.z_mesh is not None)

        #calibrate = getattr(bed_mesh, "calibrate", None)
        #if calibrate:
        #    # calibrate.probed_z_table
        #    pass

        #if fans:
        #    fan0 = self.printer.lookup_objects("fan 0")
        #    # status_block['sensors']['fanRPM'] = 0

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

        status_block["temps"].update({
            "current" : htr_current,
            "state"   : htr_state,
        })

        status_resp = dict(status_block)

        if _type >= 2:
            stat2 = self._stats_type_2
            cold_temp = 999.0
            if hasattr(curr_extruder, "get_heater"):
                heater = curr_extruder.get_heater()
                if heater.min_extrude_temp_disabled:
                    cold_temp = .0
                else:
                    cold_temp = heater.min_extrude_temp
            stat2["coldExtrudeTemp"] = stat2["coldRetractTemp"] = cold_temp
            status_resp.update(stat2)

        if _type >= 3:
            stat = self._stats_type_3

            current_time = time.time()
            printing_time = current_time - self.print_start_time
            stat["printDuration"] = int(printing_time)
            stat['currentLayerTime'] = int(
                current_time - self.last_print_layer_change)

            # SD progress
            progress = 0.
            if self.sd is not None:
                progress = self.sd.get_progress()

            # How much filament would have been printed without extrusion factors applied
            stat["extrRaw"] = [float("%0.1f" % e.raw_filament)
                for i, e in _extrs.items()]

            # ===== Print time estimation =====
            timesLeft = stat["timesLeft"]
            # 1) based on file progress
            if 0 < progress < 1.0:
                remaining_time_file = (printing_time / progress) - printing_time
                timesLeft['file'] = int(remaining_time_file)
            elif progress == 1. and self.curr_state == "P":
                # keep progress ongoing until print is end
                progress = .999
                timesLeft['file'] = 1
            # stat["fractionPrinted"] = float("%.1f" % (progress * 100.))
            stat["fractionPrinted"] = int(progress * 100.)
            '''
            # 2) Used filament amount
            fila_total = sum(e for e in info['filament'])
            if fila_total > 0:
                fila_used = sum(e.raw_filament for i, e in _extrs.items())
                fila_perc = (fila_used / fila_total)
                remaining_time_fila = (printing_time / fila_perc) - printing_time
                timesLeft['filament'] = [float("%.1f" % remaining_time_fila)]
            #'''
            '''
            # 3) Layer statistics
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
                    timesLeft['layer'] = float("%.1f" % remaining_time_layer)
            #'''
            status_resp.update(stat)

        # self.logger.debug("%s", json.dumps(status_resp, indent=4))
        return status_resp

def load_config(config):
    return GuiStats(config)
