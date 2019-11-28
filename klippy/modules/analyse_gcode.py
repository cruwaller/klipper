# This file may be distributed under the terms of the GNU GPLv3 license.

import os, re, math, json
import logging

READ_SIZE = 16384 # 16kB

class ParseError(Exception):
    pass


def analyse_gcode_file(filepath):
    # Set initial values
    info = {
        "slicer" : "unknown",
        "height" : 0,
        "layerHeight" : 0,
        "firstLayerHeight": 0,
        "filament" : [],
        "buildTime" : 0,
        "size": 0.,
    }
    if filepath is None:
        return info
    absolute_coord = True
    last_position = .0
    try:
        info["size"] = os.path.getsize(filepath)
        with open(filepath, 'rb') as f:
            f.seek(0, os.SEEK_END)
            fsize = f.tell()
            f.seek(0)
            # find used slicer
            slicer = None
            for idx in range(100):
                line = f.readline().strip()
                if "Simplify3D" in line: # S3D
                    slicer = "Simplify3D"
                elif "slic3r" in line.lower():
                    slicer = "Slic3r"
                elif 'PrusaSlicer' in line:
                    slicer = "PrusaSlicer"
                elif ";Sliced by " in line: # ideaMaker
                    slicer = "ideaMaker"
                elif "; KISSlicer" in line: # KISSlicer
                    slicer = "KISSlicer"
                elif ";Sliced at: " in line: # Cura(old)
                    slicer = "Cura (OLD)"
                elif ";Generated with Cura" in line: # Cura(new)
                    slicer = "Cura"
                elif "IceSL" in line:
                    slicer = "IceSL"
                elif "CraftWare" in line:
                    slicer = "CraftWare"
                if slicer is not None:
                    info["slicer"] = slicer
                    break

            layerHeight = None
            firstLayerHeightPercentage = None
            firstLayerHeight = None

            f.seek(0)
            if fsize <= (2 * READ_SIZE):
                # read while file
                data = f.read(fsize)
                lines = data.split('\n')
            else:
                # read head
                data = f.read(READ_SIZE)
                lines = data.split('\n')
                lines.pop()
                # read tail
                f.seek(fsize - READ_SIZE)
                data = f.read(READ_SIZE)
                lines.extend(data.split('\n')[1:])

            args_r = re.compile('([A-Z_]+|[A-Z*/"])')
            build_info_r = re.compile('([0-9\.]+)')
            for line in lines:
                line = line.strip()
                cpos = line.find(';')
                if cpos == 0:
                    # Try to parse slicer infos
                    if slicer is "Simplify3D":
                        if "Build time" in line:
                            parts = build_info_r.split(line)
                            buildTime = .0
                            offset = 1
                            if " hour " in parts:
                                buildTime += 60. * float(parts[offset])
                                offset += 2
                            if " minutes" in parts:
                                buildTime += float(parts[offset])
                            info["buildTime"] = buildTime * 60.
                        elif "Filament length: " in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]))
                        elif "layerHeight" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                        elif "firstLayerHeightPercentage" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeightPercentage = float(parts[1]) / 100.

                    elif slicer in ["Slic3r", "PrusaSlicer"]:
                        if "; filament used" in line:
                            if "mm" in line:
                                parts = build_info_r.split(line)
                                info["filament"].append(float(parts[1]))
                            elif "cm3" in line:
                                # No support at the moment
                                pass
                        elif "; first_layer_height" in line:
                            # first_layer_height = 100%
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                            if "%" in line:
                                firstLayerHeightPercentage = firstLayerHeight / 100.
                                firstLayerHeight = None
                        elif "; layer_height" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                        elif "; estimated printing time" in line:
                            buildTime = 0.
                            parts = line.split("=", 1)
                            # time format is 12h 23m 19s
                            time_re = re.compile("\s*([0-9]+)[hmsHMS]\s*")
                            time_parts = filter(None, time_re.split(parts[1]))
                            time_parts.reverse()
                            for count, time in enumerate(time_parts):
                                buildTime += float(time) * math.pow(60., count)
                            if "normal" in parts[0]:
                                info["buildTime"] = buildTime
                            elif "silent" in parts[0]:
                                # No support at the moment
                                pass

                    elif slicer is "Cura":
                        if "Filament used" in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]) * 1000.) # Convert m to mm
                        elif "Layer height" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                            firstLayerHeight = layerHeight
                    elif slicer is "IceSL":
                        if "z_layer_height_first_layer_mm" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                        elif "z_layer_height_mm" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])

                    elif slicer is "KISSlicer":
                        if ";    Ext " in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[3]))
                        elif "first_layer_thickness_mm" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                        elif "layer_thickness_mm" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])

                    elif slicer is "CraftWare":
                        # encoded settings in gcode file, need to extract....
                        pass
                    continue

                # Remove comments
                if cpos >= 0:
                    line = line[:cpos]
                # Parse args
                parts = args_r.split(line.upper())[1:]
                params = { parts[i]: parts[i+1].strip()
                           for i in range(0, len(parts), 2) }
                if parts and parts[0] == 'N':
                    # Skip line number at start of command
                    del parts[:2]
                if not parts:
                    continue
                # Find object height
                if "G" in params:
                    try:
                        gnum = int(params['G'])
                    except ValueError as err:
                        logging.error("GCode line error: %s" % line)
                        logging.error("GCode params: %s" % params)
                        # not a G1 or G0 -> continue
                        continue
                    if gnum == 0 or gnum == 1:
                        if "Z" in params:
                            if absolute_coord:
                                last_position = float(params['Z'])
                            else:
                                last_position += float(params['Z'])
                    elif gnum == 90:
                        absolute_coord = True
                    elif gnum == 91:
                        absolute_coord = False

            info["height"] = last_position
            # layer height
            if layerHeight is not None:
                info["layerHeight"] = float("%.3f" % layerHeight)
            # first layer height
            if (layerHeight is not None and
                    firstLayerHeightPercentage is not None):
                info["firstLayerHeight"] = float("%.3f" % (
                        layerHeight * firstLayerHeightPercentage))
            if firstLayerHeight is not None:
                info["firstLayerHeight"] = float("%.3f" % firstLayerHeight)
    except (IOError, ParseError) as err:
        logging.error("Parsing error: '%s'" % err)
    except ValueError as err:
        logging.error("GCode file has an issue! '%s'" % err)
    # logging.info("PARSED: %s" % info)
    return info


class GCodeAnalyser:
    def __init__(self, config, sd_path=None):
        self.dir = sd_path
        if sd_path is None:
            printer = config.get_printer()
            sd = printer.try_load_module(config, "virtual_sdcard")
            self.dir = sd.sdcard_dirname
        self._db = self._open_database()
    def _open_database(self):
        db = {}
        try:
            with open(os.path.join(self.dir, "analysed.json"), "rb") as _f:
                for line in _f:
                    parts = line.split("=", 1)
                    db[parts[0].strip()] = json.loads(parts[1])
            return db
        except IOError:
            return {}
    def _write_database(self):
        with open(os.path.join(self.dir, "analysed.json"), "w+") as _f:
            #json.dump(self._db, _f)
            for fname, data in self._db.items():
                _f.write("%s=%s\n" % (fname, json.dumps(data)))
    def _append_file_info(self, fname, data):
        with open(os.path.join(self.dir, "analysed.json"), "a+") as _f:
            # json.dump(data, _f)
            _f.write("%s=%s\n" % (fname, json.dumps(data)))
    def remove_file(self, path):
        if type(path) != list:
            path = [path]
        for fname in path:
            if fname in self._db:
                del self._db[fname]
        self._write_database()
    def move_file(self, source, target):
        if source in self._db:
            self._db[target] = self._db[source]
            self.remove_file(source)
    def get_file_info(self, path):
        if path in self._db:
            return self._db[path]
        info = analyse_gcode_file(path)
        self._db[path] = info
        self._append_file_info(path, info)
        return info


def load_config(config, sd_path=None):
    return GCodeAnalyser(config, sd_path)
