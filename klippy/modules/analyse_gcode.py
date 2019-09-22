# This file may be distributed under the terms of the GNU GPLv3 license.

import os, re
import logging

class ParseError(Exception):
    pass


ANALYSED_GCODE_FILES = {}

def analyse_gcode_file(filepath):
    # Set initial values
    info = {
        "slicer" : "unknown",
        "height" : 0,
        "layerHeight" : 0,
        "firstLayerHeight": 0,
        "filament" : [],
        "buildTime" : 0
    }
    if filepath is None:
        return info
    if filepath in ANALYSED_GCODE_FILES:
        return ANALYSED_GCODE_FILES[filepath]
    elif os.path.join("gcode", filepath) in ANALYSED_GCODE_FILES:
        return ANALYSED_GCODE_FILES[filepath]
    absolute_coord = True
    last_position = .0
    try:
        with open(filepath, 'rb') as f:
            #f.seek(0, os.SEEK_END)
            #fsize = f.tell()
            f.seek(0)
            # find used slicer
            slicer = None
            for idx in range(100):
                line = f.readline().strip()
                if "Simplify3D" in line: # S3D
                    slicer = "Simplify3D"
                elif "Slic3r" in line or "slic3r" in line: # slic3r
                    slicer = "Slic3r"
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
                    break
            # Stop if slicer is not detected!
            if slicer is None:
                raise ParseError("Cannot detect slicer")
            info["slicer"] = slicer
            # read header
            layerHeight = None
            firstLayerHeightPercentage = None
            firstLayerHeight = None
            # read footer and find object height
            f.seek(0)
            args_r = re.compile('([A-Z_]+|[A-Z*/])')
            build_info_r = re.compile('([0-9\.]+)')
            for line in f:
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
                    elif slicer is "Slic3r":
                        if "filament used" in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]))
                        elif "first_layer_height" in line:
                            # first_layer_height = 100%
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                            if "%" in line:
                                firstLayerHeightPercentage = firstLayerHeight / 100.
                                firstLayerHeight = None
                        elif "layer_height" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
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
                # Find object height
                if "G" in params:
                    gnum = int(params['G'])
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
            # first layer height
            if layerHeight is not None:
                info["layerHeight"] = float("%.3f" % layerHeight)
            if layerHeight is not None and firstLayerHeightPercentage is not None:
                info["firstLayerHeight"] = float("%.3f" % (layerHeight * firstLayerHeightPercentage))
            if firstLayerHeight is not None:
                info["firstLayerHeight"] = float("%.3f" % firstLayerHeight)
    except (IOError, ParseError):
        pass
    ANALYSED_GCODE_FILES[filepath] = info
    # logging.info("PARSED: %s" % info)
    return info
