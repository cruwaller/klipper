# This file may be distributed under the terms of the GNU GPLv3 license.

"""
files in this folder will be loaded
    - default modules are loaded always
    - if file contains
        def load_config(config) or
        def load_config_prefix(config)
      method and section (e.g. gcodes.retract) is
      defined in config file.
"""
import generic
import gui_stats

def load_gcodes(config):
    # Load all default gcode files
    generic.GenericGcode(config)
    if config.has_section('reprapgui_process') or \
            config.has_section('reprapgui'):
        gui_stats.GuiStats(config)
