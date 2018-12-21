"""
files in this folder will be loaded
    - default modules are loaded always
    - if file contains
        def load_config(config) or
        def load_config_prefix(config)
      method and section (e.g. gcodes.retract) is
      defined in config file.
"""
import babysteps
import generic
import gui_stats

def load_gcodes(config):
    # Load all default gcode files
    babysteps.BabySteps(config)
    generic.GenericGcode(config)
    gui_stats.GuiStats(config)
