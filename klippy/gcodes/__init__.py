# This file may be distributed under the terms of the GNU GPLv3 license.

import generic
import gui_stats

def load_gcodes(config):
    # Load all default gcode files
    generic.GenericGcode(config)
    if config.has_section('reprapgui_process'):
        config.get_printer().try_load_module(
            config, 'gui_stats', folder='gcodes')
