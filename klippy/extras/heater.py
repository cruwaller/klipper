# This file may be distributed under the terms of the GNU GPLv3 license.

def load_config_prefix(config):
    if 'heater bed' == config.get_name():
        pheater = config.get_printer().lookup_object('heater')
        config.get_printer().add_object(
            'heater_bed', pheater.setup_heater(config, 'B'))
