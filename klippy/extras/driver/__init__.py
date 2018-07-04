# Stepper driver init wrapper
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from driverbase import DriverBase
from tmc2130    import TMC2130
from tmc51xx    import TMC51xx

DRV_MAPPING = {
    'DEFAULT' : DriverBase,
    'A4988'   : DriverBase,
    'DRV8825' : DriverBase,
    'TMC2100' : DriverBase,
    'TMC2130' : TMC2130,
    'TMC5130' : TMC51xx,
    'TMC5160' : TMC51xx,
}

def load_driver(config):
    section = config.get_name()
    printer = config.get_printer()
    driver = DRV_MAPPING[config.get('type', 'default').upper()](config)
    printer.add_object(section, driver)
    return driver
