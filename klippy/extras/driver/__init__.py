# Stepper driver init wrapper
#
# Copyright (C) 2018  Petri Honkala <cruwaller@gmail.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.

from driverbase import DriverBase
from tmc2130    import TMC2130
from tmc2208    import TMC2208
from tmc2660    import TMC2660
from tmc51xx    import TMC51xx

DRV_MAPPING = {
    'DEFAULT' : DriverBase, 'A4988'   : DriverBase,
    'DRV8825' : DriverBase, 'TMC2100' : DriverBase,
    'TMC2130' : TMC2130,
    'TMC2208' : TMC2208, 'TMC2224' : TMC2208,
    'TMC2660' : TMC2660,
    'TMC5130' : TMC51xx,
    'TMC5160' : TMC51xx,
}

def load_driver(config):
    section = config.get_name()
    printer = config.get_printer()
    _type = config.get('type', None)
    if _type is None:
        if '2130' in section:
            _type = 'TMC2130'
        elif '2208' in section or '2224' in section:
            _type = 'TMC2208'
        elif '2660' in section:
            _type = 'TMC2660'
        else:
            raise config.error("Cannot detect driver type!"
                               "Please define type in driver section")
    driver = DRV_MAPPING[_type.upper()](config)
    printer.add_object(section, driver)
    return driver
