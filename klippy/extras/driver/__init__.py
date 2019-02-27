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
    'TMC2130' : TMC2130,
    'TMC2208' : TMC2208, 'TMC2224' : TMC2208,
    'TMC2660' : TMC2660,
    'TMC5130' : TMC51xx,
    'TMC5160' : TMC51xx,
}

def load_driver(stepper_config):
    driver_config = stepper_config
    section = _type = None
    driver_name = stepper_config.get('driver', None)
    if driver_name:
        section_name = 'driver %s' % driver_name
        if stepper_config.has_section(section_name):
            driver_config = stepper_config.getsection(section_name)
            section = driver_config.get_name()
            _type = driver_config.get('type', None)
            if _type is None:
                if '2130' in section:
                    _type = 'TMC2130'
                elif '2208' in section or '2224' in section:
                    _type = 'TMC2208'
                elif '2660' in section:
                    _type = 'TMC2660'
                elif '5130' in section:
                    _type = 'TMC5130'
                elif '5160' in section:
                    _type = 'TMC5160'
            else:
                _type = _type.upper()
    func = DRV_MAPPING.get(_type, DriverBase)
    driver = func(driver_config, stepper_config)
    if section:
        stepper_config.get_printer().add_object(section, driver)
    return driver
