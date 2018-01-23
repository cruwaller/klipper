import os, sys
'''
TODO:
  - L6470
  - TMC26XX
  - Other?
'''
from driverbase import DriverBase
from tmc2130    import TMC2130

def get_driver(printer, config, name=None, logger=None):
    if name is not None:
        config = config.getsection("driver %s" % (name,))
    mapping = { 'DEFAULT' : DriverBase,
                'A4988'   : DriverBase,
                'DRV8825' : DriverBase,
                'TMC2100' : DriverBase,
                'TMC2130' : TMC2130 }
    return mapping[config.get('type', 'default').upper()](printer,
                                                          config,
                                                          logger=logger)
