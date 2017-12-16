import logging
# import supported special drivers
import tmc2130

'''
TODO:
  - L6470
  - TMC26XX
  - Other?
'''

class DriverBase(object):
    def __init__(self, printer, config):
        # Read config and send to driver
        self.step_dist = config.getfloat('step_distance', None, above=0.)
        if self.step_dist is not None:
            self.inv_step_dist = 1. / self.step_dist
        else:
            self.inv_step_dist = config.getfloat('steps_per_mm', above=0.)
            self.step_dist = 1.0 / float(self.inv_step_dist)

def get_driver(printer, config, name=None):
    if name is not None:
        logging.info(" !!! driver name : %s" % name)
        config = config.getsection(name)
    mapping = { 'DEFAULT' : DriverBase,
                'A4988'   : DriverBase,
                'DRV8825' : DriverBase,
                'TMC2130' : tmc2130.TMC2130 }
    return mapping[config.get('type', 'default').upper()](printer, config)
