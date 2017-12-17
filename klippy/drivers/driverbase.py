
# This is just a basic stepper driver without any special features

class DriverBase(object):
    def __init__(self, printer, config, logger):
        # Read config and send to driver
        self.step_dist = config.getfloat('step_distance', None, above=0.)
        if self.step_dist is not None:
            self.inv_step_dist = 1. / self.step_dist
        else:
            self.inv_step_dist = config.getfloat('steps_per_mm', above=0.)
            self.step_dist = 1.0 / float(self.inv_step_dist)
