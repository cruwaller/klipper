
# This is just a basic stepper driver without any special features

class DriverBase(object):
    inv_step_dist = step_dist = microsteps = None
    def __init__(self, config):
        self.microsteps = config.getint('microsteps', None)
        # Driver class can override existing stepper config steps
        self.step_dist = config.getfloat('step_distance', default=None, above=0.)
        if self.step_dist is not None:
            self.inv_step_dist = 1. / self.step_dist
        else:
            self.inv_step_dist = config.getfloat('steps_per_mm', default=None, above=0.)
            if self.inv_step_dist is not None:
                self.step_dist = 1.0 / self.inv_step_dist
