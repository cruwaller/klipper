
# This is just a basic stepper driver without any special features

class DriverBase(object):
    motors_deg_map = { "7.5": 7.5, "1.8": 1.8, "0.9": 0.9, '0': None }

    def __init__(self, printer, config, config_parent, logger=None):
        self.microsteps = config.getint('microsteps', default=None)
        if self.microsteps is None:
            self.microsteps = config_parent.getint('microsteps', default=16)
        self.motor_deg = config.getfloat('motor_step_angle', above=0., default=None)
        if self.motor_deg is None:
            self.motor_deg = config_parent.getfloat('motor_step_angle', above=0., default=1.8)

        # Read config and send to driver
        self.step_dist = config.getfloat('step_distance', default=None, above=0.)
        if self.step_dist is None:
            self.step_dist = config_parent.getfloat('step_distance', default=None, above=0.)
        if self.step_dist is not None:
            self.inv_step_dist = 1. / self.step_dist
        else:
            steps_per_mm = config.getfloat('steps_per_mm', default=None, above=0.)
            if steps_per_mm is None:
                steps_per_mm = config_parent.getfloat('steps_per_mm', default=None, above=0.)
            if steps_per_mm is None:
                if (self.motor_deg is None or self.microsteps is None):
                    raise self.error("'microsteps' and 'motor_step_angle' have to be defined!")
                # Calculate base on settings
                pitch = config.getfloat('pitch', None, above=0.)
                if pitch is None:
                    pitch = config_parent.getfloat('pitch', 2., above=0.)
                teeth = config.getfloat('teeths', None, above=0.)
                if teeth is None:
                    teeth = config_parent.getfloat('teeths', 20., above=0.)
                ratio = config.getfloat('gear_ratio', None)
                if ratio is None:
                    ratio = config_parent.getfloat('gear_ratio', 1.0)
                self.motor_rev = 360. / self.motor_deg
                steps_per_mm = self.motor_rev * self.microsteps / (pitch * teeth) * ratio
            self.inv_step_dist = steps_per_mm
            self.step_dist = 1.0 / self.inv_step_dist

    # Virtual class for parents to override
    def init_home(self, *args, **kwargs):
        pass;
    def status(self, *args, **kwargs):
        pass
    def clear_faults(self, *args, **kwargs):
        pass
    def set_dir(self, *args, **kwargs):
        pass
    def set_current(self, *args, **kwargs):
        pass
    def get_current(self, *args, **kwargs):
        pass
    def is_stall(self, *args, **kwargs):
        pass
    def lost_steps(self, *args, **kwargs):
        pass
