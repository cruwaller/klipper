import extruder

class DriverGcode(object):
    def __init__(self, printer):
        self.printer = printer
        self.gcode = printer.lookup_object('gcode')
        for cmd in ['DRV_STATUS', 'DRV_CURRENT', 'DRV_SG']:
            self.gcode.register_command(cmd,
                                        getattr(self, 'cmd_' + cmd),
                                        desc=getattr(self, 'cmd_' + cmd + '_help', None))
        self.respond_info = self.gcode.respond_info
        self.axis2pos = self.gcode.axis2pos
        self.logger = self.gcode.logger
        self.drivers = []
        # register to get printer_state called
        printer.add_object("gcode driver", self)
        self.logger.info("Driver GCode extension initialized")

    def printer_state(self, state):
        if state == 'ready':
            self.drivers = self.printer.lookup_module_objects("driver")

    cmd_DRV_STATUS_help = "Return the status of the configured drivers"
    def cmd_DRV_STATUS(self, params):
        for driver in self.drivers:
            if hasattr(driver, 'status'):
                driver.status(log=self.respond_info)

    cmd_DRV_CURRENT_help = "Set the driver current by driver name"
    def cmd_DRV_CURRENT(self, params):
        for key,val in params.items():
            if key.startswith("#"):
                continue
            current = self.gcode.get_float(key, params, None)
            if current is not None:
                driver = self.printer.lookup_object('driver %s'%(key.lower()), None)
                if hasattr(driver, 'set_current'):
                    self.respond_info("%s : %s" % (key, driver.set_current(current)))

    cmd_DRV_SG_help = "Set driver stall guard by driver name"
    def cmd_DRV_SG(self, params):
        for key,val in params.items():
            if key.startswith("#"):
                continue
            sgval = self.gcode.get_int(key, params, None)
            if sgval is not None:
                driver = self.printer.lookup_object('driver %s'%(key.lower()), None)
                if hasattr(driver, 'set_stallguard'):
                    self.respond_info("%s : %s" % (key, driver.set_stallguard(sgval)))

def load_gcode(printer):
    DriverGcode(printer)
