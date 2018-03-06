
# This is loaded automatically when Klippy starts

class ExampleAutoModule(object):
    def __init__(self, printer):
        self.logger = printer.logger.getChild("module.example")
        self.logger.debug("Example module loaded...")

def load_module(printer):
    ExampleAutoModule(printer)
