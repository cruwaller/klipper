
# This is loaded if defined in config file
#
# [modules.example_manual name]
# variable: value
#

class ExampleManualModule(object):
    def __init__(self, config):
        self.printer = config.get_printer()
        self.logger = self.printer.logger.getChild("module.example_manual")
        self.logger.debug("Example module loaded...")

def load_config_prefix(config):
    ExampleManualModule(config)
