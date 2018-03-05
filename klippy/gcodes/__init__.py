'''
files in this folder will be loaded

    - automatically if file contains
        def load_module(printer)
      method.

    - if file contains
        def load_config(config) or
        def load_config_prefix(config)
      method and section (e.g. gcodes.retract) is
      defined in config file.
'''
