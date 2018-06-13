import os

class HostCmd(object):
    def __init__(self, printer, config):
        self.printer = printer
        self.gcode = gcode = printer.lookup_object('gcode')
        self.cmd_list = []
        commands = config.get("commands").splitlines()
        for cmd in commands:
            cmd = cmd.strip()
            if len(cmd) == 0:
                continue
            parts = cmd.split("=")
            try:
                alias = parts[0].strip()
                command = parts[1].strip()
            except IndexError:
                raise config.error("Command type must be 'alist = <command with args>'")
            self.cmd_list.append(Command(gcode, alias.upper(), command))
        printer.add_object("hostcmd", self)

class Command(object):
    def __init__(self, gcode, name, command):
        self.logger = gcode.logger.getChild(name)
        self.gcode = gcode
        self.command = command
        name = "HOST_CMD_" + name
        gcode.register_command(
            name, self.execute, when_not_ready=True,
            desc="Execute host command '%s'" % command)
        self.logger.info("ALIAS %s : %s registered" % (name, command))
    def execute(self, params):
        self.logger.debug("command %s executed" % self.command)
        msg = os.popen(self.command).read()
        self.gcode.respond_info(msg)

def load_module(printer, config):
    if not config.has_section("hostcmd"):
        return None
    return HostCmd(printer, config.getsection("hostcmd"))
