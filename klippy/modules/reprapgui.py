#
# This module is handling request from DuetWebControl (http://reprap.org/wiki/Duet_Web_Control)
#     Tornado webserver is needed to run page
#       Note: Tornado version 4.5 is required!
#         - install Tornado using pip ( $pip install tornado==4.5 )
#         - or download from https://github.com/tornadoweb/tornado/tree/branch4.5
#           and use environment variable 'export TORNADO_PATH=<path to tornado folder>'
#     Create SSL cert:
#         - openssl req -newkey rsa:2048 -new -nodes -x509 -days 3650 -keyout <path>/key.pem -out <path>/cert.pem
#
"""
Example config sections:

[virtual_sdcard]
path: ~/.octoprint/uploads/

[reprapgui]
name: This is my printer
user: test
password: test
http: 80
; Enable for SSL connection
;https: 443
;cert: ~/ssl/server.crt
;key: ~/ssl/server.key
; Video feed
;feedrate: 1.0
;camera_index: 0
"""

import time, sys, os, errno, threading, json, logging
import base64, uuid
import functools

try:
    sys.path.append(os.path.normpath(
        os.path.expanduser(os.environ['TORNADO_PATH'])))
except KeyError:
    pass
import tornado.ioloop
import tornado.web
import tornado.websocket

_PARENT = None
KLIPPER_CFG_NAME = 'klipper_config.cfg'
KLIPPER_LOG_NAME = "klippy.log"
MAX_STREAMED_SIZE = 1024**3

def dict_dump_json(data, logger=None):
    if logger is None:
        logger = logging
    logger.info(json.dumps(data,
                           #sort_keys=True,
                           sort_keys=False,
                           indent=4,
                           separators=(',', ': ')))


class BaseHandler(tornado.web.RequestHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user", max_age_days=5)

class MainHandler(BaseHandler):
    path = None
    def initialize(self, file):
        self.file = file
    @tornado.web.authenticated
    def get(self, path):
        self.render(self.file)

class LoginHandler(BaseHandler):
    parent = None
    def initialize(self):
        self.parent = _PARENT
    @tornado.gen.coroutine
    def get(self):
        incorrect = self.get_secure_cookie("incorrect")
        if incorrect and int(incorrect) > 20:
            self.write('<center>blocked</center>')
            return
        # Skip login if user or passwd is not set
        if not self.parent.user or not self.parent.passwd:
            self.set_secure_cookie("user", "John Doe")
            self.set_secure_cookie("incorrect", "0")
            self.redirect(self.reverse_url("main", ""))
            return
        login = '''
        <html>
        <head>
        <title>Please Log In</title>
        </head>
        <body>
        <form action="/login" method="POST">
            <p>Username: <input type="text" name="username" /><p>
            <p>Password: <input type="password" name="password" /><p>
            <p><input type="submit" value="Log In" /></p>
        </form>
        </body>
        </html>
        '''
        self.write(login)

    @tornado.gen.coroutine
    def post(self):
        incorrect = self.get_secure_cookie("incorrect")
        if incorrect and int(incorrect) > 20:
            self.write('<center>blocked</center>')
            return

        getusername = tornado.escape.xhtml_escape(self.get_argument("username"))
        getpassword = tornado.escape.xhtml_escape(self.get_argument("password"))
        if self.parent.user == getusername and \
           self.parent.passwd == getpassword:
            self.set_secure_cookie("user", self.get_argument("username"))
            self.set_secure_cookie("incorrect", "0")
            self.redirect(self.reverse_url("main", ""))
        else:
            incorrect = self.get_secure_cookie("incorrect") or 0
            increased = str(int(incorrect)+1)
            self.set_secure_cookie("incorrect", increased)
            raise tornado.web.HTTPError(401)

class LogoutHandler(BaseHandler):
    def get(self):
        self.clear_cookie("user")
        self.redirect(self.get_argument("next", self.reverse_url("main")))


@tornado.web.stream_request_body
class rrHandler(BaseHandler):
    parent = printer = sd_path = logger = None

    def initialize(self, sd_path):
        self.klipper_cfg = None
        self.upload_fp = None
        self.upload_bytes_written = 0

        self.parent = _PARENT
        self.printer = self.parent.printer
        self.sd_path = sd_path
        self.logger = self.parent.logger

    @tornado.web.authenticated
    def get(self, path, *args, **kwargs):
        sd_path = self.sd_path
        respdata = {"err" : 1}

        #self.logger.info("uri: %s", self.request.uri)

        # rr_connect?password=XXX&time=YYY
        if "rr_connect" in path:
            respdata["err"] = 1
            #_passwd = self.get_argument('password')
            #if not self.parent.passwd or self.parent.passwd == _passwd:
            if True:
                respdata["err"] = 0
                # 0 = success, 1 = wrong passwd, 2 = No more HTTP sessions available
                respdata["sessionTimeout"] = 30000 # ms
                # duetwifi10, duetethernet10, radds15, alligator2, duet06, duet07, duet085, default: unknown
                respdata["boardType"] = "klipper"

        # rr_disconnect
        elif "rr_disconnect" in path:
            respdata["err"] = 0

        # rr_status?type=XXX
        # http://reprap.org/wiki/RepRap_Firmware_Status_responses
        elif "rr_status" in path:
            _type = int(self.get_argument('type'))
            if _type < 1 or _type > 3:
                _type = 1
            # get status from Klippy
            respdata["err"] = 0
            respdata.update(self.parent.gui_stats.get_status_stats(_type))
            if self.parent.atx_state is not None:
                # update ATX power state
                if "params" in respdata:
                    respdata['params']['atxPower'] = int(self.parent.atx_state)
            respdata['seq'] = (len(self.parent.gcode_resps) +
                               len(self.parent.gcode_resps_async))

        # rr_gcode?gcode=XXX
        elif "rr_gcode" in path:
            respdata["err"] = 0
            respdata["buff"] = 99999
            gcode = self.get_argument('gcode')
            #self.logger.debug("rr_gcode={}".format(gcode))
            self.parent.send_gcode_from_handler_async(gcode)

        # rr_download?name=XXX
        elif "rr_download" in path:
            # Download a specified file from the SD card.
            path = self.get_argument('name').replace("0:/", "")
            if KLIPPER_CFG_NAME in path:
                path = os.path.abspath(
                    self.printer.get_start_arg('config_file'))
            elif KLIPPER_LOG_NAME in path:
                path = os.path.abspath(
                    self.printer.get_start_arg('logfile'))
            elif "heightmap.csv" in path:
                bed_mesh = self.printer.lookup_object('bed_mesh', None)
                calibrate = getattr(bed_mesh, "calibrate", None)
                if bed_mesh is not None and bed_mesh.z_mesh and calibrate:
                    self.set_header('Content-Type',
                                    'application/force-download')
                    self.set_header('Content-Disposition',
                                    'attachment; filename=heightmap.csv')
                    params = calibrate.get_probe_params()
                    spacing_x = (params['max_x'] - params['min_x']) / (params['x_count'] - 1)
                    spacing_y = (params['max_y'] - params['min_y']) / (params['y_count'] - 1)
                    csv = [
                        "Klipper height map",
                        "xmin,xmax,ymin,ymax,radius,xspacing,yspacing,xnum,ynum",
                        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d" % (
                            params['min_x'], params['max_x'],
                            params['min_y'], params['max_y'],
                            params.get('radius', -1),
                            params.get('xspacing', spacing_x),
                            params.get('yspacing', spacing_y),
                            params['x_count'], params['y_count']),
                    ]
                    points_csv = calibrate.print_probed_positions_to_csv()
                    data = "\n".join(csv) + "\n" + points_csv
                    # self.logger.debug("heightmap.csv:\n%s", data)
                    self.write(data)
                    self.finish()
                    return
                else:
                    raise tornado.web.HTTPError(404)
            else:
                path = os.path.abspath(os.path.join(sd_path, path))
            # Check if file exists and upload
            if not os.path.exists(path):
                raise tornado.web.HTTPError(404)
            else:
                self.set_header('Content-Type', 'application/force-download')
                self.set_header('Content-Disposition',
                                'attachment; filename=%s' % os.path.basename(path))
                try:
                    with open(path, "rb") as f:
                        self.write( f.read() )
                except IOError:
                    # raise tornado.web.HTTPError(500)
                    raise tornado.web.HTTPError(404)
                self.finish()
                return

        # rr_delete?name=XXX
        elif "rr_delete" in path:
            # resp: `{"err":[code]}`
            respdata["err"] = 0
            directory = self.get_argument('name').replace("0:/", "")
            if KLIPPER_CFG_NAME in directory or KLIPPER_LOG_NAME in directory:
                pass
            else:
                fremoved = []
                directory = os.path.abspath(os.path.join(sd_path, directory))
                #self.logger.debug("delete: absolute path {}".format(directory))
                try:
                    for root, dirs, files in os.walk(directory, topdown=False):
                        for name in files:
                            fpath = os.path.join(root, name)
                            os.remove(fpath)
                            fremoved.append(fpath)
                        for name in dirs:
                            os.rmdir(os.path.join(root, name))
                    if os.path.isdir(directory):
                        os.rmdir(directory)
                    else:
                        os.remove(directory)
                        fremoved.append(directory)
                    if fremoved:
                        handler = self.printer.lookup_object("analyse_gcode")
                        handler.remove_file(fremoved)
                except OSError as e:
                    self.logger.error("rr_delete: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_filelist?dir=XXX
        elif path in ["rr_filelist", 'rr_files']:
            lst_dirs = bool(self.get_argument('flagDirs', True))
            self.parent.get_filelist(self.get_argument('dir'),
                                     respdata, lst_dirs)

        # rr_fileinfo?name=XXX
        elif "rr_fileinfo" in path:
            name = self.get_argument('name', default=None)
            if name is None:
                sd = self.printer.lookup_object('virtual_sdcard')
                try:
                    # current file printed
                    if sd.current_file is not None:
                        name = sd.current_file.name
                    else:
                        raise AttributeError
                except AttributeError:
                    name = None
            # info about the requested file
            self.parent.get_file_info(name, respdata)

        # rr_move?old=XXX&new=YYY
        elif "rr_move" in path:
            # {"err":[code]} , code 0 if success
            respdata["err"] = 0
            _from = self.get_argument('old').replace("0:/", "")
            if KLIPPER_CFG_NAME in _from or KLIPPER_LOG_NAME in _from:
                pass
            else:
                _from = os.path.abspath(os.path.join(sd_path, _from))
                _to   = self.get_argument('new').replace("0:/", "")
                _to   = os.path.abspath(os.path.join(sd_path, _to))
                try:
                    os.rename(_from, _to)
                    handler = self.printer.lookup_object("analyse_gcode")
                    handler.move_file(_from, _to)
                except OSError as e:
                    self.logger.error("rr_move: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_mkdir?dir=XXX
        elif "rr_mkdir" in path:
            # {"err":[code]} , 0 if success
            respdata["err"] = 0
            directory = self.get_argument('dir').replace("0:/", "")
            directory = os.path.abspath(os.path.join(sd_path, directory))
            try:
                os.makedirs(directory)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    self.logger.error("rr_mkdir: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_config / rr_configfile
        elif "rr_configfile" in path:
            self.logger.info("rr_configfile: {}".format(self.request.uri))
            respdata = { "err" : 0 }

        elif "rr_config" in path:
            respdata = self.parent.gui_stats.get_config_stats()

        elif "rr_reply" in path:
            out = self.parent.get_gcode_resps()
            out.extend(self.parent.get_gcode_async_resps())
            self.write("\n".join(out))
            return

        else:
            self.logger.error("  get(path={})".format(path))
            self.logger.error("     !! uri: {}".format(self.request.uri))

        # Send response back to client
        respstr = json.dumps(respdata)
        self.write(respstr)

    def _parse_file_path(self, filename):
        filename = filename.replace("0:/", "")
        if KLIPPER_CFG_NAME in filename:
            cfgname = os.path.abspath(
                self.printer.get_start_arg('config_file'))
            datestr = time.strftime("-%Y%m%d_%H%M%S")
            backup_name = cfgname + datestr
            temp_name = cfgname + "_autosave"
            if cfgname.endswith(".cfg"):
                backup_name = cfgname[:-4] + datestr + ".cfg"
                temp_name = cfgname[:-4] + "_autosave.cfg"
            try:
                os.rename(cfgname, backup_name)
                self.klipper_cfg = (cfgname, temp_name)
            except IOError:
                raise tornado.web.HTTPError(400)
            filename = temp_name
        elif KLIPPER_LOG_NAME in filename:
            filename = None
        elif "heightmap.csv" in filename:
            filename = None
        else:
            filename = os.path.abspath(os.path.join(self.sd_path, filename))
            try:
                # try to create a dir first
                os.makedirs(os.path.dirname(filename))
            except OSError:
                pass
        return filename

    def post(self, path, *args, **kwargs):
        error = 1
        if "rr_upload" in self.request.path:
            if self.upload_fp is not None:
                self.upload_fp.close()
                if self.klipper_cfg is not None:
                    # if file was klipper.cfg then rename temp file
                    os.rename(self.klipper_cfg[1], self.klipper_cfg[0])

            file_crc32 = self.get_argument('crc32', None)
            size = int(self.request.headers['Content-Length'])
            if self.upload_bytes_written != size:
                self.logger.error("upload size error: %s != %s" % (
                    self.upload_bytes_written, size))
            error = int(self.upload_bytes_written != size)
        # Send response back to client
        self.write(json.dumps({ "err" : error }))

    def prepare(self):
        self.request.connection.set_max_body_size(MAX_STREAMED_SIZE)

    def data_received(self, chunk):
        if "rr_upload" in self.request.path:
            # /rr_upload?name=xxxxx&time=xxxx
            # /rr_upload?name=0:/filaments/PLA/unload.g&time=2017-11-30T11:46:50
            if self.upload_fp is None:
                filename = self._parse_file_path(self.get_argument('name'))
                if filename is None:
                    return
                try:
                    self.upload_fp = open(filename, "wb+")
                except IOError:
                    raise tornado.web.HTTPError(400)
            self.upload_fp.write(chunk)
            self.upload_bytes_written += len(chunk)


connections = set()

class WebSocketHandler(tornado.websocket.WebSocketHandler):
    logger = logging.getLogger("WebSocket")
    def __init__(self, *args, **kwargs):
        super(WebSocketHandler, self).__init__(*args, **kwargs)
        self._wait_ack = False
        self._lock = threading.Lock()
    def send_message(self, msg):
        with self._lock:
            self.write_message(msg) # yield ?
    def open(self):
        logging.debug("Client connected")
        status = _PARENT.gui_stats.get_status_new(True)
        if _PARENT.passwd:
            status["network"]["password"] = _PARENT.passwd
        status = json.dumps(status, separators=(',', ':'))
        self.send_status_update(status)
        connections.add(self)
    def on_message(self, commands):
        if "PING" in commands:
            self.send_message("PONG\n")
        elif "OK" in commands:
            self._wait_ack = False
        else:
            self.logger.error("[RX] unknown command '%s'" % commands)
    def on_close(self):
        logging.debug("Client left")
        connections.remove(self)
    def send_status_update(self, status):
        if self._wait_ack:
            return
        self._wait_ack = True
        self.send_message(status)


class GetDirectoryHandler(BaseHandler):
    @tornado.web.authenticated
    def get(self, value):
        respdata = {}
        _PARENT.get_filelist(value, respdata)
        self.write(json.dumps(respdata["files"]))
    def put(self, path):
        directory = path.replace("0:", _PARENT.sd_path)
        try:
            os.makedirs(directory)
        except OSError as e:
            if e.errno != errno.EEXIST:
                logging.error("mkdir: %s" % (e.strerror,))
                raise tornado.web.HTTPError(403)


class FileInfoHandler(BaseHandler):
    @tornado.web.authenticated
    def get(self, value):
        respdata = {"err" : 1}
        _PARENT.get_file_info(value, respdata)
        self.write(json.dumps(respdata))


class GcodeCommandHandler(BaseHandler):
    @tornado.web.authenticated
    def post(self):
        gcodes = self.request.body
        resp = _PARENT.send_gcode_from_handler_sync(gcodes)
        #if not resp:
        #    raise tornado.web.HTTPError(500)
        self.write(resp)


class ConfigFileHandler(tornado.web.StaticFileHandler):
    def get(self, path, include_body=True):
        self.absolute_path = self.get_absolute_path(
            self.root, self.default_filename)
        super(ConfigFileHandler, self).get(self.default_filename, include_body)
    def put(self, path):
        parent = _PARENT
        cfgname = self.get_absolute_path(self.root, self.default_filename)
        datestr = time.strftime("-%Y%m%d_%H%M%S")
        backup_name = cfgname + datestr
        if cfgname.endswith(".cfg"):
            backup_name = cfgname[:-4] + datestr + ".cfg"
        try:
            os.rename(cfgname, backup_name)
            with open(cfgname, "wb+") as _file:
                _file.write(self.request.body)
        except IOError as e:
            parent.logger.error("put: %s" % (e.strerror,))
            raise tornado.web.HTTPError(400)
    def delete(self, *args, **kwargs):
        raise tornado.web.HTTPError(405)
    def post(self, *args, **kwargs):
        raise tornado.web.HTTPError(405)


class LogFileHandler(tornado.web.StaticFileHandler):
    def get(self, path, include_body=True):
        self.absolute_path = self.get_absolute_path(
            self.root, self.default_filename)
        super(LogFileHandler, self).get(self.default_filename, include_body)
    def put(self, *args, **kwargs):
        raise tornado.web.HTTPError(405)
    def delete(self, *args, **kwargs):
        raise tornado.web.HTTPError(405)
    def post(self, *args, **kwargs):
        raise tornado.web.HTTPError(405)


class HeightmapFileHandler(tornado.web.RequestHandler):
    def get(self, path, include_body=True):
        bed_mesh = _PARENT.printer.lookup_object('bed_mesh', None)
        calibrate = getattr(bed_mesh, "calibrate", None)
        if bed_mesh is not None and bed_mesh.z_mesh and calibrate:
            params = calibrate.get_probe_params()
            spacing_x = (params['max_x'] - params['min_x']) / (params['x_count'] - 1)
            spacing_y = (params['max_y'] - params['min_y']) / (params['y_count'] - 1)
            csv = [
                "Klipper height map",
                "xmin,xmax,ymin,ymax,radius,xspacing,yspacing,xnum,ynum",
                "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%d" % (
                    params['min_x'], params['max_x'],
                    params['min_y'], params['max_y'],
                    params.get('radius', -1),
                    params.get('xspacing', spacing_x),
                    params.get('yspacing', spacing_y),
                    params['x_count'], params['y_count']),
            ]
            points_csv = calibrate.print_probed_positions_to_csv()
            data = "\n".join(csv) + "\n" + points_csv
            self.set_header('Content-Type',
                            'application/force-download')
            self.set_header('Content-Disposition',
                            'attachment; filename=heightmap.csv')
            self.write(data)
            self.finish()
        else:
            raise tornado.web.HTTPError(404)


class FileCommandHandler(tornado.web.StaticFileHandler):
    def parse_url_path(self, url_path):
        return super(FileCommandHandler, self).parse_url_path(
            url_path.replace("0:/", ""))
    def parse_abs_file_path(self, path):
        return os.path.join(self.root, self.parse_url_path(path))
    def prepare(self):
        self.request.connection.set_max_body_size(MAX_STREAMED_SIZE)
    def data_received(self, chunk):
        logging.info("data_received()")
        pass
    def put(self, path):
        parent = _PARENT
        # file upload
        path = self.parse_abs_file_path(path)
        try:
            with open(path, "wb+") as _file:
                _file.write(self.request.body)
        except IOError as e:
            parent.logger.error("put: %s" % (e.strerror,))
            raise tornado.web.HTTPError(400)
    def delete(self, path):
        parent = _PARENT
        path = self.parse_abs_file_path(path)
        if KLIPPER_CFG_NAME not in path and KLIPPER_LOG_NAME not in path:
            fremoved = []
            try:
                for root, dirs, files in os.walk(path, topdown=False):
                    for name in files:
                        fpath = os.path.join(root, name)
                        os.remove(fpath)
                        fremoved.append(fpath)
                    for name in dirs:
                        os.rmdir(os.path.join(root, name))
                if os.path.isdir(path):
                    os.rmdir(path)
                else:
                    os.remove(path)
                    fremoved.append(path)
                if fremoved:
                    handler = parent.printer.lookup_object("analyse_gcode")
                    handler.remove_file(fremoved)
            except OSError as e:
                parent.logger.error("delete: %s" % (e.strerror,))
                raise tornado.web.HTTPError(400)
    def post(self, path):
        parent = _PARENT
        if path == "move":
            _from = self.parse_abs_file_path(self.get_argument("from"))
            if KLIPPER_CFG_NAME not in _from and KLIPPER_LOG_NAME not in _from:
                _to = self.parse_abs_file_path(self.get_argument("to"))
                # force = bool(self.get_argument("force"))
                try:
                    os.rename(_from, _to)
                    handler = parent.printer.lookup_object("analyse_gcode")
                    handler.move_file(_from, _to)
                except OSError as e:
                    parent.logger.error("move: %s" % (e.strerror,))
                    raise tornado.web.HTTPError(400)
        else:
            logging.info("post(%s), uri: %s, path: %s" % (
                path, self.request.uri, self.request.path))
            logging.debug("  Arguments: %s" % { k: self.get_argument(k) for k in self.request.arguments })
            # logging.debug("  Body: %s" % self.request.body)


_TORNADO_THREAD = None

class RepRapGuiModule(object):
    warmup_time = .1
    layer_stats = []
    first_layer_start = None
    last_used_file = None
    htmlroot = None
    def __init__(self, config):
        global _TORNADO_THREAD
        global _PARENT
        _PARENT = self
        self.printer = printer = config.get_printer()
        self.logger = printer.get_logger("DWC")
        self.logger_tornado = self.logger.getChild("tornado")
        self.logger_tornado.setLevel(logging.INFO)
        self.gcode = printer.lookup_object('gcode')
        self.resp = ""
        self.resp_rcvd = False
        self.resp_cnt = 0
        self.store_resp = False
        self.gcode_resps = []
        self.gcode_resps_async = []
        self.lock = threading.Lock()
        self.lock_resps = threading.Lock()
        self.reactor = self.printer.get_reactor()
        # self.mutex = self.reactor.mutex()
        # Register events
        printer.register_event_handler('klippy:config_ready', self._config_ready)
        printer.register_event_handler("klippy:disconnect", self._shutdown)
        # Read config
        dwc_rest_api = config.getboolean('dwc_rest_api', True)
        dwc2 = config.getboolean('dwc2', True)
        htmlroot = config.get('htmlroot',
                              ["DuetWebControl", "DuetWebControl2"][dwc2])
        dwc_htm = 'reprap.htm'
        if not os.path.isabs(htmlroot):
            current_folder = os.path.normpath(os.path.join(
                os.path.dirname(__file__)))
            htmlroot = os.path.join(current_folder, htmlroot)
        if not os.path.exists(os.path.join(htmlroot, 'reprap.htm')): # DWC1
            if os.path.exists(os.path.join(htmlroot, 'index.html')): # DWC2
                dwc_htm = 'index.html'
            else:
                raise printer.config_error(
                    "DuetWebControl files not found '%s'" % htmlroot)
        self.dwc2 = dwc_htm == 'index.html'
        self.logger.debug("html root: %s" % (htmlroot,))
        self.user = config.get('user', '')
        self.passwd = config.get('password', '')
        # ------------------------------
        # - M80 / M81 ATX commands
        self.atx_state = self.atx_off = None
        self.atx_on = config.get('atx_cmd_on', default=None)
        if self.atx_on:
            self.atx_off = config.get('atx_cmd_off')
            self.atx_state = False
        # ------------------------------
        # Create paths to virtual SD
        def create_dir(_dir):
            try:
                os.makedirs(_dir)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    raise Exception("Cannot create directory: '%s'" % (_dir,))
        sd = printer.try_load_module(config, "virtual_sdcard")
        self.sd_path = sdcard_dirname = sd.sdcard_dirname
        create_dir(os.path.join(sdcard_dirname, "gcodes"))
        create_dir(os.path.join(sdcard_dirname, "macros"))
        create_dir(os.path.join(sdcard_dirname, "filaments"))
        create_dir(os.path.join(sdcard_dirname, "sys"))
        # ------------------------------
        # try to load required modules
        printer.try_load_module(config, "babysteps")
        printer.try_load_module(config, "analyse_gcode", folder="modules")
        self.gui_stats = printer.try_load_module(config, 'gui_stats',
                                                 folder='gcodes')
        # ------------------------------
        fd_r, self.pipe_write = os.pipe() # Change to PTY ?
        self.gcode.register_fd(fd_r)
        self.gcode.write_resp = self.gcode_resp_handler
        # Disable auto temperature reporting
        self.printer_write("AUTO_TEMP_REPORT AUTO=0", resp=False)
        # ------------------------------
        # Start tornado webserver
        #if not tornado.ioloop.IOLoop.initialized():
        if True:
            try:
                with open(os.path.join(sdcard_dirname, ".cookie"), "rb") as _f:
                    cookie_secret = _f.readline().strip()
            except IOError:
                cookie_secret = base64.b64encode(
                    uuid.uuid4().bytes + uuid.uuid4().bytes)
                with open(os.path.join(sdcard_dirname, ".cookie"), "w+") as _f:
                    _f.write(cookie_secret)
            log_file = printer.get_start_arg('logfile', "/tmp/klippy.log")
            cfg_file = printer.get_start_arg('config_file')
            app_urls = []
            if self.dwc2 and dwc_rest_api:
                # DWC REST API support
                app_urls.extend([
                    (r"/machine/file/(.*"+KLIPPER_CFG_NAME+")", ConfigFileHandler,
                        {"path": os.path.dirname(cfg_file),
                            "default_filename": os.path.basename(cfg_file)}),
                    (r"/machine/file/(.*"+KLIPPER_LOG_NAME+")", LogFileHandler,
                        {"path": os.path.dirname(log_file),
                            "default_filename": os.path.basename(log_file)}),
                    (r"/machine/file/.*(heightmap.csv)", HeightmapFileHandler),
                    (r"/machine/file/(.*)", FileCommandHandler,
                        {"path": sdcard_dirname}),
                    (r"/machine/code", GcodeCommandHandler),
                    (r"/machine/fileinfo/(.*)", FileInfoHandler),
                    (r"/machine/directory/(.*)", GetDirectoryHandler),
                    (r"/machine", WebSocketHandler),
                ])
            else:
                # DWC polling API support and static file serving
                app_urls.append(
                    (r"/(rr_.*)", rrHandler, {"sd_path": sdcard_dirname}),)
            # Generic file serving
            app_urls.extend([
                tornado.web.url(r"/(.*\.ico)", tornado.web.StaticFileHandler,
                                {"path": htmlroot}),
                tornado.web.url(r"/(.*\.xml)", tornado.web.StaticFileHandler,
                                {"path": htmlroot}),
                tornado.web.url(r"/fonts/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "fonts")}),
                tornado.web.url(r"/js/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "js")}),
                tornado.web.url(r"/css/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "css")}),
                tornado.web.url(r'/login', LoginHandler, name="login"),
                tornado.web.url(r'/logout', LogoutHandler, name="logout"),
                tornado.web.url(r"/(.*)", MainHandler,
                                {"file": os.path.join(htmlroot, dwc_htm)},
                                name="main"),
            ])
            application = tornado.web.Application(app_urls,
                cookie_secret=cookie_secret,
                log_function=self.__Tornado_LoggerCb,
                login_url = "/login",
                xsrf_cookies = False)
            # Put tornado to background thread
            thread = threading.Thread(target=self.__Tornado_execute,
                                      args=(config, application))
            thread.daemon = True
            thread.start()
    def __del__(self):
        self._shutdown()
    def __Tornado_LoggerCb(self, req):
        values  = [req.request.remote_ip, req.request.method, req.request.uri]
        self.logger_tornado.debug(" ".join(values))
    def __Tornado_execute(self, config, application):
        port = config.getint('http', default=80)
        ssl_options = None
        cert = config.get('cert', None)
        key = config.get('key', None)
        if cert is not None and key is not None:
            cert = os.path.normpath(os.path.expanduser(cert))
            key = os.path.normpath(os.path.expanduser(key))
            self.logger.debug("cert: %s", cert)
            self.logger.debug("key: %s", key)
            if os.path.exists(cert) and os.path.exists(key):
                https_port = config.getint('https', None)
                if https_port is not None:
                    port = https_port
                ssl_options = {"certfile": cert, "keyfile": key}
            else:
                self.logger.warning("cert or key file is missing!")
        self.logger.info("Server port %s (SSL %s)" % (
            port, ssl_options is not None))

        http_server = tornado.httpserver.HTTPServer(
            application, ssl_options=ssl_options,
            max_buffer_size=MAX_STREAMED_SIZE)
        http_server.listen(port)
        ioloop = tornado.ioloop.IOLoop.current()
        ioloop.klipper_http_server = http_server
        try:
            ioloop.start()
            self.logger.warning("server stopped!")
        except RuntimeError:
            pass
    def _config_ready(self):
        if self.dwc2:
            self.reactor.register_timer(self._status_update, self.reactor.NOW)
    def _shutdown(self):
        with self.lock:
            self.pipe_write = None
        ioloop = tornado.ioloop.IOLoop.current()
        http_server = getattr(ioloop, "klipper_http_server", None)
        if http_server is not None:
            http_server.stop()
            ioloop.klipper_http_server = None
    def _status_update(self, eventtime): # DWC2 only
        global connections
        _clients = connections
        if not len(_clients):
            return eventtime + .5
        status = dict(_PARENT.gui_stats.get_status_new())
        resps = self.get_gcode_async_resps()
        if resps:
            status['state']['gcoresp'] = "\n".join(resps)
            status['state']['displayNotification'] = {
                "type": "info",
                "title": "GCode",
                "message": "\n".join(resps),
                "timeout": 2000,
            }
        # update atx status
        if self.atx_on is not None:
            status["state"]["atxPower"] = int(self.atx_state)
        status = json.dumps(status, separators=(',', ':'))
        #ioloop = tornado.ioloop.IOLoop.instance() # current()
        for client in _clients:
            client.send_status_update(status)
            #ioloop.add_callback(
            #    functools.partial(client.send_status_update, status))
        return eventtime + .5

    def __gcode_parse(self, gcodes):
        gcodes = gcodes.strip().replace("0:/", "") # clean gcode request
        gcodes = [gcode.strip() for gcode in gcodes.split("\n")]
        out = []
        for gcode in gcodes:
            if gcode.startswith(";"):
                # ignore comments
                continue
            if "M80" in gcode and self.atx_on is not None:
                # ATX ON
                resp = "ATX ON: %s" % os.popen(self.atx_on).read()
                self.append_gcode_resp(resp)
                self.atx_state = True
            elif "M81" in gcode and self.atx_off is not None:
                # ATX OFF
                resp = "ATX OFF: %s" % os.popen(self.atx_off).read()
                self.append_gcode_resp(resp)
                self.atx_state = False
            elif "T-1" in gcode or "M292" in gcode or \
                    "M120" in gcode or "M121" in gcode:
                # ignore
                # T-1 : Active -> Standby
                pass
            else:
                out.append(gcode)
        return "\n".join(out)
    def printer_write(self, cmd, resp=True):
        cmd = self.__gcode_parse(cmd)
        if not cmd:
            with self.lock:
                self.store_resp = resp
                self.resp_rcvd = False
                self.resp_cnt = 1
            self.gcode_resp_handler("ok")
            return
        self.logger.debug("gcode send: %s" % (repr(cmd),))
        with self.lock:
            self.store_resp = resp
            self.resp_rcvd = False
            self.resp_cnt = cmd.count("\n") + 1
            if self.pipe_write is not None:
                os.write(self.pipe_write, "%s\n" % cmd)
    def send_gcode_from_handler_async(self, gcode):
        self.printer_write(gcode)
    def send_gcode_from_handler_sync(self, gcode):
        self.printer_write(gcode)
        while not self.resp_rcvd:
            time.sleep(0.1)
        return "\n".join(self.get_gcode_resps())
    def gcode_resp_handler(self, msg):
        resp = self.resp + msg
        if "ok" not in resp:
            # wait until whole resp is received
            self.resp = resp
            return
        self.resp = ""
        always_store = ("Error:" in resp or "Warning:" in resp)
        if "Klipper state" in resp:
            self.append_gcode_async_resp(resp)
        elif always_store or self.store_resp:
            if self.resp_cnt:
                self.resp_cnt -= 1
                self.resp_rcvd = self.resp_cnt <= 0
                if len(resp) > 3:
                    resp = resp.replace("ok", "")
                resp = resp.strip()
                self.append_gcode_resp(resp)
            else:
                resp = resp.replace("ok", "")
                self.append_gcode_async_resp(resp)
    def append_gcode_resp(self, msg):
        if not msg:
            return
        with self.lock_resps:
            if msg not in self.gcode_resps:
                self.logger.debug("gcode resps: %s" % (repr(msg),))
                self.gcode_resps.append(msg)
    def get_gcode_resps(self):
        with self.lock_resps:
            out = self.gcode_resps
            self.gcode_resps = []
        return out
    def append_gcode_async_resp(self, msg):
        if not msg:
            return
        with self.lock_resps:
            if msg not in self.gcode_resps_async:
                self.logger.debug("gcode resp (async): %s" % repr(msg))
                self.gcode_resps_async.append(msg)
    def get_gcode_async_resps(self):
        with self.lock_resps:
            out = self.gcode_resps_async
            self.gcode_resps_async = []
        return out

    # ============================
    def get_filelist(self, path, respdata, lst_dirs=True):
        path = path.replace("0:", self.sd_path)
        respdata["files"] = []
        respdata['next'] = 0
        respdata["err"] = error = int(not os.path.exists(path))
        if not error:
            for _local in os.listdir(path):
                if _local.startswith("."):
                    continue
                filepath = os.path.join(path, _local)
                if os.path.isfile(filepath):
                    data = {
                        "type": "f",
                        "name": os.path.relpath(filepath, path),
                        "size": os.path.getsize(filepath),
                        "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                              time.localtime(os.path.getmtime(filepath))),
                    }
                    respdata["files"].append(data)
                elif os.path.isdir(filepath) and lst_dirs:
                    data = {
                        "type": "d",
                        "name": os.path.relpath(filepath, path),
                        "size": os.path.getsize(filepath),
                        "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                              time.localtime(os.path.getmtime(filepath))),
                    }
                    respdata["files"].append(data)

            # Add printer.cfg into sys list
            if "/sys" in path:
                cfg_file = os.path.abspath(
                    self.printer.get_start_arg('config_file'))
                respdata["files"].append({
                    "type": "f",
                    "name": KLIPPER_CFG_NAME,
                    "size": os.path.getsize(cfg_file),
                    "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                          time.localtime(os.path.getmtime(cfg_file))),
                })
                logfile = self.printer.get_start_arg('logfile', None)
                if logfile is not None:
                    respdata["files"].append({
                        "type": "f",
                        "name": KLIPPER_LOG_NAME,
                        "size": os.path.getsize(logfile),
                        "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                              time.localtime(os.path.getmtime(logfile))),
                    })

    def get_file_info(self, path, respdata):
        path = path.replace("0:", self.sd_path)
        if path is None or not os.path.exists(path):
            respdata["err"] = 1
        else:
            handler = self.printer.lookup_object("analyse_gcode")
            info = handler.get_file_info(path)
            respdata["err"] = 0
            respdata["size"] = info.get("size", os.path.getsize(path))
            respdata["lastModified"] = \
                time.strftime("%Y-%m-%dT%H:%M:%S",
                              time.localtime(os.path.getmtime(path)))
            respdata["generatedBy"] = info["slicer"]
            respdata["height"] = info["height"]
            respdata["firstLayerHeight"] = info["firstLayerHeight"]
            respdata["layerHeight"] = info["layerHeight"]
            respdata["filament"] = info["filament"]
            respdata["printDuration"] = info['buildTime']
            respdata["fileName"] = os.path.relpath(path, self.sd_path)


def load_config(config):
    if not config.has_section('reprapgui_process'):
        return RepRapGuiModule(config)
    return None
