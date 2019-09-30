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

'''
Status info:
? 'C'    // Reading the configuration file - init type
? 'F'    // Flashing a new firmware binary - IGNORE
? 'H'    // Halted
? 'D'    // Pausing / Decelerating - IGNORE?
? 'R'    // Resuming
? 'T'    // Changing tool - IGNORE?
? 'S'    // Paused / Stopped
? 'P'    // Printing
? 'B'    // Busy
: 'I'    // Idle
'''

"""
Usage
  G10 Pnnn Xnnn Ynnn Znnn
Parameters
  Pnnn Tool number - SKIP
  Xnnn X offset - SKIP
  Ynnn Y offset - SKIP
  U,V,Wnnn U, V and W axis offsets - SKIP
  Znnn Z offset - SKIP
"""

import time, sys, os, errno, threading, json, logging
import analyse_gcode

try:
    sys.path.append(os.path.normpath(
        os.path.expanduser(os.environ['TORNADO_PATH'])))
except KeyError:
    pass
import tornado.ioloop
import tornado.web

_PARENT = None
KLIPPER_CFG_NAME = 'klipper_config.cfg'
KLIPPER_LOG_NAME = "klippy.log"

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
    def initialize(self, path):
        self.path = path
    @tornado.web.authenticated
    def get(self):
        self.render(os.path.join(self.path, "reprap.htm"))

class LoginHandler(BaseHandler):
    path = parent = None
    def initialize(self, path):
        self.path = path
        self.parent = _PARENT
    @tornado.gen.coroutine
    def get(self):
        incorrect = self.get_secure_cookie("incorrect")
        if incorrect and int(incorrect) > 20:
            self.write('<center>blocked</center>')
            return
        # Skip login if user or passwd is not set
        if not len(self.parent.user) or not len(self.parent.passwd):
            self.set_secure_cookie("user", "John Doe")
            self.set_secure_cookie("incorrect", "0")
            self.redirect(self.reverse_url("main"))
            return
        self.render(os.path.join(self.path, 'login.html'))

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
            self.redirect(self.reverse_url("main"))
        else:
            incorrect = self.get_secure_cookie("incorrect") or 0
            increased = str(int(incorrect)+1)
            self.set_secure_cookie("incorrect", increased)
            self.write("""<center>
                            Something Wrong With Your Data (%s)<br />
                            <a href="/">Go Home</a>
                          </center>""" % increased)

class LogoutHandler(BaseHandler):
    def get(self):
        self.clear_cookie("user")
        self.redirect(self.get_argument("next", self.reverse_url("main")))

class JpegHandler(tornado.web.RequestHandler):
    camera = logger = None
    def initialize(self, camera):
        self.camera = camera
    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        if self.camera is None:
            self.write("No camera available")
            return
        self.set_header(
            'Cache-Control',
            'no-store, no-cache, must-revalidate, '
            'pre-check=0, post-check=0, max-age=0')
        self.set_header('Connection', 'close')
        self.set_header('Content-Type',
                        'multipart/x-mixed-replace;boundary=--boundarydonotcross')
        self.set_header('Expires', 'Mon, 3 Jan 2000 12:34:56 GMT')
        self.set_header('Pragma', 'no-cache')

        img = self.camera.get_frame()
        self.write("--boundarydonotcross\n")
        self.write("Content-type: image/jpeg\r\n")
        self.write("Content-length: %s\r\n\r\n" % len(img))
        self.write(str(img))


class JpegStreamHandler(tornado.web.RequestHandler):
    camera = interval = logger = None
    def initialize(self, camera, interval):
        self.camera = camera
        self.interval = interval
    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        if self.camera is None:
            self.write("No camera available")
            return
        self.set_header(
            'Cache-Control',
            'no-store, no-cache, must-revalidate, '
            'pre-check=0, post-check=0, max-age=0')
        self.set_header('Connection', 'close')
        self.set_header('Content-Type',
                        'multipart/x-mixed-replace;boundary=--boundarydonotcross')
        self.set_header('Expires', 'Mon, 3 Jan 2000 12:34:56 GMT')
        self.set_header('Pragma', 'no-cache')

        ioloop = tornado.ioloop.IOLoop.current()
        served_image_timestamp = time.time()
        my_boundary = "--boundarydonotcross\n"
        while True:
            img = self.camera.get_frame()
            if served_image_timestamp + self.interval < time.time():
                self.write(my_boundary)
                self.write("Content-type: image/jpeg\r\n")
                self.write("Content-length: %s\r\n\r\n" % len(img))
                self.write(str(img))
                served_image_timestamp = time.time()
                yield tornado.gen.Task(self.flush)
                if not self.interval:
                    break
            else:
                yield tornado.gen.Task(ioloop.add_timeout,
                                       ioloop.time() + self.interval)

class rrHandler(tornado.web.RequestHandler):
    parent = printer = sd_path = logger = None

    def initialize(self, sd_path):
        self.parent = _PARENT
        self.printer = self.parent.printer
        self.sd_path = sd_path
        self.logger = self.parent.logger

    def get(self, path, *args, **kwargs):
        sd_path = self.sd_path
        respdata = {"err" : 10}

        # rr_connect?password=XXX&time=YYY
        if "rr_connect" in path:
            respdata["err"] = 0
            #_passwd = self.get_argument('password')
            #if self.parent.passwd != _passwd:
            #    respdata["err"] = 1
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
                respdata['params']['atxPower'] = int(self.parent.atx_state)
            respdata['seq'] += len(self.parent.gcode_resps)

        # rr_gcode?gcode=XXX
        elif "rr_gcode" in path:
            respdata["err"] = 0
            respdata["buff"] = 99999

            gcode = self.get_argument('gcode')
            #self.logger.debug("rr_gcode={}".format(gcode))
            # Clean up gcode command
            gcode = gcode.replace("0:/", "").replace("0%3A%2F", "")

            if "M80" in gcode and self.parent.atx_on is not None:
                # ATX ON
                resp = os.popen(self.parent.atx_on).read()
                self.parent.append_gcode_resp(resp)
                self.logger.info("ATX ON: %s" % resp)
                self.parent.atx_state = True
            elif "M81" in gcode and self.parent.atx_off is not None:
                # ATX OFF
                resp = os.popen(self.parent.atx_off).read()
                self.parent.append_gcode_resp(resp)
                self.logger.info("ATX OFF: %s" % resp)
                self.parent.atx_state = False
            elif "T-1" in gcode:
                # ignore
                pass
            else:
                try:
                    self.parent.printer_write(gcode)
                except self.parent.gcode.error:
                    respdata["err"] = 1

        # rr_download?name=XXX
        elif "rr_download" in path:
            # Download a specified file from the SD card.
            path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            if KLIPPER_CFG_NAME in path:
                path = os.path.abspath(
                    self.printer.get_start_arg('config_file'))
            elif KLIPPER_LOG_NAME in path:
                path = os.path.abspath(
                    self.printer.get_start_arg('logfile'))
            elif "heightmap.csv" in path:
                bed_mesh = self.printer.lookup_object('bed_mesh', None)
                calibrate = getattr(bed_mesh, "calibrate", None)
                if calibrate:
                    # calibrate.print_probed_positions_to_csv()
                    self.set_header('Content-Type',
                                    'application/force-download')
                    self.set_header('Content-Disposition',
                                    'attachment; filename=heightmap.csv')
                    self.write(calibrate.print_probed_positions_to_csv())
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
                self.set_header('Content-Disposition', 'attachment; filename=%s' % os.path.basename(path))
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
            directory = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            if KLIPPER_CFG_NAME in directory or KLIPPER_LOG_NAME in directory:
                pass
            else:
                directory = os.path.abspath(os.path.join(sd_path, directory))
                #self.logger.debug("delete: absolute path {}".format(directory))
                try:
                    for root, dirs, files in os.walk(directory, topdown=False):
                        for name in files:
                            os.remove(os.path.join(root, name))
                        for name in dirs:
                            os.rmdir(os.path.join(root, name))
                    if os.path.isdir(directory):
                        os.rmdir(directory)
                    else:
                        os.remove(directory)
                except OSError as e:
                    self.logger.error("rr_delete: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_filelist?dir=XXX
        elif "rr_filelist" in path:
            '''
            resp: `{"type":[type],"name":"[name]","size":[size],"lastModified":"[datetime]"}`
            resp error: `{"err":[code]}`
                where code is
                    1 = the directory doesn't exist
                    2 = the requested volume is not mounted
            '''
            _dir = self.get_argument('dir')
            respdata["dir"]   = _dir
            respdata["files"] = []

            _dir = _dir.replace("0:/", "").replace("0%3A%2F", "")
            path = os.path.abspath(os.path.join(sd_path, _dir))

            if not os.path.exists(path):
                respdata["err"] = 1
            else:
                respdata["err"] = 0
                del respdata["err"] # err keyword need to be deleted
                for _local in os.listdir(path):
                    if _local.startswith("."):
                        continue
                    filepath = os.path.join(path, _local)
                    if os.path.isfile(filepath):
                        data = {
                            "type" : "f",
                            "name" : os.path.relpath(filepath, path),
                            "size" : os.path.getsize(filepath),
                            "date" : time.strftime("%Y-%m-%dT%H:%M:%S",
                                                   time.localtime(os.path.getmtime(filepath))),
                        }
                        respdata["files"].append(data)
                    elif os.path.isdir(filepath):
                        data = {
                            "type" : "d",
                            "name" : os.path.relpath(filepath, path),
                            "size" : os.path.getsize(filepath),
                            "date" : time.strftime("%Y-%m-%dT%H:%M:%S",
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

        # rr_fileinfo?name=XXX
        elif "rr_fileinfo" in path:
            name = self.get_argument('name', default=None)
            #self.logger.debug("rr_fileinfo: {} , name: {}".format(self.request.uri, name))
            if name is None:
                sd = self.printer.lookup_object('virtual_sdcard')
                try:
                    # current file printed
                    if sd.current_file is not None:
                        path = sd.current_file.name
                    else:
                        raise AttributeError
                except AttributeError:
                    path = None
            else:
                path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
                path = os.path.abspath(os.path.join(sd_path, path))
            # info about the requested file
            if path is None or not os.path.exists(path):
                respdata["err"] = 1
            else:
                info = analyse_gcode.analyse_gcode_file(path)
                respdata["err"] = 0
                respdata["size"] = os.path.getsize(path)
                respdata["lastModified"] = \
                    time.strftime("%Y-%m-%dT%H:%M:%S",
                                  time.localtime(os.path.getmtime(path)))
                respdata["generatedBy"]      = info["slicer"]
                respdata["height"]           = info["height"]
                respdata["firstLayerHeight"] = info["firstLayerHeight"]
                respdata["layerHeight"]      = info["layerHeight"]
                respdata["filament"]         = info["filament"]
                respdata["printDuration"]    = info['buildTime']
                respdata["fileName"] = os.path.relpath(path, sd_path) # os.path.basename ?

        # rr_move?old=XXX&new=YYY
        elif "rr_move" in path:
            # {"err":[code]} , code 0 if success
            respdata["err"] = 0
            _from = self.get_argument('old').replace("0:/", "").replace("0%3A%2F", "")
            if KLIPPER_CFG_NAME in _from or KLIPPER_LOG_NAME in _from:
                pass
            else:
                _from = os.path.abspath(os.path.join(sd_path, _from))
                _to   = self.get_argument('new').replace("0:/", "").replace("0%3A%2F", "")
                _to   = os.path.abspath(os.path.join(sd_path, _to))
                try:
                    os.rename(_from, _to)
                except OSError as e:
                    self.logger.error("rr_move: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_mkdir?dir=XXX
        elif "rr_mkdir" in path:
            # {"err":[code]} , 0 if success
            respdata["err"] = 0
            directory = self.get_argument('dir').replace("0:/", "").replace("0%3A%2F", "")
            directory = os.path.abspath(os.path.join(sd_path, directory))
            try:
                os.makedirs(directory)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    self.logger.error("rr_mkdir: %s" % (e.strerror,))
                    respdata["err"] = 1

        # rr_config / rr_configfile
        elif "rr_configfile" in path:
            respdata = { "err" : 0 }

        elif "rr_config" in path:
            respdata = self.parent.gui_stats.get_config_stats()

        elif "rr_reply" in path:
            try:
                self.write(self.parent.gcode_resps.pop(0))
            except IndexError:
                self.write("")
            return

        else:
            self.logger.error("  get(path={})".format(path))
            self.logger.error("     !! uri: {}".format(self.request.uri))

        # Send response back to client
        respstr = json.dumps(respdata)
        self.write(respstr)

    def post(self, path, *args, **kwargs):
        respdata = { "err" : 1 }
        if "rr_upload" in self.request.path:
            # /rr_upload?name=xxxxx&time=xxxx
            # /rr_upload?name=0:/filaments/PLA/unload.g&time=2017-11-30T11:46:50

            size = int(self.request.headers['Content-Length'])
            body_len = len(self.request.body)
            if body_len == 0:
                # e.g. filament create
                respdata["err"] = 0
            elif body_len != size or not size:
                self.logger.error("upload size error: %s != %s" % (body_len, size))
            else:
                target_path = self.get_argument('name').replace("0:/", ""). \
                    replace("0%3A%2F", "")
                if KLIPPER_CFG_NAME in target_path:
                    cfgname = os.path.abspath(
                        self.printer.get_start_arg('config_file'))
                    datestr = time.strftime("-%Y%m%d_%H%M%S")
                    backup_name = cfgname + datestr
                    temp_name = cfgname + "_autosave"
                    if cfgname.endswith(".cfg"):
                        backup_name = cfgname[:-4] + datestr + ".cfg"
                        temp_name = cfgname[:-4] + "_autosave.cfg"
                    try:
                        f = open(temp_name, 'wb')
                        f.write(self.request.body)
                        f.close()
                        os.rename(cfgname, backup_name)
                        os.rename(temp_name, cfgname)
                        respdata['err'] = 0
                    except IOError as err:
                        self.logger.error("Upload, cfg: %s" % err)
                elif KLIPPER_LOG_NAME in target_path:
                    respdata['err'] = 0
                elif "heightmap.csv" in target_path:
                    # skip just in case
                    respdata['err'] = 0
                else:
                    target_path = os.path.abspath(os.path.join(self.sd_path, target_path))
                    # Create a dir first
                    try:
                        os.makedirs(os.path.dirname(target_path))
                    except OSError:
                        pass
                    # Try to save content
                    try:
                        with open(target_path, 'w') as output_file:
                            output_file.write(self.request.body)
                            respdata['err'] = 0
                    except IOError as err:
                        self.logger.error("Upload, g-code: %s" % err)
        else:
            self.logger.error("Unknown req path: %s" % self.request.path)
        # Send response back to client
        self.write(json.dumps(respdata))


def create_dir(_dir):
    try:
        os.makedirs(_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise Exception("cannot create directory {}".format(_dir))


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
        self.gui_stats = printer.lookup_object('gui_stats')
        self.gcode_resps = []
        self.lock = threading.Lock()
        # Read config
        htmlroot = os.path.normpath(os.path.join(os.path.dirname(__file__)))
        htmlroot = os.path.join(htmlroot, "DuetWebControl")
        if not os.path.exists(os.path.join(htmlroot, 'reprap.htm')):
            raise printer.config_error("DuetWebControl files not found '%s'" % htmlroot)
        self.logger.debug("html root: %s" % (htmlroot,))
        self.user = config.get('user', '')
        self.passwd = config.get('password', '')
        # Camera information
        self.feed_interval = config.getfloat('feedrate', minval=.0, default=.1)
        self.camera = printer.try_load_module(
            config, "videocam", folder="modules")
        # - M80 / M81 ATX commands
        self.atx_state = self.atx_off = None
        self.atx_on = config.get('atx_cmd_on', default=None)
        if self.atx_on:
            self.atx_off = config.get('atx_cmd_off')
            self.atx_state = False
        # ------------------------------
        # Create paths to virtual SD
        sd = printer.try_load_module(config, "virtual_sdcard")
        sdcard_dirname = sd.sdcard_dirname
        create_dir(os.path.join(sdcard_dirname, "gcodes"))
        create_dir(os.path.join(sdcard_dirname, "macros"))
        create_dir(os.path.join(sdcard_dirname, "filaments"))
        create_dir(os.path.join(sdcard_dirname, "sys"))
        # ------------------------------
        # try to load required modules
        printer.try_load_module(config, "babysteps")
        # ------------------------------
        # Start tornado webserver
        if _TORNADO_THREAD is None or not _TORNADO_THREAD.isAlive():
            application = tornado.web.Application(
                [
                    tornado.web.url(r"/", MainHandler,
                                    {"path": htmlroot}, name="main"),
                    tornado.web.url(r'/login', LoginHandler,
                                    {"path": htmlroot}, name="login"),
                    tornado.web.url(r'/logout', LogoutHandler, name="logout"),
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
                    tornado.web.url(r"/(rr_.*)", rrHandler, {"sd_path": sdcard_dirname}),
                    tornado.web.url(r"/jpeg", JpegHandler, {"camera": self.camera}),
                    tornado.web.url(r"/video", JpegStreamHandler,
                                    { "camera": self.camera,
                                      "interval": self.feed_interval}),
                ],
                cookie_secret="16d35553-3331-4569-b419-8748d22aa599",
                log_function=self.Tornado_LoggerCb,
                max_buffer_size=104857600*20,
                login_url = "/login",
                xsrf_cookies = False)

            # Put tornado to background thread
            _TORNADO_THREAD = threading.Thread(
                target=self.Tornado_execute, args=(config, application))
            _TORNADO_THREAD.daemon = True
            _TORNADO_THREAD.start()

        # ------------------------------
        fd_r, self.pipe_write = os.pipe() # Change to PTY ?
        self.gcode.register_fd(fd_r)

        self.gcode.write_resp = self.gcode_resp_handler
        # Disable auto temperature reporting
        self.printer_write_no_update("AUTO_TEMP_REPORT AUTO=0")
        # ------------------------------
        self.logger.info("RepRep Web GUI loaded")

    def Tornado_LoggerCb(self, req):
        values  = [req.request.remote_ip, req.request.method, req.request.uri]
        self.logger_tornado.debug(" ".join(values))

    def Tornado_execute(self, config, application):
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
                ssl_options = {
                    "certfile": cert,
                    "keyfile": key,
                }
            else:
                self.logger.warning("cert or key file is missing!")
        self.logger.info("Server port %s (SSL %s)" % (
            port, ssl_options is not None))

        http_server = tornado.httpserver.HTTPServer(application,
                                                    ssl_options=ssl_options)
        http_server.listen(port)
        tornado.ioloop.IOLoop.current().start()
        self.logger.warning("Something went wrong, server exited!")

    resp = ""
    resp_rcvd = False
    store_resp = False
    def _write(self, cmd):
        # self.logger.debug("GCode send: %s" % (cmd,))
        with self.lock:
            self.resp_rcvd = False
            self.resp = ""
            os.write(self.pipe_write, "%s\n" % cmd)
    def printer_write_no_update(self, cmd):
        self.store_resp = False
        self._write(cmd)
    def printer_write(self, cmd):
        self.store_resp = True
        self._write(cmd)
    def gcode_resp_handler(self, msg):
        self.resp += msg
        if "ok" not in self.resp:
            return
        resp = self.resp
        self.logger.debug("GCode resps: %s" % (repr(resp),))
        if "Klipper state" in resp:
            self.append_gcode_resp(resp)
        elif not self.resp_rcvd or "Error:" in resp or "Warning:" in resp:
            resp = resp.strip()
            if len(resp) > 2:
                resp = resp.replace("ok", "")
            if self.store_resp or "Error:" in resp or "Warning:" in resp:
                self.append_gcode_resp(resp)
            # self.resp_rcvd = True
        self.resp = ""
    def append_gcode_resp(self, msg):
        self.gcode_resps.append(msg)


def load_config(config):
    return RepRapGuiModule(config)
