#
# This module is handling request from DuetWebControl (http://reprap.org/wiki/Duet_Web_Control)
#     Tornado webserver is needed to run page
#       Note: Tornado version 4.5 is required!
#         - install Tornado using pip ( $pip install tornado==4.5 )
#         - or download from https://github.com/tornadoweb/tornado/tree/branch4.5
#           and use environment variable 'export TORNADO_PATH=<path to tornado folder>'
#     Install opencv for video streaming
#         - $sudo apt-get install python-opencv
#
'''
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
'''

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

import time, sys, os, errno, threading, json, re, logging
import extruder, util

try:
    sys.path.append(os.path.normpath(
        os.path.expanduser(os.environ['TORNADO_PATH'])))
except KeyError:
    pass
import tornado.ioloop
import tornado.web

_PARENT = None

def dict_dump_json(data, logger):
    logger.info(json.dumps(data,
                           #sort_keys=True,
                           sort_keys=False,
                           indent=4,
                           separators=(',', ': ')))


try:
    import cv2
    class VideoCamera(object):
        def __init__(self, index=0):
            self.video = cv2.VideoCapture(index)
        def __del__(self):
            self.video.release()
        def get_frame(self):
            success, image = self.video.read()
            if image is not None:
                ret, jpeg = cv2.imencode('.jpg', image)
                return jpeg.tostring()
            return ""
except ImportError:
    class VideoCamera(object):
        def __init__(self, index=0):
            pass
        def __del__(self):
            pass
        def get_frame(self):
            return ""


class MJPEGHandler(tornado.web.RequestHandler):
    def initialize(self, camera, interval, logger):
        self.camera = camera
        self.interval = interval if 0 <= interval else 0
        self.logger = logger
    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        #self.logger.info("MJPEGHandler get() args: %s boby: %s" % (self.request.arguments, self.request.body));
        #self.logger.info("  header: %s" % (self.request.headers))
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, pre-check=0, post-check=0, max-age=0')
        self.set_header('Connection', 'close')
        self.set_header('Content-Type', 'multipart/x-mixed-replace;boundary=--boundarydonotcross')
        self.set_header('Expires', 'Mon, 3 Jan 2000 12:34:56 GMT')
        self.set_header('Pragma', 'no-cache')

        if self.interval == 0:
            img = self.camera.get_frame()
            self.write("--boundarydonotcross\n")
            self.write("Content-type: image/jpeg\r\n")
            self.write("Content-length: %s\r\n\r\n" % len(img))
            self.write(str(img))
            return

        ioloop = tornado.ioloop.IOLoop.current()
        self.served_image_timestamp = time.time()
        my_boundary = "--boundarydonotcross\n"
        while True:
            img = self.camera.get_frame()
            if self.served_image_timestamp + self.interval < time.time():
                self.write(my_boundary)
                self.write("Content-type: image/jpeg\r\n")
                self.write("Content-length: %s\r\n\r\n" % len(img))
                self.write(str(img))
                self.served_image_timestamp = time.time()
                yield tornado.gen.Task(self.flush)
            else:
                yield tornado.gen.Task(ioloop.add_timeout, ioloop.time() + self.interval)

class BaseHandler(tornado.web.RequestHandler):
    def get_current_user(self):
        return self.get_secure_cookie("user", max_age_days=5)

class MainHandler(BaseHandler):
    def initialize(self, path):
        self.path = path
    @tornado.web.authenticated
    def get(self):
        self.render(os.path.join(self.path, "reprap.htm"))

class LoginHandler(BaseHandler):
    def initialize(self, path, parent):
        self.path = path
        self.parent = _PARENT #parent
    @tornado.gen.coroutine
    def get(self):
        incorrect = self.get_secure_cookie("incorrect")
        if incorrect and int(incorrect) > 20:
            self.write('<center>blocked</center>')
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

class rrHandler(tornado.web.RequestHandler):
    def initialize(self, parent):
        self.parent = _PARENT #parent
        self.sd_path = parent.sd.sdcard_dirname
        self.logger = parent.logger

    def get(self, path, *args, **kwargs):
        respdata = {
            "err" : 10
        }

        #### rr_connect?password=XXX&time=YYY
        if "rr_connect" in path:
            respdata["err"] = 0
            _passwd = self.get_argument('password')
            #if self.parent.passwd != _passwd:
            #    respdata["err"] = 1
            # 0 = success, 1 = wrong passwd, 2 = No more HTTP sessions available
            respdata["sessionTimeout"] = 30000 # ms
            # duetwifi10, duetethernet10, radds15, alligator2, duet06, duet07, duet085, default: unknown
            respdata["boardType"] = "unknown" #"radds15"

        #### rr_disconnect
        elif "rr_disconnect" in path:
            respdata["err"] = 0

        #### rr_status?type=XXX
        # http://reprap.org/wiki/RepRap_Firmware_Status_responses
        elif "rr_status" in path:
            _type = int(self.get_argument('type'))
            if (_type < 1 or _type > 3):
                _type = 1
            # get status from Klippy
            respdata["err"] = 0;
            respdata.update(self.parent.web_getstatus(_type))

        #### rr_gcode?gcode=XXX
        elif "rr_gcode" in path:
            respdata["err"] = 0
            respdata["buff"] = 99999

            gcode = self.get_argument('gcode')
            #self.logger.debug("rr_gcode={}".format(gcode))
            # Clean up gcode command
            gcode = gcode.replace("0:/", "").replace("0%3A%2F", "")

            '''
            special commands:
            * Start print: M32 [root]/gcodes/<file>
            * Deselect filament: T-1
            M122 ?

            Add new tool: ??
            M563 P127 S"hhh" D0 H0

            M0 H1 = Cancel print
            '''
            printer_write = self.parent.printer_write

            if "M32" in gcode:
                # M32 <GCodeFile> - Select and start gcode file
                try:
                    self.parent.gcode.simulate_print = False
                    printer_write(gcode.replace("M32 ", "M23 gcodes/"))
                    self.parent.current_file = gcode[4:]
                    printer_write("M24")
                except self.parent.gcode.error as e:
                    respdata["err"] = 1
            elif "T-1" in gcode:
                # Skip...
                pass
            else:
                if "G10" in gcode:
                    # G10 - retract if no params (G10 Pnnn Xnnn Ynnn Znnn Rnnn Snnn)
                    if bool(re.search('P|X|Y|Z|R|S', gcode)):
                        gcode = gcode.replace('G10', 'M1010') # M1000 + cmd nbr for RepRap
                if "M106" in gcode:
                    gcode = gcode.replace('M106', 'M1106')
                try:
                    resp = printer_write(gcode)
                except self.parent.gcode.error as e:
                    respdata["err"] = 1

        #### rr_download?name=XXX
        elif "rr_download" in path:
            # Download a specified file from the SD card.
            path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            path = os.path.abspath(os.path.join(self.sd_path, path))

            #self.logger.debug("GET rr_download: name={} [path: {}]".
            #                  format(self.get_argument('name'),
            #                         path))

            if not os.path.exists(path):
                raise tornado.web.HTTPError(404)
            else:
                self.set_header('Content-Type', 'application/force-download')
                self.set_header('Content-Disposition', 'attachment; filename=%s' % os.path.basename(path))
                with open(path, "rb") as f:
                    try:
                        while True:
                            _buffer = f.read(4096)
                            if _buffer:
                                self.write(_buffer)
                            else:
                                f.close()
                                self.finish()
                            return
                    except:
                        raise tornado.web.HTTPError(404)
                raise tornado.web.HTTPError(500)

        #### rr_delete?name=XXX
        elif "rr_delete" in path:
            # resp: `{"err":[code]}`
            respdata["err"] = 0
            directory = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            directory = os.path.abspath(os.path.join(self.sd_path, directory))
            #self.logger.debug("delete: absolute path {}".format(directory))
            try:
                for root, dirs, files in os.walk(directory, topdown=False):
                    for name in files:
                        os.remove(os.path.join(root, name))
                    for name in dirs:
                        os.rmdir(os.path.join(root, name))
                if (os.path.isdir(directory)):
                    os.rmdir(directory)
                else:
                    os.remove(directory)
            except OSError as e:
                self.logger.error("rr_delete: %s" % (e.strerror,))
                respdata["err"] = 1

        #### rr_filelist?dir=XXX
        elif "rr_filelist" in path:
            '''
            resp: `{"type":[type],"name":"[name]","size":[size],"lastModified":"[datetime]"}`
            resp error: `{"err":[code]}`
                where code is
                    1 = the directory doesn't exist
                    2 = the requested volume is not mounted
            '''
            _dir = self.get_argument('dir').replace("0:/", "").replace("0%3A%2F", "")
            respdata["dir"]   = self.get_argument('dir')
            respdata["files"] = []

            path = os.path.abspath(os.path.join(self.sd_path, _dir))

            if not os.path.exists(path):
                respdata["err"] = 1
            else:
                respdata["err"] = 0
                del respdata["err"]

                for _local in os.listdir(path):
                    if _local.startswith("."):
                        continue

                    filepath = os.path.join(path, _local)

                    if os.path.isfile(filepath):
                        data = {
                            "type" : "f",
                            "name" : os.path.relpath(filepath, path), #os.path.basename(filepath),
                            "size" : os.path.getsize(filepath),
                            "date" : time.strftime("%Y-%m-%dT%H:%M:%S",
                                                   time.gmtime(os.path.getmtime(filepath))),
                        }
                        respdata["files"].append(data)
                    elif os.path.isdir(filepath):
                        data = {
                            "type" : "d",
                            "name" : os.path.relpath(filepath, path), #os.path.basename(filepath),
                            "size" : os.path.getsize(filepath),
                            "date" : time.strftime("%Y-%m-%dT%H:%M:%S",
                                                   time.gmtime(os.path.getmtime(filepath))),
                        }
                        respdata["files"].append(data)
            #self.logger.debug("rr_filelist: {}".format(respdata))

        #### rr_fileinfo?name=XXX
        elif "rr_fileinfo" in path:
            name = self.get_argument('name', default=None)
            #self.logger.debug("rr_fileinfo: {} , name: {}".format(self.request.uri, name))
            path = None
            is_printing = False
            if name is None:
                try:
                    is_printing = (self.parent.sd.current_file is not None and
                                   self.parent.sd.work_timer is not None)
                    # current file printed
                    if self.parent.sd.current_file is not None:
                        path = self.parent.sd.current_file.name
                    else:
                        raise AttributeError
                except AttributeError:
                    path = None
            else:
                path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
                path = os.path.abspath(os.path.join(self.sd_path, path))

            # info about the requested file
            if not os.path.exists(path):
                respdata["err"] = 1
            else:
                respdata["err"] = 0
                respdata["size"] = os.path.getsize(path)
                respdata["lastModified"] = \
                        time.strftime("%Y-%m-%dT%H:%M:%S",
                                      time.gmtime(os.path.getmtime(path)))
                respdata["generatedBy"]      = "unknown"
                respdata["height"]           = "NA"
                respdata["firstLayerHeight"] = "NA"
                respdata["layerHeight"]      = "NA"
                respdata["filament"]         = []

                if is_printing is True:
                    respdata["printDuration"] = 1234
                    respdata["fileName"]      = os.path.relpath(path, self.sd_path)

        #### rr_move?old=XXX&new=YYY
        elif "rr_move" in path:
            # {"err":[code]} , code 0 if success
            respdata["err"] = 0
            _from = self.get_argument('old').replace("0:/", "").replace("0%3A%2F", "")
            _from = os.path.abspath(os.path.join(self.sd_path, _from))
            _to   = self.get_argument('new').replace("0:/", "").replace("0%3A%2F", "")
            _to   = os.path.abspath(os.path.join(self.sd_path, _to))
            try:
                os.rename(_from, _to)
            except OSError as e:
                self.logger.error("rr_move: %s" % (e.strerror,))
                respdata["err"] = 1

        #### rr_mkdir?dir=XXX
        elif "rr_mkdir" in path:
            #self.logger.debug("GET rr_mkdir: dir={}".
            #                  format(self.get_argument('dir')))
            # {"err":[code]} , 0 if success
            respdata["err"] = 0
            directory = self.get_argument('dir').replace("0:/", "").replace("0%3A%2F", "")
            directory = os.path.abspath(os.path.join(self.sd_path, directory))
            try:
                os.makedirs(directory)
            except OSError as e:
                if e.errno != errno.EEXIST:
                    self.logger.error("rr_mkdir: %s" % (e.strerror,))
                    respdata["err"] = 1

        #### rr_config / rr_configfile
        elif "rr_configfile" in path:
            respdata = {
                "err" : 0,
            }
        elif "rr_config" in path:
            respdata = self.parent.web_getconfig()

        elif "rr_reply" in path:
            if len(self.parent.gcode_resps):
                self.write(self.parent.gcode_resps.pop(0))
            #else:
            #    self.write("GCode resps queue is empty")
            return

        else:
            self.logger.error("  get(path={})".format(path))
            self.logger.error("     !! uri: {}".format(self.request.uri))

        # Send response back to client
        respstr = json.dumps(respdata)
        self.write(respstr)
        #self.logger.info(json.dumps(respdata,
        #                        sort_keys=True,
        #                        indent=4, separators=(',', ': ')))

    def post(self, path, *args, **kwargs):
        respdata = {
            "err" : 1
        }

        if "rr_upload" in self.request.path:
            # /rr_upload?name=xxxxx&time=xxxx
            # /rr_upload?name=0:/filaments/PLA/unload.g&time=2017-11-30T11:46:50

            path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            path = os.path.abspath(os.path.join(self.sd_path, path))

            #self.logger.debug("POST rr_upload: name={} time={} [path: {}]".
            #                  format(self.get_argument('name'),
            #                         self.get_argument('time'),
            #                         path))
            try:
                os.makedirs(os.path.dirname(path))
            except OSError as e:
                if e.errno != errno.EEXIST:
                    pass
            try:
                output_file = open(path, 'w')
                if self.request.body:
                    output_file.write(self.request.body)
                output_file.close()

                respdata['err'] = 0
            except IOError:
                pass

        # Send response back to client
        respstr = json.dumps(respdata)
        self.write(respstr)

def create_dir(_dir):
    try:
        os.makedirs(_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise Exception("cannot create directory {}".format(_dir))


_TORNADO_THREAD = None

class RepRapGuiModule(object):
    htmlroot = None
    def __init__(self, printer, config):
        global _TORNADO_THREAD
        global _PARENT
        _PARENT = self
        self.printer = printer
        self.logger = printer.logger.getChild("DuetWebControl")
        self.logger_tornado = self.logger.getChild("tornado")
        self.logger_tornado.setLevel(logging.INFO)
        self.gcode = printer.lookup_object('gcode')
        self.toolhead = printer.lookup_object('toolhead')
        self.babysteps = printer.lookup_object('babysteps')
        printer.try_load_module(config, "virtual_sdcard")
        self.sd = printer.lookup_object('virtual_sdcard')
        self.starttime = time.time()
        self.curr_state = 'C'
        self.gcode_resps = []
        self.current_file = None

        # Read config
        self.name = config.getsection('printer').get(
            'name', default="Klipper printer")
        # Read config
        htmlroot = os.path.normpath(os.path.join(os.path.dirname(__file__)))
        htmlroot = os.path.join(htmlroot, "DuetWebControl")
        if not os.path.exists(os.path.join(htmlroot, 'reprap.htm')):
            raise self.printer.config_error("DuetWebControl files not found '%s'" % htmlroot)
        self.logger.debug("html root: %s" % (htmlroot,))
        http_port = config.getint('http', default=80)
        https_port = config.getint('https', default=None)
        ssl_options = None
        if https_port is not None:
            ssl_options = {
                "certfile": os.path.normpath(os.path.expanduser(config.get('cert'))),
                "keyfile": os.path.normpath(os.path.expanduser(config.get('key'))),
            }
        self.user = config.get('user')
        self.passwd = config.get('password')
        feed_interval = config.getfloat('feedrate', minval=0., default=0.)
        camera = VideoCamera(config.getint('camera_index', default=0))
        # ------------------------------
        # Create paths to virtual SD
        create_dir(os.path.join(self.sd.sdcard_dirname, "gcodes"))
        create_dir(os.path.join(self.sd.sdcard_dirname, "macros"))
        create_dir(os.path.join(self.sd.sdcard_dirname, "filaments"))
        create_dir(os.path.join(self.sd.sdcard_dirname, "sys"))
        self.sd.register_done_cb(self.sd_print_done)
        # ------------------------------
        # Start tornado webserver
        if _TORNADO_THREAD is None or not _TORNADO_THREAD.isAlive():
            application = tornado.web.Application(
                [
                    tornado.web.url(r"/", MainHandler,
                                    {"path": htmlroot}, name="main"),
                    tornado.web.url(r'/login', LoginHandler,
                                    {"path": htmlroot, "parent": self}, name="login"),
                    tornado.web.url(r'/logout', LogoutHandler, name="logout"),
                    tornado.web.url(r"/(.*)xml",    tornado.web.StaticFileHandler,
                                    {"path": htmlroot}),
                    tornado.web.url(r"/fonts/(.*)", tornado.web.StaticFileHandler,
                                    {"path": os.path.join(htmlroot, "fonts")}),
                    tornado.web.url(r"/js/(.*)",    tornado.web.StaticFileHandler,
                                    {"path": os.path.join(htmlroot, "js")}),
                    tornado.web.url(r"/css/(.*)",   tornado.web.StaticFileHandler,
                                    {"path": os.path.join(htmlroot, "css")}),
                    tornado.web.url(r"/(rr_.*)", rrHandler, {"parent": self}),
                    tornado.web.url(r"/video", MJPEGHandler,
                                    {"camera": camera, "interval": feed_interval, "logger": self.logger}),
                ],
                cookie_secret="16d35553-3331-4569-b419-8748d22aa599",
                log_function=self.Tornado_LoggerCb,
                max_buffer_size=104857600*20,
                login_url = "/login",
                xsrf_cookies = False)

            http_server = tornado.httpserver.HTTPServer(
                application,
                ssl_options=ssl_options)
            if https_port is not None:
                http_server.bind(https_port)
                self.logger.debug("HTTPS port %s" % (https_port))
            else:
                http_server.bind(http_port)
                self.logger.debug("HTTPS port %s" % (http_port))
            http_server.start()

            # Put tornado to background thread
            _TORNADO_THREAD = threading.Thread(
                target=self.Tornado_execute, args=())
            _TORNADO_THREAD.daemon = True
            _TORNADO_THREAD.start()

        # ------------------------------
        fd_r, self.pipe_write = os.pipe() # Change to PTY ?
        self.gcode.register_fd(fd_r)
        self.gcode.temperature_auto_report(False)
        self.gcode.register_command('M550',
                                    self.cmd_M550,
                                    when_not_ready=True,
                                    desc="Set printer name for DuetWebControl")
        for cmd in ['M98', 'M37', 'M1010', 'M1106']:
            self.gcode.register_command(cmd, getattr(self, 'cmd_' + cmd))

        # GCodes to Handle ? M561,

        self.gcode.write_resp = self.gcode_resp_handler
        # ------------------------------
        printer.add_object("webgui", self)
        self.logger.info("RepRep Web GUI loaded")

    def printer_state(self, state):
        if state == 'shutdown':
            pass
        elif state == 'connect':
            pass
        elif state == 'ready':
            pass

    def Tornado_LoggerCb(self, req):
        values  = [req.request.remote_ip, req.request.method, req.request.uri]
        self.logger_tornado.debug(" ".join(values))

    def Tornado_execute(self):
        tornado.ioloop.IOLoop.instance().start()

    send_resp = False
    def printer_write(self, cmd):
        self.logger.debug("GCode command: %s" % (cmd,))
        os.write(self.pipe_write, "%s\n" % cmd)
        self.send_resp = True

    def gcode_resp_handler(self, msg):
        self.logger.debug("GCode resps: %s" % (msg.strip(),))
        if self.send_resp:
            self.gcode_resps.append(msg)
            self.send_resp = False

    def printer_state(self, state):
        if state == "connect":
            self.curr_state = "B"
        elif state == "ready":
            self.curr_state = "I"
        elif state == "disconnect":
            self.curr_state = "C"
        elif state == "shutdown":
            self.curr_state = "H"

    def sd_print_done(self, status):
        if status == 'pause':
            self.curr_state = "S"
        elif status == 'start':
            self.curr_state = "P"
        elif status == 'error':
            self.curr_state = "I"
        elif status == 'done':
            self.curr_state = "I"

    # ================================================================================
    def cmd_M1106(self, params):
        self.gcode.set_fan_speed(self.gcode.get_float('S', params, 1.),
                                 self.gcode.get_int('P', params, 0))
    def cmd_M1010(self, params):
        '''
        Usage
          G10 Pnnn Xnnn Ynnn Znnn Rnnn Snnn
        Parameters
          Pnnn Tool number - SKIP
          Xnnn X offset - SKIP
          Ynnn Y offset - SKIP
          U,V,Wnnn U, V and W axis offsets - SKIP
          Znnn Z offset - SKIP
          Rnnn Standby temperature(s)
          Snnn Active temperature(s)
        '''
        if 'P' in params:
            # Set correct tool index keyword
            params['T'] = params['P']
        if 'S' in params:
            self.gcode.set_temp(params)
        elif 'R' in params:
            params['S'] = params['R']
            self.gcode.set_temp(params)
    def cmd_M37(self, params):
        if 'P' in params:
            gco_f = params['#original'][5:]
            self.logger.info("Simulating file %s" % (gco_f,))
            self.gcode.simulate_print = True
            self.printer_write("M23 %s" % (gco_f))
            self.printer_write("M24")
            # TODO: Disable simulation??
    def cmd_M98(self, params):
        macro = params['#original'][5:]
        self.logger.info("Executing macro %s" % (macro,))
        self.gcode.simulate_print = False
        self.printer_write("M23 %s" % (macro))
        self.printer_write("M24")
    def cmd_M550(self, params):
        if 'P' in params:
            self.name = params['P']
        self.logger.info("My name is now {}".format(self.name))

    # ================================================================================
    def web_getconfig(self):
        # self.logger.info("****** KLIPPER: web_getconfig() *******")
        _extrs   = extruder.get_printer_extruders(self.printer)
        num_extruders = len(_extrs)
        toolhead = self.toolhead
        steppers = toolhead.kin.get_steppers()
        num_steppers = len(steppers)
        currents = [0.00] * (num_steppers + num_extruders)
        for idx,stp in enumerate(steppers):
            get_current = getattr(stp.driver, "get_current", None)
            if get_current is not None:
                currents[idx] = get_current()
        for idx,e in _extrs.items():
            get_current = getattr(e.stepper.driver, "get_current", None)
            if get_current is not None:
                currents[idx] = get_current()
        return {
            "axisMins"            : [ s.position_min for s in steppers ],
            "axisMaxes"           : [ s.position_max for s in steppers ],
            "accelerations"       : [ toolhead.max_accel ] * (num_steppers + num_extruders),
            "currents"            : currents, #[1.00] * (num_steppers + num_extruders),
            "firmwareElectronics" : util.get_cpu_info(),
            "firmwareName"        : "Klipper",
            "firmwareVersion"     : self.printer.get_start_args().get('software_version'),
            "firmwareDate"        : "2017-12-01",
            "idleCurrentFactor"   : 1.0,
            "idleTimeout"         : toolhead.motor_off_time,
            "minFeedrates"        : [0.00] * (num_steppers + num_extruders),
            "maxFeedrates"        : [toolhead.max_velocity] * (num_steppers + num_extruders)
            }

    def web_getstatus(self, _type=1):
        toolhead = self.toolhead
        states = {
            False : 0,
            True  : 2
        }
        curr_pos = toolhead.get_position()
        fans     = [ fan.last_fan_value * 100.0 for fan in self.printer.lookup_module_objects("fan") ]
        heatbed  = self.printer.lookup_object('heater bed')
        _heaters = self.printer.lookup_module_objects("heater")
        total_htrs = len(_heaters)
        _extrs   = extruder.get_printer_extruders(self.printer)

        # _type == 1 is always included
        status_block = {
            # sequence number to indicate a new G-code response
            "status": self.curr_state,
            "seq"   : len(self.gcode_resps),
            "coords": {
                "axesHomed" : toolhead.kin.is_homed(),
                "extr"      : [ curr_pos[3] if toolhead.extruder == e else 0.0
                                for i,e in _extrs.items() ], #coords_extr,
                "xyz"       : curr_pos[:3],
            },
            "currentTool": toolhead.extruder.index,        # -1 means none
            "params": {
                "atxPower"    : 0,
                "fanPercent"  : fans,
                "speedFactor" : self.gcode.speed_factor * 60. * 100.0,
                "extrFactors" : [ e.extrude_factor * 100.0 for i,e in _extrs.items() ],
                "babystep"    : float("%.3f" % self.babysteps.babysteps),
            },
            # This must be included....
            "sensors": {
                "probeValue"     : -1, # 0
                "probeSecondary" : [0,0],  # Hidden for unmodulated probes, otherwise its array size depends on the probe type (usually 1 or 2)
                "fanRPM"         : 0,
            },
            "time" : (time.time() - self.starttime)  # time since last reset
        }

        status_block["temps"] = {}
        if (heatbed is not None):
            status_block["temps"].update( {
                "bed": {
                    "active"  : float("%.2f" % heatbed.target_temp),
                    "heater"  : (total_htrs-1),
                },
            } )

        '''
        if chamber is not None:
            status_block["temps"].update( {
                "chamber": {
                    "active"  : float("%.2f" % heatbed.target_temp),
                    "heater"  : chamber.heater.index,
                },
            } )
        if cabinet is not None:
            status_block["temps"].update( {
                "cabinet": {
                    "active"  : float("%.2f" % heatbed.target_temp),
                    "heater"  : cabinet.heater.index,
                },
            } )
        '''
        htr_current = [0.0] * total_htrs
        htr_state   = [  3] * total_htrs # HS_off = 0, HS_standby = 1, HS_active = 2, HS_fault = 3, HS_tuning = 4
        for htr in _heaters:
            index = htr.index
            if htr == heatbed:
                index = (total_htrs-1)
            htr_current[index] = float("%.2f" % htr.last_temp)
            htr_state[index]   = states[True if htr.last_pwm_value > 0.0 else False]
        status_block["temps"].update( {
            "current" : htr_current,
            "state"   : htr_state, # 0: off, 1: standby, 2: active, 3: fault (same for bed)
        } )

        # Tools target temps
        status_block["temps"].update( {
            'tools': {
                "active"  : [ [ float("%.2f" % (e.get_heater().target_temp)) ]
                              for i,e in _extrs.items() ],
                "standby" : [ [ 0.0 ] for i,e in _extrs.items() ],
            },
        } )

        if (_type == 2):
            max_temp  = 0.0
            cold_temp = 0.0
            if hasattr(toolhead.extruder, "get_heater"):
                heater = toolhead.extruder.get_heater()
                max_temp  = heater.max_temp
                cold_temp = heater.min_extrude_temp
                if heater.min_extrude_temp_disabled:
                    cold_temp = 0.0
            status_block.update( {
                "coldExtrudeTemp" : cold_temp,
                "coldRetractTemp" : cold_temp,
                "tempLimit"       : max_temp,
                "endstops_IGN"    : 7,                # NEW: As of 1.09n-ch, this field provides a bitmap of all stopped drive endstops
                "firmwareName"    : "Klipper",
                "geometry"        : toolhead.kin.name, # cartesian, coreXY, delta
                "axes"            : 3,                # Subject to deprecation - may be dropped in RRF 1.20
                "volumes"         : 1,                # Num of SD cards
                "mountedVolumes"  : 1,                # Bitmap of all mounted volumes
                "name"            : self.name,
                #"probe": {
                #    "threshold" : 500,
                #    "height"    : 2.6,
                #    "type"      : 1
                #},
                #"mcutemp": { # Not available on RADDS
                #    "min": 26.4,
                #    "cur": 30.5,
                #    "max": 43.4
                #},
                #"vin": { # Only DuetNG (Duet Ethernet + WiFi)
                #    "min": 10.4,
                #    "cur": 12.3,
                #    "max": 12.5
                #},
            } )

            tools = []
            for key,extr in _extrs.items():
                values = {
                    "number"   : extr.index,
                    "name"     : extr.name,
                    "heaters"  : [ extr.heater.index ],
                    "drives"   : [ 3+extr.index ],
                    #"filament" : "N/A",
                }
                tools.append(values)
            status_block["tools"] = tools

        elif (_type == 3):
            # TODO : update at some day
            status_block.update( {
                "currentLayer"       : 0,
                "currentLayerTime"   : 0.0,
                "extrRaw"            : [ 0.0 for i,e in _extrs.items() ],  # How much filament would have been printed without extrusion factors applied
                "fractionPrinted"    : 0.0,         # one decimal place

                "firstLayerDuration" : 0.0,
                "firstLayerHeight"   : 0.0,
                "printDuration"      : 0.0,
                "warmUpDuration"     : 0.0,

                "timesLeft": {
                    "file"     : 0.0,
                    "filament" : 0.0,
                    "layer"    : 0.0
                }
            } )
        return status_block


def load_module(printer, config):
    if not config.has_section("reprapgui"):
        return None
    return RepRapGuiModule(
        printer, config.getsection("reprapgui"))
