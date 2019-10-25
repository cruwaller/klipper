#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tornado.ioloop
import tornado.web
import time, sys, os, errno, threading, json, re, logging
import argparse, ConfigParser
# import Queue
# from multiprocessing import Queue as QueueMulti

# Local modules
import queuelogger, reactor, configfile
import modules.videocam as videocam
import modules.analyse_gcode as analyse_gcode


# Include www data to search dir
sys.path.append(os.path.abspath(
    os.path.join(os.path.dirname(__file__), "www_data")))

'''
RepRap Web Gui - Status info:
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

KLIPPER_CFG_NAME = 'klipper_config.cfg'
KLIPPER_LOG_NAME = "klippy.log"
MAX_STREAMED_SIZE = 1024**3

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
            self.index = index
            self.video = video = cv2.VideoCapture(index)
            video.set(cv2.CAP_PROP_FPS, 0.)
            self.lock = threading.Lock()
        def __del__(self):
            self.video.release()
        def get_frame(self, skip=0):
            with self.lock:
                video = self.video
                for idx in range(0, skip):
                    success, image = video.read()
                success, image = video.read()
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
        def get_frame(self, skip=0):
            return ""


class JpegHandler(tornado.web.RequestHandler):
    camera = logger = None
    def initialize(self, camera):
        self.camera = camera
    @tornado.web.asynchronous
    @tornado.gen.coroutine
    def get(self):
        self.set_header(
            'Cache-Control',
            'no-store, no-cache, must-revalidate, '
            'pre-check=0, post-check=0, max-age=0')
        self.set_header('Connection', 'close')
        self.set_header('Content-Type',
                        'multipart/x-mixed-replace;boundary=--boundarydonotcross')
        self.set_header('Expires', 'Mon, 3 Jan 2000 12:34:56 GMT')
        self.set_header('Pragma', 'no-cache')

        img = self.camera.get_frame(skip=4)
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
            else:
                yield tornado.gen.Task(ioloop.add_timeout,
                                       ioloop.time() + self.interval)


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
    def initialize(self, path, parent):
        self.path = path
        self.parent = parent
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

@tornado.web.stream_request_body
class rrHandler(tornado.web.RequestHandler):
    parent = printer = sd_path = logger = None

    def initialize(self, sd_path, parent):
        self.klipper_cfg = None
        self.fp = None
        self.bytes_written = 0

        self.parent = parent
        self.sd_path = self.parent.sd_path
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
            respdata["boardType"] = "unknown" # "radds15"

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
            respdata.update(self.parent.web_getstatus(_type))
            respdata['seq'] += len(self.parent.gcode_resps)

        # rr_gcode?gcode=XXX
        elif "rr_gcode" in path:
            respdata["err"] = 0
            respdata["buff"] = 99999

            gcode = self.get_argument('gcode')
            #self.logger.debug("rr_gcode={}".format(gcode))
            # Clean up gcode command
            gcode = gcode.replace("0:/", "").replace("0%3A%2F", "")

            atx_on = self.parent.atx_on
            atx_off = self.parent.atx_off
            if atx_on is not None and "M80" in gcode:
                # ATX ON
                resp = os.popen(atx_on).read()
                self.parent.append_gcode_resp(resp)
                self.logger.info("ATX ON: %s" % resp)
            elif atx_off is not None and "M81" in gcode:
                # ATX OFF
                resp = os.popen(atx_off).read()
                self.parent.append_gcode_resp(resp)
                self.logger.info("ATX OFF: %s" % resp)
            elif "T-1" in gcode:
                # ignore
                pass
            else:
                self.parent.write_async_with_resp(gcode)

        # rr_download?name=XXX
        elif "rr_download" in path:
            # Download a specified file from the SD card.
            path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
            if KLIPPER_CFG_NAME in path:
                path = self.parent.get_printer_start_arg('config_file', None)
            elif KLIPPER_LOG_NAME in path:
                path = self.parent.get_printer_start_arg('logfile')
            else:
                path = os.path.join(sd_path, path)
            if path is not None:
                path = os.path.abspath(path)
            # Check if file exists and upload
            if path is None or not os.path.exists(path):
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
                        self.parent.gcode_analyse.remove_file(fremoved)
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
                                                   time.gmtime(os.path.getmtime(filepath))),
                        }
                        respdata["files"].append(data)
                    elif os.path.isdir(filepath):
                        data = {
                            "type" : "d",
                            "name" : os.path.relpath(filepath, path),
                            "size" : os.path.getsize(filepath),
                            "date" : time.strftime("%Y-%m-%dT%H:%M:%S",
                                                   time.gmtime(os.path.getmtime(filepath))),
                        }
                        respdata["files"].append(data)

                # Add printer.cfg into sys list
                if "/sys" in path:
                    cfg_file = self.parent.get_printer_start_arg('config_file', None)
                    if cfg_file is not None:
                        cfg_file = os.path.abspath(cfg_file)
                        respdata["files"].append({
                            "type": "f",
                            "name": KLIPPER_CFG_NAME,
                            "size": os.path.getsize(cfg_file),
                            "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                                  time.gmtime(os.path.getmtime(cfg_file))),
                        })
                    logfile = self.parent.get_printer_start_arg('logfile', None)
                    if logfile is not None:
                        respdata["files"].append({
                            "type": "f",
                            "name": KLIPPER_LOG_NAME,
                            "size": os.path.getsize(logfile),
                            "date": time.strftime("%Y-%m-%dT%H:%M:%S",
                                                  time.gmtime(os.path.getmtime(logfile))),
                        })

        # rr_fileinfo?name=XXX
        elif "rr_fileinfo" in path:
            name = self.get_argument('name', default=None)
            # self.logger.debug("rr_fileinfo: {} , name: {}".format(self.request.uri, name))
            if name is None:
                stat = self.parent.web_getsd()
                path = stat.get('file', None)
            else:
                path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
                path = os.path.abspath(os.path.join(sd_path, path))
            # info about the requested file
            if path is None or not os.path.exists(path):
                respdata["err"] = 1
            else:
                info = self.parent.gcode_analyse.get_file_info(path)
                respdata["err"] = 0
                respdata["size"] = os.path.getsize(path)
                respdata["lastModified"] = \
                    time.strftime("%Y-%m-%dT%H:%M:%S",
                                  time.gmtime(os.path.getmtime(path)))
                respdata["generatedBy"]      = info["slicer"]
                respdata["height"]           = info["height"]
                respdata["firstLayerHeight"] = info["firstLayerHeight"]
                respdata["layerHeight"]      = info["layerHeight"]
                respdata["filament"]         = info["filament"]

                #if is_printing is True:
                #    # Current file information
                respdata["printDuration"] = info['buildTime']
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
            respdata = self.parent.web_getconfig()

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

    def _parse_file_path(self, target_path):
        # /rr_upload?name=xxxxx&time=xxxx
        # /rr_upload?name=0:/filaments/PLA/unload.g&time=2017-11-30T11:46:50

        target_path = target_path.replace("0:/", "").replace("0%3A%2F", "")
        if KLIPPER_CFG_NAME in target_path:
            path = self.parent.get_printer_start_arg('config_file', None)
            if path is not None:
                cfgname = os.path.abspath(path)
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
                target_path = temp_name
        elif KLIPPER_LOG_NAME in target_path:
            raise tornado.web.HTTPError(503)
        else:
            target_path = os.path.abspath(os.path.join(self.sd_path, target_path))
            # Create a dir first
            try:
                os.makedirs(os.path.dirname(target_path))
            except OSError:
                pass
        return target_path

    def post(self, path, *args, **kwargs):
        respdata = { "err" : 1 }
        if "rr_upload" in self.request.path:
            # /rr_upload?name=xxxxx&time=xxxx
            # /rr_upload?name=0:/filaments/PLA/unload.g&time=2017-11-30T11:46:50

            '''
            size = int(self.request.headers['Content-Length'])
            # body_len = len(self.request.body)
            body_len = self.bytes_written
            if body_len == 0:
                # e.g. filament create
                respdata["err"] = 0
            elif body_len != size or not size:
                self.logger.error("upload size error: %s != %s" % (body_len, size))
            else:
                target_path = self.get_argument('name').replace("0:/", ""). \
                    replace("0%3A%2F", "")
                if KLIPPER_CFG_NAME in target_path:
                    path = self.parent.get_printer_start_arg('config_file', None)
                    if path is not None:
                        cfgname = os.path.abspath(path)
                        datestr = time.strftime("-%Y%m%d_%H%M%S")
                        backup_name = cfgname + datestr
                        temp_name = cfgname + "_autosave"
                        if cfgname.endswith(".cfg"):
                            backup_name = cfgname[:-4] + datestr + ".cfg"
                            temp_name = cfgname[:-4] + "_autosave.cfg"
                        try:
                            f = open(temp_name, 'wb')
                            # f.write(self.request.body)
                            f.write(self.data)
                            f.close()
                            os.rename(cfgname, backup_name)
                            os.rename(temp_name, cfgname)
                            respdata['err'] = 0
                        except IOError as err:
                            self.logger.error("Upload, cfg: %s" % err)
                elif KLIPPER_LOG_NAME in target_path:
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
                        with open(target_path, 'wb') as output_file:
                            # output_file.write(self.request.body)
                            output_file.write(self.data)
                            respdata['err'] = 0
                    except IOError as err:
                        self.logger.error("Upload, g-code: %s" % err)
            '''
            if self.fp is not None:
                self.fp.close()
                if self.klipper_cfg is not None:
                    os.rename(self.klipper_cfg[1], self.klipper_cfg[0])

            size = int(self.request.headers['Content-Length'])
            if self.bytes_written != size:
                self.logger.error("upload size error: %s != %s" % (
                    self.bytes_written, size))
            else:
                respdata['err'] = 0

        # Send response back to client
        respstr = json.dumps(respdata)
        self.write(respstr)

    def prepare(self):
        self.request.connection.set_max_body_size(MAX_STREAMED_SIZE)

    def data_received(self, chunk):
        if "rr_upload" in self.request.path:
            if self.fp is None:
                filename = self._parse_file_path(self.get_argument('name'))
                try:
                    self.fp = open(filename, "wb+")
                    self.fp.write(chunk)
                except IOError:
                    raise tornado.web.HTTPError(400)
            else:
                self.fp.write(chunk)
            self.bytes_written += len(chunk)


def create_dir(_dir):
    try:
        os.makedirs(_dir)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise Exception("cannot create directory {}".format(_dir))


class CommunicationError(Exception):
    pass


class RepRapGuiModule(object):
    warmup_time = .1
    layer_stats = []
    first_layer_start = None
    last_used_file = None
    htmlroot = None
    def __init__(self, config, args):
        self.logger = logging.getLogger("DuetWebControl")
        self.logger_tornado = self.logger.getChild("tornado")
        if args.very_verbose:
            self.logger_tornado.setLevel(logging.DEBUG)
        else:
            self.logger_tornado.setLevel(logging.INFO)
        self.starttime = time.time()
        self.curr_state = 'C'
        # Read config
        htmlroot = os.path.normpath(os.path.join(os.path.dirname(__file__)))
        htmlroot = os.path.join(htmlroot, "modules", "DuetWebControl")
        if not os.path.exists(os.path.join(htmlroot, 'reprap.htm')):
            raise config.error("DuetWebControl files not found '%s'" % htmlroot)
        self.logger.debug("html root: %s" % (htmlroot,))
        self.user = config.get('user', default="")
        self.passwd = config.get('password', default="")
        # Camera information
        self.feed_interval = config.getfloat('feedrate', minval=.0, default=.1)
        self.camera = videocam.load_config(config)
        # - M80 / M81 ATX commands
        self.atx_on = config.get('atx_cmd_on', default=None)
        self.atx_off = config.get('atx_cmd_off', default=None)
        # ------------------------------
        # Create paths to virtual SD
        if config.has_section('virtual_sdcard'):
            vsd_cfg = config.getsection('virtual_sdcard')
            sdcard_dirname = vsd_cfg.get("path")
        else:
            sdcard_dirname = config.get("sd_path")
        self.sd_path = sdcard_dirname = \
            os.path.normpath(os.path.expanduser(sdcard_dirname))
        create_dir(sdcard_dirname)
        create_dir(os.path.join(sdcard_dirname, "gcodes"))
        create_dir(os.path.join(sdcard_dirname, "macros"))
        create_dir(os.path.join(sdcard_dirname, "filaments"))
        create_dir(os.path.join(sdcard_dirname, "sys"))
        self.logger.debug("Gcode path: %s" % sdcard_dirname)
        # ------------------------------
        self.gcode_analyse = analyse_gcode.load_config(config, sdcard_dirname)
        # ------------------------------
        # Open a communication pipe
        self.reactor = reactor.Reactor()
        self.partial_input = ""
        self.printer_start_args = {}
        self.input_fd = None
        self.gcode_resps = []
        self.write_count = 0
        # self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        self.resp_event = threading.Event()
        self.resp_sync = threading.Event()
        # ------------------------------
        # Configure tornado webserver
        application = tornado.web.Application(
            [
                tornado.web.url(r"/", MainHandler,
                                {"path": htmlroot}, name="main"),
                tornado.web.url(r'/login', LoginHandler,
                                {"path": htmlroot, "parent": self}, name="login"),
                tornado.web.url(r'/logout', LogoutHandler, name="logout"),
                tornado.web.url(r"/(.*\.xml)", tornado.web.StaticFileHandler,
                                {"path": htmlroot}),
                tornado.web.url(r"/fonts/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "fonts")}),
                tornado.web.url(r"/js/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "js")}),
                tornado.web.url(r"/css/(.*)", tornado.web.StaticFileHandler,
                                {"path": os.path.join(htmlroot, "css")}),
                tornado.web.url(r"/(rr_.*)", rrHandler,
                                {"sd_path": sdcard_dirname, "parent": self}),
                tornado.web.url(r"/jpeg", JpegHandler, {"camera": self.camera}),
                tornado.web.url(r"/video", JpegStreamHandler,
                                { "camera": self.camera,
                                  "interval": self.feed_interval}),
            ],
            cookie_secret="16d35553-3331-4569-b419-8748d22aa599",
            log_function=self.Tornado_LoggerCb,
            # max_buffer_size=104857600*20,
            login_url = "/login",
            xsrf_cookies = False)
        # ------------------------------
        # Start communication thread
        _comm_thread = threading.Thread(target=self.open_pipe)
        _comm_thread.daemon = True
        _comm_thread.start()
        # ------------------------------
        # Start Tornado server
        self.Tornado_execute(config, application)

    # ================================================================================
    def is_printing(self):
        return self.curr_state == "P"
    def is_halted(self):
        return self.curr_state == 'H'

    # ================================================================================
    def Tornado_LoggerCb(self, req):
        values  = [req.request.remote_ip, req.request.method, req.request.uri]
        self.logger_tornado.debug(" ".join(values))

    def Tornado_execute(self, config, application):
        http_port = config.getint('http', default=80)
        https_port = config.getint('https', None)
        port = http_port
        ssl_options = None

        if https_port is not None:
            port = https_port
            ssl_options = {
                "certfile": os.path.normpath(os.path.expanduser(config.get('cert'))),
                "keyfile": os.path.normpath(os.path.expanduser(config.get('key'))),
            }
            self.logger.debug("HTTPS port %s" % (https_port,))
        else:
            self.logger.debug("HTTP port %s" % (http_port,))

        http_server = tornado.httpserver.HTTPServer(
            application, ssl_options=ssl_options)
        http_server.listen(port)
        try:
            tornado.ioloop.IOLoop.current().start()
        except (Exception, RuntimeError) as err:
            self.logger.error("Serving failure! %s" % err)
        except KeyboardInterrupt:
            self.logger.info("Exiting...")

    # ================================================================================
    def open_pipe(self):
        # Keep thread running...
        while True:
            try:
                comm_path = open("/tmp/printer", "wb+")
            except IOError:
                time.sleep(0.5)
                continue
            input_fd = comm_path.fileno()
            self.partial_input = ""
            fd_handle = self.reactor.register_fd_thread(input_fd, self.input_handler)
            fd_handle.start()
            self.logger.debug("Klipper communication ok...")
            try:
                self.logger.info("Version: %s" % self.write_sync("M115", fd=input_fd))
                # ------------------------------
                # Disable auto temperature reporting
                self.write_sync("AUTO_TEMP_REPORT AUTO=0", fd=input_fd)
                # ------------------------------
                # Get start arguments from printer
                resp = self.write_sync("GUISTATS_GET_ARGS", fd=input_fd)
                self.printer_start_args = json.loads(resp.replace("ok", ""))
                self.logger.info("Printer start args: %s" % self.printer_start_args)
                # ------------------------------
                # Enable auto status reporting
                self.write_sync("GUISTATS_AUTO_REPORT ENABLE=1", fd=input_fd)
            except ValueError as e:
                fd_handle.stop()
                self.logger.error("Unable to parse json: '%s' - restart..." % e)
                time.sleep(1.0) # 1sec delay
                continue
            except CommunicationError as err:
                fd_handle.stop()
                self.logger.error("Communication failure: '%s' - restart..." % err)
                time.sleep(1.0) # 1sec delay
                continue
            except (KeyboardInterrupt, Exception):
                # Exit if something else has happened
                return
            #with self.write_lock:
            self.input_fd = input_fd
            # ------------------------------
            # Poll the pipe alive
            while fd_handle.is_running():
                time.sleep(0.5)
            self.logger.error("Connection lost, restart...")
            # reset params
            self.staus_report = 'NA'
            self.input_fd = None
            self.curr_state = 'C'
            self.write_count = 0
            self.partial_input = ""
            self.printer_start_args = {}

    class sentinel:
        pass
    def get_printer_start_arg(self, name, default=sentinel):
        if default is not self.sentinel:
            return self.printer_start_args.get(name, default)
        return self.printer_start_args[name]

    # ================================================================================
    def write(self, cmd, fd=None):
        if fd is None:
            if self.input_fd is None:
                raise CommunicationError("Pipe is not open")
            fd = self.input_fd
        # self.logger.debug("GCode > %s" % repr(cmd))
        try:
            os.write(fd, "%s\n" % cmd)
        except OSError as e:
            self.logger.error("write failed! %s" % e)
    def write_async(self, cmd, fd=None):
        with self.write_lock:
            try:
                self.write(cmd, fd)
            except ValueError:
                pass
    def write_async_with_resp(self, cmd, fd=None):
        with self.write_lock:
            try:
                self.write_count = 1
                self.write(cmd, fd)
            except (ValueError, CommunicationError):
                pass
    sync_resp = None
    def write_sync(self, cmd, fd=None):
        with self.write_lock:
            self.sync_resp = None
            self.resp_event.clear()
            self.resp_sync.set()
            self.write(cmd, fd)
            self.resp_event.wait(timeout=5.)
            if self.sync_resp is None:
                raise CommunicationError("Read timeout")
        return self.sync_resp

    # Callback method for input fd poller thread
    def input_handler(self, eventtime, handler):
        try:
            data_in = self.read(handler.fileno())
        except OSError:
            self.input_fd = None
            return True # Exit to reconnect...
        # self.logger.debug("GCode < %s" % repr(data_in))
        if 'GUISTATS_REPORT=' in data_in:
            self.staus_report = data_in[16:].replace("ok", "")
        elif 'Klipper state' in data_in:
            self.logger.debug("Klipper state: %s" % data_in)
            self.append_gcode_resp(data_in)
            if 'Shutdown' in data_in or 'Disconnect' in data_in:
                return True # Exit to reconnect...
        elif (self.resp_sync.is_set() and self.write_count == 0) or \
                "axesHomed" in data_in:
            self.sync_resp = data_in.replace("ok", "")
            self.resp_sync.clear()
            self.resp_event.set()
        else:
            self.append_gcode_resp(data_in)
        return False
    def read(self, fileno):
        data = ""
        while "ok" not in data:
            data += os.read(fileno, 4096)
        lines = data.split('\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        data = "\n".join(lines)
        return data.strip()

    staus_report = 'NA'
    def append_gcode_resp(self, msg):
        if type(msg) is not str:
            self.logger.warning("Not valid resp '%s' received!", msg)
            return
        if not len(msg):
            return
        msg = msg.strip()
        if len(msg) > 2:
            msg = msg.replace("ok", "")
        # self.logger.debug("GCode resp to GUI: '%s'" % (msg,))
        if "Error" in msg or "Warning" in msg or 'Klipper state' in msg:
            self.gcode_resps.append(msg)
        elif self.write_count > 0:
            self.gcode_resps.append(msg)
            self.logger.info("resp appended")
            self.write_count -= 1

    # ================================================================================
    def web_getconfig(self):
        try:
            resp = json.loads(self.write_sync("GUISTATS_GET_CONFIG"))
        except (ValueError, CommunicationError):
            resp = {'seq': 0, "err": 1}
        return resp

    def web_getstatus(self, _type=1):
        try:
            resp = json.loads(self.staus_report)
            self.curr_state = resp['status']
            return resp
        except ValueError:
            try:
                resp = json.loads(self.write_sync("GUISTATS_GET_STATUS TYPE=%d" % _type))
                self.curr_state = resp['status']
                return resp
            except (ValueError, CommunicationError):
                pass
        return {'seq': 0,  "status": self.curr_state, "err": 1}

    def web_getsd(self):
        try:
            resp = json.loads(self.write_sync("GUISTATS_GET_SD_INFO"))
        except (ValueError, CommunicationError):
            resp = {}
        return resp


# =================================================================================================

if __name__ == "__main__":
    bglogger = None
    parser = argparse.ArgumentParser(
        description='RepRap Web Gui Loader - OULWare')
    parser.add_argument('configfile',
                        help='config file path')
    parser.add_argument('-d', dest='simulation',
                        action='store_true',
                        help='Start in simulation mode')
    parser.add_argument('-v', dest='verbose',
                        action='store_true',
                        help='Change logging level to debug')
    parser.add_argument('-vv', dest='very_verbose',
                        action='store_true',
                        help='Change Tornado logging level to debug')
    parser.add_argument('-l', dest='logfile', default=None,
                        help='write to log file')
    args = parser.parse_args()

    debuglevel = logging.INFO
    if args.verbose:
        debuglevel = logging.DEBUG
    if args.logfile:
        bglogger = queuelogger.setup_bg_logging(
            args.logfile, debuglevel)
    else:
        logging.basicConfig(level=debuglevel,
                            format=queuelogger.LOGFORMAT)
    logging.getLogger().setLevel(debuglevel)
    logger = logging.getLogger('server')
    logger.info("Starting ...")

    config_file = args.configfile
    fileconfig = ConfigParser.RawConfigParser()
    if not fileconfig.read(config_file):
        raise Exception("Unable to open config file %s" % (config_file,))
    config = configfile.ConfigWrapper(
        None, fileconfig, {}, 'reprapgui_process')
    gui = RepRapGuiModule(config, args)
