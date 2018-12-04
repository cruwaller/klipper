#!/usr/bin/env python
# -*- coding: utf-8 -*-
import tornado.ioloop
import tornado.web
import time, sys, os, errno, threading, json, re, logging
import argparse, ConfigParser
import Queue
# Local modules
import util, queuelogger, reactor
import modules.videocam as videocam

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

_PARENT = None
KLIPPER_CFG_NAME = 'klipper_config.cfg'
KLIPPER_LOG_NAME = "klippy.log"

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


class ParseError(Exception):
    pass


ANALYSED_GCODE_FILES = {}

def analyse_gcode_file(filepath):
    # Set initial values
    info = {
        "slicer" : "unknown",
        "height" : 0,
        "layerHeight" : 0,
        "firstLayerHeight": 0,
        "filament" : []
    }
    if filepath is None:
        return info
    if filepath in ANALYSED_GCODE_FILES:
        return ANALYSED_GCODE_FILES[filepath]
    elif os.path.join("gcode", filepath) in ANALYSED_GCODE_FILES:
        return ANALYSED_GCODE_FILES[filepath]
    absolutecoord = True
    last_position = .0
    try:
        with open(filepath, 'rb') as f:
            #f.seek(0, os.SEEK_END)
            #fsize = f.tell()
            f.seek(0)
            # find used slicer
            slicer = None
            for idx in range(100):
                line = f.readline().strip()
                if "Simplify3D" in line: # S3D
                    slicer = "Simplify3D"
                elif "Slic3r" in line: # slic3r
                    slicer = "Slic3r"
                elif ";Sliced by " in line: # ideaMaker
                    slicer = "ideaMaker"
                elif "; KISSlicer" in line: # KISSlicer
                    slicer = "KISSlicer"
                elif ";Sliced at: " in line: # Cura(old)
                    slicer = "Cura (OLD)"
                elif ";Generated with Cura" in line: # Cura(new)
                    slicer = "Cura"
                elif "IceSL" in line:
                    slicer = "IceSL"
                elif "CraftWare" in line:
                    slicer = "CraftWare"
                if slicer is not None:
                    break
            # Stop if slicer is not detected!
            if slicer is None:
                raise ParseError("Cannot detect slicer")
            info["slicer"] = slicer
            # read header
            layerHeight = None
            firstLayerHeightPercentage = None
            firstLayerHeight = None
            # read footer and find object height
            f.seek(0)
            args_r = re.compile('([A-Z_]+|[A-Z*/])')
            build_info_r = re.compile('([0-9\.]+)')
            for line in f:
                line = line.strip()
                cpos = line.find(';')
                if cpos == 0:
                    # Try to parse slicer infos
                    if slicer is "Simplify3D":
                        if "Build time" in line:
                            parts = build_info_r.split(line)
                            buildTime = .0
                            offset = 1
                            if " hour " in parts:
                                buildTime += 60. * float(parts[offset])
                                offset += 2
                            if " minutes" in parts:
                                buildTime += float(parts[offset])
                            info["buildTime"] = buildTime * 60.
                        elif "Filament length: " in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]))
                        elif "layerHeight" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                        elif "firstLayerHeightPercentage" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeightPercentage = float(parts[1]) / 100.
                    elif slicer is "Slic3r":
                        if "filament used" in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]))
                        elif "first_layer_height" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                        elif "layer_height" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                    elif slicer is "Cura":
                        if "Filament used" in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[1]) * 1000.) # Convert m to mm
                        elif "Layer height" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                            firstLayerHeight = layerHeight
                    elif slicer is "IceSL":
                        if "z_layer_height_first_layer_mm" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                        elif "z_layer_height_mm" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                    elif slicer is "KISSlicer":
                        if ";    Ext " in line:
                            parts = build_info_r.split(line)
                            info["filament"].append(float(parts[3]))
                        elif "first_layer_thickness_mm" in line:
                            parts = build_info_r.split(line)
                            firstLayerHeight = float(parts[1])
                        elif "layer_thickness_mm" in line:
                            parts = build_info_r.split(line)
                            layerHeight = float(parts[1])
                    elif slicer is "CraftWare":
                        # encoded settings in gcode file, need to extract....
                        pass
                    continue

                # Remove comments
                if cpos >= 0:
                    line = line[:cpos]
                # Parse args
                parts = args_r.split(line.upper())[1:]
                params = { parts[i]: parts[i+1].strip()
                           for i in range(0, len(parts), 2) }
                # Find object height
                if "G" in params:
                    gnum = int(params['G'])
                    if gnum == 0 or gnum == 1:
                        if "Z" in params:
                            if absolutecoord:
                                last_position = float(params['Z'])
                            else:
                                last_position += float(params['Z'])
                    elif gnum == 90:
                        absolutecoord = True
                    elif gnum == 91:
                        absolutecoord = False

            info["height"] = last_position
            # first layer height
            if layerHeight is not None:
                info["layerHeight"] = float("%.3f" % layerHeight)
            if layerHeight is not None and firstLayerHeightPercentage is not None:
                info["firstLayerHeight"] = float("%.3f" % (layerHeight * firstLayerHeightPercentage))
            if firstLayerHeight is not None:
                info["firstLayerHeight"] = float("%.3f" % firstLayerHeight)
    except (IOError, ParseError):
        pass
    ANALYSED_GCODE_FILES[filepath] = info
    # logging.info("PARSED: %s" % info)
    return info

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
    parent = sd_path = logger = None

    def initialize(self, sd_path):
        self.parent = _PARENT
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

            printer_write = self.parent.write_async

            if "M80" in gcode:
                atx_on = self.parent.atx_on
                if atx_on is not None:
                    resp = os.popen(atx_on).read()
                    self.parent.append_gcode_resp(resp)
                    self.logger.info("ATX ON: %s" % resp)
            elif "M81" in gcode:
                # ATX OFF
                atx_off = self.parent.atx_off
                if atx_off is not None:
                    resp = os.popen(atx_off).read()
                    self.parent.append_gcode_resp(resp)
                    self.logger.info("ATX OFF: %s" % resp)
            elif "T-1" in gcode:
                # Skip...
                pass
            else:
                if "M0" in gcode and self.parent.curr_state == "S":
                    # Cancel print after pause, change state to idle
                    self.parent.curr_state = "I"
                try:
                    printer_write(gcode)
                except self.parent.gcode.error as e:
                    respdata["err"] = 1

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
            #self.logger.debug("rr_fileinfo: {} , name: {}".format(self.request.uri, name))
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
                path = os.path.abspath(os.path.join(sd_path, path))
            # info about the requested file
            if path is None or not os.path.exists(path):
                respdata["err"] = 1
            else:
                info = analyse_gcode_file(path)
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

                if is_printing is True:
                    # Current file information
                    respdata["printDuration"] = self.parent.toolhead.get_print_time()
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
        #self.logger.info(json.dumps(respdata,
        #                        sort_keys=True,
        #                        indent=4, separators=(',', ': ')))

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
                path = self.get_argument('name').replace("0:/", "").replace("0%3A%2F", "")
                if KLIPPER_CFG_NAME in path:
                    path = self.parent.get_printer_start_arg('config_file', None)
                    if path is not None:
                        path = os.path.abspath(path)
                elif KLIPPER_LOG_NAME in path:
                    path = None
                    respdata['err'] = 0
                else:
                    path = os.path.abspath(os.path.join(self.sd_path, path))
                if path is not None:
                    try:
                        os.makedirs(os.path.dirname(path))
                    except OSError as e:
                        if e.errno != errno.EEXIST:
                            pass
                    try:
                        # Write request content to file
                        with open(path, 'w') as output_file:
                            output_file.write(self.request.body)
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
    warmup_time = .1
    layer_stats = []
    first_layer_start = None
    last_used_file = None
    htmlroot = None
    def __init__(self, config, args):
        global _TORNADO_THREAD
        global _PARENT
        _PARENT = self
        self.logger = config.get_logger("DuetWebControl")
        self.logger_tornado = self.logger.getChild("tornado")
        if args.very_verbose:
            self.logger_tornado.setLevel(logging.DEBUG)
        else:
            self.logger_tornado.setLevel(logging.INFO)
        self.starttime = time.time()
        self.curr_state = 'C'
        self.gcode_resps = []
        self.lock = threading.Lock()
        self.write_lock = threading.Lock()
        # Read config
        htmlroot = os.path.normpath(os.path.join(os.path.dirname(__file__)))
        htmlroot = os.path.join(htmlroot, "modules", "DuetWebControl")
        if not os.path.exists(os.path.join(htmlroot, 'reprap.htm')):
            raise ConfigWrapper.error("DuetWebControl files not found '%s'" % htmlroot)
        self.logger.debug("html root: %s" % (htmlroot,))
        self.user = config.get('user')
        self.passwd = config.get('password')
        # Camera information
        self.feed_interval = config.getfloat('feedrate', minval=.0, default=.1)
        self.camera = None
        if config.has_section('videocam'):
            self.camera = videocam.VideoCamera(config.getsection('videocam'))
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
        # Open a communication pipe
        self.reactor = reactor.Reactor()
        self.partial_input = ""
        self.printer_start_args = {}
        self.input_fd = None
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
        # Start communication thread
        _comm_thread = threading.Thread(target=self.open_pipe)
        _comm_thread.daemon = True
        _comm_thread.start()
        # ------------------------------
        self.logger.info("RepRep Web GUI loaded")

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

        http_server = tornado.httpserver.HTTPServer(application,
                                                    ssl_options=ssl_options)
        http_server.listen(port)
        tornado.ioloop.IOLoop.current().start()

    # ================================================================================
    def open_pipe(self):
        while True:
            try:
                comm_path = open("/tmp/reprapgui", "wb+")
            except IOError:
                time.sleep(0.5)
                continue
            self.input_fd = input_fd = comm_path.fileno()
            self.partial_input = ""
            fd_handle = self.reactor.register_fd_thread(input_fd, self.input_handler)
            fd_handle.start()
            self.logger.debug("Klipper communication pipe ok...")
            try:
                # ------------------------------
                # Disable auto temperature reporting
                self.write_sync("AUTO_TEMP_REPORT AUTO=0")
                # ------------------------------
                # Get start arguments from printer
                self.printer_start_args = \
                    json.loads(self.write_sync("GUISTATS_GET_ARGS"))
                self.logger.info("Printer start args: %s" % self.printer_start_args)
            except ValueError as e:
                self.logger.error("Communication failure: '%s' - restart..." % e)
                time.sleep(1.0) # 1sec delay
                continue
            # ------------------------------
            # Poll the pipe alive
            while fd_handle.is_running():
                time.sleep(0.5)
            # reset params
            self.partial_input = ""
            self.printer_start_args = {}
            self.input_fd = None

    class sentinel:
        pass
    def get_printer_start_arg(self, name, default=sentinel):
        if default is not self.sentinel:
            return self.printer_start_args.get(name, default)
        return self.printer_start_args[name]

    # ================================================================================
    def read(self, fileno):
        try:
            data = os.read(fileno, 8192) # 8192 -> 4096
            # self.logger.info("DATA < %s", data)
        except OSError as e:
            self.logger.error("read failed! %s" % e)
            return "FAILURE"
        lines = data.split('\n')
        lines[0] = self.partial_input + lines[0]
        self.partial_input = lines.pop()
        data = "\n".join(lines)
        return data
    def write(self, cmd):
        if self.input_fd is None:
            return
        # self.logger.debug("Write: %s" % (cmd,))
        try:
            os.write(self.input_fd, "%s\n" % cmd)
        except OSError as e:
            self.logger.error("write failed! %s" % e)
    def write_async(self, cmd):
        with self.write_lock:
            self.write(cmd)
    sync_resp = None
    def write_sync(self, cmd):
        if self.input_fd is None:
            raise ValueError("Pipe is not open")
        with self.write_lock:
            self.sync_resp = None
            self.input_state = 3
            self.write(cmd)
            timeout = 0.
            while self.sync_resp is None:
                time.sleep(0.05)
                timeout += 0.05
                if timeout > 1.:
                    self.input_state = 0
                    raise ValueError("Read timeout")
            self.input_state = 0
            resp = self.sync_resp
        #self.logger.debug("write_sync: resp %s" % resp)
        return resp

    def printer_write(self, cmd):
        with self.write_lock:
            self.write(cmd)

    # Callback method for input fd poller thread
    input_state = 0
    def input_handler(self, eventtime, handler):
        # self.logger.info("input handler called at %s" % eventtime)
        data_in = self.read(handler.fileno())
        if self.input_state == 3:
            self.sync_resp = data_in
        else:
            self.append_gcode_resp(data_in)
        return False

    def append_gcode_resp(self, msg):
        if type(msg) is not str:
            self.logger.warning("Not valid resp '%s' received!", msg)
            return
        msg = msg.strip()
        if msg not in self.gcode_resps:
            self.logger.debug("append_gcode_resp(%s)" % (msg,))
            self.gcode_resps.append(msg)

    # ================================================================================
    def web_getconfig(self):
        try:
            resp = json.loads(self.write_sync("GUISTATS_GET_CONFIG"))
        except ValueError as e:
            # self.logger.error("Config fail: %s" % e)
            resp = {'seq': 0, "err": 1}
        return resp

    def web_getstatus(self, _type=1):
        try:
            resp = json.loads(self.write_sync("GUISTATS_GET_STATUS TYPE=%d" % _type))
        except ValueError as e:
            # self.logger.error("Status fail: %s" % e)
            resp = {'seq': 0,  "status": "C", "err": 1}
        return resp

# =================================================================================================

class ConfigWrapper:
    error = ConfigParser.Error
    class sentinel:
        pass
    def __init__(self, fileconfig, section, logger):
        self.logger = logger
        self.fileconfig = fileconfig
        self.section = section
    def get_logger(self, name=None):
        if name is None:
            return self.logger.getChild(self.section)
        return self.logger.getChild(name)
    def get_name(self):
        return self.section
    def _get_wrapper(self, parser, option, default,
                     minval=None, maxval=None, above=None, below=None):
        if (default is not self.sentinel
            and not self.fileconfig.has_option(self.section, option)):
            return default
        try:
            v = parser(self.section, option)
            if type(v) == str:
                v = v.strip('"|\'')
        except self.error:
            raise
        except:
            raise self.error("Unable to parse option '%s' in section '%s'" % (
                option, self.section))
        if minval is not None and v < minval:
            raise self.error(
                "Option '%s' in section '%s' must have minimum of %s" % (
                    option, self.section, minval))
        if maxval is not None and v > maxval:
            raise self.error(
                "Option '%s' in section '%s' must have maximum of %s" % (
                    option, self.section, maxval))
        if above is not None and v <= above:
            raise self.error(
                "Option '%s' in section '%s' must be above %s" % (
                    option, self.section, above))
        if below is not None and v >= below:
            raise self.error(
                "Option '%s' in section '%s' must be below %s" % (
                    option, self.section, below))
        return v
    def get(self, option, default=sentinel):
        return self._get_wrapper(self.fileconfig.get, option, default)
    def getint(self, option, default=sentinel, minval=None, maxval=None):
        return self._get_wrapper(
            self.fileconfig.getint, option, default, minval, maxval)
    def getfloat(self, option, default=sentinel,
                 minval=None, maxval=None, above=None, below=None):
        return self._get_wrapper(self.fileconfig.getfloat, option, default,
                                 minval, maxval, above, below)
    def getboolean(self, option, default=sentinel):
        return self._get_wrapper(self.fileconfig.getboolean, option, default)
    def getchoice(self, option, choices, default=sentinel):
        c = self.get(option, default)
        if c not in choices:
            raise self.error(
                "Choice '%s' for option '%s' in section '%s'"
                " is not a valid choice" % (c, option, self.section))
        return choices[c]
    def getsection(self, section):
        return ConfigWrapper(self.fileconfig, section, self.logger)
    def has_section(self, section):
        return self.fileconfig.has_section(section)
    def get_prefix_sections(self, prefix):
        return [self.getsection(s) for s in self.fileconfig.sections()
                if s.startswith(prefix)]


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
    logger = logging.getLogger('start')
    logger.info("Starting ...")

    config_file = args.configfile
    fileconfig = ConfigParser.RawConfigParser()
    if not fileconfig.read(config_file):
        raise Exception("Unable to open config file %s" % (config_file,))
    config = ConfigWrapper(fileconfig, 'reprapgui_process', logger)
    gui = RepRapGuiModule(config, args)
    try:
        while 1:
            time.sleep(10)
    except KeyboardInterrupt:
        pass
