#  sudo update-rc.d -f octoprint remove
#  sudo update-rc.d -f webcamd remove

import os, time
import logging
import requests
import subprocess
import threading

"""
[videocam]
camera_index: 0
fps: 1
resolution: 640x480
;resolution_width: 640
;resolution_height: 480
port: 8080
path: /home/pi/mjpg_streamer/
"""

# Default to octoprint's default address
STREAM_DEFAULT = "http://127.0.0.1:8080/?action=stream"
SNAPSHOT_DEFAULT = "http://127.0.0.1:8080/?action=snapshot"


class VideoStreamerHelper:
    def __init__(self, config):
        self.logger = config.get_printer().logger.getChild("VideoStreamerHelper")
        resolution = config.get('resolution', default="640x480")
        if "x" not in resolution:
            raise config.error("Invalid resolution format! ")
        resolution = resolution.split('x')
        width = config.getint('resolution_width',
            default=int(resolution[0]), above=0)
        height = config.getint('resolution_height',
            default=int(resolution[1]), above=0)
        resolution = "%sx%s" % (width, height)
        fps = config.getint('fps', default=1, minval=1)
        video = "/dev/video%d" % config.getint('camera_index', default=0)
        port = config.getint('port', default=8080, minval=0, maxval=65535)
        self.url = "http://127.0.0.1:%d/?action=snapshot" % (port,)
        self.logger.info("VideoCamera url: %s" % (self.url,))
        # Resolve streamer path
        streamer_path = config.get('path')
        streamer = os.path.join(streamer_path, "mjpg_streamer")
        if not os.path.exists(streamer):
            raise config.error("streamer does not exist")
        cmd = [streamer, '-i "%s' % os.path.join(streamer_path, "input_uvc.so"),
            '-n', '-r %s' % resolution, '-f %d' % fps, '-d %s"' % video,
            '-o "%s' % os.path.join(streamer_path, "output_http.so"),
            '-p %d"' % port]
        # Start into background thread
        bg_thread = threading.Thread(
            target=self.__execute, args=(cmd,))
        bg_thread.daemon = True
        bg_thread.start()
    def __execute(self, *cmd):
        proc = subprocess.Popen([" ".join(*cmd)], shell=True, bufsize=1,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        proc.wait()
        self.logger.warning("videocam process exit!")
        for line in iter(proc.stdout.readline, b''):
            self.logger.error("  %s", line.strip())
        self.url = SNAPSHOT_DEFAULT
    def get_url(self):
        return self.url


class VideoCamera(object):
    def __init__(self, config):
        self.get_url = lambda : SNAPSHOT_DEFAULT
        if config.get('path', None) is not None:
            self.helper = VideoStreamerHelper(config)
            self.get_url = self.helper.get_url
    def get_frame(self):
        try:
            r = requests.get(self.get_url(), timeout=2.)
            return r.content
        except Exception:
            return ""
    def get_pic(self):
        return self.get_frame()


class VideoCameraDummy(object):
    def __init__(self):
        pass
    def __del__(self):
        pass
    def get_frame(self):
        return ""
    def get_pic(self):
        return ""


def load_config(config):
    if config.has_section('videocam'):
        return VideoCamera(config.getsection('videocam'))
    return VideoCameraDummy()
