#     Install opencv for video streaming
#         - $sudo apt-get install python-opencv
#
#  sudo update-rc.d -f octoprint remove
#  sudo update-rc.d -f webcamd remove
#
#  cp /usr/lib/python2.7/dist-packages/cv* ~/klippy-env/lib/python2.7/site-packages/
#  ~/klippy-env/bin/pip install numpy

import threading

"""
[videocam]
camera_index: 0
"""

class VideoCamera(object):
    import cv2
    def __init__(self, config):
        # Find OpenCV version
        major_ver = int(self.cv2.__version__.split('.')[0])
        resolution = config.get('resolution', default="640x480")
        if "x" not in resolution:
            raise config.error("Invalid resolution format! ")
        resolution = resolution.split('x')
        width = config.getint('resolution_width',
                              default=int(resolution[0]), above=0)
        height = config.getint('resolution_height',
                               default=int(resolution[1]), above=0)
        fps = config.getfloat('fps', default=0, minval=0.)
        self.index = index = config.getint('camera_index', default=0)
        # Validate version
        if major_ver < 3:
            # version 2.x
            prop_fps = self.cv2.cv.CV_CAP_PROP_FPS
            prop_width = self.cv2.cv.CV_CAP_PROP_FRAME_WIDTH
            prop_height = self.cv2.cv.CV_CAP_PROP_FRAME_HEIGHT
        elif major_ver == 3:
            # version 3.x
            prop_fps = self.cv2.CAP_PROP_FPS
            prop_width = self.cv2.CAP_PROP_FRAME_WIDTH
            prop_height = self.cv2.CAP_PROP_FRAME_HEIGHT
        else:
            raise config.error("opencv version '%s' is not supported!",
                major_ver)
        self.video = video = self.cv2.VideoCapture(index)
        if not video.isOpened():
            raise config.error("Could not open video device")
        video.set(prop_fps, fps)
        video.set(prop_width, width)
        video.set(prop_height, height)
        self.lock = threading.Lock()
    def __del__(self):
        self.video.release()
    def get_frame(self):
        with self.lock:
            video = self.video
            if video.grab():
                success, image = video.retrieve()
                if success and image is not None:
                    ret, jpeg = self.cv2.imencode('.jpg', image)
                    return jpeg.tostring()
            return ""
    def get_pic(self):
        return self.get_frame()

class VideoCameraDummy(object):
    def __init__(self, config):
        print("VideoCameraDummy loaded!")
        pass
    def __del__(self):
        pass
    def get_frame(self):
        return ""
    def get_pic(self):
        return ""

def load_config(config):
    if config.has_section('videocam'):
        config = config.getsection('videocam')
        try:
            return VideoCamera(config)
        except ImportError:
            pass
    return VideoCameraDummy(config)
