#     Install opencv for video streaming
#         - $sudo apt-get install python-opencv

import threading

"""
[videocam]
camera_index: 0
"""

class VideoCamera(object):
    import cv2
    def __init__(self, config):
        resolution = config.get('resolution', default="640x480")
        if "x" not in resolution:
            raise config.error("Invalid resolution format! ")
        resolution = resolution.split('x')
        width = config.getint('resolution_width',
                              default=int(resolution[0]), above=0)
        height = config.getint('resolution_height',
                               default=int(resolution[1]), above=0)
        fps = config.getfloat('fps', default=0, minval=0.)
        self.skip = config.getint('skip', default=4)
        self.index = index = config.getint('camera_index', default=0)
        self.video = video = self.cv2.VideoCapture(index)
        if not video.isOpened():
            raise config.error("Could not open video device")
        video.set(self.cv2.CAP_PROP_FPS, fps)
        video.set(self.cv2.CAP_PROP_FRAME_WIDTH, width)
        video.set(self.cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.lock = threading.Lock()
        print("VIDEO CAM LOADED!")
    def __del__(self):
        self.video.release()
    def get_frame(self):
        with self.lock:
            video = self.video
            for idx in range(0, self.skip):
                success, image = video.read()
            success, image = video.read()
            if image is not None:
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
