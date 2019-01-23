#     Install opencv for video streaming
#         - $sudo apt-get install python-opencv

import threading

"""
[videocam]
camera_index: 0
"""

try:
    import cv2
    class VideoCamera(object):
        def __init__(self, config):
            resolution = config.get('resolution', default="640x480").split('x')
            fps = config.getfloat('fps', default=0, minval=0.)
            self.skip = config.getint('skip', default=4)
            self.index = index = config.getint('camera_index', default=0)
            self.video = video = cv2.VideoCapture(index)
            video.set(cv2.CAP_PROP_FPS, fps)
            video.set(cv2.CAP_PROP_FRAME_WIDTH, int(resolution[0]))
            video.set(cv2.CAP_PROP_FRAME_HEIGHT, int(resolution[1]))
            self.lock = threading.Lock()
        def __del__(self):
            self.video.release()
        def get_frame(self):
            with self.lock:
                video = self.video
                for idx in range(0, self.skip):
                    success, image = video.read()
                success, image = video.read()
                if image is not None:
                    ret, jpeg = cv2.imencode('.jpg', image)
                    return jpeg.tostring()
                return ""
        def get_pic(self):
            return self.get_frame()

except ImportError:
    class VideoCamera(object):
        def __init__(self, config):
            pass
        def __del__(self):
            pass
        def get_frame(self, skip=0):
            return ""
        def get_pic(self):
            return self.get_frame(skip=4)


def load_config(config):
    return VideoCamera(config)
