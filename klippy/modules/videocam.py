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
            self.index = index = config.getint('camera_index', default=0)
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
        def get_pic(self):
            return self.get_frame(skip=4)

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
