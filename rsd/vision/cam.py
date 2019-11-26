import time
import picamera
import picamera.array


class Cam:
    def __init__(self):
        self.camera = picamera.PiCamera(
            resolution=(640, 480),
            framerate=30,
        )
        self.camera.start_preview()
        time.sleep(2)

    def capture(self, file=None):
        if file:
            self.camera.capture(file, use_video_port=True)
        else:
            with picamera.array.PiRGBArray(self.camera) as stream:
                self.camera.capture(stream, format='bgr', use_video_port=True)
                image = stream.array
            return image

