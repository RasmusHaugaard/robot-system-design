import time

import numpy as np

from cam import Cam
import vision
from modbus_callback import start_cb_register_modbus_server

cam = Cam()
time.sleep(2)
px_mu, px_cov = vision.load_color_stats()


def cb():
    img = vision.brick_crop(cam.capture())
    img = img.astype(np.float) / 255
    return vision.classify(img, px_mu, px_cov)


start_cb_register_modbus_server({0x00: cb})
