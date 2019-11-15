from modbus_callback import start_cb_register_modbus_server
from cam import Cam

i = 0
cam = Cam()


def capture():
    global i
    cam.capture("imgs/{}.jpg".format(i))
    i += 1
    return i


start_cb_register_modbus_server({0x00: capture})
