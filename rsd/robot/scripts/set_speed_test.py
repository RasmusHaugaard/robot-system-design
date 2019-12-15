from rtde_io import RTDEIOInterface
from rsd import conf
import time

ur_io = RTDEIOInterface(conf.UR_IP)

for _ in range(1):
    ur_io.setSpeedSlider(0.0)
    time.sleep(1)
    ur_io.setSpeedSlider(0.5)
    time.sleep(1)
    ur_io.setSpeedSlider(1.0)
    time.sleep(1)
