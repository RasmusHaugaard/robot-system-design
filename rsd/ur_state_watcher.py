from rsd.utils.rsd_redis import RsdRedis
from rsd.utils.warnings import Warnings
import time

wt = "ur"
r = RsdRedis()
w = Warnings(r)

while True:
    time.sleep(1)
    r_mode = r.service_call("getRobotModes")
    s_mode = r.service_call("getSafetyMode")

    if s_mode == 6:
        w.set(wt, "Table safety stop engaged")
    elif s_mode == 7:
        w.set(wt, "Teach pendant stop engaged")
    elif r_mode != 7:
        w.set(wt, "Please initialize the robot")
    else:
        w.rem(wt)
