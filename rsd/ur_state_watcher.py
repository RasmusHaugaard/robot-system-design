from rsd.utils.rsd_redis import RsdRedis
from rsd.utils.warnings import Warnings
from rsd.packml.packml import PackMLState as S, PackMLActions as A
import time

wt = "ur"
r = RsdRedis()
w = Warnings(r)

while True:
    time.sleep(1)
    r_mode = r.service_call("getRobotModes")
    s_mode = r.service_call("getSafetyMode")
    stop = False

    if s_mode == 6:
        w.set(wt, "Table safety stop engaged")
        stop = True
    elif s_mode == 7:
        w.set(wt, "Teach pendant stop engaged")
        stop = True
    elif r_mode != 7:
        w.set(wt, "Please initialize the robot")
        stop = True
    else:
        w.rem(wt)

    if stop and r.get("state") != S.ABORTED:
        r.publish("action", A.ABORT)
