import rtde_control
from rsd import conf
from rsd.robot import q
import time

ur_ctrl = rtde_control.RTDEControlInterface(conf.UR_IP)
ur_ctrl.moveJ(q.IDLE)
time.sleep(5)
