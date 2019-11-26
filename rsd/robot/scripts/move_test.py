import rtde_control
from rsd import conf
from rsd.robot import q

ur_ctrl = rtde_control.RTDEControlInterface(conf.UR_IP)
ur_ctrl.moveJ(q.IDLE)
