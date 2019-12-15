import rtde_control
from rsd import conf

ur_ctrl = rtde_control.RTDEControlInterface(conf.UR_IP)
#ur_ctrl.stopRobot()
#ur_ctrl.reconnect()
ur_ctrl.teachMode()
