import rtde_control
import rtde_io
from rsd import conf
from rsd.robot import q
import time

ur_ctrl = rtde_control.RTDEControlInterface(conf.UR_IP)
ur_io = rtde_io.RTDEIOInterface(conf.UR_IP)

ur_io.setSpeedSlider(1)

ur_ctrl.moveJ(q.TRAY_RELEASE_B_ABOVE)
if True:
    ur_ctrl.moveJ(q.TRAY_RELEASE_B_ABOVE)
    ur_ctrl.moveJ(q.EMPTY_BOXES_WAYPOINT)
    ur_ctrl.moveJ(q.EMPTY_BOXES_0)

    ur_ctrl.moveJ(q.EMPTY_BOXES_ROTATE_TCP)
    ur_ctrl.moveJ(q.EMPTY_BOXES_EMPTYING)
    ur_ctrl.moveJ(q.EMPTY_BOXES_ROTATE_TCP)

    ur_ctrl.moveJ(q.EMPTY_BOXES_0)
    ur_ctrl.moveJ(q.EMPTY_BOXES_WAYPOINT)
    ur_ctrl.moveJ(q.TRAY_RELEASE_B_ABOVE)
    ur_ctrl.moveJ(q.TRAY_RELEASE_B)
    