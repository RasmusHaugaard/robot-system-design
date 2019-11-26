from rtde_control import RTDEControlInterface
from rsd import conf
from rsd.robot import q

ur_ctrl = RTDEControlInterface(conf.UR_IP)

for _q in (
        q.IDLE,
        q.ABOVE_PUSH_START_BOX_TRAY,
        q.PUSH_START_BOX_TRAY,
        q.PUSH_END_BOX_TRAY,
        q.ABOVE_PUSH_END_BOX_TRAY,
        q.IDLE,
):
    ur_ctrl.moveJ(_q)
