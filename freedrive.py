import rtde_control
import rsd_conf

ur_ctrl = rtde_control.RTDEControlInterface(rsd_conf.UR_IP)
ur_ctrl.teachMode()
