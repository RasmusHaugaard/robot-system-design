import rtde_receive
import rsd_conf

rtde_r = rtde_receive.RTDEReceiveInterface(rsd_conf.UR_IP)
print(tuple(rtde_r.getActualQ()))
