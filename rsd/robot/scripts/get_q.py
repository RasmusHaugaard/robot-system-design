import rtde_receive
from rsd import conf

ur_rcv = rtde_receive.RTDEReceiveInterface(conf.UR_IP)
print(tuple(ur_rcv.getActualQ()))
