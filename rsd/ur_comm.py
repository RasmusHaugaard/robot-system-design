#!/usr/bin/python3

from functools import partial

from rtde_io import RTDEIOInterface
from rtde_receive import RTDEReceiveInterface

from rsd.utils.rsd_redis import RsdRedis
from rsd import conf
from rsd.utils.function_queue import FunctionQueue

r = RsdRedis()

ur_rcv = RTDEReceiveInterface(conf.UR_IP)
ur_io = RTDEIOInterface(conf.UR_IP)

io_fq = FunctionQueue()
rcv_fq = FunctionQueue()

# io
r.subscribe("setStandardDigitalOut", partial(io_fq, ur_io.setStandardDigitalOut))
r.subscribe("setSpeedSlider", partial(io_fq, ur_io.setSpeedSlider))

r.service("setStandardDigitalOut", partial(io_fq, ur_io.setStandardDigitalOut))
r.service("setSpeedSlider", partial(io_fq, ur_io.setSpeedSlider))

# recv
r.service("getActualQ", partial(rcv_fq, lambda x: ur_rcv.getActualQ()))
r.service("getSafetyMode", partial(rcv_fq, lambda x: ur_rcv.getSafetyMode()))
r.service("getRobotModes", partial(rcv_fq, lambda x: ur_rcv.getRobotModes()))
# TODO: poll robot state and publish abort in case of emergency stop

#r.join()
