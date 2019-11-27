from rsd.utils.rsd_redis import RsdRedis
from rtde_io import RTDEIOInterface
from rsd import conf
from rsd.utils.function_queue import FunctionQueue
from functools import partial

ur_io = RTDEIOInterface(conf.UR_IP)
r = RsdRedis()
fq = FunctionQueue()

r.subscribe("setStandardDigitalOut", partial(fq, ur_io.setStandardDigitalOut))
r.subscribe("setSpeedSlider", partial(fq, ur_io.setSpeedSlider))
r.join()
