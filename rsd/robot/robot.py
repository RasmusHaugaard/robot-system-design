from rsd.utils.rsd_redis import RsdRedis
from rtde_control import RTDEControlInterface
from rsd import conf


class Robot:
    def __init__(self, r: RsdRedis = RsdRedis()):
        self.r = r
        self.ur_ctrl = RTDEControlInterface(conf.UR_IP)

    def move(self, *q):
        for _q in q:
            while not self.ur_ctrl.moveJ(_q):
                pass
        return self

    def grasp(self, do_grasp=True):
        self.r.set_standard_digital_out(conf.IO_GRIPPER, do_grasp)
        return self

    def release(self):
        return self.grasp(False)

    def get_q(self):
        return self.r.get_q()
