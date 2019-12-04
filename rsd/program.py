#!/usr/bin/python3
from subprocess import Popen

from rsd.utils.rsd_redis import RsdRedis
from rsd.packml.packml import PackML, PackMLState as S, PackMLActions as A, ACTIONS


class Program:
    def __init__(self):
        self.r = RsdRedis()
        self.pml = PackML({
            S.RESETTING: self.resetting,
            S.STARTING: self.starting,

            S.HOLDING: self.speed_zero,
            S.SUSPENDING: self.speed_zero,
            S.UNHOLDING: self.speed_one,
            S.UNSUSPENDING: self.speed_one,

            S.STOPPING: self.stopping,
            S.ABORTING: self.stopping,

            S.CLEARING: self.clearing,

            S.EXECUTE: lambda: False,  # don't finish this state
        }, cb_state_change=self.on_state_change)
        self.on_state_change(None, self.pml.state)
        self.p_ur_comm = Popen(["python3", "ur_comm.py"])
        self.ligt_tower_process = Popen(["python3", "light_tower.py"])
        self.robot_process = None  # type: Popen
        self.r.subscribe("action", self.on_action_req)
        self.gui_process = Popen(["python3", "gui.py"], cwd="gui")
        # TODO: start gui process
        # TODO: start mes state logger
        self.r.join()

    def on_action_req(self, a):
        s = self.pml.state
        if a is A.START:
            if s is S.HELD:
                a = A.UNHOLD
            if s is S.SUSPENDED:
                a = A.UNSUSPEND
            if s is S.ABORTED:
                a = A.CLEAR
        self.pml(a)

    def on_state_change(self, old_state, new_state):
        self.r.set("state", new_state)
        self.r.publish("state_changed", (old_state, new_state))

    def resetting(self):
        # TODO: ensure safety devices are running
        pass

    def starting(self):
        self.stopping()
        self.speed_one()
        self.robot_process = Popen(["python3", "execute.py"], shell=False)

    def stopping(self):
        self.speed_zero()
        if self.robot_process:
            self.robot_process.terminate()

    def clearing(self):
        # TODO: possibly clear up state
        pass

    def speed_zero(self):
        self.r.set_speed(0)

    def speed_one(self):
        self.r.set_speed(1)


Program()