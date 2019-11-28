from packml.packml import PackMLActions as A, PackMLState as S, PackML
from multiprocessing import Process
import subprocess
import time
from rtde_io import RTDEIOInterface
import rsd_conf


class Program:
    process = None
    time = 0
    ur_io = None  # RTDE

    def __init__(self):
        pass

    def resetting(self, pml):
        # Make sure safety features are on, etc
        print("Resetting")
        return True

    def starting(self, pml):
        # Check for valid parameters, ramp up speed, pressure, etc for production
        print("STARTING")
        # self.ur_io = RTDEIOInterface(rsd_conf.UR_IP)
        # self.ur_io.setSpeedSlider(1.)
        return True

    def execute(self, pml):
        print("EXECUTING")
        if self.process is None:
            print("No execute process. Starting new process.")
            self.process = Process(target=lambda: subprocess.call(["python3", "executing.py"]))
            self.process.start()
        else:
            print("Execute process already running.")
        return False

    def stop_process(self):
        self.process.terminate()
        self.process = None

    def stopping(self, pml):
        print("STOPPING")
        self.stop_process()
        # self.ur_io.setSpeedSlider(0.)
        return True

    def holding(self, pml):
        print("HOLDING")
        # self.ur_io.setSpeedSlider(0.)
        return True

    def unholding(self, pml):
        print("UNHOLDING")
        #self.ur_io.setSpeedSlider(1.)
        return True


program = Program()

pml = PackML({
    S.RESETTING: program.resetting,
    S.STARTING: program.starting,
    S.EXECUTE: program.execute,
    S.STOPPING: program.stopping,
    S.HOLDING: program.holding,
    S.UNHOLDING: program.unholding,
})

pml(A.RESET)
pml(A.START)

print("machine works for some time")
time.sleep(1)

print("user presses hold to access workspace")
pml(A.HOLD)
time.sleep(1)

print("user starts machine again")
pml(A.UNHOLD)
time.sleep(1)
