import time
import threading
from rsd.packml.packml import PackMLState as S, STATES
from rsd.utils.rsd_redis import RsdRedis

GREEN, YELLOW, RED = "GREEN", "YELLOW", "RED"
COLORS = (GREEN, YELLOW, RED)

OFF, SOLID, FLASH = "OFF", "SOLID", "FLASH"

# colors not defined default to OFF
light_config = {
    S.ABORTING: {RED: FLASH},
    S.ABORTED: {RED: FLASH},
    S.CLEARING: {RED: FLASH},
    S.STOPPING: {RED: SOLID},
    S.STOPPED: {RED: SOLID},
    S.RESETTING: {YELLOW: FLASH},
    S.IDLE: {GREEN: FLASH},
    S.STARTING: {GREEN: SOLID},
    S.EXECUTE: {GREEN: SOLID},
    S.HOLDING: {YELLOW: FLASH, GREEN: FLASH},
    S.HELD: {YELLOW: FLASH, GREEN: FLASH},
    S.UNHOLDING: {GREEN: SOLID},
    S.SUSPENDING: {YELLOW: SOLID},
    S.SUSPENDED: {YELLOW: SOLID},
    S.UNSUSPENDING: {GREEN: SOLID},
}


class LightTower:
    odd = False

    def __init__(self, debug=False):
        super().__init__()
        self.ur_io = ur_io
        self.should_stop = False
        self.debug = debug
        self.r = RsdRedis()
        self.state = S.STOPPED
        self.t = None

    def set_state(self, state):
        self.state = state

    def start(self, blocking=True):
        assert self.t is None
        sub = self.r.subscribe("state", self.set_state)

        def loop():
            while not self.should_stop:
                self.set_lights()
                time.sleep(.5)
            self.r.unsubscribe(sub)

        self.t = threading.Thread(target=loop)
        self.t.start()
        if blocking:
            self.t.join()
        return self

    def set_lights(self):
        state = self.state
        if state is None:
            state = S.STOPPED
        l_conf = light_config[state]
        condition = (SOLID, FLASH) if self.odd else (SOLID,)
        lights_on = [l_conf.get(color, OFF) in condition for color in COLORS]
        self.odd = not self.odd

        if self.debug:
            print(STATES[state], "ODD" if self.odd else "EVEN", lights_on)
        else:
            for output_id, light_on in enumerate(lights_on):
                self.r.publish("setStandardDigitalOutput", (output_id, light_on))

    def stop(self):
        self.should_stop = True
        self.t.join()

    def __del__(self):
        self.stop()


def main():
    lt = LightTower(ur_io=None, debug=True)
    lt.start(blocking=False)

    r = RsdRedis()
    for state in S.RESETTING, S.EXECUTE, S.HOLDING:
        time.sleep(1)
        r.publish("state", state)

    lt.stop()


if __name__ == '__main__':
    main()
