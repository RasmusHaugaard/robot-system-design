#!/usr/bin/python3

import rest_api_functions as rest
import time

STATE_MOVING_TO_WORKCELL  = 1
STATE_AT_WORKCELL         = 2
STATE_MISSION_COMPLETE    = 3

class Mir():
    def __init__(self, state_register, release_register, mission_name):
        self.state_register     = state_register
        self.release_register   = release_register
        self.mission_name       = mission_name

    def get_state(self):
        return rest.get_register_value( self.state_register)
    
    def come_to_workcell(self):
        guid = rest.get_mission_guid(self.mission_name)
        rest.add_to_mission_queue(guid)

        while(self.get_state() != STATE_AT_WORKCELL ):
            print("State is:", self.get_state())
            time.sleep(1)

    def release_from_workcell(self):
        rest.set_register_value(self.release_register, 1)


if __name__ == "__main__":
    mir = Mir(9, 11, "G9_mission")
    mir.get_state()

    mir.come_to_workcell()
    mir.release_from_workcell()
    

