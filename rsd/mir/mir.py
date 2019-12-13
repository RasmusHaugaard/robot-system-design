#!/usr/bin/python3

import rsd.mir.rest_api_functions as rest
import time

STATE_AWAY = 0
STATE_MOVING_TO_WORKCELL = 1
STATE_AT_WORKCELL = 2
STATE_MISSION_COMPLETE = 3
mir_states = {
    STATE_AWAY : 'AWAY'
    STATE_MOVING_TO_WORKCELL: 'MOVING_TO_WORKCELL',
    STATE_AT_WORKCELL: 'AT_WORKCELL',
    STATE_MISSION_COMPLETE: 'MISSION_COMPLETE'
}


class Mir:
    def __init__(self, state_register, release_register, mission_name):
        self.state_register = state_register
        self.release_register = release_register
        self.mission_name = mission_name
        self.guid = rest.get_mission_guid(self.mission_name)

    def get_state(self):
        return rest.get_register_value(self.state_register)

    def come_to_workcell(self):
        self.remove_old_missions()
        rest.add_to_mission_queue(self.guid)

        while self.get_state() != STATE_AT_WORKCELL:
            time.sleep(1)

    def release_from_workcell(self):
        rest.set_register_value(self.release_register, 1)

    """Find all pending and executing missions
        And then remove the ones which are posted by our group"""

    def remove_old_missions(self):
        mq = rest.get_mission_queue()
        l = []
        for d in mq:
            state = d['state']
            if state in ('Pending', 'Executing'):
                l.append(d['id'])
        for id in l:
            m = rest.get_mission_info_by_id(id)
            if m['mission_id'] == rest.get_mission_guid(self.mission_name):
                rest.remove_mission(id)
        rest.set_register_value(self.state_register, STATE_AWAY)


def main():
    mir = Mir(9, 11, "G9_mission")
    mir.get_state()

    mir.come_to_workcell()
    mir.release_from_workcell()


if __name__ == "__main__":
    main()
