#!/usr/bin/env python3

import rsd.mir.rest_api_functions as rest
import time

STATE_AWAY = 0
STATE_MOVING_TO_WORKCELL = 1
STATE_AT_WORKCELL = 2
STATE_MISSION_COMPLETE = 3
STATE_CHARGING = 4
mir_states = {
    STATE_AWAY: 'Away',
    STATE_MOVING_TO_WORKCELL: 'Moving to workcell',
    STATE_AT_WORKCELL: 'At workcell',
    STATE_MISSION_COMPLETE: 'Mission complete',
    STATE_CHARGING: "Charging"
}

MIR_BATTERY_REGISTER = 1 #Set this register high if battery percentage below 30%
MIR_STATE_REGISTER = 2
MIR_RELEASE_REGISTER = 3

G10_BATTERY_REGISTER = 11
G11_BATTERY_REGISTER = 21
G12_BATTERY_REGISTER = 31

GLOBAL_CHARGING_REGISTER = 90

BATTERY_THRESHOLD = 30.0

class Mir:
    def __init__(self, battery_register, state_register, release_register, g9_mission_name, charging_mission_name):
        self.battery_register = battery_register
        self.state_register = state_register
        self.release_register = release_register
        
        self.g9_mission_name = g9_mission_name
        self.g9_mission_guid = rest.get_mission_guid(self.g9_mission_name)
        
        self.charging_mission_name = charging_mission_name
        self.charging_mission_guid = rest.get_mission_guid(self.charging_mission_name)
        
    def set_state(self, state):
        rest.set_register_value(self.state_register, state)

    def get_state(self):
        return rest.get_register_value(self.state_register)

    def get_battery_percentage(self):
        return rest.get_status()["battery_percentage"]


    def all_other_groups_want_charging(self):
        g10 = rest.get_register_value(G10_BATTERY_REGISTER)
        g11 = rest.get_register_value(G11_BATTERY_REGISTER)
        g12 = rest.get_register_value(G12_BATTERY_REGISTER)

        return g10 and g11 and g12

    def go_to_charging_station(self):
        rest.add_to_mission_queue(self.charging_mission_guid)

    def should_go_to_charging_station(self):
        if self.get_battery_percentage() < BATTERY_THRESHOLD:
            rest.set_register_value(self.battery_register, 1)
            if self.all_other_groups_want_charging():
                return True
        return False
    
    def is_already_charging(self):
        return rest.get_register_value(GLOBAL_CHARGING_REGISTER)


    def battery_check(self):
        if self.should_go_to_charging_station():
            rest.set_register_value(GLOBAL_CHARGING_REGISTER, 1)
            self.go_to_charging_station()
            

    def come_to_workcell(self):
        self.remove_old_missions()
        rest.add_to_mission_queue(self.g9_mission_guid)
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
            if m['mission_id'] == rest.get_mission_guid(self.g9_mission_name):
                rest.remove_mission(id)
        rest.set_register_value(self.state_register, STATE_AWAY)


def main():
    mir = Mir(MIR_BATTERY_REGISTER, MIR_STATE_REGISTER, MIR_RELEASE_REGISTER, "G9_mission", "G9G10G11G12Recharging")
    mir.get_battery_percentage()
    print(mir.should_go_to_charging_station())
    #mir.go_to_charging_station()
    # mir.get_state()
    mir.come_to_workcell()
    # mir.release_from_workcell()


if __name__ == "__main__":
    main()
