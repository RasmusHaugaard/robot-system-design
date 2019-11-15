#!/usr/bin/python2

"""Python imports"""
import threading
import time

"""ROS imports"""
import rospy
from std_msgs.msg import String
from std_msgs.msg import Int16
#from std_msgs.msg import 

"""SHP imports"""
import rest_api_functions as rest

state_register = 0
state_lock = threading.Lock()

def read_register_thread(register_id):
    global state_register, state_lock
    while not rospy.is_shutdown():
        state_lock.acquire()

        state_register = int( rest.get_register_value(register_id) )
        release_register = int(rest.get_register_value(11))

        state_lock.release()
        time.sleep(1)


def mir_command_callback(data):
    if(data.data == "release"):
        print("Release the robot")
        rest.set_register_value(11, 1) #Set register 11 to 1, breaking the "PLC wait while loop"

    elif "mission_" in data.data: #Publish "mission_name" on mir_command topic - where name is the actual name of the mission. e.g. : mission_G9TestMission
        mission_name = data.data[8:len(data.data)] #remove 'mission_' from data received.
        guid = rest.get_mission_guid(mission_name)
        rest.add_to_mission_queue(guid)    
        print("Added mission: %s to queue" % mission_name )



def mir_node():
    rospy.init_node('mir_node', anonymous=True) 
    pub = rospy.Publisher('mir_state', Int16, queue_size=10)
    sub = rospy.Subscriber('mir_command', String, mir_command_callback)
    rate = rospy.Rate(1)

    # rospy.spin()
    global state_register, state_lock
    while not rospy.is_shutdown():
        state_lock.acquire()

        pub.publish(state_register)
        print("Published state %10d"  %state_register)
        state_lock.release()

        rate.sleep()

if __name__ == '__main__':
    t_read_state = threading.Thread(target = read_register_thread, args=(9,))
    t_read_state.start()
    mir_node()
