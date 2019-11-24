#!/usr/bin/python2

"""ROS imports"""
import rospy
from std_msgs.msg import Int16

import rest_api_functions as rest

MIR_STATE_REGISTER = 9

def mir_node():
    rospy.init_node('mir_node', anonymous=True) 
    pub = rospy.Publisher("mir_state", Int16, queue_size=10)
    rate = rospy.Rate(5)

    while(not rospy.is_shutdown()):
        pub.publish( rest.get_register_value(MIR_STATE_REGISTER) )
        rate.sleep()


if __name__ == "__main__":
    mir_node()