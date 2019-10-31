//UR rtde:
#include <rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>

//C++
#include <iostream> // only needed for the printout


//Ros
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ur_rtde;

#define UR_IP "127.0.0.1"

#define RED_OUTPUT_PIN 0
#define YELLOW_OUTPUT_PIN 1
#define GREEN_OUTPUT_PIN 2



void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char* argv[])
{

    RTDEIOInterface rtde_io(UR_IP);
    rtde_io.setStandardDigitalOut(RED_OUTPUT_PIN, true);


    ros::init(argc, argv,"light_tower");

    ros::NodeHandle nodehandle;
    ros::Subscriber lightstate_sub = nodehandle.subscribe("lightstate",1000,callback);

    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

//    ros::spin();


    return 0;


}

