//UR rtde:
#include <rtde_control_interface.h>
#include <ur_rtde/rtde_io_interface.h>

//C++
#include <iostream> // only needed for the printout
#include <string>
#include <memory>


//Ros
#include "ros/ros.h"
#include "std_msgs/String.h"

using namespace ur_rtde;
using namespace std;

#define UR_IP "192.168.100.54"

#define RED_OUTPUT_PIN 2
#define YELLOW_OUTPUT_PIN 1
#define GREEN_OUTPUT_PIN 0


//Global variables:
string state = "idle";
bool flash = false;
RTDEIOInterface* rtde_io_ptr;


void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
    state = msg->data;
}

void timer_callback(const ros::TimerEvent& event)
{
    ROS_INFO("Timer timed out");
    flash = !flash;
    cout << "Should flash: " << flash << endl;

}

void stateMachine();
void setDigitalOutputs(RTDEIOInterface* iface, bool red, bool yellow, bool green);


int main(int argc, char* argv[])
{

    rtde_io_ptr = new RTDEIOInterface(UR_IP);

    ros::init(argc, argv,"light_tower");

    ros::NodeHandle nodehandle;
    ros::Subscriber lightstate_sub = nodehandle.subscribe("lightstate",1000,callback);

    ros::Timer timer = nodehandle.createTimer(ros::Duration(1),timer_callback);
    ros::Rate loop_rate(10);

    while(ros::ok())
    {
        ros::spinOnce();
        stateMachine();

        loop_rate.sleep();
    }

    //Clean up:
    delete rtde_io_ptr;

    return 0;

}

void stateMachine()
{
    if(state == "unholding")
    {
        setDigitalOutputs(rtde_io_ptr, false, false, true);
        cout << "stateMachine(): unholding state" << endl;
    }
    else if(state == "held")
    {
        setDigitalOutputs(rtde_io_ptr, false, flash, flash);
        cout << "stateMachine(): held state" << endl;
    }
    else if(state =="holding")
    {
        setDigitalOutputs(rtde_io_ptr, false, flash, flash);
        cout << "stateMachine(): holding state" << endl;
    }
    else if(state =="idle")
    {
        setDigitalOutputs(rtde_io_ptr, false, false, flash);
        cout << "stateMachine(): idle" << endl;
    }

    else if(state =="starting")
    {
        setDigitalOutputs(rtde_io_ptr, false, false, true);
        cout << "stateMachine(): starting " << endl;
    }

    else if(state =="execute")
    {
        setDigitalOutputs(rtde_io_ptr, false, false, true);
        cout << "stateMachine(): execute" << endl;
    }
    else if(state =="completing")
    {
        setDigitalOutputs(rtde_io_ptr, flash, flash, flash); //Note this state is ont part of Christian Quist's standard...
        cout << "stateMachine(): completing" << endl;
    }

    else if(state =="complete")
    {
        setDigitalOutputs(rtde_io_ptr, flash, flash, flash);
        cout << "stateMachine(): complete" << endl;
    }

    else if(state =="resetting")
    {
        setDigitalOutputs(rtde_io_ptr, false, flash, false);
        cout << "stateMachine(): resetting" << endl;
    }
    else if(state =="unsuspending")
    {
        setDigitalOutputs(rtde_io_ptr, false, false, true);
        cout << "stateMachine(): unsuspending" << endl;
    }
    else if(state =="suspended")
    {
        setDigitalOutputs(rtde_io_ptr, false, true, false);
        cout << "stateMachine(): suspended" << endl;
    }

    else if(state =="suspending")
    {
        setDigitalOutputs(rtde_io_ptr, false, true, false);
        cout << "stateMachine(): suspending" << endl;
    }

    else if(state =="stopped")
    {
        setDigitalOutputs(rtde_io_ptr, true, false, false);
        cout << "stateMachine(): stopped" << endl;
    }

    else if(state =="stopping")
    {
        setDigitalOutputs(rtde_io_ptr, true, false, false);
        cout << "stateMachine(): stopping" << endl;
    }

    else if(state =="clearing")
    {
        setDigitalOutputs(rtde_io_ptr, flash, false, false);
        cout << "stateMachine(): clearing" << endl;
    }

    else if(state =="aborted")
    {
        setDigitalOutputs(rtde_io_ptr, flash, false, false);
        cout << "stateMachine(): aborted" << endl;
    }

    else if(state =="aborting")
    {
        setDigitalOutputs(rtde_io_ptr, flash, false, false);
        cout << "stateMachine(): aborting" << endl;
    }

}



void setDigitalOutputs(RTDEIOInterface* iface, bool red, bool yellow, bool green)
{

     iface->setStandardDigitalOut(RED_OUTPUT_PIN, red);
     iface->setStandardDigitalOut(YELLOW_OUTPUT_PIN, yellow);
     iface->setStandardDigitalOut(GREEN_OUTPUT_PIN, green);

    //Defensive programming:
//    if(! iface->setStandardDigitalOut(RED_OUTPUT_PIN, red))
//        cout << "Could not change signal level on RED_OUTPUT_PIN << (" << RED_OUTPUT_PIN << ")" << endl;

//    if(! iface->setStandardDigitalOut(YELLOW_OUTPUT_PIN, yellow))
//            cout << "Could not change signal level on YELLOW_OUTPUT_PIN << (" << YELLOW_OUTPUT_PIN << ")" << endl;

//    if(! iface->setStandardDigitalOut(GREEN_OUTPUT_PIN, green))
//            cout << "Could not change signal level on GREEN_OUTPUT_PIN << (" << GREEN_OUTPUT_PIN << ")" << endl;


}
