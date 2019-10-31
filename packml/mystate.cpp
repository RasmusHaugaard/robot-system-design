#include "mystate.h"

MyState::MyState()
{

}

MyState::MyState(QString name)
{
    this->setObjectName(name);
}

MyState::MyState(QString name, ros::Publisher *pub_ptr)
{
    this->setObjectName(name);
    _pub_ptr = pub_ptr;
}

void MyState::onEntry(QEvent *)
{
    std_msgs::String msg;
    msg.data = objectName().toStdString();
    _pub_ptr->publish(msg);
    ROS_INFO("Sent message: %s", msg.data.c_str());

}
