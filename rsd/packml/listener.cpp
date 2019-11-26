#include "listener.h"
#include <QDebug>

Listener::Listener(QObject *parent) : QObject(parent)
{
}

void Listener::listen()
{
    qDebug() << "Hello from listen()";

    _subscriber = _nodehandle.subscribe("gui_command",1000, &Listener::callback, this);

    ros::spin();

}


void Listener::callback(const std_msgs::String::ConstPtr &msg)
{
    QString command = msg->data.c_str();
    qDebug() << "Listener heard: " << command;
    emit commandSignal(command);
}
