#ifndef LISTENER_H
#define LISTENER_H

#include <QObject>
#include "ros/ros.h"
#include "std_msgs/String.h"

class Listener : public QObject
{
    Q_OBJECT
public:
    explicit Listener(QObject *parent = nullptr);
    void listen();
    void callback(const std_msgs::String::ConstPtr& msg);

signals:
    void commandSignal(QString command);

public slots:

private:
    ros::NodeHandle _nodehandle;
    ros::Subscriber _subscriber;

};



#endif // LISTENER_H
