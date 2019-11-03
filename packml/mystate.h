#ifndef MYSTATE_H
#define MYSTATE_H
#include <QState>
#include <QDebug>
#include <QString>
#include "ros/ros.h"
#include "std_msgs/String.h"


class MyState : public QState
{
public:
    MyState();
    MyState(QString name);
    MyState(QString name, ros::Publisher* pub_ptr);
    MyState(QString name, ros::Publisher* pub_ptr, QState* parent);


protected:
    void onEntry(QEvent * ) override;
    ros::Publisher* _pub_ptr;

};

#endif // MYSTATE_H
