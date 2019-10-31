#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QDebug>
#include <QList>
#include <QString>

#include <QStateMachine>
#include <QPropertyAnimation>


#include <ros/ros.h>
#include <std_msgs/String.h>

#include "mystate.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:

private:
    Ui::MainWindow *ui;
    QStateMachine* _state_machine;

    //Ros:
    ros::NodeHandle _node_handle;
    ros::Publisher _lightstate_pub;

    //packML states:

    MyState *_my_state;
    QState* _state_unholding;
    QState* _state_held;
    QState* _state_holding;
    QState* _state_idle;
    QState* _state_starting;
    QState* _state_execute;
    QState* _state_complete;
    QState* _state_completing;
    QState* _state_resetting;
    QState* _state_unsuspending;
    QState* _state_suspended;
    QState* _state_suspending;
    QState* _state_stopped;
    QState* _state_stopping;
    QState* _state_clearing;
    QState* _state_aborted;
    QState* _state_aborting;

    QList<QState*> _packML_states;
    QList<QState*> _top_box_states; //The top states which can directly stop & abort.
    QList<QState*> _active_states;  //An active state is a green one in the state machine diagram on. Execute state is dual (both acting & waiting)
                                    //https://infosys.beckhoff.com/content/1033/tcplclib_tc3_packml_v2/Images/png/1336618251__Web.png
    //QList<QState*> _waiting_states; //A waiting state is a yellow one.

    QList<QString> _state_names;

    void initStateMachine();


};

#endif // MAINWINDOW_H
