#include "mainwindow.h"
//#include "/home/soren/hobby/packML_cmake/build/packml_autogen/include/ui_mainwindow.h"
#include "ui_mainwindow.h"


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Ros:
    _lightstate_pub = _node_handle.advertise<std_msgs::String>("packml_state",100);

    initStateMachine();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::receivedCommandSlot(QString command)
{
    qDebug() << "Mainwindow received the command: " << command;
    if(command.compare("abort") == 0)
    {
        qDebug() << "You sent the command 'abort'";
        emit abortCommandReceivedSignal();
    }
    else if(command.compare("stop") == 0)
    {
        qDebug() << "You sent the command 'stop'";
        emit stopCommandReceivedSignal();
    }
}



void MainWindow::initStateMachine()
{
    _state_machine          = new QStateMachine(this);

    _top_box_state = new QState();  //Parent states
    _lower_box_state = new QState();

    _state_unholding        = new MyState("unholding",  &_lightstate_pub,       _top_box_state);
    _state_held             = new MyState("held",       &_lightstate_pub,       _top_box_state);
    _state_holding          = new MyState("holding",    &_lightstate_pub,       _top_box_state);
    _state_idle             = new MyState("idle",       &_lightstate_pub,       _top_box_state);
    _state_starting         = new MyState("starting",   &_lightstate_pub,       _top_box_state);
    _state_execute          = new MyState("execute",    &_lightstate_pub,       _top_box_state);
    _state_completing       = new MyState("completing", &_lightstate_pub,       _top_box_state);
    _state_complete         = new MyState("complete",   &_lightstate_pub,       _top_box_state);
    _state_resetting        = new MyState("resetting",  &_lightstate_pub,       _top_box_state);
    _state_unsuspending     = new MyState("unsuspending", &_lightstate_pub,     _top_box_state);
    _state_suspended        = new MyState("suspended",  &_lightstate_pub,       _top_box_state);
    _state_suspending       = new MyState("suspending", &_lightstate_pub,       _top_box_state);
    _state_stopped          = new MyState("stopped",    &_lightstate_pub,       _lower_box_state);
    _state_stopping         = new MyState("stopping",   &_lightstate_pub,       _lower_box_state);
    _state_clearing         = new MyState("clearing",   &_lightstate_pub,       _lower_box_state);
    _state_aborted          = new MyState("aborted",    &_lightstate_pub,       _lower_box_state);
    _state_aborting         = new MyState("aborting",   &_lightstate_pub,       _lower_box_state);


    _top_box_state->addTransition(ui->abortButton,SIGNAL(clicked(bool)),_state_aborting);  //This makes every state in the "top part" of packml chart go to aborting / stopping state
    _top_box_state->addTransition(ui->stopButton,SIGNAL(clicked(bool)),_state_stopping);   //when the 'abort' or 'stop' buttons are pushed


    _top_box_state->addTransition(this, &MainWindow::abortCommandReceivedSignal, _state_aborting);  //If somebody sends "abort" or "stop" to command topic
    _top_box_state->addTransition(this, &MainWindow::stopCommandReceivedSignal, _state_stopping);   //the GUI will update accordingly


    _state_unholding->addTransition (ui->stateCompleteButton,SIGNAL(clicked(bool)),_state_execute);
    _state_held->addTransition      (ui->unHoldButton, SIGNAL(clicked(bool)), _state_unholding);
    _state_holding->addTransition   (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_held);
    _state_idle->addTransition      (ui->startButton, SIGNAL(clicked(bool)), _state_starting);
    _state_starting->addTransition  (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_execute);
    _state_execute->addTransition   (ui->holdButton, SIGNAL(clicked(bool)), _state_holding);
    _state_execute->addTransition   (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_completing);
    _state_execute->addTransition   (ui->suspendButton, SIGNAL(clicked(bool)), _state_suspending);
    _state_completing->addTransition(ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_complete);
    _state_complete->addTransition  (ui->resetButton, SIGNAL(clicked(bool)), _state_resetting);
    _state_resetting->addTransition (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_idle);
    _state_unsuspending->addTransition(ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_execute);
    _state_suspended->addTransition (ui->unsuspendButton, SIGNAL(clicked(bool)), _state_unsuspending);
    _state_suspending->addTransition(ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_suspended);
    _state_stopped->addTransition   (ui->resetButton, SIGNAL(clicked(bool)), _state_resetting);
    _state_stopping->addTransition  (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_stopped);
    _state_clearing->addTransition  (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_stopped);
    _state_aborted->addTransition   (ui->clearButton, SIGNAL(clicked(bool)), _state_clearing);
    _state_aborting->addTransition  (ui->stateCompleteButton, SIGNAL(clicked(bool)), _state_aborted);


    _state_unholding->assignProperty(ui->stateLabel,"text","Unholding");
    _state_held->assignProperty(ui->stateLabel,"text","Held");
    _state_holding->assignProperty(ui->stateLabel,"text","Holding");
    _state_idle->assignProperty(ui->stateLabel,"text","Idle");
    _state_starting->assignProperty(ui->stateLabel,"text","Starting");
    _state_execute->assignProperty(ui->stateLabel,"text","Execute");
    _state_completing->assignProperty(ui->stateLabel,"text","Completing");
    _state_complete->assignProperty(ui->stateLabel,"text","Complete");
    _state_resetting->assignProperty(ui->stateLabel,"text","Resetting");
    _state_unsuspending->assignProperty(ui->stateLabel,"text","Unsuspending");
    _state_suspended->assignProperty(ui->stateLabel,"text","Suspended");
    _state_suspending->assignProperty(ui->stateLabel,"text","Suspending");
    _state_stopped->assignProperty(ui->stateLabel,"text","Stopped");
    _state_stopping->assignProperty(ui->stateLabel,"text","Stopping");
    _state_clearing->assignProperty(ui->stateLabel,"text","Clearing");
    _state_aborted->assignProperty(ui->stateLabel,"text","Aborted");
    _state_aborting->assignProperty(ui->stateLabel,"text","Aborting");

    _top_box_state->setInitialState(_state_idle);
    _state_machine->addState(_top_box_state); //Adds every top box state to statemachine
    _state_machine->addState(_lower_box_state);
    _state_machine->setInitialState(_top_box_state);

    _state_machine->start();




//    _state_names = {"unholding", "held", "holding", "idle", "starting", "execute", "completing",
//                    "complete", "resetting", "unsuspending", "suspended", "suspending",
//                    "stopped", "stopping", "clearing", "aborted", "aborting"};


//    for(int i = 0; i < _packML_states.size(); i++)
//        _packML_states[i]->setObjectName(_state_names[i]);


//    _packML_states = {_state_unholding, _state_held, _state_holding,_state_idle, _state_starting, _state_execute,
//                      _state_completing, _state_complete,_state_resetting, _state_unsuspending, _state_suspended,
//                      _state_suspending, _state_stopped, _state_stopping, _state_clearing, _state_aborted, _state_aborting};


//    _top_box_states = {_state_unholding, _state_held, _state_holding,_state_idle, _state_starting, _state_execute, _state_complete,
//                       _state_completing, _state_resetting, _state_unsuspending, _state_suspended, _state_suspending};

//    _active_states = {_state_unholding, _state_holding, _state_starting, _state_execute, _state_completing,
//                     _state_resetting, _state_unsuspending, _state_suspending, _state_stopping,
//                     _state_clearing, _state_aborting};


}




