#include "mainwindow.h"

//Qt:
#include <QApplication>
#include <QThread>

//Ros:
#include "ros/ros.h"

//C++
#include <iostream>

//
#include "listener.h"

using namespace std;


int main(int argc, char *argv[])
{
    //Ros:
    ros::init(argc, argv, "qt_packML");

    QThread thread;
    Listener listener;
    listener.moveToThread(&thread);
    QObject::connect(&thread, &QThread::started, &listener, &Listener::listen);
    thread.start();


    //Qt:
    QApplication a(argc, argv);
    MainWindow w;

    QObject::connect(&listener, &Listener::commandSignal, &w, &MainWindow::receivedCommandSlot); //Whenever listener picks up a command
                                                                                                 //The mainwindow is notified


    w.show();

    return a.exec();
}
