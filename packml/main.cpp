#include "mainwindow.h"

//Qt:
#include <QApplication>

//Ros:
#include "ros/ros.h"

//C++
#include <iostream>

using namespace std;



int main(int argc, char *argv[])
{
    //Ros:
    ros::init(argc, argv, "qt_packML");

    //Qt:
    QApplication a(argc, argv);
    MainWindow w;
    w.show();

    return a.exec();
}
