#!/usr/bin/python

import sys
from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from ui_mainwindow import Ui_MainWindow

import rospy
from std_msgs.msg import String

class MainWindow(QtWidgets.QMainWindow, QObject):
    def __init__(self, *args, **kwargs):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.pub = rospy.Publisher("packml_state", String, queue_size=10)

        #Connect button signals to slots:
        self.ui.startButton.clicked.connect(self.on_start_button_clicked)

        

    @Slot(str)
    def update_packml_state_slot(self, message):
        self.ui.packmlStateLabel.setText(message)
    
    @Slot()
    def on_start_button_clicked(self):
        self.pub.publish("Start butt0n clicked")


class RosNode(QObject):
    def __init__(self):
        QObject.__init__(self) #<-- but why?
        rospy.init_node("gui_node",anonymous=True)
        self.sub = rospy.Subscriber("packml_state", String, self.packml_callback)
    
    def packml_callback(self, data):
        self.packml_signal.emit(data.data)
    
    packml_signal = Signal(str)

if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()

    node = RosNode()
    node.packml_signal.connect(window.update_packml_state_slot)

    window.show()
    app.exec_()