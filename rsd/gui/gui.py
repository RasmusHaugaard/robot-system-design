#!/usr/bin/python2

import sys
from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from ui_mainwindow import Ui_MainWindow

class MainWindow(QtWidgets.QMainWindow, QObject):
    def __init__(self, *args, **kwargs):
        QtWidgets.QMainWindow.__init__(self)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.packmlStateLabel.setText("Hey what's up!?")


class RosNode(QObject):
    @Slot(str)
    def message_received_slot(self, message):
        print "I received:", message




if __name__ == "__main__":

    

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()