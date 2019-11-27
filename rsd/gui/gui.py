#!/usr/bin/python3

import sys
from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from ui_mainwindow import Ui_MainWindow
from PySide2 import QtGui
from PySide2 import QtCore

class MainWindow(QtWidgets.QMainWindow, QObject):
    def __init__(self, *args, **kwargs):
        super().__init__()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.packmlStateLabel.setText("Hey what's up!?")
        image_profile = QtGui.QImage('tower_states/101.png')
        image_profile = image_profile.scaled(183,281, aspectRatioMode=QtCore.Qt.KeepAspectRatio, transformMode=QtCore.Qt.SmoothTransformation) # To scale image for example and keep its Aspect Ration    
        self.ui.lightTowerStatus.setPixmap(QtGui.QPixmap.fromImage(image_profile)) 




if __name__ == "__main__":

    

    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    

