#!/usr/bin/python3

import sys
from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from PySide2 import QtGui
from PySide2 import QtCore

from rsd.gui.ui_mainwindow import Ui_MainWindow
from rsd.utils.rsd_redis import RsdRedis
from rsd.packml.packml import STATES, PackMLActions as A


class MainWindow(QtWidgets.QMainWindow, QObject):
    state_change = Signal(tuple)

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.r = RsdRedis()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.packmlStateLabel.setText("ABORTED")
        image_profile = QtGui.QImage('tower_states/101.png')
        image_profile = image_profile.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                             transformMode=QtCore.Qt.SmoothTransformation)
        self.ui.lightTowerStatus.setPixmap(QtGui.QPixmap.fromImage(image_profile))

        # state
        self.state_change.connect(self.on_state_change)
        self.r.subscribe("state_changed", lambda data: window.state_change.emit(data))

        self.ui.abortButton.clicked.connect(lambda: self.r.publish("action", A.ABORT))
        self.ui.stopButton.clicked.connect(lambda: self.r.publish("action", A.STOP))
        self.ui.pauseButton.clicked.connect(lambda: self.r.publish("action", A.HOLD))
        self.ui.startButton.clicked.connect(lambda: self.r.publish("action", A.START))
        self.ui.resetButton.clicked.connect(lambda: self.r.publish("action", A.RESET))

    @Slot(tuple)
    def on_state_change(self, data):
        old_state, new_state = data
        self.ui.packmlStateLabel.setText(STATES[new_state])


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    window.r.unsubscribe()
