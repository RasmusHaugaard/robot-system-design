#!/usr/bin/python3

import sys
import time
import signal

from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from PySide2 import QtGui
from PySide2 import QtCore

from rsd.gui.ui_mainwindow import Ui_MainWindow
from rsd.utils.rsd_redis import RsdRedis
from rsd.packml.packml import STATES, PackMLActions as A

signal.signal(signal.SIGINT, signal.SIG_DFL)  # https://stackoverflow.com/questions/5160577/ctrl-c-doesnt-work-with-pyqt

state_images = (
    ["tower_states/001.png", 1], ["tower_states/011.png", 0], ["tower_states/011.png", 0], ["tower_states/001.png", 0],
    ["tower_states/001.png", 1], ["tower_states/001.png", 1], ["tower_states/000.png", 1], ["tower_states/000.png", 1],
    ["tower_states/010.png", 0], ["tower_states/001.png", 1], ["tower_states/010.png", 1], ["tower_states/010.png", 1],
    ["tower_states/100.png", 1], ["tower_states/100.png", 1], ["tower_states/100.png", 0], ["tower_states/100.png", 0],
    ["tower_states/100.png", 0]
)


class MainWindow(QtWidgets.QMainWindow, QObject):
    state_change = Signal(tuple)

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.r = RsdRedis()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        self.ui.packmlStateLabel.setText("ABORTED")
        image_profile = QtGui.QImage('tower_states/000.png')
        image_profile = image_profile.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                             transformMode=QtCore.Qt.SmoothTransformation)
        self.ui.lightTowerStatus.setPixmap(
            QtGui.QPixmap.fromImage(image_profile))
        logo = QtGui.QImage('logo.jpg')
        self.ui.label_10.setPixmap(QtGui.QPixmap.fromImage(logo))

        # state
        self.state_change.connect(self.on_state_change)
        self.r.subscribe("state_changed", self.state_change.emit)

        self.ui.abortButton.clicked.connect(
            lambda: self.r.publish("action", A.ABORT))
        self.ui.stopButton.clicked.connect(
            lambda: self.r.publish("action", A.STOP))
        self.ui.pauseButton.clicked.connect(
            lambda: self.r.publish("action", A.HOLD))
        self.ui.startButton.clicked.connect(
            lambda: self.r.publish("action", A.START))
        self.ui.resetButton.clicked.connect(
            lambda: self.r.publish("action", A.RESET))

        # OEE - https://www.oee.com/calculating-oee.html
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.on_timer_timeout)
        self.timer.start(1000)

        self.uptime = 0  # aka planned production time.
        self.runtime = 0

        self.ideal_cycle_time = 120  # Average of packing 10 orders
        self.new_state = 0
        self.flash = True

    @Slot(tuple)
    def on_state_change(self, data):
        old_state, self.new_state = data
        self.ui.packmlStateLabel.setText(STATES[self.new_state])
        self.flash = True

    @Slot()
    def on_timer_timeout(self):

        self.uptime = self.uptime + 1
        if (self.uptime < 15):  # Change to if state == executing?
            self.runtime = self.runtime + 1

        time_hrs = int(self.uptime / 3600)
        time_mins = int((self.uptime - time_hrs * 3600) / 60)
        time_secs = self.uptime - (time_hrs * 3600) - (time_mins * 60)

        string_hrs = str(time_hrs) if time_hrs > 10 else "0" + str(time_hrs)
        string_mins = str(time_mins) if time_mins > 10 else "0" + str(time_mins)
        string_secs = str(time_secs) if time_secs > 10 else "0" + str(time_secs)

        uptime_string = string_hrs + ":" + string_mins + ":" + string_secs
        self.ui.uptimeLabel.setText(uptime_string)

        self.update_OEE()

        # Keep light tower image still
        if state_images[self.new_state][1] == 0:
            state_image = QtGui.QImage(state_images[self.new_state][0])
            state_image = state_image.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                             transformMode=QtCore.Qt.SmoothTransformation)
            self.ui.lightTowerStatus.setPixmap(
                QtGui.QPixmap.fromImage(state_image))
        # Flash the light tower image
        else:
            if self.flash == True:
                self.flash = False
                state_image = QtGui.QImage(state_images[self.new_state][0])
                state_image = state_image.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                                 transformMode=QtCore.Qt.SmoothTransformation)
                self.ui.lightTowerStatus.setPixmap(
                    QtGui.QPixmap.fromImage(state_image))
            else:
                self.flash = True
                state_image = QtGui.QImage("tower_states/000.png")
                state_image = state_image.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                                 transformMode=QtCore.Qt.SmoothTransformation)
                self.ui.lightTowerStatus.setPixmap(
                    QtGui.QPixmap.fromImage(state_image))

    def update_OEE(self):
        # Availability:
        self.availibity = (self.runtime / self.uptime)
        self.ui.availabilityLabel.setText("%0.2f" % self.availibity)

        # Performance:
        self.total_count = 10  # self.r.get("total count")
        self.performance = (self.ideal_cycle_time *
                            self.total_count) / self.runtime
        self.ui.performanceLabel.setText("%0.2f" % self.performance)

        # Quality:
        self.good_count = 7  # r.get("good count")
        self.quality = self.good_count / self.total_count
        self.ui.qualityLabel.setText("%0.2f" % self.quality)

        # OEE:
        self.OEE = self.availibity * self.performance * self.quality
        self.ui.OEELabel.setText("%0.2f" % self.OEE)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    window.r.unsubscribe()
