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
from rsd.packml.packml import PackMLState as S, STATES, PackMLActions as A
from rsd import light_tower

signal.signal(signal.SIGINT, signal.SIG_DFL)  # https://stackoverflow.com/questions/5160577/ctrl-c-doesnt-work-with-pyqt


class MainWindow(QtWidgets.QMainWindow, QObject):
    state_change = Signal(tuple)
    state = S.ABORTED
    light_tower_odd = False
    uptime = 0.
    runtime = 0.

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.r = RsdRedis()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # setup logo
        logo = QtGui.QImage('logo.jpg')
        self.ui.label_10.setPixmap(QtGui.QPixmap.fromImage(logo))

        # state subscriber
        self.state_change.connect(self.on_state_change)
        self.r.subscribe("state_changed", self.state_change.emit)

        # button callbacks
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
        self.timer.start(500)

    @Slot(tuple)
    def on_state_change(self, data):
        old_state, self.state = data
        self.ui.packmlStateLabel.setText(STATES[self.state])

    @Slot()
    def on_timer_timeout(self):
        self.uptime += .5
        if self.state == S.EXECUTE:
            self.runtime += .5

        time_hrs = int(self.uptime / 3600)
        time_mins = int(self.uptime % 3600 / 60)
        time_secs = int(self.uptime % 60)

        string_hrs = str(time_hrs) if time_hrs >= 10 else "0" + str(time_hrs)
        string_mins = str(time_mins) if time_mins >= 10 else "0" + str(time_mins)
        string_secs = str(time_secs) if time_secs >= 10 else "0" + str(time_secs)

        uptime_string = string_hrs + ":" + string_mins + ":" + string_secs
        self.ui.uptimeLabel.setText(uptime_string)

        self.update_OEE()
        self.update_light_tower()

    def update_light_tower(self):
        conf = light_tower.light_config[self.state]
        cond = (light_tower.FLASH, light_tower.SOLID) if self.light_tower_odd else (light_tower.SOLID,)
        self.light_tower_odd = not self.light_tower_odd
        on = [conf.get(c, light_tower.OFF) in cond for c in light_tower.COLORS]  # g, y, r
        binary_str = "".join([str(int(b)) for b in reversed(on)])  # r, y, g
        img_path = "tower_states/{}.png".format(binary_str)
        img = QtGui.QImage(img_path)
        img = img.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                         transformMode=QtCore.Qt.SmoothTransformation)
        self.ui.lightTowerStatus.setPixmap(
            QtGui.QPixmap.fromImage(img))

    def update_OEE(self):
        # Availability:
        availability = self.runtime / max(self.uptime, 1e-6)
        self.ui.availabilityLabel.setText("%0.2f" % availability)

        # Performance:
        total_count = self.r.get("total_count") or 0
        ideal_cycle_time = 120  # Average of packing 10 orders
        performance = ideal_cycle_time * total_count / max(self.runtime, 1e-6)
        self.ui.performanceLabel.setText("%0.2f" % performance)

        # Quality:
        good_count = self.r.get("good count") or 0
        quality = good_count / (total_count or 1)
        self.ui.qualityLabel.setText("%0.2f" % quality)

        # OEE:
        OEE = availability * performance * quality
        self.ui.OEELabel.setText("%0.2f" % OEE)


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    window.r.unsubscribe()
