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
from rsd.utils.warnings import Warnings

signal.signal(signal.SIGINT, signal.SIG_DFL)  # https://stackoverflow.com/questions/5160577/ctrl-c-doesnt-work-with-pyqt


class MainWindow(QtWidgets.QMainWindow, QObject):
    state_change = Signal(tuple)
    state = None
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
        self.on_state_change((None, S.ABORTED))

        # button callbacks
        self.ui.stopButton.clicked.connect(
            lambda: self.r.publish("action", A.STOP))
        self.ui.pauseButton.clicked.connect(
            lambda: self.r.publish("action", A.HOLD))
        self.ui.startButton.clicked.connect(
            lambda: self.r.publish("action", A.START))
        self.ui.resetButton.clicked.connect(
            lambda: self.r.publish("action", A.RESET))

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

        uptime_string = convert_from_seconds_to_timestamp(self.uptime)
        runtime_string = convert_from_seconds_to_timestamp(self.runtime)

        self.ui.uptimeLabel.setText(uptime_string)
        self.ui.runtimeLabel.setText(runtime_string)

        self.total_count = self.r.get("total_count")

        self.update_OEE()
        self.update_order_status()
        self.update_light_tower()
        self.update_warnings()
        self.update_mir_state()

    def update_order_status(self):
        orders_packed = self.r.get("total_count")
        order = self.r.get("current_order") or {"blue": 0, "red": 0, "yellow": 0, "id": "no order"}
        remaining_bricks = self.r.get("remaining_bricks") or {"blue": 0, "red": 0, "yellow": 0}

        self.ui.ordersPackedLabel.setText(str(orders_packed))
        self.ui.orderIDLabel.setText(str(order["id"]))
        self.ui.numBlueLabel.setText(str(order["blue"] - remaining_bricks["blue"]) + "/" + str(order["blue"]))
        self.ui.numRedLabel.setText(str(order["red"] - remaining_bricks["red"]) + "/" + str(order["red"]))
        self.ui.numYellowLabel.setText(str(order["yellow"] - remaining_bricks["yellow"]) + "/" + str(order["yellow"]))

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
        ideal_cycle_time = 180 / 4  # 4 orders take 180 seconds approximately
        performance = ideal_cycle_time * total_count / max(self.runtime, 1e-6)
        self.ui.performanceLabel.setText("%0.2f" % performance)

        # Quality:
        good_count = total_count
        quality = good_count / (total_count or 1)
        self.ui.qualityLabel.setText("%0.2f" % quality)

        # OEE:
        OEE = availability * performance * quality
        self.ui.OEELabel.setText("%0.2f" % OEE)

    def update_mir_state(self):
        mir_state = self.r.get("mir_state")
        self.ui.MIRstatusLabel.setText(mir_state)

    def update_warnings(self):
        w = Warnings(self.r)
        warnings = w.get_all_warnings()
        lines = ["Warnings:"] if warnings else []
        for key, val in warnings.items():
            lines.append("{}: {}".format(key, val))
        text = "\n".join(lines)
        self.ui.warningLabel.setText(text)


def convert_from_seconds_to_timestamp(seconds):
    time_hrs = int(seconds / 3600)
    time_mins = int((seconds - time_hrs * 3600) / 60)
    time_secs = seconds - (time_hrs * 3600) - (time_mins * 60)

    string_hrs = str(time_hrs) if time_hrs > 10 else "0" + str(time_hrs)
    string_mins = str(time_mins) if time_mins > 10 else "0" + str(time_mins)
    string_secs = str(time_secs) if time_secs > 10 else "0" + str(time_secs)

    return string_hrs + ":" + string_mins + ":" + string_secs


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
    window.r.unsubscribe()
