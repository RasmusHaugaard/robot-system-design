#!/usr/bin/python3

import sys
import time
import signal
signal.signal(signal.SIGINT, signal.SIG_DFL) #https://stackoverflow.com/questions/5160577/ctrl-c-doesnt-work-with-pyqt

from PySide2 import QtWidgets
from PySide2.QtCore import QObject, Slot, Signal
from PySide2 import QtGui
from PySide2 import QtCore

from rsd.gui.ui_mainwindow import Ui_MainWindow
from rsd.utils.rsd_redis import RsdRedis
from rsd.packml.packml import STATES, PackMLState as S, PackMLActions as A


state_images = (["tower_states/001.png", 1], ["tower_states/011.png", 0], ["tower_states/011.png", 0], ["tower_states/001.png", 0], ["tower_states/001.png", 1], ["tower_states/001.png", 1], ["tower_states/000.png", 1], ["tower_states/000.png", 1], ["tower_states/010.png", 0],
                ["tower_states/001.png", 1], ["tower_states/010.png", 1], ["tower_states/010.png", 1], ["tower_states/100.png", 1], ["tower_states/100.png", 1], ["tower_states/100.png", 0], ["tower_states/100.png", 0], ["tower_states/100.png", 0])


class MainWindow(QtWidgets.QMainWindow, QObject):
    state_change = Signal(tuple)

    def __init__(self, *args, **kwargs):
        super().__init__()
        self.r = RsdRedis()
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)
        image_profile = QtGui.QImage('tower_states/000.png')
        image_profile = image_profile.scaled(183, 281, aspectRatioMode=QtCore.Qt.KeepAspectRatio,
                                             transformMode=QtCore.Qt.SmoothTransformation)
        self.ui.lightTowerStatus.setPixmap(
            QtGui.QPixmap.fromImage(image_profile))
        logo = QtGui.QImage('logo.jpg')
        self.ui.label_10.setPixmap(QtGui.QPixmap.fromImage(logo))

        # state
        self.state_change.connect(self.on_state_change)
        self.new_state = S.EXECUTE
        self.ui.packmlStateLabel.setText(STATES[self.new_state])
        #self.r.subscribe("state_changed", window.state_change.emit) #<-- Bug

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

        self.ideal_cycle_time = 180  #[seconds] Average of packing 4 orders
        # self.new_state = 0
        self.flash = True

    @Slot(tuple)
    def on_state_change(self, data):
        old_state, self.new_state = data
        self.ui.packmlStateLabel.setText(STATES[self.new_state])
        self.flash = True

    @Slot()
    def on_timer_timeout(self):

        self.uptime = self.uptime + 1
        if(self.new_state == S.EXECUTE):  #Only increase runtime if the robot is in execute state
            self.runtime = self.runtime + 1

        uptime_string = convert_from_seconds_to_timestamp(self.uptime)
        runtime_string = convert_from_seconds_to_timestamp(self.runtime)
        self.ui.uptimeLabel.setText(uptime_string)
        self.ui.runtimeLabel.setText(runtime_string)

        self.total_count = self.r.get("total count")

        self.update_OEE()
        self.update_order_status()
        self.show_gui_lighttower()
        
        
    def update_order_status(self):
        orders_packed = self.r.get("total_count")
        order = self.r.get("current_order")
        remaining_bricks = self.r.get("remaining_bricks")

        self.ui.ordersPackedLabel.setText   (str(orders_packed))
        self.ui.orderIDLabel.setText        (str(order["id"]))
        self.ui.numBlueLabel.setText        (str(order["blue"] - remaining_bricks["blue"]) + "/" + str(order["blue"]))
        self.ui.numRedLabel.setText         (str(order["red"] - remaining_bricks["red"]) + "/"  + str(order["red"]))
        self.ui.numYellowLabel.setText      (str(order["yellow"] - remaining_bricks["yellow"]) + "/"  + str(order["yellow"]))

        print("Orders packed so far: %d ", orders_packed)
        print(order)


    def update_OEE(self):
        # Availability:
        self.availibity = (self.runtime / self.uptime)
        self.ui.availabilityLabel.setText("%0.2f" % self.availibity)

        # Performance:
        #self.total_count = 10  # self.r.get("total count")
        self.performance = (self.ideal_cycle_time * self.total_count) / self.runtime if self.runtime > 0 else 0
        self.ui.performanceLabel.setText("%0.2f" % self.performance)

        # Quality:
        # self.quality = self.good_count / self.total_count if self.total_count > 0 else 0
        self.quality = 1 #Quality is always 1.00 in this system.
        self.ui.qualityLabel.setText("%0.2f" % self.quality)

        # OEE:
        self.OEE = self.availibity * self.performance * self.quality
        self.ui.OEELabel.setText("%0.2f" % self.OEE)

    def show_gui_lighttower(self):
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


def convert_from_seconds_to_timestamp(seconds):
        time_hrs = int(seconds / 3600)
        time_mins = int((seconds - time_hrs * 3600) / 60)
        time_secs = seconds - (time_hrs * 3600) - (time_mins * 60)

        string_hrs = str(time_hrs) if time_hrs > 10 else "0"+str(time_hrs)
        string_mins = str(time_mins) if time_mins > 10 else "0"+str(time_mins)
        string_secs = str(time_secs) if time_secs > 10 else "0"+str(time_secs)

        return string_hrs + ":" + string_mins + ":" + string_secs
        


if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    window = MainWindow()
    window.show()
    app.exec_()
