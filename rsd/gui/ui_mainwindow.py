# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'ui_mainwindow.ui',
# licensing of 'ui_mainwindow.ui' applies.
#
# Created: Wed Nov 27 10:58:34 2019
#      by: pyside2-uic  running on PySide2 5.13.2
#
# WARNING! All changes made in this file will be lost!

from PySide2 import QtCore, QtGui, QtWidgets

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(980, 535)
        MainWindow.setStyleSheet("QLabel\n"
"{\n"
"font:  20px;\n"
"}\n"
"\n"
"QPushButton\n"
"{\n"
"font: 20px\n"
"}\n"
"\n"
"QPushButton:disabled {\n"
"background-color:#888888;\n"
"}\n"
"\n"
"#leftPanel\n"
"{\n"
"border: 1px solid;\n"
"}\n"
"\n"
"#middlePanel_2\n"
"{\n"
"border: 1px solid;\n"
"}\n"
"\n"
"#rightPanel\n"
"{\n"
"border: 1px solid;\n"
"}")
        self.centralwidget = QtWidgets.QWidget(MainWindow)
        self.centralwidget.setStyleSheet("QFrame[frameShape=\"5\"]\n"
"{\n"
"    border: none;\n"
"    background: grey;\n"
"}\n"
"\n"
"\n"
"\n"
"")
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.centralwidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.leftPanel = QtWidgets.QWidget(self.centralwidget)
        self.leftPanel.setStyleSheet("")
        self.leftPanel.setObjectName("leftPanel")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.leftPanel)
        self.verticalLayout.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout.setObjectName("verticalLayout")
        self.label_7 = QtWidgets.QLabel(self.leftPanel)
        self.label_7.setStyleSheet("font: 30px")
        self.label_7.setObjectName("label_7")
        self.verticalLayout.addWidget(self.label_7)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.label_4 = QtWidgets.QLabel(self.leftPanel)
        self.label_4.setLayoutDirection(QtCore.Qt.LeftToRight)
        self.label_4.setStyleSheet("QLabel\n"
"{\n"
"alignment: AlignHLeft\n"
"}")
        self.label_4.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_4.setObjectName("label_4")
        self.horizontalLayout_2.addWidget(self.label_4)
        self.statusLabel = QtWidgets.QLabel(self.leftPanel)
        self.statusLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.statusLabel.setObjectName("statusLabel")
        self.horizontalLayout_2.addWidget(self.statusLabel)
        self.verticalLayout.addLayout(self.horizontalLayout_2)
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        self.label_13 = QtWidgets.QLabel(self.leftPanel)
        self.label_13.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_13.setObjectName("label_13")
        self.horizontalLayout_3.addWidget(self.label_13)
        self.packmlStateLabel = QtWidgets.QLabel(self.leftPanel)
        self.packmlStateLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.packmlStateLabel.setObjectName("packmlStateLabel")
        self.horizontalLayout_3.addWidget(self.packmlStateLabel)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout.addItem(spacerItem)
        self.startButton = QtWidgets.QPushButton(self.leftPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.startButton.sizePolicy().hasHeightForWidth())
        self.startButton.setSizePolicy(sizePolicy)
        self.startButton.setStyleSheet("QPushButton\n"
"{\n"
"color: rgb(0,0, 0);\n"
"background-color: #00FF00;\n"
"}\n"
"\n"
"")
        self.startButton.setObjectName("startButton")
        self.verticalLayout.addWidget(self.startButton)
        self.pauseButton = QtWidgets.QPushButton(self.leftPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.pauseButton.sizePolicy().hasHeightForWidth())
        self.pauseButton.setSizePolicy(sizePolicy)
        self.pauseButton.setStyleSheet("backgrond: black")
        self.pauseButton.setObjectName("pauseButton")
        self.verticalLayout.addWidget(self.pauseButton)
        self.stopButton = QtWidgets.QPushButton(self.leftPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.stopButton.sizePolicy().hasHeightForWidth())
        self.stopButton.setSizePolicy(sizePolicy)
        self.stopButton.setObjectName("stopButton")
        self.verticalLayout.addWidget(self.stopButton)
        self.abortButton = QtWidgets.QPushButton(self.leftPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.abortButton.sizePolicy().hasHeightForWidth())
        self.abortButton.setSizePolicy(sizePolicy)
        self.abortButton.setStyleSheet("QPushButton\n"
"{\n"
"color: rgb(0,0, 0);\n"
"background-color: #FF0000;\n"
"}\n"
"\n"
"")
        self.abortButton.setObjectName("abortButton")
        self.verticalLayout.addWidget(self.abortButton)
        self.resetButton = QtWidgets.QPushButton(self.leftPanel)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.resetButton.sizePolicy().hasHeightForWidth())
        self.resetButton.setSizePolicy(sizePolicy)
        self.resetButton.setObjectName("resetButton")
        self.verticalLayout.addWidget(self.resetButton)
        self.horizontalLayout.addWidget(self.leftPanel)
        self.middlePanel_2 = QtWidgets.QWidget(self.centralwidget)
        self.middlePanel_2.setStyleSheet("")
        self.middlePanel_2.setObjectName("middlePanel_2")
        self.middlePanel = QtWidgets.QVBoxLayout(self.middlePanel_2)
        self.middlePanel.setContentsMargins(0, 0, 0, 0)
        self.middlePanel.setObjectName("middlePanel")
        self.label_6 = QtWidgets.QLabel(self.middlePanel_2)
        font = QtGui.QFont()
        font.setPointSize(-1)
        font.setWeight(50)
        font.setItalic(False)
        font.setBold(False)
        self.label_6.setFont(font)
        self.label_6.setStyleSheet("font: 30px")
        self.label_6.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_6.setObjectName("label_6")
        self.middlePanel.addWidget(self.label_6)
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.label_2 = QtWidgets.QLabel(self.middlePanel_2)
        self.label_2.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_2.setObjectName("label_2")
        self.horizontalLayout_4.addWidget(self.label_2)
        self.orderIDLabel = QtWidgets.QLabel(self.middlePanel_2)
        self.orderIDLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.orderIDLabel.setObjectName("orderIDLabel")
        self.horizontalLayout_4.addWidget(self.orderIDLabel)
        self.middlePanel.addLayout(self.horizontalLayout_4)
        self.horizontalLayout_8 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_8.setObjectName("horizontalLayout_8")
        self.label_14 = QtWidgets.QLabel(self.middlePanel_2)
        self.label_14.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_14.setObjectName("label_14")
        self.horizontalLayout_8.addWidget(self.label_14)
        self.numRedLabel = QtWidgets.QLabel(self.middlePanel_2)
        self.numRedLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.numRedLabel.setObjectName("numRedLabel")
        self.horizontalLayout_8.addWidget(self.numRedLabel)
        self.middlePanel.addLayout(self.horizontalLayout_8)
        self.horizontalLayout_10 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_10.setObjectName("horizontalLayout_10")
        self.label_17 = QtWidgets.QLabel(self.middlePanel_2)
        self.label_17.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_17.setObjectName("label_17")
        self.horizontalLayout_10.addWidget(self.label_17)
        self.numYellowLabel = QtWidgets.QLabel(self.middlePanel_2)
        self.numYellowLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.numYellowLabel.setObjectName("numYellowLabel")
        self.horizontalLayout_10.addWidget(self.numYellowLabel)
        self.middlePanel.addLayout(self.horizontalLayout_10)
        self.horizontalLayout_9 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_9.setObjectName("horizontalLayout_9")
        self.label_19 = QtWidgets.QLabel(self.middlePanel_2)
        self.label_19.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_19.setObjectName("label_19")
        self.horizontalLayout_9.addWidget(self.label_19)
        self.numBlueLabel = QtWidgets.QLabel(self.middlePanel_2)
        self.numBlueLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.numBlueLabel.setObjectName("numBlueLabel")
        self.horizontalLayout_9.addWidget(self.numBlueLabel)
        self.middlePanel.addLayout(self.horizontalLayout_9)
        spacerItem1 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.middlePanel.addItem(spacerItem1)
        self.horizontalLayout.addWidget(self.middlePanel_2)
        self.rightPanel = QtWidgets.QWidget(self.centralwidget)
        self.rightPanel.setStyleSheet("")
        self.rightPanel.setObjectName("rightPanel")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.rightPanel)
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.label_3 = QtWidgets.QLabel(self.rightPanel)
        self.label_3.setStyleSheet("font: 30px\n"
"")
        self.label_3.setObjectName("label_3")
        self.verticalLayout_2.addWidget(self.label_3)
        self.horizontalLayout_7 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_7.setObjectName("horizontalLayout_7")
        self.label_11 = QtWidgets.QLabel(self.rightPanel)
        self.label_11.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_11.setObjectName("label_11")
        self.horizontalLayout_7.addWidget(self.label_11)
        self.qualityLabel = QtWidgets.QLabel(self.rightPanel)
        self.qualityLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.qualityLabel.setObjectName("qualityLabel")
        self.horizontalLayout_7.addWidget(self.qualityLabel)
        self.verticalLayout_2.addLayout(self.horizontalLayout_7)
        self.horizontalLayout_6 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_6.setObjectName("horizontalLayout_6")
        self.label_9 = QtWidgets.QLabel(self.rightPanel)
        self.label_9.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_9.setObjectName("label_9")
        self.horizontalLayout_6.addWidget(self.label_9)
        self.performanceLabel = QtWidgets.QLabel(self.rightPanel)
        self.performanceLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.performanceLabel.setObjectName("performanceLabel")
        self.horizontalLayout_6.addWidget(self.performanceLabel)
        self.verticalLayout_2.addLayout(self.horizontalLayout_6)
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.label_8 = QtWidgets.QLabel(self.rightPanel)
        self.label_8.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_8.setObjectName("label_8")
        self.horizontalLayout_5.addWidget(self.label_8)
        self.availabilityLabel = QtWidgets.QLabel(self.rightPanel)
        self.availabilityLabel.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.availabilityLabel.setObjectName("availabilityLabel")
        self.horizontalLayout_5.addWidget(self.availabilityLabel)
        self.verticalLayout_2.addLayout(self.horizontalLayout_5)
        self.horizontalLayout_11 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_11.setObjectName("horizontalLayout_11")
        self.label_5 = QtWidgets.QLabel(self.rightPanel)
        self.label_5.setAlignment(QtCore.Qt.AlignLeading|QtCore.Qt.AlignLeft|QtCore.Qt.AlignVCenter)
        self.label_5.setObjectName("label_5")
        self.horizontalLayout_11.addWidget(self.label_5)
        self.label = QtWidgets.QLabel(self.rightPanel)
        self.label.setAlignment(QtCore.Qt.AlignRight|QtCore.Qt.AlignTrailing|QtCore.Qt.AlignVCenter)
        self.label.setObjectName("label")
        self.horizontalLayout_11.addWidget(self.label)
        self.verticalLayout_2.addLayout(self.horizontalLayout_11)
        spacerItem2 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Expanding)
        self.verticalLayout_2.addItem(spacerItem2)
        self.horizontalLayout.addWidget(self.rightPanel)
        MainWindow.setCentralWidget(self.centralwidget)
        self.statusbar = QtWidgets.QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QtWidgets.QApplication.translate("MainWindow", "MainWindow", None, -1))
        self.label_7.setText(QtWidgets.QApplication.translate("MainWindow", "System control", None, -1))
        self.label_4.setText(QtWidgets.QApplication.translate("MainWindow", "Status", None, -1))
        self.statusLabel.setText(QtWidgets.QApplication.translate("MainWindow", "Picking from feeder", None, -1))
        self.label_13.setText(QtWidgets.QApplication.translate("MainWindow", "PackML state", None, -1))
        self.packmlStateLabel.setText(QtWidgets.QApplication.translate("MainWindow", "Stopped", None, -1))
        self.startButton.setText(QtWidgets.QApplication.translate("MainWindow", "Start", None, -1))
        self.pauseButton.setText(QtWidgets.QApplication.translate("MainWindow", "Hold", None, -1))
        self.stopButton.setText(QtWidgets.QApplication.translate("MainWindow", "Stop", None, -1))
        self.abortButton.setText(QtWidgets.QApplication.translate("MainWindow", "Abort", None, -1))
        self.resetButton.setText(QtWidgets.QApplication.translate("MainWindow", "Reset", None, -1))
        self.label_6.setText(QtWidgets.QApplication.translate("MainWindow", "Order status", None, -1))
        self.label_2.setText(QtWidgets.QApplication.translate("MainWindow", "Current order ID", None, -1))
        self.orderIDLabel.setText(QtWidgets.QApplication.translate("MainWindow", "1337", None, -1))
        self.label_14.setText(QtWidgets.QApplication.translate("MainWindow", "#red", None, -1))
        self.numRedLabel.setText(QtWidgets.QApplication.translate("MainWindow", "1/2", None, -1))
        self.label_17.setText(QtWidgets.QApplication.translate("MainWindow", "#yellow", None, -1))
        self.numYellowLabel.setText(QtWidgets.QApplication.translate("MainWindow", "0/1", None, -1))
        self.label_19.setText(QtWidgets.QApplication.translate("MainWindow", "#blue", None, -1))
        self.numBlueLabel.setText(QtWidgets.QApplication.translate("MainWindow", "1/5", None, -1))
        self.label_3.setText(QtWidgets.QApplication.translate("MainWindow", "OEE statistics", None, -1))
        self.label_11.setText(QtWidgets.QApplication.translate("MainWindow", "Quality", None, -1))
        self.qualityLabel.setText(QtWidgets.QApplication.translate("MainWindow", "0.65", None, -1))
        self.label_9.setText(QtWidgets.QApplication.translate("MainWindow", "Performance", None, -1))
        self.performanceLabel.setText(QtWidgets.QApplication.translate("MainWindow", "0.95", None, -1))
        self.label_8.setText(QtWidgets.QApplication.translate("MainWindow", "Availability", None, -1))
        self.availabilityLabel.setText(QtWidgets.QApplication.translate("MainWindow", "0.3", None, -1))
        self.label_5.setText(QtWidgets.QApplication.translate("MainWindow", "Uptime", None, -1))
        self.label.setText(QtWidgets.QApplication.translate("MainWindow", "02:34:13", None, -1))

