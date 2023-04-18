# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'design/control.ui'
#
# Created by: PyQt5 UI code generator 5.14.1
#
# WARNING! All changes made in this file will be lost!


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName("MainWindow")
        MainWindow.resize(1250, 993)
        MainWindow.setStyleSheet("")
        self.control_window = QtWidgets.QWidget(MainWindow)
        self.control_window.setStyleSheet("QWidget {\n"
"    background-color: rgb(56, 57, 60);\n"
"}\n"
"\n"
"\n"
"QFrame {\n"
"    background-color: rgb(86, 88, 93);\n"
"    border-radius: 10px;\n"
"}\n"
"\n"
"\n"
"QPushButton {\n"
"    background-color: rgb(114, 159, 207);\n"
"    font-size: 30px;\n"
"    font-weight: bold;\n"
"    color: white;\n"
"    border-radius: 10px;\n"
"}\n"
"QPushButton:disabled {\n"
"    color: gray;\n"
"    background-color: rgb(86, 88, 93);\n"
"}\n"
"\n"
"\n"
"QStackedWidget > QWidget {\n"
"    background-color: rgb(86, 88, 93);\n"
"    border-radius: 10px;\n"
"}\n"
"QStackedWidget QLabel {\n"
"    color: white;\n"
"    font-size: 30px;\n"
"    font-weight: bold;\n"
"}\n"
"QStackedWidget QPushButton {\n"
"    background-color: rgb(114, 159, 207);\n"
"    font-size: 25px;\n"
"}\n"
"QStackedWidget QRadioButton {\n"
"    font-size: 25px;\n"
"    color: white;\n"
"    background-color: rgb(86, 88, 93);\n"
"}")
        self.control_window.setObjectName("control_window")
        self.verticalLayout = QtWidgets.QVBoxLayout(self.control_window)
        self.verticalLayout.setContentsMargins(10, 10, 10, 10)
        self.verticalLayout.setSpacing(10)
        self.verticalLayout.setObjectName("verticalLayout")
        self.frame = QtWidgets.QFrame(self.control_window)
        self.frame.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame.setObjectName("frame")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.frame)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.horizontalLayout = QtWidgets.QHBoxLayout()
        self.horizontalLayout.setContentsMargins(0, -1, -1, -1)
        self.horizontalLayout.setSpacing(15)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.bt_setting = QtWidgets.QPushButton(self.frame)
        self.bt_setting.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bt_setting.sizePolicy().hasHeightForWidth())
        self.bt_setting.setSizePolicy(sizePolicy)
        self.bt_setting.setObjectName("bt_setting")
        self.horizontalLayout.addWidget(self.bt_setting)
        self.bt_init = QtWidgets.QPushButton(self.frame)
        self.bt_init.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bt_init.sizePolicy().hasHeightForWidth())
        self.bt_init.setSizePolicy(sizePolicy)
        self.bt_init.setObjectName("bt_init")
        self.horizontalLayout.addWidget(self.bt_init)
        self.bt_control = QtWidgets.QPushButton(self.frame)
        self.bt_control.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bt_control.sizePolicy().hasHeightForWidth())
        self.bt_control.setSizePolicy(sizePolicy)
        self.bt_control.setObjectName("bt_control")
        self.horizontalLayout.addWidget(self.bt_control)
        self.bt_log = QtWidgets.QPushButton(self.frame)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.bt_log.sizePolicy().hasHeightForWidth())
        self.bt_log.setSizePolicy(sizePolicy)
        self.bt_log.setObjectName("bt_log")
        self.horizontalLayout.addWidget(self.bt_log)
        self.verticalLayout_2.addLayout(self.horizontalLayout)
        self.verticalLayout.addWidget(self.frame)
        self.frame_2 = QtWidgets.QFrame(self.control_window)
        self.frame_2.setEnabled(True)
        self.frame_2.setFrameShape(QtWidgets.QFrame.StyledPanel)
        self.frame_2.setFrameShadow(QtWidgets.QFrame.Raised)
        self.frame_2.setObjectName("frame_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.frame_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.stackedWidget = QtWidgets.QStackedWidget(self.frame_2)
        self.stackedWidget.setStyleSheet("")
        self.stackedWidget.setObjectName("stackedWidget")
        self.page_2 = QtWidgets.QWidget()
        self.page_2.setObjectName("page_2")
        self.label_3 = QtWidgets.QLabel(self.page_2)
        self.label_3.setGeometry(QtCore.QRect(40, 450, 301, 61))
        self.label_3.setObjectName("label_3")
        self.label_4 = QtWidgets.QLabel(self.page_2)
        self.label_4.setGeometry(QtCore.QRect(20, 20, 301, 61))
        self.label_4.setObjectName("label_4")
        self.lineEdit = QtWidgets.QLineEdit(self.page_2)
        self.lineEdit.setGeometry(QtCore.QRect(60, 130, 113, 31))
        self.lineEdit.setObjectName("lineEdit")
        self.widget = QtWidgets.QWidget(self.page_2)
        self.widget.setGeometry(QtCore.QRect(30, 540, 661, 81))
        self.widget.setObjectName("widget")
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout(self.widget)
        self.horizontalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        self.radioButton = QtWidgets.QRadioButton(self.widget)
        self.radioButton.setObjectName("radioButton")
        self.horizontalLayout_2.addWidget(self.radioButton)
        self.radioButton_2 = QtWidgets.QRadioButton(self.widget)
        self.radioButton_2.setObjectName("radioButton_2")
        self.horizontalLayout_2.addWidget(self.radioButton_2)
        self.stackedWidget.addWidget(self.page_2)
        self.pg_init = QtWidgets.QWidget()
        self.pg_init.setObjectName("pg_init")
        self.label = QtWidgets.QLabel(self.pg_init)
        self.label.setGeometry(QtCore.QRect(10, 10, 191, 51))
        self.label.setObjectName("label")
        self.pushButton = QtWidgets.QPushButton(self.pg_init)
        self.pushButton.setGeometry(QtCore.QRect(40, 100, 221, 181))
        self.pushButton.setObjectName("pushButton")
        self.pushButton_2 = QtWidgets.QPushButton(self.pg_init)
        self.pushButton_2.setGeometry(QtCore.QRect(280, 100, 211, 81))
        self.pushButton_2.setObjectName("pushButton_2")
        self.pushButton_3 = QtWidgets.QPushButton(self.pg_init)
        self.pushButton_3.setGeometry(QtCore.QRect(280, 200, 211, 81))
        self.pushButton_3.setObjectName("pushButton_3")
        self.label_2 = QtWidgets.QLabel(self.pg_init)
        self.label_2.setGeometry(QtCore.QRect(10, 420, 191, 51))
        self.label_2.setObjectName("label_2")
        self.stackedWidget.addWidget(self.pg_init)
        self.page_3 = QtWidgets.QWidget()
        self.page_3.setObjectName("page_3")
        self.stackedWidget.addWidget(self.page_3)
        self.page_4 = QtWidgets.QWidget()
        self.page_4.setObjectName("page_4")
        self.stackedWidget.addWidget(self.page_4)
        self.verticalLayout_3.addWidget(self.stackedWidget)
        self.verticalLayout.addWidget(self.frame_2)
        self.verticalLayout.setStretch(0, 1)
        self.verticalLayout.setStretch(1, 8)
        MainWindow.setCentralWidget(self.control_window)

        self.retranslateUi(MainWindow)
        self.stackedWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "MainWindow"))
        self.bt_setting.setText(_translate("MainWindow", "设置"))
        self.bt_init.setText(_translate("MainWindow", "初始化"))
        self.bt_control.setText(_translate("MainWindow", "操作"))
        self.bt_log.setText(_translate("MainWindow", "日志"))
        self.label_3.setText(_translate("MainWindow", "步进电机"))
        self.label_4.setText(_translate("MainWindow", "USBCAN卡"))
        self.radioButton.setText(_translate("MainWindow", "协议位置模式"))
        self.radioButton_2.setText(_translate("MainWindow", "速度模式"))
        self.label.setText(_translate("MainWindow", "USBCAN卡"))
        self.pushButton.setText(_translate("MainWindow", "打开设备"))
        self.pushButton_2.setText(_translate("MainWindow", "打开通道0"))
        self.pushButton_3.setText(_translate("MainWindow", "打开通道1"))
        self.label_2.setText(_translate("MainWindow", "步进电机"))
