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
        MainWindow.resize(800, 589)
        self.control_window = QtWidgets.QWidget(MainWindow)
        self.control_window.setObjectName("control_window")
        self.pushButton = QtWidgets.QPushButton(self.control_window)
        self.pushButton.setGeometry(QtCore.QRect(90, 70, 117, 32))
        self.pushButton.setObjectName("pushButton")
        MainWindow.setCentralWidget(self.control_window)

        self.retranslateUi(MainWindow)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        _translate = QtCore.QCoreApplication.translate
        MainWindow.setWindowTitle(_translate("MainWindow", "控制面板"))
        self.pushButton.setText(_translate("MainWindow", "PushButton"))
