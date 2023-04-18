# -*- coding:utf-8 -*-


from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore

from ui_login import Ui_MainWindow as Ui_LoginPanel
from ui_control import Ui_MainWindow as Ui_ControlPanel


class LoginPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_LoginPanel()
        self.ui.setupUi(self)
        
        self.show() # 显示登录窗口
        self.__fix_window_size() # 固定窗口大小
        
        self.__login_check() # 登录
    
    def __fix_window_size(self, width = 1300, height = 630) -> None:
        self.resize(width, height)
        self.setFixedSize(width, height)

    def __login_check(self):
        self.ui.button_login.clicked.connect(self.__jump_to_control_panel)

    def __jump_to_control_panel(self):
        account = self.ui.box_user.currentText()
        password = self.ui.password.text()
        if account == "测试人员" and password == "123":
            self.control_panel = ControlPanel()
            self.close()
        elif account == "管理员" and password == "456":
            self.control_panel = ControlPanel()
            self.close()
        else: pass


class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        # self.setWindowFlag(QtCore.Qt.FramelessWindowHint)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground)
        
        self.show()


if __name__ == "__main__":
    app = QApplication([])
    login_panel = LoginPanel()
    app.exec_()