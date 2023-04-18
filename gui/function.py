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

    def __login_check(self) -> None:
        self.ui.button_login.clicked.connect(self.__jump_to_control_panel)

    def __jump_to_control_panel(self) -> None:
        account = self.ui.box_user.currentText()
        password = self.ui.password.text()
        if account == "测试人员" and password == "1":
            self.control_panel = ControlPanel(account)
            self.close()
        elif account == "管理员" and password == "9":
            self.control_panel = ControlPanel(account)
            self.close()
        else: pass


class ControlPanel(QMainWindow):
    def __init__(self, account) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        self.show()
        self.__account = account

        self.set_menu_jumping() # 设置菜单的按钮跳转
        
        self.set_default_show() # 根据权限设置显示内容
        
        

    def set_menu_jumping(self):
        self.ui.bt_init.clicked.connect(self.__jump_to_init_page)
        self.ui.bt_control.clicked.connect(self.__jump_to_control_page)
        self.ui.bt_log.clicked.connect(self.__jump_to_log_page)
    def __jump_to_init_page(self):
        self.ui.stackedWidget.setCurrentIndex(0)
    def __jump_to_control_page(self):
        self.ui.stackedWidget.setCurrentIndex(1)
    def __jump_to_log_page(self):
        self.ui.stackedWidget.setCurrentIndex(2)

    def set_default_show(self):
        self.ui.bt_init.setEnabled(True) # 初始化按钮激活
        self.ui.bt_control.setEnabled(False) # 控制按钮失效
        self.ui.bt_log.setEnabled(False) # 日志按钮失效

        self.__jump_to_init_page() # 显示初始化页面
        self.__set_default_usbcan() # usbcan部分
        is_admin = False if self.__account == "测试人员" else True
        self.__set_default_motor(is_admin) # motor部分
    def __set_default_usbcan(self):
        self.ui.group_usbcan.setEnabled(True) # usbcan部分 激活
        self.ui.bt_open.setEnabled(True) # 只激活打开设备按钮
        self.ui.bx_rate0.setEnabled(False)
        self.ui.bx_rate1.setEnabled(False)
        self.ui.bt_channel0.setEnabled(False)
        self.ui.bt_channel1.setEnabled(False)
        self.ui.bt_reset.setEnabled(False)
        self.ui.bt_close.setEnabled(False)
    def __set_default_motor(self, is_admin):
        self.ui.r_pos.setChecked(True) # 选择位置模式
        if is_admin:
            self.ui.group_motor.setEnabled(True) # 激活
            self.ui.r_pos.setEnabled(False)
            self.ui.r_vel.setEnabled(False)
            self.ui.tx_speed.setEnabled(False)
            self.ui.le_speed.setEnabled(False)
        else:
            self.ui.group_motor.setEnabled(False) # 全部失效


if __name__ == "__main__":
    app = QApplication([])
    win = LoginPanel()
    app.exec_()