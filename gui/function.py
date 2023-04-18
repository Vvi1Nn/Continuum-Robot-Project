# -*- coding:utf-8 -*-


from PyQt5.QtWidgets import QApplication, QMainWindow
from PyQt5 import QtCore

from ui_login import Ui_MainWindow as Ui_LoginPanel
from ui_control import Ui_MainWindow as Ui_ControlPanel

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from usbcan.function import *
from motor.function import *


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
            self.close()
            self.control_panel = ControlPanel(account)
        elif account == "管理员" and password == "9":
            self.close()
            self.control_panel = ControlPanel(account)
        else: pass

class ControlPanel(QMainWindow):
    def __init__(self, account) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        self.show()
        
        self.__account = account
        self.__usbcan_0 = None
        self.__usbcan_1 = None

        self.set_menu_jumping() # 设置菜单的按钮跳转

        self.set_default_show() # 根据权限设置显示内容

        self.set_usbcan_jumping() # 设置usbcan初始化部分的按钮跳转
        self.set_motor_jumping() # 设置motor初始化部分的按钮跳转


    def set_menu_jumping(self) -> None:
        self.ui.bt_init.clicked.connect(self.__jump_to_init_page)
        self.ui.bt_control.clicked.connect(self.__jump_to_control_page)
        self.ui.bt_log.clicked.connect(self.__jump_to_log_page)
    def __jump_to_init_page(self) -> None:
        self.ui.stackedWidget.setCurrentIndex(0)
    def __jump_to_control_page(self) -> None:
        self.ui.stackedWidget.setCurrentIndex(1)
    def __jump_to_log_page(self) -> None:
        self.ui.stackedWidget.setCurrentIndex(2)

    def set_default_show(self) -> None:
        self.ui.bt_init.setEnabled(True) # 初始化按钮激活
        self.ui.bt_control.setEnabled(False) # 控制按钮失效
        self.ui.bt_log.setEnabled(False) # 日志按钮失效

        self.__jump_to_init_page() # 显示初始化页面
        self.__set_default_usbcan() # usbcan部分
        is_admin = False if self.__account == "测试人员" else True
        self.__set_default_motor(is_admin) # motor部分
    def __set_default_usbcan(self) -> None:
        self.ui.group_usbcan.setEnabled(True) # usbcan部分 激活
        self.ui.bt_open.setEnabled(True) # 只激活打开设备按钮
        self.ui.bx_rate0.setEnabled(False)
        self.ui.bx_rate1.setEnabled(False)
        self.ui.bt_channel0.setEnabled(False)
        self.ui.bt_channel1.setEnabled(False)
        self.ui.bt_reset0.setEnabled(False)
        self.ui.bt_reset1.setEnabled(False)
        self.ui.bt_close.setEnabled(False)
    def __set_default_motor(self, is_admin) -> None:
        self.ui.r_pos.setChecked(True) # 选择位置模式
        if is_admin:
            self.ui.r_pos.setEnabled(False)
            self.ui.r_vel.setEnabled(False)
            self.ui.tx_speed.setEnabled(False)
            self.ui.le_speed.setEnabled(False)
            self.ui.bt_launch.setEnabled(False)
        else:
            self.ui.r_pos.setEnabled(False)
            self.ui.r_vel.setEnabled(False)
            self.ui.tx_speed.setEnabled(False)
            self.ui.le_speed.setEnabled(False)
            self.ui.tx_acc.setEnabled(False)
            self.ui.le_acc.setEnabled(False)
            self.ui.tx_dec.setEnabled(False)
            self.ui.le_dec.setEnabled(False)
            self.ui.tx_vel.setEnabled(False)
            self.ui.le_vel.setEnabled(False)
            self.ui.bt_save.setEnabled(False)
            self.ui.bt_launch.setEnabled(False)

    def set_usbcan_jumping(self) -> None:
        self.ui.bt_open.clicked.connect(self.__interface_open_usbcan)
        self.ui.bt_channel0.clicked.connect(self.__interface_start_channel_0)
        self.ui.bt_channel1.clicked.connect(self.__interface_start_channel_1)
        self.ui.bt_reset0.clicked.connect(self.__interface_reset_cannel_0)
        self.ui.bt_reset1.clicked.connect(self.__interface_reset_cannel_1)
        self.ui.bt_close.clicked.connect(self.__interface_close_usbcan)
    def __interface_open_usbcan(self) -> None:
        UsbCan.open_device()
        if UsbCan.is_open:
            self.ui.bt_open.setEnabled(False)
            self.ui.bt_open.setText("设备已打开")
            self.ui.bt_close.setEnabled(True)
            self.ui.bt_close.setText("关闭设备")
            self.ui.bx_rate0.setEnabled(True)
            self.ui.bx_rate1.setEnabled(True)
            self.ui.bt_channel0.setEnabled(True)
            self.ui.bt_channel1.setEnabled(True)
        else: pass
    def __interface_start_channel_0(self) -> None:
        self.__usbcan_0 = UsbCan("0", self.ui.bx_rate0.currentText())
        if self.__usbcan_0.is_init and self.__usbcan_0.is_start:
            self.ui.bx_rate0.setEnabled(False)
            self.ui.bt_channel0.setEnabled(False)
            self.ui.bt_channel0.setText("通道0已打开")
            self.ui.bt_reset0.setEnabled(True)
            Motor.config(self.__usbcan_0) # 保存电机的参数
            self.ui.bt_save.setEnabled(True)
            self.ui.bt_launch.setEnabled(True)
    def __interface_start_channel_1(self) -> None:
        self.__usbcan_1 = UsbCan("1", self.ui.bx_rate1.currentText())
        if self.__usbcan_1.is_init and self.__usbcan_1.is_start:
            self.ui.bx_rate1.setEnabled(False)
            self.ui.bt_channel1.setEnabled(False)
            self.ui.bt_channel1.setText("通道1已打开")
            self.ui.bt_reset1.setEnabled(True)
    def __interface_reset_cannel_0(self) -> None:
        self.__usbcan_0.reset_can()
    def __interface_reset_cannel_1(self) -> None:
        self.__usbcan_1.reset_can()
    def __interface_close_usbcan(self) -> None:
        UsbCan.close_device()
        if not UsbCan.is_open:
            self.ui.bt_close.setEnabled(False)
            self.ui.bt_close.setText("设备已关闭")
            self.ui.bx_rate0.setEnabled(False)
            self.ui.bx_rate1.setEnabled(False)
            self.ui.bt_channel0.setEnabled(False)
            self.ui.bt_channel0.setText("打开通道0")
            self.ui.bt_channel1.setEnabled(False)
            self.ui.bt_channel1.setText("打开通道1")
            self.ui.bt_reset0.setEnabled(False)
            self.ui.bt_reset1.setEnabled(False)
            self.ui.bt_open.setEnabled(True)
            self.ui.bt_open.setText("打开设备")
            self.ui.tx_acc.setEnabled(False)
            self.ui.le_acc.setEnabled(False)
            self.ui.tx_dec.setEnabled(False)
            self.ui.le_dec.setEnabled(False)
            self.ui.tx_vel.setEnabled(False)
            self.ui.le_vel.setEnabled(False)
            self.ui.bt_save.setEnabled(False)
            self.ui.bt_launch.setEnabled(False)
        else: pass

    def set_motor_jumping(self):
        self.ui.bt_save.clicked.connect(self.__interface_save_config)
        self.ui.bt_launch.clicked.connect(self.__interface_launch_motor)
    def __interface_save_config(self):
        acc = self.ui.le_acc.text()
        dec = self.ui.le_dec.text()
        vel = self.ui.le_vel.text()
        if acc == "": acc = "1000"
        if dec == "": dec = "10000"
        if vel == "": vel = "100"
        self.ui.le_acc.clear()
        self.ui.le_dec.clear()
        self.ui.le_vel.clear()
        self.ui.le_acc.setPlaceholderText("当前值{}".format(acc))
        self.ui.le_dec.setPlaceholderText("当前值{}".format(dec))
        self.ui.le_vel.setPlaceholderText("当前值{}".format(vel))
        Motor.config(device=self.__usbcan_0, acceleration=int(acc), deceleration=int(dec), velocity=int(vel))
    def __interface_launch_motor(self):
        self.ui.bt_save.setEnabled(False)
        self.ui.tx_acc.setEnabled(False)
        self.ui.le_acc.setEnabled(False)
        self.ui.tx_dec.setEnabled(False)
        self.ui.le_dec.setEnabled(False)
        self.ui.tx_vel.setEnabled(False)
        self.ui.le_vel.setEnabled(False)
        self.__motor_1 = Motor(1)
        self.__motor_2 = Motor(2)
        # self.__motor_3 = Motor(3)
        # self.__motor_4 = Motor(4)
        # self.__motor_5 = Motor(5)
        # self.__motor_6 = Motor(6)
        # self.__motor_7 = Motor(7)
        # self.__motor_8 = Motor(8)
        # self.__motor_9 = Motor(9)
        # self.__motor_10 = Motor(10)
        self.ui.bt_launch.setEnabled(False)
        self.ui.bt_launch.setText("成功")
    
