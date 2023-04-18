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
        
        self.__is_admin = False if account == "测试人员" else True
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
        self.__enable_menu(True, False, False) # menu 初始化激活 控制失效 日志失效
        
        self.__jump_to_init_page() # 显示初始化页面

        self.__enable_open_device(True, "打开设备") # usbcan
        self.__enable_channel0(False, "打开通道0", False, "重置通道0")
        self.__enable_channel1(False, "打开通道1", False, "重置通道1")
        self.__enable_close_device(False, "关闭设备")

        self.__enable_default() # motor
        self.__enable_set_param(False, "默认值1000", "默认值10000", "默认值100")
        self.__enable_save_param(False)
        self.__enable_init_motor(False, "生效")
    def __enable_menu(self, init, control, log):
        self.ui.bt_init.setEnabled(init)
        self.ui.bt_control.setEnabled(control)
        self.ui.bt_log.setEnabled(log)
    def __enable_open_device(self, flag, text):
        self.ui.bt_open.setEnabled(flag)
        self.ui.bt_open.setText(text)
    def __enable_channel0(self, flag_open, text_open, flag_reset, text_reset):
        self.ui.bx_rate0.setEnabled(flag_open)
        self.ui.bt_channel0.setEnabled(flag_open)
        self.ui.bt_channel0.setText(text_open)
        self.ui.bt_reset0.setEnabled(flag_reset)
        self.ui.bt_reset0.setText(text_reset)
    def __enable_channel1(self, flag_open, text_open, flag_reset, text_reset):
        self.ui.bx_rate1.setEnabled(flag_open)
        self.ui.bt_channel1.setEnabled(flag_open)
        self.ui.bt_channel1.setText(text_open)
        self.ui.bt_reset1.setEnabled(flag_reset)
        self.ui.bt_reset1.setText(text_reset)
    def __enable_close_device(self, flag, text):
        self.ui.bt_close.setEnabled(flag)
        self.ui.bt_close.setText(text)
    def __enable_default(self):
        self.ui.r_pos.setChecked(True) # 选择位置模式
        self.ui.r_pos.setEnabled(False)
        self.ui.r_vel.setEnabled(False) # 禁用模式选择
        self.ui.tx_speed.setEnabled(False)
        self.ui.le_speed.setEnabled(False) # 禁用速度设置
    def __enable_set_param(self, flag, text0, text1, text2):
        self.ui.le_acc.clear()
        self.ui.tx_acc.setEnabled(flag)
        self.ui.le_acc.setEnabled(flag)
        self.ui.le_acc.setPlaceholderText(text0)
        self.ui.le_dec.clear()
        self.ui.tx_dec.setEnabled(flag)
        self.ui.le_dec.setEnabled(flag)
        self.ui.le_dec.setPlaceholderText(text1)
        self.ui.le_vel.clear()
        self.ui.tx_vel.setEnabled(flag)
        self.ui.le_vel.setEnabled(flag)
        self.ui.le_vel.setPlaceholderText(text2)
    def __enable_save_param(self, flag):
        self.ui.bt_save.setEnabled(flag)
    def __enable_init_motor(self, flag, text):
        self.ui.bt_launch.setEnabled(flag)
        self.ui.bt_launch.setText(text)
    

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
            self.__enable_open_device(False, "设备已打开")
            self.__enable_close_device(True, "关闭设备")
            self.__enable_channel0(True, "打开通道0", False, "重置通道0")
            self.__enable_channel1(True, "打开通道1", False, "重置通道1")
    def __interface_start_channel_0(self) -> None:
        self.__usbcan_0 = UsbCan("0", self.ui.bx_rate0.currentText())
        if self.__usbcan_0.is_init and self.__usbcan_0.is_start:
            self.__enable_channel0(False, "通道0已打开", True, "重置通道0")
            if self.__is_admin:
                self.__enable_save_param(True) # 激活 保存
                self.__enable_set_param(True, "默认值1000", "默认值10000", "默认值100") # 激活输入
            else:
                Motor.config(self.__usbcan_0) # 直接保存默认参数
            self.__enable_init_motor(True, "生效") # 激活 生效
    def __interface_start_channel_1(self) -> None:
        self.__usbcan_1 = UsbCan("1", self.ui.bx_rate1.currentText())
        if self.__usbcan_1.is_init and self.__usbcan_1.is_start:
            self.__enable_channel1(False, "通道1已打开", True, "重置通道1")
            pass # 其他的配置 后续补齐
    def __interface_reset_cannel_0(self) -> None:
        self.__usbcan_0.reset_can()
    def __interface_reset_cannel_1(self) -> None:
        self.__usbcan_1.reset_can()
    def __interface_close_usbcan(self) -> None:
        UsbCan.close_device()
        if not UsbCan.is_open:
            self.__enable_open_device(True, "打开设备")
            self.__enable_channel0(False, "打开通道0", False, "重置通道0")
            self.__enable_channel1(False, "打开通道1", False, "重置通道1")
            self.__enable_close_device(False, "设备已关闭")
            self.__enable_set_param(False, "默认值1000", "默认值10000", "默认值100")
            self.__enable_save_param(False)
            self.__enable_init_motor(False, "生效")
    def set_motor_jumping(self):
        self.ui.bt_save.clicked.connect(self.__interface_save_config)
        self.ui.bt_launch.clicked.connect(self.__interface_init_motor)
    def __interface_save_config(self):
        acc = self.ui.le_acc.text()
        if acc == "": acc = "1000"
        dec = self.ui.le_dec.text()
        if dec == "": dec = "10000"
        vel = self.ui.le_vel.text()
        if vel == "": vel = "100"
        Motor.config(device=self.__usbcan_0, acceleration=int(acc), deceleration=int(dec), velocity=int(vel))
        self.__enable_set_param(True, "当前值{}".format(Motor.acceleration), "当前值{}".format(Motor.deceleration), "当前值{}".format(Motor.velocity))
    def __interface_init_motor(self):
        self.__enable_set_param(False, "当前值{}".format(Motor.acceleration), "当前值{}".format(Motor.deceleration), "当前值{}".format(Motor.velocity))
        self.__enable_save_param(False)
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
        self.__enable_init_motor(False, "成功")
    
