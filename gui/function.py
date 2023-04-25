# -*- coding:utf-8 -*-


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QThread, QMutex, QObject, pyqtSignal

from ui_login import Ui_MainWindow as Ui_LoginPanel
from ui_control import Ui_MainWindow as Ui_ControlPanel

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from usbcan.function import UsbCan
from canopen.processor import CanOpenBusProcessor
from canopen.motor import Motor
from canopen.io_module import IoModule


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
        
        self.is_admin = False if account == "测试人员" else True
        
        self.usbcan_0 = UsbCan(channel="0")
        self.usbcan_1 = UsbCan(channel="1")
        
        CanOpenBusProcessor.link_device(self.usbcan_0)
        
        self.motor_checked_num = 0
        self.motor_ischeck_num = 1
        self.motor_1 = Motor(1)
        self.motor_2 = Motor(2)
        self.motor_3 = Motor(3)
        self.motor_4 = Motor(4)
        self.motor_5 = Motor(5)
        self.motor_6 = Motor(6)
        self.motor_7 = Motor(7)
        self.motor_8 = Motor(8)
        self.motor_9 = Motor(9)
        self.motor_10 = Motor(10)
        
        self.io_module_checked_num = 0
        self.io_module_ischeck_num = 1
        self.io_module = IoModule(11)

        self.initial_status() # 根据权限设置显示内容
        
        self.set_menu_jumping() # 设置菜单的按钮跳转
        
        self.set_usbcan_jumping() # 设置USBCAN初始化部分的按钮跳转
        
        self.set_motor_jumping() # 设置电机初始化部分的按钮跳转

        self.set_io_jumping() # 设置IO初始化部分按钮跳转


    # 菜单
    def enable_menu(self, init, control, log):
        self.ui.bt_init.setEnabled(init)
        self.ui.bt_control.setEnabled(control)
        self.ui.bt_log.setEnabled(log)
    # USBCAN
    def enable_open_device(self, flag, text=None):
        self.ui.bt_open.setEnabled(flag)
        if text != None: self.ui.bt_open.setText(text)
    def enable_channel0(self, flag_open, flag_reset, text_open=None, text_reset=None):
        self.ui.bx_rate0.setEnabled(flag_open)
        self.ui.bt_channel0.setEnabled(flag_open)
        if text_open != None: self.ui.bt_channel0.setText(text_open)
        self.ui.bt_reset0.setEnabled(flag_reset)
        if text_reset != None: self.ui.bt_reset0.setText(text_reset)
    def enable_channel1(self, flag_open, flag_reset, text_open=None, text_reset=None):
        self.ui.bx_rate1.setEnabled(flag_open)
        self.ui.bt_channel1.setEnabled(flag_open)
        if text_open != None: self.ui.bt_channel1.setText(text_open)
        self.ui.bt_reset1.setEnabled(flag_reset)
        if text_reset != None: self.ui.bt_reset1.setText(text_reset)
    def enable_close_device(self, flag, text=None):
        self.ui.bt_close.setEnabled(flag)
        if text != None: self.ui.bt_close.setText(text)
    # 电机
    def enable_check_all(self, flag, text=None):
        self.ui.check_all.setEnabled(flag)
        if text != None: self.ui.check_all.setText(text)
    def enable_check_1(self, flag, text=None):
        self.ui.check_1.setEnabled(flag)
        if text != None: self.ui.check_1.setText(text)
    def enable_check_2(self, flag, text=None):
        self.ui.check_2.setEnabled(flag)
        if text != None: self.ui.check_2.setText(text)
    def enable_check_3(self, flag, text=None):
        self.ui.check_3.setEnabled(flag)
        if text != None: self.ui.check_3.setText(text)
    def enable_check_4(self, flag, text=None):
        self.ui.check_4.setEnabled(flag)
        if text != None: self.ui.check_4.setText(text)
    def enable_check_5(self, flag, text=None):
        self.ui.check_5.setEnabled(flag)
        if text != None: self.ui.check_5.setText(text)
    def enable_check_6(self, flag, text=None):
        self.ui.check_6.setEnabled(flag)
        if text != None: self.ui.check_6.setText(text)
    def enable_check_7(self, flag, text=None):
        self.ui.check_7.setEnabled(flag)
        if text != None: self.ui.check_7.setText(text)
    def enable_check_8(self, flag, text=None):
        self.ui.check_8.setEnabled(flag)
        if text != None: self.ui.check_8.setText(text)
    def enable_check_9(self, flag, text=None):
        self.ui.check_9.setEnabled(flag)
        if text != None: self.ui.check_9.setText(text)
    def enable_check_10(self, flag, text=None):
        self.ui.check_10.setEnabled(flag)
        if text != None: self.ui.check_10.setText(text)
    def enable_choose_mode(self, flag, mode="position_control"):
        if mode == "position_control": self.ui.r_pos.setChecked(True) # 选择位置模式
        elif mode == "speed_control": self.ui.r_vel.setChecked(True) # 选择速度模式
        else: pass
        self.ui.r_pos.setEnabled(flag)
        self.ui.r_vel.setEnabled(flag)
    def enable_default(self):
        self.ui.tx_speed.setEnabled(False)
        self.ui.le_speed.setEnabled(False) # 禁用速度设置
        self.ui.le_speed.setPlaceholderText("默认值0")
        self.ui.tx_position.setEnabled(False)
        self.ui.le_position.setEnabled(False) # 禁用位置设置
        self.ui.le_position.setPlaceholderText("默认值0")
    def enable_set_param(self, flag, text0=None, text1=None, text2=None):
        self.ui.le_acc.clear()
        self.ui.tx_acc.setEnabled(flag)
        self.ui.le_acc.setEnabled(flag)
        if text0 != None: self.ui.le_acc.setPlaceholderText(text0)
        self.ui.le_dec.clear()
        self.ui.tx_dec.setEnabled(flag)
        self.ui.le_dec.setEnabled(flag)
        if text1 != None: self.ui.le_dec.setPlaceholderText(text1)
        self.ui.le_vel.clear()
        self.ui.tx_vel.setEnabled(flag)
        self.ui.le_vel.setEnabled(flag)
        if text2 != None: self.ui.le_vel.setPlaceholderText(text2)
    def enable_save_param(self, flag):
        self.ui.bt_save.setEnabled(flag)
    def enable_init_motor(self, flag, text=None):
        self.ui.bt_launch.setEnabled(flag)
        if text != None: self.ui.bt_launch.setText(text)
    # IO
    def enable_check_status(self, flag, text=None):
        self.ui.bt_check.setEnabled(flag)
        if text != None: self.ui.bt_check.setText(text)
    def enable_start(self, flag, text=None):
        self.ui.bt_start.setEnabled(flag)
        if text != None: self.ui.bt_start.setText(text)
    # 采集卡
    ...

    def initial_status(self) -> None:
        # 菜单 初始化激活 控制失效 日志失效
        self.enable_menu(True, False, False)
        # 显示第1页
        self.ui.stackedWidget.setCurrentIndex(0)
        # USBCSN
        self.enable_open_device(True, "打开设备")
        self.enable_channel0(False, False, "打开通道0", "重置通道0")
        self.enable_channel1(False, False, "打开通道1", "重置通道1")
        self.enable_close_device(False, "关闭设备")
        # 电机
        self.enable_check_all(False, "一键检查")
        self.enable_check_1(False, "电机1")
        self.enable_check_2(False, "电机2")
        self.enable_check_3(False, "电机3")
        self.enable_check_4(False, "电机4")
        self.enable_check_5(False, "电机5")
        self.enable_check_6(False, "电机6")
        self.enable_check_7(False, "电机7")
        self.enable_check_8(False, "电机8")
        self.enable_check_9(False, "电机9")
        self.enable_check_10(False, "电机10")
        self.enable_choose_mode(False, "position_control")
        self.enable_default()
        self.enable_set_param(False, "默认值{}".format(Motor.acceleration), "默认值{}".format(Motor.deceleration), "默认值{}".format(Motor.velocity))
        self.enable_save_param(False)
        self.enable_init_motor(False, "生效")
        # IO
        self.enable_check_status(False, "检查总线状态")
        self.enable_start(False, "启动RPDO传输")
    
    def set_menu_jumping(self) -> None:
        self.ui.bt_init.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.bt_control.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.bt_log.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))

    def set_usbcan_jumping(self) -> None:
        def open_usbcan():
            UsbCan.open_device()
            if UsbCan.is_open:
                self.enable_open_device(False, "设备已打开")
                self.enable_close_device(True, "关闭设备")
                self.enable_channel0(True, False)
                self.enable_channel1(True, False)
        def start_channel_0():
            self.usbcan_0.timer = self.ui.bx_rate0.currentText()
            self.usbcan_0.init_can()
            self.usbcan_0.start_can()
            if self.usbcan_0.is_start:
                # USBCAN
                self.enable_channel0(False, True, "通道0已打开")
                # 电机
                self.enable_check_all(True)
                self.enable_check_1(True)
                self.enable_check_2(True)
                self.enable_check_3(True)
                self.enable_check_4(True)
                self.enable_check_5(True)
                self.enable_check_6(True)
                self.enable_check_7(True)
                self.enable_check_8(True)
                self.enable_check_9(True)
                self.enable_check_10(True)
        def start_channel_1():
            self.usbcan_1.timer = self.ui.bx_rate1.currentText()
            self.usbcan_1.init_can()
            self.usbcan_1.start_can()
            if self.usbcan_1.is_start:
                self.enable_channel1(False, True, "通道1已打开")
                # 其他的配置 后续补齐
                ...
        def reset_cannel_0():
            self.usbcan_0.reset_can()
        def reset_cannel_1():
            self.usbcan_1.reset_can()
        def close_usbcan():
            UsbCan.close_device()
            if not UsbCan.is_open:
                # 菜单
                self.enable_menu(True, False, False)
                # USBCAN
                self.enable_open_device(True, "打开设备")
                self.enable_channel0(False, False, "打开通道0", "重置通道0")
                self.enable_channel1(False, False, "打开通道1", "重置通道1")
                self.enable_close_device(False, "设备已关闭")
                # 电机
                self.enable_check_all(False)
                self.enable_check_1(False, "电机1")
                self.enable_check_2(False, "电机2")
                self.enable_check_3(False, "电机3")
                self.enable_check_4(False, "电机4")
                self.enable_check_5(False, "电机5")
                self.enable_check_6(False, "电机6")
                self.enable_check_7(False, "电机7")
                self.enable_check_8(False, "电机8")
                self.enable_check_9(False, "电机9")
                self.enable_check_10(False, "电机10")
                self.enable_choose_mode(False)
                self.enable_set_param(False, "默认值1000", "默认值10000", "默认值100")
                self.enable_save_param(False)
                self.enable_init_motor(False, "生效")
                # IO
                self.enable_check_status(False, "状态总线检查")
                self.enable_start(False, "启动RPDO传输")
        self.ui.bt_open.clicked.connect(lambda: open_usbcan())
        self.ui.bt_channel0.clicked.connect(lambda: start_channel_0())
        self.ui.bt_channel1.clicked.connect(lambda: start_channel_1())
        self.ui.bt_reset0.clicked.connect(lambda: reset_cannel_0())
        self.ui.bt_reset1.clicked.connect(lambda: reset_cannel_1())
        self.ui.bt_close.clicked.connect(lambda: close_usbcan())
    
    def set_motor_jumping(self):
        def ischecked():
            if self.motor_checked_num == self.motor_ischeck_num:
                if self.is_admin:
                    self.enable_choose_mode(True) # 激活 模式选择
                    self.enable_set_param(True) # 激活 设置参数
                self.enable_save_param(True) # 激活 保存参数
                # IO
                self.enable_check_status(True)
        def check_bus_all():
            check_bus_1()
            check_bus_2()
            check_bus_3()
            check_bus_4()
            check_bus_5()
            check_bus_6()
            check_bus_7()
            check_bus_8()
            check_bus_9()
            check_bus_10()
            self.enable_check_all(False)
            ischecked()
        def check_bus_1():
            if self.motor_1.check_bus_status():
                if self.motor_1.check_motor_status():
                    self.enable_check_1(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_2():
            if self.motor_2.check_bus_status():
                if self.motor_2.check_motor_status():
                    self.enable_check_2(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_3():
            if self.motor_3.check_bus_status():
                if self.motor_3.check_motor_status():
                    self.enable_check_3(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_4():
            if self.motor_4.check_bus_status():
                if self.motor_4.check_motor_status():
                    self.enable_check_4(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_5():
            if self.motor_5.check_bus_status():
                if self.motor_5.check_motor_status():
                    self.enable_check_5(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_6():
            if self.motor_6.check_bus_status():
                if self.motor_6.check_motor_status():
                    self.enable_check_6(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_7():
            if self.motor_7.check_bus_status():
                if self.motor_7.check_motor_status():
                    self.enable_check_7(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_8():
            if self.motor_8.check_bus_status():
                if self.motor_8.check_motor_status():
                    self.enable_check_8(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_9():
            if self.motor_9.check_bus_status():
                if self.motor_9.check_motor_status():
                    self.enable_check_9(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def check_bus_10():
            if self.motor_10.check_bus_status():
                if self.motor_10.check_motor_status():
                    self.enable_check_10(False, "就绪")
                    self.motor_checked_num += 1
            ischecked()
        def save_config():
            if not self.is_admin: Motor.config() # 直接保存默认参数
            else:
                mode = "position_control" if self.ui.r_pos.isChecked() else "speed_control"
                acc = self.ui.le_acc.text() if self.ui.le_acc.text() != "" else "1000"
                dec = self.ui.le_dec.text() if self.ui.le_dec.text() != "" else "10000"
                vel = self.ui.le_vel.text() if self.ui.le_vel.text() != "" else "100"
                Motor.config(mode, int(acc), int(dec), int(vel))
                self.enable_set_param(True, "当前值{}".format(Motor.acceleration), "当前值{}".format(Motor.deceleration), "当前值{}".format(Motor.velocity))
            self.enable_init_motor(True, "生效") # 激活 生效
        def init_motor():
            self.motor_2.init_config()
            ...
            self.enable_init_motor(False, "完成")
            self.enable_menu(True, True, False) # 菜单
        self.ui.check_all.clicked.connect(lambda: check_bus_all())
        self.ui.check_1.clicked.connect(lambda: check_bus_1())
        self.ui.check_2.clicked.connect(lambda: check_bus_2())
        self.ui.check_3.clicked.connect(lambda: check_bus_3())
        self.ui.check_4.clicked.connect(lambda: check_bus_4())
        self.ui.check_5.clicked.connect(lambda: check_bus_5())
        self.ui.check_6.clicked.connect(lambda: check_bus_6())
        self.ui.check_7.clicked.connect(lambda: check_bus_7())
        self.ui.check_8.clicked.connect(lambda: check_bus_8())
        self.ui.check_9.clicked.connect(lambda: check_bus_9())
        self.ui.check_10.clicked.connect(lambda: check_bus_10())
        self.ui.bt_save.clicked.connect(lambda: save_config())
        self.ui.bt_launch.clicked.connect(lambda: init_motor())
    
    def set_io_jumping(self):
        def check_status():
            if self.io_module.check_bus_status():
                self.enable_check_status(False, "就绪")
                self.io_module_checked_num += 1
            if self.io_module_checked_num == self.io_module_ischeck_num:
                self.enable_start(True) # 激活 生效
        def start_output():
            if self.io_module.start_output():
                self.enable_start(False, "传输已启动")
                self.enable_menu(True, True, False) # 菜单
        self.ui.bt_check.clicked.connect(lambda: check_status())
        self.ui.bt_start.clicked.connect(lambda: start_output())