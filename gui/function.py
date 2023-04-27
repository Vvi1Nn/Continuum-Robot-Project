# -*- coding:utf-8 -*-


''' function.py GUI功能函数 v1.8 '''


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

''' 登录界面 '''
class LoginPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_LoginPanel()
        self.ui.setupUi(self)

        self.user_pw = ""
        self.admin_pw = ""
        
        self.show() # 显示登录窗口
        self.__fix_window_size() # 固定窗口大小
        
        self.__login_check() # 登录
    
    def __fix_window_size(self, width = 788, height = 506) -> None:
        self.resize(width, height)
        self.setFixedSize(width, height)

    def __login_check(self) -> None:
        self.ui.button_login.clicked.connect(self.__jump_to_control_panel)

    def __jump_to_control_panel(self) -> None:
        account = self.ui.box_user.currentText()
        password = self.ui.password.text()
        if account == "测试人员" and password == self.user_pw:
            self.close()
            self.control_panel = ControlPanel(account)
        elif account == "管理员" and password == self.admin_pw:
            self.close()
            self.control_panel = ControlPanel(account)
        else: pass
 

''' 控制界面 '''
class ControlPanel(QMainWindow):
    def __init__(self, account) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        
        # 是否是管理员权限
        self.is_admin = False if account == "测试人员" else True
        
        # CAN卡实例化
        self.usbcan_0 = UsbCan.is_show_log(False)("0") # 通道0
        self.usbcan_1 = UsbCan.is_show_log(False)("1") # 通道1
        
        CanOpenBusProcessor.link_device(self.usbcan_0) # 将CANopen总线绑定至CAN卡的通道0
        
        # 电机实例化
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
        self.motor_checked_num = 0 # 已检查成功的数量
        self.motor_ischeck_num = 1 # 应当检查成功的数量
        
        # IO实例化
        self.io_module = IoModule(11)
        self.io_module_checked_num = 0 # 已检查成功的数量
        self.io_module_ischeck_num = 1 # 应当检查成功的数量

        self.initial_status() # 根据权限 显示初始页面
        
        # 按钮跳转的设置 也就是把按钮的signal和slot绑定
        self.set_menu_jumping() # 菜单
        self.set_usbcan_jumping() # USBCAN
        self.set_motor_jumping() # 电机
        self.set_io_jumping() # IO

        self.show() # 显示界面


    '''
        以下是控制界面中 所有按钮的状态设置函数 包含是否激活和显示文字
    '''
    ''' 菜单 '''
    def enable_menu(self, init, control, log):
        self.ui.bt_init.setEnabled(init)
        self.ui.bt_control.setEnabled(control)
        self.ui.bt_log.setEnabled(log)
    
    '''
        第一页
    '''
    ''' USBCAN '''
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
    
    ''' 电机 '''
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
    def enable_start_pdo(self, flag, text=None):
        self.ui.start.setEnabled(flag)
        if text != None: self.ui.start.setText(text)
    def enable_choose_mode(self, flag, mode="position_control"):
        if mode == "position_control": self.ui.r_pos.setChecked(True) # 选择位置模式
        elif mode == "speed_control": self.ui.r_vel.setChecked(True) # 选择速度模式
        else: pass
        self.ui.r_pos.setEnabled(flag)
        self.ui.r_vel.setEnabled(flag)
    def enable_set_param(self, flag, text0=None, text1=None, text2=None, text3=None, text4=None):
        self.ui.le_acc.clear()
        self.ui.tx_acc.setEnabled(flag)
        self.ui.le_acc.setEnabled(flag)
        if text0 != None: self.ui.le_acc.setPlaceholderText(text0) # 加速度
        self.ui.le_dec.clear()
        self.ui.tx_dec.setEnabled(flag)
        self.ui.le_dec.setEnabled(flag)
        if text1 != None: self.ui.le_dec.setPlaceholderText(text1) # 减速度
        self.ui.le_vel.clear()
        self.ui.tx_vel.setEnabled(flag)
        self.ui.le_vel.setEnabled(flag)
        if text2 != None: self.ui.le_vel.setPlaceholderText(text2) # 动作速度
        self.ui.le_position.clear()
        self.ui.tx_position.setEnabled(flag)
        self.ui.le_position.setEnabled(flag)
        if text2 != None: self.ui.le_position.setPlaceholderText(text3) # 目标位置 也就是运动间隔
        self.ui.le_inhibit.clear()
        self.ui.tx_inhibit.setEnabled(flag)
        self.ui.le_inhibit.setEnabled(flag)
        if text3 != None: self.ui.le_inhibit.setPlaceholderText(text4) # TPDO禁止时间
    def enable_save_param(self, flag, text=None):
        self.ui.bt_save.setEnabled(flag)
        if text != None: self.ui.bt_save.setText(text)
    def enable_init_motor(self, flag, text=None):
        self.ui.bt_launch.setEnabled(flag)
        if text != None: self.ui.bt_launch.setText(text)
    
    ''' IO '''
    def enable_check_status(self, flag, text=None):
        self.ui.bt_check.setEnabled(flag)
        if text != None: self.ui.bt_check.setText(text)
    def enable_start_rpdo(self, flag, text=None):
        self.ui.bt_start.setEnabled(flag)
        if text != None: self.ui.bt_start.setText(text)
    
    ''' 采集卡 '''
    ...
    
    '''
        第二页
    '''
    ''' 状态控制 '''
    def enable_quick_stop(self, flag, text=None):
        self.ui.bt_quick_stop.setEnabled(flag)
        if text != None: self.ui.bt_quick_stop.setText(text)

    '''
        第三页
    '''
    '''  '''
    ...
    

    '''
        界面的初始显示状态
    '''
    def initial_status(self) -> None:
        ''' 菜单 '''
        self.enable_menu(True, False, False) # 初始化激活 控制失效 日志失效
        ''' 页面 '''
        self.ui.stackedWidget.setCurrentIndex(0) # 显示第1页
        ''' USBCAN '''
        self.enable_open_device(True, "打开设备") # 激活
        self.enable_channel0(False, False, "打开通道0", "重置0") # 失效
        self.enable_channel1(False, False, "打开通道1", "重置1") # 失效
        self.enable_close_device(False, "关闭") # 失效
        ''' 电机 '''
        self.enable_check_all(False, "一键检查") # 失效
        self.enable_check_1(False, "M1") # 失效
        self.enable_check_2(False, "M2") # 失效
        self.enable_check_3(False, "M3") # 失效
        self.enable_check_4(False, "M4") # 失效
        self.enable_check_5(False, "M5") # 失效
        self.enable_check_6(False, "M6") # 失效
        self.enable_check_7(False, "M7") # 失效
        self.enable_check_8(False, "M8") # 失效
        self.enable_check_9(False, "M9") # 失效
        self.enable_check_10(False, "M10") # 失效
        self.enable_start_pdo(False, "启动PDO") # 失效
        self.enable_choose_mode(False, "position_control") # 失效
        # 参数设置 失效 并显示默认值
        self.enable_set_param(False, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
        self.enable_save_param(False, "保存") # 失效
        self.enable_init_motor(False, "生效") # 失效
        ''' IO '''
        self.enable_check_status(False, "检查耦合器")
        self.enable_start_rpdo(False, "启动RPDO")
        ''' 传感器 '''
    
    '''
        以下是控制界面中 按钮signal的slot函数
    '''
    ''' 菜单 '''
    def set_menu_jumping(self) -> None:
        self.ui.bt_init.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.bt_control.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.bt_log.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))
    
    ''' CAN卡 '''
    def set_usbcan_jumping(self) -> None:
        '''
            定义slot函数
        '''
        ''' 打开设备 '''
        def open_usbcan():
            if UsbCan.open_device():
                self.enable_open_device(False, "设备已打开")
                self.enable_close_device(True)
                self.enable_channel0(True, False)
                self.enable_channel1(True, False)
        ''' 开启通道0 '''
        def start_channel_0():
            self.usbcan_0.set_timer(self.ui.bx_rate0.currentText()) # 记录波特率
            if self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.enable_channel0(False, True, "通道0已打开") # 打开失效 激活重置
                # 电机检查全部激活
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
        ''' 开启通道1 '''
        def start_channel_1():
            self.usbcan_1.set_timer(self.ui.bx_rate1.currentText()) # 记录波特率
            if self.usbcan_1.init_can() and self.usbcan_1.start_can():
                self.enable_channel1(False, True, "通道1已打开") # 打开失效 激活重置
                # 传感器的配置 后续补齐
                ...
                ...
                ...
        ''' 重置通道0 '''
        def reset_cannel_0():
            self.usbcan_0.reset_can() # 直接重置就行
        ''' 重置通道1 '''
        def reset_cannel_1():
            self.usbcan_1.reset_can() # 直接重置就行
        ''' 关闭设备 '''
        def close_usbcan():
            if UsbCan.close_device():
                self.initial_status()
        '''
            绑定signal和slot
        '''
        self.ui.bt_open.clicked.connect(lambda: open_usbcan())
        self.ui.bt_channel0.clicked.connect(lambda: start_channel_0())
        self.ui.bt_channel1.clicked.connect(lambda: start_channel_1())
        self.ui.bt_reset0.clicked.connect(lambda: reset_cannel_0())
        self.ui.bt_reset1.clicked.connect(lambda: reset_cannel_1())
        self.ui.bt_close.clicked.connect(lambda: close_usbcan())
    
    ''' 电机 '''
    def set_motor_jumping(self) -> None:
        ''' 检查是否完成 '''
        def __ischecked():
            if self.motor_checked_num == self.motor_ischeck_num:
                if self.is_admin:
                    self.enable_choose_mode(True) # 激活 模式选择
                    self.enable_set_param(True) # 激活 设置参数
                self.enable_save_param(True) # 激活 保存参数
        ''' 一键检查 '''
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
            __ischecked()
        ''' 单个检查 '''
        def check_bus_1():
            if self.motor_1.check_bus_status():
                if self.motor_1.check_motor_status():
                    self.enable_check_1(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_2():
            if self.motor_2.check_bus_status():
                if self.motor_2.check_motor_status():
                    self.enable_check_2(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_3():
            if self.motor_3.check_bus_status():
                if self.motor_3.check_motor_status():
                    self.enable_check_3(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_4():
            if self.motor_4.check_bus_status():
                if self.motor_4.check_motor_status():
                    self.enable_check_4(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_5():
            if self.motor_5.check_bus_status():
                if self.motor_5.check_motor_status():
                    self.enable_check_5(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_6():
            if self.motor_6.check_bus_status():
                if self.motor_6.check_motor_status():
                    self.enable_check_6(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_7():
            if self.motor_7.check_bus_status():
                if self.motor_7.check_motor_status():
                    self.enable_check_7(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_8():
            if self.motor_8.check_bus_status():
                if self.motor_8.check_motor_status():
                    self.enable_check_8(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_9():
            if self.motor_9.check_bus_status():
                if self.motor_9.check_motor_status():
                    self.enable_check_9(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        def check_bus_10():
            if self.motor_10.check_bus_status():
                if self.motor_10.check_motor_status():
                    self.enable_check_10(False, "就绪")
                    self.motor_checked_num += 1
            __ischecked()
        ''' 保存参数 '''
        def save_config():
            if not self.is_admin: Motor.config() # 无权限 直接保存默认参数
            else: # 管理员权限
                mode = "position_control" if self.ui.r_pos.isChecked() else "speed_control" # 记录模式
                acc = self.ui.le_acc.text() if self.ui.le_acc.text() != "" else "1000" # 记录加速度
                dec = self.ui.le_dec.text() if self.ui.le_dec.text() != "" else "10000" # 记录减速度
                vel = self.ui.le_vel.text() if self.ui.le_vel.text() != "" else "100" # 记录动作速度
                position = self.ui.le_position.text() if self.ui.le_position.text() != "" else "50" # 记录动作幅度
                inhibit = self.ui.le_inhibit.text() if self.ui.le_inhibit.text() != "" else "500" # 记录禁止时间
                # 保存上述参数
                Motor.config(mode, int(acc), int(dec), int(vel), int(position), int(inhibit))
                # 
                self.enable_set_param(True, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
            self.enable_init_motor(True, "生效") # 激活 生效
        def init_motor():
            self.motor_2.init_config()
            ...
            ...
            ...
            self.enable_init_motor(False, "完成")
            self.enable_start_pdo(True) # 激活 开启TPDO
        def start_pdo():
            self.motor_2.start_feedback()
            ...
            ...
            ...
            
            self.enable_menu(True, True, False) # 菜单

        self.ui.check_all.clicked.connect(lambda: check_bus_all())
        self.ui.start.clicked.connect(lambda: start_pdo())
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
    
    ''' IO '''
    def set_io_jumping(self):
        def check_status():
            if self.io_module.check_bus_status():
                self.enable_check_status(False, "OK")
                self.io_module_checked_num += 1
            if self.io_module_checked_num == self.io_module_ischeck_num:
                self.enable_start_rpdo(True) # 激活 开启RPDO
        def start_output():
            if self.io_module.start_output():
                self.enable_start_rpdo(False, "传输已启动")
                self.enable_start_pdo(True) # 激活 开启TPDO
        self.ui.bt_check.clicked.connect(lambda: check_status())
        self.ui.bt_start.clicked.connect(lambda: start_output())