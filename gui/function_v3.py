# -*- coding:utf-8 -*-


''' function_new.py 新GUI功能函数 v3.2 '''


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QMutex


# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gui.ui_login import Ui_MainWindow as Ui_LoginPanel
from gui.ui_control_v3 import Ui_MainWindow as Ui_ControlPanel
from usbcan.function import UsbCan
import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor
from motor.function import Motor
from motor.threads import CANopenUpdateThread, JointControlThread, JointControlSpeedModeThread, InitMotorThread, CheckMotorThread, StartPDO, StopPDO
from sensor.function import Sensor, SensorUpdateThread
from io_module.function import IoModule
from joystick.function import JoystickThread


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
        self.usbcan_0 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("0") # 通道0
        self.usbcan_1 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("1") # 通道1
        self.usbcan_is_running = False
        
        CanOpenBusProcessor.link_device(self.usbcan_0) # 将CANopen总线绑定至CAN卡的通道0
        CanOpenBusProcessor.is_show_log(False)
        
        # 电机实例化
        self.motor_1 = Motor(1, [-10000000,10000000], [-200,200])
        self.motor_2 = Motor(2, [-10000000,10000000], [-200,200])
        self.motor_3 = Motor(3, [-10000000,10000000], [-200,200])
        self.motor_4 = Motor(4, [-10000000,10000000], [-200,200])
        self.motor_5 = Motor(5, [-10000000,10000000], [-200,200])
        self.motor_6 = Motor(6, [-10000000,10000000], [-200,200])
        self.motor_7 = Motor(7, [-10000000,10000000], [-200,200])
        self.motor_8 = Motor(8, [-10000000,10000000], [-200,200])
        self.motor_9 = Motor(9, [-10000000,10000000], [-200,200])
        self.motor_10 = Motor(10, [-10000000,10000000], [-200,200])
        self.motor_is_running = False

        Sensor.link_device(self.usbcan_1)
        # 力传感器
        self.sensor_1 = Sensor(1)
        self.sensor_2 = Sensor(2)
        self.sensor_3 = Sensor(3)
        self.sensor_4 = Sensor(4)
        self.sensor_5 = Sensor(5)
        self.sensor_6 = Sensor(6)
        self.sensor_7 = Sensor(7)
        self.sensor_8 = Sensor(8)
        self.sensor_9 = Sensor(9)
        self.sensor_10 = Sensor(10)
        self.sensor_is_running = False

        # IO模块
        self.io = IoModule(11, self.update_magnetic_valve) # IO
        self.io_is_running = False

        self.check_motor_thread = CheckMotorThread() # 检查电机
        self.init_motor_thread = InitMotorThread() # 初始化电机
        self.sensor_mutex = QMutex()
        self.sensor_update = SensorUpdateThread(self.sensor_mutex, self.update_sensor) # 力传感器
        self.start_pdo_thread = StartPDO() # 开启PDO
        self.stop_pdo_thread = StopPDO() # 关闭PDO
        self.joystick = JoystickThread() # 操纵杆
        self.motor_update = CANopenUpdateThread(self.update_canopen) # 电机状态更新

        self.initial_status() # 根据权限 显示初始页面
        
        # 按钮跳转的设置 也就是把按钮的signal和slot绑定
        self.set_menu_jumping() # 菜单
        self.set_auto_jumping()
        self.set_usbcan_jumping() # USBCAN
        self.set_motor_jumping() # 电机
        self.set_sensor_jumping() # 传感器
        self.set_io_jumping() # IO
        self.set_control_jumping() # 控制

        self.show() # 显示界面

       

    
    ''' 界面的初始显示状态 '''
    def initial_status(self) -> None:
        ''' 菜单 '''
        self.enable_menu(True, True, True) # 初始化激活 控制激活 日志激活
        ''' 页面 '''
        self.ui.stackedWidget.setCurrentIndex(0) # 显示第1页
        ''' USBCAN '''
        self.enable_open_device(True, "打开") # 激活
        self.enable_channel0(False, False, "打开通道0", "重置0") # 失效
        self.enable_channel1(False, False, "打开通道1", "重置1") # 失效
        self.enable_close_device(False, "关闭") # 失效
        ''' 电机 '''
        self.enable_check_all(False, "电机状态确认") # 失效
        for i in range(1, 11):
            getattr(self, f"enable_check_{i}")(False, "M{}".format(i)) # 失效
        self.enable_choose_mode(False, "position_control") # 失效
        # 参数设置 失效 并显示默认值
        self.enable_set_param(False, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
        self.enable_save_param(False, "保存") # 失效
        self.enable_init_motor(False, "生效") # 失效
        self.enable_start_pdo(False, "启动状态读取") # 失效
        self.enable_stop_pdo(False, "停止状态读取") # 失效
        ''' 传感器 '''
        self.enable_start_force(False, "启动拉力读取") # 失效
        self.enable_stop_force(False, "停止拉力读取") # 失效
        ''' IO '''
        self.enable_open_io(False, "启动IO输出") # 失效
        self.enable_close_io(False, "关闭IO输出") # 失效
        ''' 状态控制 '''
        self.enable_quick_stop(False, "急停") # 失效
        self.enable_release_break(False, "解除抱闸") # 失效
        self.enable_enable_servo(False, "伺服使能") # 失效
        ''' 关节操作 '''
        self.enable_joint_control(False, "关节操作") # 失效
        self.enable_quit(False, "退出") # 失效
        for i in range(1, 11):
            getattr(self, f"enable_motor_group_{i}")(False) # 失效
        self.enable_io(False) # 失效
        ''' 遥操作 '''
        self.enable_end_control(False, "末端操作") # 失效
        self.enable_exit(False, "退出") # 失效
        ''' 状态 '''
        for i in range(1, 11):
            getattr(self, f"enable_status_{i}")(False) # 失效
        self.enable_status_io(False) # 失效
















    ''' 菜单 '''
    # 跳转
    def set_menu_jumping(self) -> None:
        self.ui.bt_init.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.bt_control.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.bt_log.clicked.connect(lambda: self.ui.stackedWidget.setCurrentIndex(2))
    # 界面 状态设置
    def enable_menu(self, init, control, log):
        self.ui.bt_init.setEnabled(init)
        self.ui.bt_control.setEnabled(control)
        self.ui.bt_log.setEnabled(log)
    
    
    
    '''
        自动
    '''
    def enable_auto(self, flag, text=None):
        self.ui.bt_auto.setEnabled(flag)
        if text != None: self.ui.bt_auto.setText(text)
    
    def set_auto_jumping(self) -> None:
        self.ui.bt_auto.clicked.connect(self.auto)
    
    def auto(self):
        self.open_usbcan()
        time.sleep(0.1)
        self.start_channel_0()
        time.sleep(0.1)
        self.start_channel_1()
        time.sleep(0.1)
        # self.check_motor()
        # time.sleep(5)
        self.open_module()
        # time.sleep(1)

        # self.start_channel_1()
        # time.sleep(0.1)
        # self.start_force()
        # time.sleep(8)

        self.enable_auto(False)
        self.usbcan_is_running = True
    



    
    
    
    












    ''' CAN卡 '''
    # 界面 状态设置
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
    
    # 按钮clicked信号绑定slot函数
    def set_usbcan_jumping(self) -> None:
        self.ui.bt_open.clicked.connect(self.open_usbcan)
        self.ui.bt_channel0.clicked.connect(self.start_channel_0)
        self.ui.bt_channel1.clicked.connect(self.start_channel_1)
        # self.ui.bt_reset0.clicked.connect(self.reset_cannel_0)
        # self.ui.bt_reset1.clicked.connect(self.reset_cannel_1)
        # self.ui.bt_close.clicked.connect(self.close_usbcan)


        self.ui.bt_reset0.clicked.connect(self.test_1)
        self.ui.bt_reset1.clicked.connect(self.test_2)
    
    # 打开设备
    def open_usbcan(self):
        if UsbCan.open_device():
            self.enable_open_device(False, "设备已打开")
            self.enable_close_device(True)
            self.enable_channel0(True, False)
            self.enable_channel1(True, False)
        else: pass
    
    # 打开通道0
    def start_channel_0(self):
        self.usbcan_0.set_timer(self.ui.bx_rate0.currentText()) # 记录波特率
        if self.usbcan_0.init_can() and self.usbcan_0.start_can():
            self.enable_channel0(False, True, "通道0已打开") # 打开失效 激活重置
            # 电机检查全部激活
            self.enable_check_all(True)
            for i in range(1,11):
                getattr(self, f"enable_check_{i}")(True)
            # 激活IO
            self.enable_open_io(True)
        else: pass
    
    # 打开通道1
    def start_channel_1(self):
        self.usbcan_1.set_timer(self.ui.bx_rate1.currentText()) # 记录波特率
        if self.usbcan_1.init_can() and self.usbcan_1.start_can():
            self.enable_channel1(False, True, "通道1已打开") # 打开失效 激活重置
            # 传感器的配置
            self.enable_start_force(True)
        else: pass
    
    # 重置通道0
    def reset_cannel_0(self):
        self.usbcan_0.reset_can() # 直接重置就行
    
    # 重置通道1
    def reset_cannel_1(self):
        self.usbcan_1.reset_can() # 直接重置就行
    
    # 关闭设备
    def close_usbcan(self):
        if UsbCan.close_device():
            self.initial_status()
        else: pass
    



    
    
    
    
    










    
    
    ''' 电机 '''
    # 界面 状态设置
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
    
    def enable_start_pdo(self, flag, text=None):
        self.ui.start.setEnabled(flag)
        if text != None: self.ui.start.setText(text)
    
    def enable_stop_pdo(self, flag, text=None):
        self.ui.stop.setEnabled(flag)
        if text != None: self.ui.stop.setText(text)
    
    def enable_quick_stop(self, flag, text=None):
        self.ui.bt_quick_stop.setEnabled(flag)
        if text != None: self.ui.bt_quick_stop.setText(text)
    
    def enable_release_break(self, flag, text=None):
        self.ui.bt_unlock.setEnabled(flag)
        if text != None: self.ui.bt_unlock.setText(text)
    
    def enable_enable_servo(self, flag, text=None):
        self.ui.bt_enable.setEnabled(flag)
        if text != None: self.ui.bt_enable.setText(text)
    
    def enable_joint_control(self, flag, text=None):
        self.ui.bt_joint_control.setEnabled(flag)
        if text != None: self.ui.bt_joint_control.setText(text)
    
    def enable_quit(self, flag, text=None):
        self.ui.bt_quit.setEnabled(flag)
        if text != None: self.ui.bt_quit.setText(text)
    
    def enable_motor_group_1(self, flag):
        self.ui.group_1.setEnabled(flag)
    def enable_motor_group_2(self, flag):
        self.ui.group_2.setEnabled(flag)
    def enable_motor_group_3(self, flag):
        self.ui.group_3.setEnabled(flag)
    def enable_motor_group_4(self, flag):
        self.ui.group_4.setEnabled(flag)
    def enable_motor_group_5(self, flag):
        self.ui.group_5.setEnabled(flag)
    def enable_motor_group_6(self, flag):
        self.ui.group_6.setEnabled(flag)
    def enable_motor_group_7(self, flag):
        self.ui.group_7.setEnabled(flag)
    def enable_motor_group_8(self, flag):
        self.ui.group_8.setEnabled(flag)
    def enable_motor_group_9(self, flag):
        self.ui.group_9.setEnabled(flag)
    def enable_motor_group_10(self, flag):
        self.ui.group_10.setEnabled(flag)
    
    # 按钮信号绑定slot函数
    def set_motor_jumping(self) -> None:
        self.ui.check_all.clicked.connect(self.check_motor) # 检查
        self.ui.bt_save.clicked.connect(self.save_config) # 保存
        self.ui.bt_launch.clicked.connect(self.init_motor) # 生效
        self.ui.start.clicked.connect(self.start_pdo) # 开启PDO
        self.ui.stop.clicked.connect(self.stop_pdo) # 关闭PDO

        self.ui.bt_quick_stop.clicked.connect(self.quick_stop) # 急停
        self.ui.bt_quick_stop.setShortcut("space")
        self.ui.bt_unlock.clicked.connect(self.release_break) # 掉电
        self.ui.bt_enable.clicked.connect(self.enable_servo) # 使能

        self.ui.bt_joint_control.clicked.connect(self.joint_control) # 进入单电机控制
        self.ui.bt_quit.clicked.connect(self.quit_joint_control) # 退出单电机控制
        
        # 10组电机控制按钮
        for node_id in Motor.motor_dict:
            getattr(self.ui, f"bt_positive_{node_id}").pressed.connect(getattr(self, f"joint_forward_{node_id}"))
            getattr(self.ui, f"bt_positive_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
            getattr(self.ui, f"bt_negative_{node_id}").pressed.connect(getattr(self, f"joint_reverse_{node_id}"))
            getattr(self.ui, f"bt_negative_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
    
    # 检查状态
    def check_motor(self):
        def change(status):
            if status == True:
                self.enable_check_all(False, "进行中...")
                # IO的按钮
                if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
                else: self.enable_open_io(False) # 失效按钮 打开IO
            else:
                self.enable_check_all(True, "再次检查")
                self.check_motor_thread = CheckMotorThread()
                # IO的按钮
                if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
                else: self.enable_open_io(True) # 生效按钮 打开IO
        def update(node_id):
            getattr(self, f"enable_check_{node_id}")(False, "OK")
            getattr(self, f"enable_status_{node_id}")(True)
        def status(node_id):
            getattr(self.ui, f"servo_{node_id}").setText(getattr(self, f"motor_{node_id}").motor_status)
            getattr(self.ui, f"position_{node_id}").setText(str(getattr(self, f"motor_{node_id}").current_position))
            getattr(self.ui, f"speed_{node_id}").setText(str(getattr(self, f"motor_{node_id}").current_speed))
        def next():
            if self.is_admin:
                self.enable_choose_mode(True) # 激活 模式选择
                self.enable_set_param(True) # 激活 设置参数
            else: pass
            self.enable_save_param(True) # 激活 保存参数
            self.enable_check_all(False, "完成")
            # IO的按钮
            if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
            else: self.enable_open_io(True) # 生效按钮 打开IO
        self.check_motor_thread.running_signal.connect(change)
        self.check_motor_thread.check_signal.connect(update)
        self.check_motor_thread.status_signal.connect(status)
        self.check_motor_thread.finish_signal.connect(next)
        self.check_motor_thread.start()
    
    # 保存参数
    def save_config(self):
        if not self.is_admin: Motor.config() # 无权限 直接保存默认参数
        else: # 管理员权限
            mode = "position_control" if self.ui.r_pos.isChecked() else "speed_control" # 记录模式
            acc = self.ui.le_acc.text() if self.ui.le_acc.text() != "" else "100" # 记录加速度
            dec = self.ui.le_dec.text() if self.ui.le_dec.text() != "" else "100" # 记录减速度
            vel = self.ui.le_vel.text() if self.ui.le_vel.text() != "" else "100" # 记录动作速度
            position = self.ui.le_position.text() if self.ui.le_position.text() != "" else "100" # 记录动作幅度
            inhibit = self.ui.le_inhibit.text() if self.ui.le_inhibit.text() != "" else "10" # 记录禁止时间
            # 保存上述参数
            Motor.config(mode, int(acc), int(dec), int(vel), int(position), int(inhibit))
            # 生效
            self.enable_set_param(True, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
        self.enable_init_motor(True, "生效") # 激活 生效
    
    # 初始化电机参数
    def init_motor(self):
        def change(status):
            if status == True:
                self.enable_init_motor(False, "等待")
                self.enable_start_pdo(False)
                # IO的按钮
                if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
                else: self.enable_open_io(False) # 失效按钮 打开IO
            else:
                self.enable_init_motor(False, "完成")
                self.enable_start_pdo(True) # 激活 开启TPDO
                # IO的按钮
                if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
                else: self.enable_open_io(True) # 生效按钮 打开IO
        # self.init_motor_thread = InitMotorThread()
        self.init_motor_thread.running_signal.connect(change)
        self.init_motor_thread.start()
    
    # 开启PDO通讯
    def start_pdo(self):
        def change(status):
            if status == True:
                self.enable_start_pdo(False, "启动中")
                # IO的按钮
                if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
                else: self.enable_open_io(False) # 失效按钮 打开IO
            else:
                self.motor_update.start() # 开启线程
                # self.enable_close_device(False)
                self.enable_start_pdo(False, "已启动")
                self.enable_stop_pdo(True, "关闭状态读取")
                self.enable_set_param(False) # 失效
                self.enable_save_param(False) # 失效
                self.enable_choose_mode(False, mode=None) # 失效
                ''' 状态控制 '''
                self.enable_quick_stop(True) # 生效
                self.enable_release_break(True) # 生效
                ''' 操作 '''
                if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
                    self.enable_joint_control(True) # 生效
                elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
                    self.enable_joint_control(True) # 生效
                    self.enable_end_control(True) # 生效
                else: pass
                # 调用一下解除 安全起见
                self.release_break()
                # 设置状态flag
                self.motor_is_running = True
                # IO的按钮
                if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
                else: self.enable_open_io(True) # 生效按钮 打开IO
        self.start_pdo_thread.running_signal.connect(change)
        self.start_pdo_thread.start()
        self.stop_pdo_thread = StopPDO()
    
    # 关闭PDO通讯
    def stop_pdo(self):
        def change(status):
            if status == True:
                self.enable_stop_pdo(False, "关闭中")
                # IO的按钮
                if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
                else: self.enable_open_io(False) # 失效按钮 打开IO
            else:
                self.enable_close_device(True)
                self.enable_start_pdo(True, "启动状态读取")
                self.enable_stop_pdo(False, "已关闭")
                if self.is_admin:
                    self.enable_set_param(True)
                    self.enable_choose_mode(True, mode=None)
                self.enable_save_param(True)
                ''' 状态控制 '''
                self.enable_quick_stop(False) # 失效
                self.enable_release_break(False) # 失效
                self.enable_enable_servo(False) # 失效
                ''' 遥操作 '''
                self.enable_joint_control(False) # 失效
                for i in range(1, 11):
                    getattr(self, f"enable_motor_group_{i}")(False) # 失效
                self.enable_end_control(False) # 失效
                self.enable_exit(False) # 失效
                # 设置状态flag
                self.motor_is_running = False
                # IO的按钮
                if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
                else: self.enable_open_io(True) # 生效按钮 打开IO
        self.motor_update.stop()
        self.motor_update.wait()
        self.motor_update = CANopenUpdateThread(self.update_canopen) # 重新创建线程
        self.stop_pdo_thread.running_signal.connect(change)
        self.stop_pdo_thread.start()
        self.start_pdo_thread = StartPDO()
    
    # 急停所有电机
    def quick_stop(self):
        # 功能
        Motor.quick_stop()
        # 开关状态
        self.enable_quick_stop(False)
        self.enable_release_break(False)
        self.enable_enable_servo(True)
        # 停止控制
        self.quit_joint_control()
        self.exit_end_control()
        # 开关状态
        self.enable_joint_control(False)
        self.enable_end_control(False)
    
    # 解除抱闸 所有电机
    def release_break(self):
        # 功能
        Motor.release_brake()
        # 开关状态
        self.enable_quick_stop(False)
        self.enable_release_break(False)
        self.enable_enable_servo(True)
        # 停止控制
        self.quit_joint_control()
        self.exit_end_control()
        # 开关状态
        self.enable_joint_control(False)
        self.enable_end_control(False)
    
    # 使能所有电机
    def enable_servo(self):
        # 开关状态
        self.enable_quick_stop(True)
        self.enable_release_break(True)
        self.enable_enable_servo(False)
        # 控制 生效
        if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
            self.enable_joint_control(True) # 生效
        elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
            self.enable_joint_control(True) # 生效
            self.enable_end_control(True) # 生效
        else: pass
        # 功能
        Motor.enable_servo()
    
    # 进入单电机控制
    def joint_control(self):
        self.enable_joint_control(False)
        self.enable_end_control(False)
        self.enable_quit(True)
        for node_id in Motor.motor_dict:
            if Motor.motor_dict[node_id].motor_is_checked:
                getattr(self, f"enable_motor_group_{node_id}")(True)
            else: pass
    
    # 函数工厂 电机的正反转功能
    def joint_forward_factory(self, i):
        # 位置模式
        if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
            setattr(self, f"joint_{i}", JointControlThread(getattr(self, f"motor_{i}"), True, Motor.position, Motor.velocity))
        # 速度模式
        elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
            setattr(self, f"joint_{i}", JointControlSpeedModeThread(getattr(self, f"motor_{i}"), True, 200))
        else: pass
        getattr(self, f"joint_{i}").start()
        getattr(self.ui, f"bt_negative_{i}").setEnabled(False)
    def joint_reverse_factory(self, i):
        # 位置模式
        if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
            setattr(self, f"joint_{i}", JointControlThread(getattr(self, f"motor_{i}"), False, Motor.position, Motor.velocity))
        # 速度模式
        elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
            setattr(self, f"joint_{i}", JointControlSpeedModeThread(getattr(self, f"motor_{i}"), False, 200))
        else: pass
        getattr(self, f"joint_{i}").start()
        getattr(self.ui, f"bt_positive_{i}").setEnabled(False)
    def joint_stop_factory(self, i):
        getattr(self, f"joint_{i}").stop()
        getattr(self, f"joint_{i}").wait()
        getattr(self.ui, f"bt_positive_{i}").setEnabled(True)
        getattr(self.ui, f"bt_negative_{i}").setEnabled(True)
    for i in range(1,11):
        exec(f"def joint_forward_{i}(self): self.joint_forward_factory({i})")
        exec(f"def joint_reverse_{i}(self): self.joint_reverse_factory({i})")
        exec(f"def joint_stop_{i}(self): self.joint_stop_factory({i})")

    # 退出单电机控制
    def quit_joint_control(self):
        self.enable_joint_control(True) # 生效
        self.enable_end_control(True) # 生效
        self.enable_quit(False)
        for i in range(1,11):
            getattr(self, f"enable_motor_group_{i}")(False)
    
    # 电机10 归位
    def homing_pdo(self):
        print("\033[0;33mmotor 10 is homing ...\033[0m")
        if not self.io.switch_1:
            self.motor_10.sdo_write_32("control_mode", protocol.CONTROL_MODE["speed_control"], check=False)
            self.motor_10.set_speed(100)
            self.motor_10.set_servo_status("servo_enable/start")
            while True:
                if self.io.switch_1:
                    self.motor_10.set_servo_status("quick_stop")
                    break
    
    def homing_sdo(self):
        print("\033[0;33mmotor 10 is homing ...\033[0m")
        for motor in Motor.motor_dict.values():
            motor.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_close"])
            time.sleep(0.001)
        if not self.io.switch_1:
            self.motor_10.sdo_write_32("target_speed", 100)
            self.motor_10.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_enable/start"])
            time.sleep(0.001)
            while True:
                if self.io.switch_1:
                    self.motor_10.sdo_write_32("control_word", protocol.CONTROL_WORD["quick_stop"])
                    time.sleep(0.001)
                    break

























    ''' 力传感器 '''
    # 界面 状态设置
    def enable_start_force(self, flag, text=None):
        self.ui.force_open.setEnabled(flag)
        if text != None: self.ui.force_open.setText(text)
    
    def enable_stop_force(self, flag, text=None):
        self.ui.force_stop.setEnabled(flag)
        if text != None: self.ui.force_stop.setText(text)
    
    # 按钮clicked信号绑定slot函数
    def set_sensor_jumping(self):
        self.ui.force_open.clicked.connect(self.start_force) # 开始读取
        self.ui.force_stop.clicked.connect(self.stop_force) # 停止读取
    
    # 开启数值读取
    def start_force(self):
        def running():
            self.enable_start_force(False, "正在调零...")
        def next():
            self.enable_start_force(False, "力值更新已开启")
            self.enable_stop_force(True)
            # 状态flag
            self.sensor_is_running = True
        if Sensor.check_status():
            self.sensor_update.start_signal.connect(running)
            self.sensor_update.init_signal.connect(next)
            self.sensor_update.start()
        else: pass
    
    # 停止数值读取
    def stop_force(self):
        self.sensor_update.stop()
        self.sensor_update.wait()
        # self.sensor_update = SensorUpdateThread(self.sensor_mutex, self.update_sensor)
        self.enable_start_force(True, "获取力值")
        self.enable_stop_force(False)
        # 状态flag
        self.sensor_is_running = False
    
    # 在界面上显示数值
    def update_sensor(self):
        if not self.sensor_update.is_stop:
            for i in range(1, 11):
                force = getattr(self, f"sensor_{i}").force
                if force == None: force_str = "<span style=\"color:#ffffff;\">{}</span>".format("No Data")
                else: 
                    if force > 0: color = "#ff0000"
                    else:
                        if abs(force) <= 10: color = "#00ff00"
                        else: color = "#ffff00"
                    force_str = "<span style=\"color:{};\">{}</span>".format(color, abs(force))
                getattr(self.ui, f"force_{i}").setText(force_str)
        else:
            self.sensor_update = SensorUpdateThread(self.sensor_mutex, self.update_sensor)
            no_data = "<span style=\"color:#ffff00;\">{}</span>".format("NO DATA")
            for i in range(1, 11):
                getattr(self.ui, f"force_{i}").setText(no_data)


    




    












    ''' IO模块 '''
    # 界面 状态设置
    def enable_open_io(self, flag, text=None):
        self.ui.io_open.setEnabled(flag)
        if text != None: self.ui.io_open.setText(text)
    
    def enable_close_io(self, flag, text=None):
        self.ui.io_stop.setEnabled(flag)
        if text != None: self.ui.io_stop.setText(text)
    
    def enable_io(self, flag):
        self.ui.fa_1.setEnabled(flag)
        self.ui.fa_2.setEnabled(flag)
        self.ui.fa_3.setEnabled(flag)
        self.ui.fa_4.setEnabled(flag)
    
    def enable_io_1(self, flag):
        self.ui.bt_close_1.setEnabled(flag)
        self.ui.bt_open_1.setEnabled(not flag)
    
    def enable_io_2(self, flag):
        self.ui.bt_close_2.setEnabled(flag)
        self.ui.bt_open_2.setEnabled(not flag)
    
    def enable_io_3(self, flag):
        self.ui.bt_close_3.setEnabled(flag)
        self.ui.bt_open_3.setEnabled(not flag)
    
    def enable_io_4(self, flag):
        self.ui.bt_close_4.setEnabled(flag)
        self.ui.bt_open_4.setEnabled(not flag)
    
    def enable_status_io(self, flag):
        self.ui.fa_status_1.setEnabled(flag)
        self.ui.fa_status_2.setEnabled(flag)
        self.ui.fa_status_3.setEnabled(flag)
        self.ui.fa_status_4.setEnabled(flag)
        self.ui.switch_status_1.setEnabled(flag)
        self.ui.switch_status_2.setEnabled(flag)
    
    # 按钮clicked信号绑定slot函数
    def set_io_jumping(self):
        self.ui.io_open.clicked.connect(self.open_module) # 开启
        self.ui.io_stop.clicked.connect(self.close_module) # 关闭
        # 电磁阀的4组开关
        for i in range(1,5):
            getattr(self.ui, f"bt_open_{i}").clicked.connect(getattr(self, f"io_open_{i}"))
            getattr(self.ui, f"bt_close_{i}").clicked.connect(getattr(self, f"io_close_{i}"))
    
    # 启动模块PDO
    def open_module(self):
        self.io.start_output()
        if self.io.is_start:
            self.enable_open_io(False, "已打开")
            self.enable_close_io(True, "关闭IO输出")
            self.enable_io(True)
            self.enable_status_io(True)
            # for i in range (1,5):
            #     getattr(self, f"enable_io_{i}")(False)
            # 电磁阀的初始状态
            self.io_close_1() # 夹紧
            self.io_close_2() # 夹紧
            self.io_close_3() # 夹紧
            self.io_open_4() # 张开
            # 状态flag
            self.io_is_running = True
        else: pass
    
    # 关闭模块PDO
    def close_module(self):
        # 电磁阀的初始状态
        self.io_close_1() # 夹紧
        self.io_close_2() # 夹紧
        self.io_close_3() # 夹紧
        self.io_open_4() # 张开
        self.io.stop_output()
        if not self.io.is_start:
            self.enable_open_io(True, "启动IO输出")
            self.enable_close_io(False, "IO已关闭")
            self.enable_io(False)
            # 状态flag
            self.io_is_running = False
        else: pass

    # 函数工厂 四组电磁阀开关按钮的slot函数
    def io_open_factory(self, i):
        if self.io.set_channel_status(True, f"{i}"): getattr(self, f"enable_io_{i}")(True)
    def io_close_factory(self, i):
        if self.io.set_channel_status(False, f"{i}"): getattr(self, f"enable_io_{i}")(False)
    for i in range(1,4):
        # io_open_1 io_open_2 io_open_3
        exec(f"def io_open_{i}(self): self.io_open_factory({i})")
        # io_close_1 io_close_2 io_close_3
        exec(f"def io_close_{i}(self): self.io_close_factory({i})")
    
    def io_open_4(self):
        if self.io.set_channel_status(False, "4"): # 不通电的时候 爪子是打开的
            self.enable_io_4(True)
            self.motor_10.permission = True # 手爪打开的时候 电机10是可以动的
        else: pass
    
    def io_close_4(self):
        if self.io.set_channel_status(True, "4"): # 通电的时候 爪子是关闭的
            self.enable_io_4(False)
            self.motor_10.permission = True # 手爪关闭的时候 电机10不可以移动
        else: pass

    # 显示电磁阀的状态
    def update_magnetic_valve(self):
        # 小爪 开启状态较为危险
        open = "<span style=\"color:#ffff00;\">{}</span>".format("OPEN")
        close = "<span style=\"color:#00ff00;\">{}</span>".format("CLOSE")
        self.ui.isopen_1.setText(open if self.io.channel_1 else close)
        self.ui.isopen_2.setText(open if self.io.channel_2 else close)
        self.ui.isopen_3.setText(open if self.io.channel_3 else close)
        # 大爪 关闭状态危险
        open = "<span style=\"color:#00ff00;\">{}</span>".format("OPEN")
        close = "<span style=\"color:#ff0000;\">{}</span>".format("CLOSE")
        self.ui.isopen_4.setText(open if self.io.channel_4 else close)

    
    
    









    
    
    
    

    
    ''' 操纵杆 '''
    # 界面 状态设置
    def enable_end_control(self, flag, text=None):
        self.ui.bt_end_control.setEnabled(flag)
        if text != None: self.ui.bt_end_control.setText(text)
    
    def enable_exit(self, flag, text=None):
        self.ui.bt_exit.setEnabled(flag)
        if text != None: self.ui.bt_exit.setText(text)
    
    # 按钮clicked信号绑定slot函数
    def set_control_jumping(self):
        # self.ui.bt_quick_stop.clicked.connect(self.quick_stop)
        # self.ui.bt_quick_stop.setShortcut("space")
        # self.ui.bt_unlock.clicked.connect(self.release_break)
        # self.ui.bt_enable.clicked.connect(self.enable_servo)
        # self.ui.bt_joint_control.clicked.connect(self.joint_control)
        # self.ui.bt_quit.clicked.connect(self.quit_joint_control)
        # for node_id in Motor.motor_dict:
        #     getattr(self.ui, f"bt_positive_{node_id}").pressed.connect(getattr(self, f"joint_forward_{node_id}"))
        #     getattr(self.ui, f"bt_positive_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
        #     getattr(self.ui, f"bt_negative_{node_id}").pressed.connect(getattr(self, f"joint_reverse_{node_id}"))
        #     getattr(self.ui, f"bt_negative_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
        
        # for i in range(1,5):
        #     getattr(self.ui, f"bt_open_{i}").clicked.connect(getattr(self, f"io_open_{i}"))
        #     getattr(self.ui, f"bt_close_{i}").clicked.connect(getattr(self, f"io_close_{i}"))
        
        self.ui.bt_end_control.clicked.connect(self.end_control)
        self.ui.bt_exit.clicked.connect(self.exit_end_control)
    
    # 进入末端操作
    def end_control(self):
        self.ui.stackedWidget.setCurrentIndex(2) # 显示第3页
        self.enable_joint_control(False)
        self.enable_end_control(False)
        self.enable_exit(True)
        ''' 操纵 '''
        def start_manipulate(status):
            if status == 0: self.motor_10.set_servo_status("quick_stop") # 不操作
            else: # 操作
                if self.motor_10.is_in_range(): self.motor_10.set_servo_status("servo_enable/start") # 在范围内
                else: # 超出范围
                    if self.motor_10.current_position >= self.motor_10.max_position: # 大于max
                        if self.motor_10.target_speed < 0: self.motor_10.set_servo_status("servo_enable/start") # 反方向速度 可动
                        else: self.motor_10.set_servo_status("quick_stop") # 继续超出范围 不可动
                    elif self.motor_10.current_position <= self.motor_10.min_position: # 小于min
                        if self.motor_10.target_speed > 0: self.motor_10.set_servo_status("servo_enable/start") # 反方向速度 可动
                        else: self.motor_10.set_servo_status("quick_stop") # 继续超出范围 不可动
        # def test_stop(status):
        #     if status == 1: self.quick_stop()
        # def test_speed_1(value):
        #     if abs(value) < 0.1:
        #         self.motor_1.set_servo_status("quick_stop")
        #         self.action = False
        #     else:
        #         if not self.action:
        #             self.motor_1.set_servo_status("servo_enable/start")
        #             self.action = True
        #         self.motor_1.set_speed(int(value*100))
        def test_speed(value):
            if abs(value) < 0.1: value = 0
            self.motor_10.set_speed(int(value*200))
        ''' 绑定 '''
        self.joystick.button_signal_0.connect(start_manipulate)
        # self.joystick.button_signal_1.connect(test_stop)
        self.joystick.axis_signal_2.connect(test_speed)
        # self.joystick.axis_signal_1.connect(test_speed_1)
        self.joystick.start() # 开启joystick线程
    
    # 退出末端操作
    def exit_end_control(self):
        self.enable_end_control(True)
        self.enable_exit(False)
        self.joystick.stop() # 终止joystick线程
        self.joystick.wait() # 等待退出
        self.joystick = JoystickThread() # 必须重新创建线程！！！
    
      
    
    
    
    










    
    
    
    
    ''' CANOPEN总线下的状态更新 '''
    # 界面 状态设置
    def enable_status_1(self, flag):
        self.ui.status_1.setEnabled(flag)
    def enable_status_2(self, flag):
        self.ui.status_2.setEnabled(flag)
    def enable_status_3(self, flag):
        self.ui.status_3.setEnabled(flag)
    def enable_status_4(self, flag):
        self.ui.status_4.setEnabled(flag)
    def enable_status_5(self, flag):
        self.ui.status_5.setEnabled(flag)
    def enable_status_6(self, flag):
        self.ui.status_6.setEnabled(flag)
    def enable_status_7(self, flag):
        self.ui.status_7.setEnabled(flag)
    def enable_status_8(self, flag):
        self.ui.status_8.setEnabled(flag)
    def enable_status_9(self, flag):
        self.ui.status_9.setEnabled(flag)
    def enable_status_10(self, flag):
        self.ui.status_10.setEnabled(flag)
    
    # 将总线上的TPDO的信息显示到界面上
    def update_canopen(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
            # 状态
            status = getattr(self, f"motor_{node_id}").motor_status
            if status == "ready_to_switch_on":
                color = "#ff0000"
                status = "RELEASE"
            elif status == "switch_on_disabled":
                color = "#ff0000"
                status = "STOP"
            elif status == "switched_on":
                color = "#ffff00"
                status = "READY"
            elif status == "operation_enabled":
                color = "#00ff00"
                status = "RUNNING"
            else:
                color = "#0000ff"
                status = "UNKNOWN"
            status_str = "<span style=\"color:{};\">{}</span>".format(color, status)
            getattr(self.ui, f"servo_{node_id}").setText(status_str)
            
            # 位置
            position = getattr(self, f"motor_{node_id}").current_position
            max_position = getattr(self, f"motor_{node_id}").max_position
            min_position = getattr(self, f"motor_{node_id}").min_position
            range = max_position - min_position
            if position < max_position and position > min_position:
                if position <= min_position+range*0.1 or position >= max_position-range*0.1: color = "#ff0000"
                elif position > min_position+range*0.1 and position <= min_position+range*0.3 or position < max_position-range*0.1 and position >= max_position-range*0.3: color = "#ffff00"
                else: color = "#00ff00"
            else: color = "#0000ff"
            position_str = "<span style=\"color:{};\">{}</span>".format(color, position)
            getattr(self.ui, f"position_{node_id}").setText(position_str)
            
            # 速度
            speed = getattr(self, f"motor_{node_id}").current_speed
            max_speed = getattr(self, f"motor_{node_id}").max_speed
            min_speed = getattr(self, f"motor_{node_id}").min_speed
            range = max_speed - min_speed
            if speed <= max_speed and speed >= min_speed:
                if speed <= min_speed+range*0.1 or speed >= max_speed-range*0.1: color = "#ff0000"
                elif speed > min_speed+range*0.1 and speed <= min_speed+range*0.3 or speed < max_speed-range*0.1 and speed >= max_speed-range*0.3: color = "#ffff00"
                else: color = "#00ff00"
            else: color = "#0000ff"
            speed_str = "<span style=\"color:{};\">{}</span>".format(color, speed)
            getattr(self.ui, f"speed_{node_id}").setText(speed_str)
        
        # IO模块下的接近开关
        elif node_id in IoModule.io_dict.keys():
            # 当检测到接近开关 做出动作
            if self.io.switch_1 or self.io.switch_2:
                # 电机10关键动作
                # 电机10关键动作
                
                
                


                self.motor_10.set_servo_status("quick_stop") # 先停止电机10
                # self.motor_10.permission = False # 取消运动权限
                
                


                
                # 电机10关键动作 
                # 电机10关键动作
            else: pass
            # 更新接近开关状态
            warning = "<span style=\"color:#ff0000;\">{}</span>".format("WARNING")
            clear = "<span style=\"color:#00ff00;\">{}</span>".format("CLEAR")
            self.ui.isclose_1.setText(warning if self.io.switch_2 else clear)
            self.ui.isclose_2.setText(warning if self.io.switch_1 else clear)
        
        else: pass
    



    
    ''' test '''
    def test_1(self):
        # self.test_thread = NewTest(self.motor_10, self.sensor_1)

        # self.test_thread.start()

        print("==================test_1==================")

        print("set 1: ", self.motor_2.set_position_and_velocity(1000, 100))
        time.sleep(0.001)
        print("set 2: ", self.motor_5.set_position_and_velocity(1000, 100))
        time.sleep(0.001)
        print("set 3: ", self.motor_8.set_position_and_velocity(1000, 100))
        time.sleep(0.001)

        print("ready 1: ", self.motor_2.set_servo_status("position_mode_ready"))
        time.sleep(0.001)
        print("ready 3: ", self.motor_5.set_servo_status("position_mode_ready"))
        time.sleep(0.001)
        print("ready 2: ", self.motor_8.set_servo_status("position_mode_ready"))
        time.sleep(0.001)

        print("action 1: ", self.motor_2.set_servo_status("position_mode_action"))
        time.sleep(0.001)
        print("action 3: ", self.motor_5.set_servo_status("position_mode_action"))
        time.sleep(0.001)
        print("action 2: ", self.motor_8.set_servo_status("position_mode_action"))
        
    
    def test_2(self):
        print("==================test_2==================")

        print("set 1: ", self.motor_2.set_position_and_velocity(-1000, 100))
        time.sleep(0.001)
        print("set 2: ", self.motor_5.set_position_and_velocity(-1000, 100))
        time.sleep(0.001)
        print("set 3: ", self.motor_8.set_position_and_velocity(-1000, 100))
        time.sleep(0.001)

        print("ready 1: ", self.motor_2.set_servo_status("position_mode_ready"))
        time.sleep(0.001)
        print("ready 3: ", self.motor_5.set_servo_status("position_mode_ready"))
        time.sleep(0.001)
        print("ready 2: ", self.motor_8.set_servo_status("position_mode_ready"))
        time.sleep(0.001)
        

        print("action 1: ", self.motor_2.set_servo_status("position_mode_action"))
        time.sleep(0.001)
        print("action 3: ", self.motor_5.set_servo_status("position_mode_action"))
        time.sleep(0.001)
        print("action 2: ", self.motor_8.set_servo_status("position_mode_action"))
        
    
    def stop_test(self):
        # self.test_thread.stop()
        # self.test_thread.wait()
        ...

from PyQt5.QtCore import QThread
class NewTest(QThread):
    
    def __init__(self, motor_1, motor_2, motor_3) -> None:
        super().__init__()
        self.__is_stop = False
        self.__motor_1 = motor_1
        self.__motor_2 = motor_2
        self.__motor_3 = motor_3
    
    def run(self):
        self.__motor.set_position_and_velocity(25600, 100)
        
        while not self.__is_stop:
            
            if self.__sensor.force < 0 and abs(self.__sensor.force) < 1:
                self.__motor.set_servo_status("servo_enable/start")
            else:
                self.__motor.set_servo_status("quick_stop")

        self.__motor.set_speed(0)
        self.__motor.set_servo_status("quick_stop")
    
    def stop(self):
        self.__is_stop = True
        self.__motor.set_speed(0)
        self.__motor.set_servo_status("quick_stop")