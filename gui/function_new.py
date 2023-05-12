# -*- coding:utf-8 -*-


''' function_new.py 新GUI功能函数 v1.0 '''


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QThread, QMutex, QObject, pyqtSignal


# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from gui.ui_login import Ui_MainWindow as Ui_LoginPanel
from gui.ui_control_new import Ui_MainWindow as Ui_ControlPanel
from usbcan.function import UsbCan
import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor
from motor.function import Motor
from motor.update import MotorUpdateThread
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
        self.usbcan_0 = UsbCan.is_show_log(False)("0") # 通道0
        self.usbcan_1 = UsbCan.is_show_log(False)("1") # 通道1
        
        CanOpenBusProcessor.link_device(self.usbcan_0) # 将CANopen总线绑定至CAN卡的通道0
        CanOpenBusProcessor.is_show_log(False)
        
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
        self.motor_11 = Motor(11)
        self.motor_12 = Motor(12)
        self.motor_13 = Motor(13)
        self.motor_14 = Motor(14)
        self.check_list = [False] * 14 # 记录电机检查状态的列表
        self.checked_num = 0

        self.joystick = JoystickThread() # 操纵杆

        self.motor_update = MotorUpdateThread() # 电机状态更新

        self.initial_status() # 根据权限 显示初始页面
        
        # 按钮跳转的设置 也就是把按钮的signal和slot绑定
        self.set_menu_jumping() # 菜单
        self.set_usbcan_jumping() # USBCAN
        self.set_motor_jumping() # 电机
        self.set_control_jumping() # 控制面板
        self.set_update_jumping() # 状态更新

        self.show() # 显示界面

    
    '''
        界面的初始显示状态
    '''
    def initial_status(self) -> None:
        ''' 菜单 '''
        self.enable_menu(True, True, True) # 初始化激活 控制激活 日志激活
        ''' 页面 '''
        self.ui.stackedWidget.setCurrentIndex(0) # 显示第1页
        ''' USBCAN '''
        self.enable_open_device(True, "打开设备") # 激活
        self.enable_channel0(False, False, "打开通道0", "重置0") # 失效
        self.enable_channel1(False, False, "打开通道1", "重置1") # 失效
        self.enable_close_device(False, "关闭") # 失效
        ''' 电机 '''
        self.enable_check_all(False, "一键检查所有电机") # 失效
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
        self.enable_check_11(False, "M11") # 失效
        self.enable_check_12(False, "M12") # 失效
        self.enable_check_13(False, "M13") # 失效
        self.enable_check_14(False, "M14") # 失效
        self.enable_choose_mode(False, "position_control") # 失效
        # 参数设置 失效 并显示默认值
        self.enable_set_param(False, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
        self.enable_save_param(False, "保存") # 失效
        self.enable_init_motor(False, "生效") # 失效
        self.enable_start_pdo(False, "启动PDO") # 失效
        self.enable_stop_pdo(False, "停止PDO") # 失效
        ''' 状态控制 '''
        self.enable_quick_stop(False, "急停") # 失效
        self.enable_release_break(False, "解除抱闸") # 失效
        self.enable_enable_servo(False, "伺服使能") # 失效
        ''' 遥操作 '''
        self.enable_joint_control(False, "关节操作") # 失效
        self.enable_quit(False, "退出") # 失效
        self.enable_motor_group_1(False) # 失效
        self.enable_motor_group_2(False) # 失效
        self.enable_motor_group_3(False) # 失效
        self.enable_motor_group_4(False) # 失效
        self.enable_motor_group_5(False) # 失效
        self.enable_motor_group_6(False) # 失效
        self.enable_motor_group_7(False) # 失效
        self.enable_motor_group_8(False) # 失效
        self.enable_motor_group_9(False) # 失效
        self.enable_motor_group_10(False) # 失效
        self.enable_motor_group_11(False) # 失效
        self.enable_motor_group_12(False) # 失效
        self.enable_motor_group_13(False) # 失效
        self.enable_motor_group_14(False) # 失效
        self.enable_end_control(False, "末端操作") # 失效
        self.enable_exit(False, "退出") # 失效
        ''' 状态 '''
        self.enable_status_1(False)
        self.enable_status_2(False)
        self.enable_status_3(False)
        self.enable_status_4(False)
        self.enable_status_5(False)
        self.enable_status_6(False)
        self.enable_status_7(False)
        self.enable_status_8(False)
        self.enable_status_9(False)
        self.enable_status_10(False)
        self.enable_status_11(False)
        self.enable_status_12(False)
        self.enable_status_13(False)
        self.enable_status_14(False)
    

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
                self.enable_check_11(True)
                self.enable_check_12(True)
                self.enable_check_13(True)
                self.enable_check_14(True)
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
                self.check_list = [False] * 14
                self.checked_num = 0
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
            for checked in self.check_list:
                if checked: self.checked_num += 1
            if self.checked_num == 1:
                if self.is_admin:
                    self.enable_choose_mode(True) # 激活 模式选择
                    self.enable_set_param(True) # 激活 设置参数
                self.enable_save_param(True) # 激活 保存参数
                self.enable_check_all(False, "结束")
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
            check_bus_11()
            check_bus_12()
            check_bus_13()
            check_bus_14()
        ''' 单个检查 '''
        def check_bus_1():
            if self.motor_1.check_bus_status():
                if self.motor_1.check_motor_status():
                    self.enable_check_1(False, "就绪")
                    self.check_list[0] = True
                    self.enable_status_1(True)
            self.ui.servo_1.setText(self.motor_1.motor_status)
            self.ui.position_1.setText(str(self.motor_1.current_position))
            self.ui.speed_1.setText(str(self.motor_1.current_speed))
            __ischecked()
        def check_bus_2():
            if self.motor_2.check_bus_status():
                if self.motor_2.check_motor_status():
                    self.enable_check_2(False, "就绪")
                    self.check_list[1] = True
                    self.enable_status_2(True)
            self.ui.servo_2.setText(self.motor_2.motor_status)
            self.ui.position_2.setText(str(self.motor_2.current_position))
            self.ui.speed_2.setText(str(self.motor_2.current_speed))
            __ischecked()
        def check_bus_3():
            if self.motor_3.check_bus_status():
                if self.motor_3.check_motor_status():
                    self.enable_check_3(False, "就绪")
                    self.check_list[2] = True
                    self.enable_status_3(True)
            self.ui.servo_3.setText(self.motor_3.motor_status)
            self.ui.position_3.setText(str(self.motor_3.current_position))
            self.ui.speed_3.setText(str(self.motor_3.current_speed))
            __ischecked()
        def check_bus_4():
            if self.motor_4.check_bus_status():
                if self.motor_4.check_motor_status():
                    self.enable_check_4(False, "就绪")
                    self.check_list[3] = True
                    self.enable_status_4(True)
            self.ui.servo_4.setText(self.motor_4.motor_status)
            self.ui.position_4.setText(str(self.motor_4.current_position))
            self.ui.speed_4.setText(str(self.motor_4.current_speed))
            __ischecked()
        def check_bus_5():
            if self.motor_5.check_bus_status():
                if self.motor_5.check_motor_status():
                    self.enable_check_5(False, "就绪")
                    self.check_list[4] = True
                    self.enable_status_5(True)
            self.ui.servo_5.setText(self.motor_5.motor_status)
            self.ui.position_5.setText(str(self.motor_5.current_position))
            self.ui.speed_5.setText(str(self.motor_5.current_speed))
            __ischecked()
        def check_bus_6():
            if self.motor_6.check_bus_status():
                if self.motor_6.check_motor_status():
                    self.enable_check_6(False, "就绪")
                    self.check_list[5] = True
                    self.enable_status_6(True)
            self.ui.servo_6.setText(self.motor_6.motor_status)
            self.ui.position_6.setText(str(self.motor_6.current_position))
            self.ui.speed_6.setText(str(self.motor_6.current_speed))
            __ischecked()
        def check_bus_7():
            if self.motor_7.check_bus_status():
                if self.motor_7.check_motor_status():
                    self.enable_check_7(False, "就绪")
                    self.check_list[6] = True
                    self.enable_status_7(True)
            self.ui.servo_7.setText(self.motor_7.motor_status)
            self.ui.position_7.setText(str(self.motor_7.current_position))
            self.ui.speed_7.setText(str(self.motor_7.current_speed))
            __ischecked()
        def check_bus_8():
            if self.motor_8.check_bus_status():
                if self.motor_8.check_motor_status():
                    self.enable_check_8(False, "就绪")
                    self.check_list[7] = True
                    self.enable_status_8(True)
            self.ui.servo_8.setText(self.motor_8.motor_status)
            self.ui.position_8.setText(str(self.motor_8.current_position))
            self.ui.speed_8.setText(str(self.motor_8.current_speed))
            __ischecked()
        def check_bus_9():
            if self.motor_9.check_bus_status():
                if self.motor_9.check_motor_status():
                    self.enable_check_9(False, "就绪")
                    self.check_list[8] = True
                    self.enable_status_9(True)
            self.ui.servo_9.setText(self.motor_9.motor_status)
            self.ui.position_9.setText(str(self.motor_9.current_position))
            self.ui.speed_9.setText(str(self.motor_9.current_speed))
            __ischecked()
        def check_bus_10():
            if self.motor_10.check_bus_status():
                if self.motor_10.check_motor_status():
                    self.enable_check_10(False, "就绪")
                    self.check_list[9] = True
                    self.enable_status_10(True)
            self.ui.servo_10.setText(self.motor_10.motor_status)
            self.ui.position_10.setText(str(self.motor_10.current_position))
            self.ui.speed_10.setText(str(self.motor_10.current_speed))
            __ischecked()
        def check_bus_11():
            if self.motor_11.check_bus_status():
                if self.motor_11.check_motor_status():
                    self.enable_check_11(False, "就绪")
                    self.check_list[10] = True
                    self.enable_status_11(True)
            self.ui.servo_11.setText(self.motor_11.motor_status)
            self.ui.position_11.setText(str(self.motor_11.current_position))
            self.ui.speed_11.setText(str(self.motor_11.current_speed))
            __ischecked()
        def check_bus_12():
            if self.motor_12.check_bus_status():
                if self.motor_12.check_motor_status():
                    self.enable_check_12(False, "就绪")
                    self.check_list[11] = True
                    self.enable_status_12(True)
            self.ui.servo_12.setText(self.motor_12.motor_status)
            self.ui.position_12.setText(str(self.motor_12.current_position))
            self.ui.speed_12.setText(str(self.motor_12.current_speed))
            __ischecked()
        def check_bus_13():
            if self.motor_13.check_bus_status():
                if self.motor_13.check_motor_status():
                    self.enable_check_13(False, "就绪")
                    self.check_list[12] = True
                    self.enable_status_13(True)
            self.ui.servo_13.setText(self.motor_13.motor_status)
            self.ui.position_13.setText(str(self.motor_13.current_position))
            self.ui.speed_13.setText(str(self.motor_13.current_speed))
            __ischecked()
        def check_bus_14():
            if self.motor_14.check_bus_status():
                if self.motor_14.check_motor_status():
                    self.enable_check_14(False, "就绪")
                    self.check_list[13] = True
                    self.enable_status_14(True)
            self.ui.servo_14.setText(self.motor_14.motor_status)
            self.ui.position_14.setText(str(self.motor_14.current_position))
            self.ui.speed_14.setText(str(self.motor_14.current_speed))
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
            Motor.init_config() # 将所有参数生效给所有电机
            self.enable_init_motor(False, "完成")
            self.enable_start_pdo(True) # 激活 开启TPDO
        def start_pdo():
            Motor.start_feedback()
            self.motor_update.start() # 开启线程
            self.enable_close_device(False)
            self.enable_start_pdo(False)
            self.enable_stop_pdo(True)
            self.enable_set_param(False) # 失效
            self.enable_save_param(False) # 失效
            self.enable_choose_mode(False, mode=None) # 失效
            ''' 状态控制 '''
            self.enable_quick_stop(True) # 生效
            self.enable_release_break(True) # 生效
            ''' 遥操作 '''
            if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
                self.enable_joint_control(True) # 生效
            self.enable_end_control(True) # 生效
        def stop_pdo():
            Motor.stop_feedback()
            self.motor_update.stop()
            self.motor_update.wait()
            self.motor_update = MotorUpdateThread() # 重新创建线程
            self.enable_close_device(True)
            self.enable_start_pdo(True)
            self.enable_stop_pdo(False)
            self.enable_set_param(True)
            self.enable_save_param(True)
            self.enable_choose_mode(True, mode=None)
            ''' 状态控制 '''
            self.enable_quick_stop(False) # 失效
            self.enable_release_break(False) # 失效
            self.enable_enable_servo(False) # 失效
            ''' 遥操作 '''
            self.enable_joint_control(False) # 失效
            self.enable_motor_group_1(False) # 失效
            self.enable_motor_group_2(False) # 失效
            self.enable_motor_group_3(False) # 失效
            self.enable_motor_group_4(False) # 失效
            self.enable_motor_group_5(False) # 失效
            self.enable_motor_group_6(False) # 失效
            self.enable_motor_group_7(False) # 失效
            self.enable_motor_group_8(False) # 失效
            self.enable_motor_group_9(False) # 失效
            self.enable_motor_group_10(False) # 失效
            self.enable_motor_group_11(False) # 失效
            self.enable_motor_group_12(False) # 失效
            self.enable_motor_group_13(False) # 失效
            self.enable_motor_group_14(False) # 失效
            self.enable_end_control(False) # 失效
            self.enable_exit(False) # 失效
        '''
            绑定signal和slot
        '''
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
        self.ui.check_11.clicked.connect(lambda: check_bus_11())
        self.ui.check_12.clicked.connect(lambda: check_bus_12())
        self.ui.check_13.clicked.connect(lambda: check_bus_13())
        self.ui.check_14.clicked.connect(lambda: check_bus_14())
        self.ui.bt_save.clicked.connect(lambda: save_config())
        self.ui.bt_launch.clicked.connect(lambda: init_motor())
        self.ui.start.clicked.connect(lambda: start_pdo())
        self.ui.stop.clicked.connect(lambda: stop_pdo())
    
    ''' 控制 '''
    def set_control_jumping(self):
        ''' 状态控制 '''
        def quick_stop():
            Motor.quick_stop()
            self.enable_quick_stop(False)
            self.enable_release_break(False)
            self.enable_enable_servo(True)
            self.enable_joint_control(False)
            self.enable_end_control(False)
            self.enable_exit(False)
            self.joystick.stop() # 终止joystick线程
            self.joystick.wait() # 等待退出
            self.joystick = JoystickThread() # 必须重新创建线程！！！
        def release_break():
            Motor.release_brake()
            self.enable_quick_stop(False)
            self.enable_release_break(False)
            self.enable_enable_servo(True)
            self.enable_joint_control(False)
            self.enable_end_control(False)
            self.enable_exit(False)
            self.joystick.stop() # 终止joystick线程
            self.joystick.wait() # 等待退出
            self.joystick = JoystickThread() # 必须重新创建线程！！！
        def enable_servo():
            self.enable_quick_stop(True)
            self.enable_release_break(True)
            self.enable_enable_servo(False)
            if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
                self.enable_joint_control(True) # 生效
            self.enable_end_control(True)
            Motor.enable_servo()
        ''' 关节控制 '''
        def joint_control():
            self.enable_joint_control(False)
            self.enable_end_control(False)
            self.enable_quit(True)
            for i, checked in enumerate(self.check_list):
                if checked:
                    method = getattr(self, f"enable_motor_group_{i+1}")
                    method(True)
        def quit_joint_control():
            if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
                self.enable_joint_control(True) # 生效
            self.enable_end_control(True)
            self.enable_quit(False)
            for i in range(14):
                getattr(self, f"enable_motor_group_{i+1}")(False)
        ''' 各个关节 '''
        def motor_1_positive():
            self.motor_1.action()
        def motor_1_negative():
            self.motor_1.action(reverse=True)
        def motor_2_positive():
            self.motor_2.action()
        def motor_2_negative():
            self.motor_2.action(reverse=True)
        def motor_3_positive():
            self.motor_3.action()
        def motor_3_negative():
            self.motor_3.action(reverse=True)
        def motor_4_positive():
            self.motor_4.action()
        def motor_4_negative():
            self.motor_4.action(reverse=True)
        def motor_5_positive():
            self.motor_5.action()
        def motor_5_negative():
            self.motor_5.action(reverse=True)
        def motor_6_positive():
            self.motor_6.action()
        def motor_6_negative():
            self.motor_6.action(reverse=True)
        def motor_7_positive():
            self.motor_7.action()
        def motor_7_negative():
            self.motor_7.action(reverse=True)
        def motor_8_positive():
            self.motor_8.action()
        def motor_8_negative():
            self.motor_8.action(reverse=True)
        def motor_9_positive():
            self.motor_9.action()
        def motor_9_negative():
            self.motor_9.action(reverse=True)
        def motor_10_positive():
            self.motor_10.action()
        def motor_10_negative():
            self.motor_10.action(reverse=True)
        def motor_11_positive():
            self.motor_11.action()
        def motor_11_negative():
            self.motor_11.action(reverse=True)
        def motor_12_positive():
            self.motor_12.action()
        def motor_12_negative():
            self.motor_12.action(reverse=True)
        def motor_13_positive():
            self.motor_13.action()
        def motor_13_negative():
            self.motor_13.action(reverse=True)
        def motor_14_positive():
            self.motor_14.action()
        def motor_14_negative():
            self.motor_14.action(reverse=True)
        ''' 操纵杆 '''
        def end_control():
            self.enable_joint_control(False)
            self.enable_end_control(False)
            self.enable_exit(True)
            def test_front(status):
                if status == 1:
                    self.motor_1.action()
                    self.motor_2.action()
                    self.motor_3.action()
                    self.motor_4.action()
                    self.motor_5.action()
                    self.motor_6.action()
                    self.motor_7.action()
                    self.motor_8.action()
                    self.motor_9.action()
            def test_back(status):
                if status == 1:
                    self.motor_1.action(reverse=True)
                    self.motor_2.action(reverse=True)
                    self.motor_3.action(reverse=True)
                    self.motor_4.action(reverse=True)
                    self.motor_5.action(reverse=True)
                    self.motor_6.action(reverse=True)
                    self.motor_7.action(reverse=True)
                    self.motor_8.action(reverse=True)
                    self.motor_9.action(reverse=True)
            def test_speed_1(value):
                if abs(value) < 0.5: value = 0
                speed = int(value*100)
                self.motor_1.action_speed(speed)
            def test_speed_4(value):
                if abs(value) < 0.5: value = 0
                speed = int(value*100)
                self.motor_4.action_speed(speed)
            self.joystick.button_signal_5.connect(test_front)
            self.joystick.button_signal_4.connect(test_back)
            self.joystick.axis_signal_4.connect(test_speed_4)
            self.joystick.axis_signal_1.connect(test_speed_1)
            self.joystick.start() # 开启joystick线程
        def exit_end_control():
            if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
                self.enable_joint_control(True) # 生效
            self.enable_end_control(True)
            self.enable_exit(False)
            self.joystick.stop() # 终止joystick线程
            self.joystick.wait() # 等待退出
            self.joystick = JoystickThread() # 必须重新创建线程！！！
        '''
            绑定signal和slot
        '''
        self.ui.bt_quick_stop.clicked.connect(lambda: quick_stop())
        self.ui.bt_unlock.clicked.connect(lambda: release_break())
        self.ui.bt_enable.clicked.connect(lambda: enable_servo())
        self.ui.bt_joint_control.clicked.connect(lambda: joint_control())
        self.ui.bt_quit.clicked.connect(lambda: quit_joint_control())
        # 1
        self.ui.bt_positive_1.clicked.connect(lambda: motor_1_positive())
        self.ui.bt_positive_1.setAutoRepeat(True)
        self.ui.bt_positive_1.setAutoRepeatInterval(50)
        self.ui.bt_positive_1.setAutoRepeatDelay(100)
        self.ui.bt_negative_1.clicked.connect(lambda: motor_1_negative())
        self.ui.bt_negative_1.setAutoRepeat(True)
        self.ui.bt_negative_1.setAutoRepeatInterval(50)
        self.ui.bt_negative_1.setAutoRepeatDelay(100)
        # 2
        self.ui.bt_positive_2.clicked.connect(lambda: motor_2_positive())
        self.ui.bt_positive_2.setAutoRepeat(True)
        self.ui.bt_positive_2.setAutoRepeatInterval(50)
        self.ui.bt_positive_2.setAutoRepeatDelay(100)
        self.ui.bt_negative_2.clicked.connect(lambda: motor_2_negative())
        self.ui.bt_negative_2.setAutoRepeat(True)
        self.ui.bt_negative_2.setAutoRepeatInterval(50)
        self.ui.bt_negative_2.setAutoRepeatDelay(100)
        # 3
        self.ui.bt_positive_3.clicked.connect(lambda: motor_3_positive())
        self.ui.bt_positive_3.setAutoRepeat(True)
        self.ui.bt_positive_3.setAutoRepeatInterval(50)
        self.ui.bt_positive_3.setAutoRepeatDelay(100)
        self.ui.bt_negative_3.clicked.connect(lambda: motor_3_negative())
        self.ui.bt_negative_3.setAutoRepeat(True)
        self.ui.bt_negative_3.setAutoRepeatInterval(50)
        self.ui.bt_negative_3.setAutoRepeatDelay(100)
        # 4
        self.ui.bt_positive_4.clicked.connect(lambda: motor_4_positive())
        self.ui.bt_positive_4.setAutoRepeat(True)
        self.ui.bt_positive_4.setAutoRepeatInterval(50)
        self.ui.bt_positive_4.setAutoRepeatDelay(100)
        self.ui.bt_negative_4.clicked.connect(lambda: motor_4_negative())
        self.ui.bt_negative_4.setAutoRepeat(True)
        self.ui.bt_negative_4.setAutoRepeatInterval(50)
        self.ui.bt_negative_4.setAutoRepeatDelay(100)
        # 5
        self.ui.bt_positive_5.clicked.connect(lambda: motor_5_positive())
        self.ui.bt_positive_5.setAutoRepeat(True)
        self.ui.bt_positive_5.setAutoRepeatInterval(50)
        self.ui.bt_positive_5.setAutoRepeatDelay(100)
        self.ui.bt_negative_5.clicked.connect(lambda: motor_5_negative())
        self.ui.bt_negative_5.setAutoRepeat(True)
        self.ui.bt_negative_5.setAutoRepeatInterval(50)
        self.ui.bt_negative_5.setAutoRepeatDelay(100)
        # 6
        self.ui.bt_positive_6.clicked.connect(lambda: motor_6_positive())
        self.ui.bt_positive_6.setAutoRepeat(True)
        self.ui.bt_positive_6.setAutoRepeatInterval(50)
        self.ui.bt_positive_6.setAutoRepeatDelay(100)
        self.ui.bt_negative_6.clicked.connect(lambda: motor_6_negative())
        self.ui.bt_negative_6.setAutoRepeat(True)
        self.ui.bt_negative_6.setAutoRepeatInterval(50)
        self.ui.bt_negative_6.setAutoRepeatDelay(100)
        # 7
        self.ui.bt_positive_7.clicked.connect(lambda: motor_7_positive())
        self.ui.bt_positive_7.setAutoRepeat(True)
        self.ui.bt_positive_7.setAutoRepeatInterval(50)
        self.ui.bt_positive_7.setAutoRepeatDelay(100)
        self.ui.bt_negative_7.clicked.connect(lambda: motor_7_negative())
        self.ui.bt_negative_7.setAutoRepeat(True)
        self.ui.bt_negative_7.setAutoRepeatInterval(50)
        self.ui.bt_negative_7.setAutoRepeatDelay(100)
        # 8
        self.ui.bt_positive_8.clicked.connect(lambda: motor_8_positive())
        self.ui.bt_positive_8.setAutoRepeat(True)
        self.ui.bt_positive_8.setAutoRepeatInterval(50)
        self.ui.bt_positive_8.setAutoRepeatDelay(100)
        self.ui.bt_negative_8.clicked.connect(lambda: motor_8_negative())
        self.ui.bt_negative_8.setAutoRepeat(True)
        self.ui.bt_negative_8.setAutoRepeatInterval(50)
        self.ui.bt_negative_8.setAutoRepeatDelay(100)
        # 9
        self.ui.bt_positive_9.clicked.connect(lambda: motor_9_positive())
        self.ui.bt_positive_9.setAutoRepeat(True)
        self.ui.bt_positive_9.setAutoRepeatInterval(50)
        self.ui.bt_positive_9.setAutoRepeatDelay(100)
        self.ui.bt_negative_9.clicked.connect(lambda: motor_9_negative())
        self.ui.bt_negative_9.setAutoRepeat(True)
        self.ui.bt_negative_9.setAutoRepeatInterval(50)
        self.ui.bt_negative_9.setAutoRepeatDelay(100)
        # 10
        self.ui.bt_positive_10.clicked.connect(lambda: motor_10_positive())
        self.ui.bt_positive_10.setAutoRepeat(True)
        self.ui.bt_positive_10.setAutoRepeatInterval(50)
        self.ui.bt_positive_10.setAutoRepeatDelay(100)
        self.ui.bt_negative_10.clicked.connect(lambda: motor_10_negative())
        self.ui.bt_negative_10.setAutoRepeat(True)
        self.ui.bt_negative_10.setAutoRepeatInterval(50)
        self.ui.bt_negative_10.setAutoRepeatDelay(100)
        # 11
        self.ui.bt_positive_11.clicked.connect(lambda: motor_11_positive())
        self.ui.bt_positive_11.setAutoRepeat(True)
        self.ui.bt_positive_11.setAutoRepeatInterval(50)
        self.ui.bt_positive_11.setAutoRepeatDelay(100)
        self.ui.bt_negative_11.clicked.connect(lambda: motor_11_negative())
        self.ui.bt_negative_11.setAutoRepeat(True)
        self.ui.bt_negative_11.setAutoRepeatInterval(50)
        self.ui.bt_negative_11.setAutoRepeatDelay(100)
        # 12
        self.ui.bt_positive_12.clicked.connect(lambda: motor_12_positive())
        self.ui.bt_positive_12.setAutoRepeat(True)
        self.ui.bt_positive_12.setAutoRepeatInterval(50)
        self.ui.bt_positive_12.setAutoRepeatDelay(100)
        self.ui.bt_negative_12.clicked.connect(lambda: motor_12_negative())
        self.ui.bt_negative_12.setAutoRepeat(True)
        self.ui.bt_negative_12.setAutoRepeatInterval(50)
        self.ui.bt_negative_12.setAutoRepeatDelay(100)
        # 13
        self.ui.bt_positive_13.clicked.connect(lambda: motor_13_positive())
        self.ui.bt_positive_13.setAutoRepeat(True)
        self.ui.bt_positive_13.setAutoRepeatInterval(50)
        self.ui.bt_positive_13.setAutoRepeatDelay(100)
        self.ui.bt_negative_13.clicked.connect(lambda: motor_13_negative())
        self.ui.bt_negative_13.setAutoRepeat(True)
        self.ui.bt_negative_13.setAutoRepeatInterval(50)
        self.ui.bt_negative_13.setAutoRepeatDelay(100)
        # 14
        self.ui.bt_positive_14.clicked.connect(lambda: motor_14_positive())
        self.ui.bt_positive_14.setAutoRepeat(True)
        self.ui.bt_positive_14.setAutoRepeatInterval(50)
        self.ui.bt_positive_14.setAutoRepeatDelay(100)
        self.ui.bt_negative_14.clicked.connect(lambda: motor_14_negative())
        self.ui.bt_negative_14.setAutoRepeat(True)
        self.ui.bt_negative_14.setAutoRepeatInterval(50)
        self.ui.bt_negative_14.setAutoRepeatDelay(100)
        self.ui.bt_end_control.clicked.connect(lambda: end_control())
        self.ui.bt_exit.clicked.connect(lambda: exit_end_control())
    
    ''' 状态更新 '''
    def set_update_jumping(self):
        def update(node_id):
            status = getattr(self, f"motor_{node_id}").motor_status
            position = getattr(self, f"motor_{node_id}").current_position
            speed = getattr(self, f"motor_{node_id}").current_speed
            getattr(self.ui, f"servo_{node_id}").setText(status)
            getattr(self.ui, f"position_{node_id}").setText(str(position))
            getattr(self.ui, f"speed_{node_id}").setText(str(speed))
        self.motor_update.update_signal.connect(update)

    '''
        以下是控制界面中 所有按钮的状态设置函数 包含是否激活和显示文字
    '''
    ''' 
        菜单 
    '''
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
    def enable_check_11(self, flag, text=None):
        self.ui.check_11.setEnabled(flag)
        if text != None: self.ui.check_11.setText(text)
    def enable_check_12(self, flag, text=None):
        self.ui.check_12.setEnabled(flag)
        if text != None: self.ui.check_12.setText(text)
    def enable_check_13(self, flag, text=None):
        self.ui.check_13.setEnabled(flag)
        if text != None: self.ui.check_13.setText(text)
    def enable_check_14(self, flag, text=None):
        self.ui.check_14.setEnabled(flag)
        if text != None: self.ui.check_14.setText(text)
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
    
    '''
        第二页
    '''
    ''' 状态控制 '''
    def enable_quick_stop(self, flag, text=None):
        self.ui.bt_quick_stop.setEnabled(flag)
        if text != None: self.ui.bt_quick_stop.setText(text)
    def enable_release_break(self, flag, text=None):
        self.ui.bt_unlock.setEnabled(flag)
        if text != None: self.ui.bt_unlock.setText(text)
    def enable_enable_servo(self, flag, text=None):
        self.ui.bt_enable.setEnabled(flag)
        if text != None: self.ui.bt_enable.setText(text)
    ''' 遥操作 '''
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
    def enable_motor_group_11(self, flag):
        self.ui.group_11.setEnabled(flag)
    def enable_motor_group_12(self, flag):
        self.ui.group_12.setEnabled(flag)
    def enable_motor_group_13(self, flag):
        self.ui.group_13.setEnabled(flag)
    def enable_motor_group_14(self, flag):
        self.ui.group_14.setEnabled(flag)
    def enable_end_control(self, flag, text=None):
        self.ui.bt_end_control.setEnabled(flag)
        if text != None: self.ui.bt_end_control.setText(text)
    def enable_exit(self, flag, text=None):
        self.ui.bt_exit.setEnabled(flag)
        if text != None: self.ui.bt_exit.setText(text)

    '''
        第三页
    '''
    def enable_status_1(self, flag):
        self.ui.status_1.setEnabled(flag)
    def enable_status_2(self, flag):
        self.ui.status_2.setEnabled(flag)
    def enable_status_3(self, flag):
        self.ui.status_3.setEnabled(flag)
    def enable_status_4(self, flag):
        self.ui.status_4.setEnabled(flag)
    def enable_status_5(self, flag):
        self.ui.status_6.setEnabled(flag)
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
    def enable_status_11(self, flag):
        self.ui.status_11.setEnabled(flag)
    def enable_status_12(self, flag):
        self.ui.status_12.setEnabled(flag)
    def enable_status_13(self, flag):
        self.ui.status_13.setEnabled(flag)
    def enable_status_14(self, flag):
        self.ui.status_14.setEnabled(flag)
    