# -*- coding:utf-8 -*-


''' gui.py maxon motor v1.0 '''


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QMutex


# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from maxon_motor_epos2.control_panel import Ui_MainWindow as Ui_ControlPanel
from maxon_motor_epos2.usbcan import UsbCan
import maxon_motor_epos2.protocol as protocol
from maxon_motor_epos2.processor import CanOpenBusProcessor
from maxon_motor_epos2.motor import Motor
from maxon_motor_epos2.threads import CANopenUpdateThread, InitMotorThread





''' 控制界面 '''
class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        
        # CAN卡实例化
        self.usbcan_0 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("0") # 通道0
        self.usbcan_1 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("1") # 通道1
        self.usbcan_is_running = False
        
        CanOpenBusProcessor.link_device(self.usbcan_0) # 将CANopen总线绑定至CAN卡的通道0
        CanOpenBusProcessor.is_show_log(False)
        
        # 电机实例化
        self.motor_1 = Motor(1, [-10000000,10000000], [-200,200])
        self.motor_2 = Motor(2, [-10000000,10000000], [-200,200])
        self.motor_is_running = False

        
        self.initial_screen()
        
        self.set_jumping()

        self.show()

       

    

    def initial_screen(self) -> None:
        # 显示页面
        self.ui.stackedWidget.setCurrentIndex(0)
        
        # 波特率可选
        self.ui.bx_rate0.setEnabled(True)
        self.ui.bx_rate1.setEnabled(True)

        # 可打开设备
        self.ui.bt_open_device.setEnabled(True)
        self.ui.bt_open_device.setText("Open Device")

        # 不可初始化电机
        self.ui.bt_init_motor.setEnabled(False)
        self.ui.bt_init_motor.setText("Initialize Motor")

        # 电机状态
        self.ui.motor_check_1.setEnabled(False)
        self.ui.motor_check_1.setText("Motor 1 Unchecked")

        self.ui.motor_check_2.setEnabled(False)
        self.ui.motor_check_2.setText("Motor 2 Unchecked")

        # 状态控制按钮
        self.ui.bt_shut_down.setEnabled(False)
        self.ui.bt_switch_on.setEnabled(False)
        self.ui.bt_enable_operation.setEnabled(False)
        self.ui.bt_disable_operation.setEnabled(False)
        self.ui.bt_disable_voltage.setEnabled(False)
        self.ui.bt_quick_stop.setEnabled(False)
        self.ui.bt_fault_reset.setEnabled(False)

        # 参数
        self.ui.type_t.setEnabled(False)
        self.ui.type_t.setChecked(True)

        self.ui.type_s.setEnabled(False)

        self.ui.tx_acc.setEnabled(False)
        self.ui.le_acc.setEnabled(False)

        self.ui.tx_dec.setEnabled(False)
        self.ui.le_dec.setEnabled(False)

        self.ui.tx_quick.setEnabled(False)
        self.ui.le_quick.setEnabled(False)

        self.ui.tx_inhibit.setEnabled(False)
        self.ui.le_inhibit.setEnabled(False)

        self.ui.bt_save.setEnabled(False)

        self.ui.bt_launch.setEnabled(False)
        self.ui.bt_launch.setText("Launch")

        # 电机
        self.ui.bt_lock.setEnabled(False)
        self.ui.bt_adaptive.setEnabled(False)
        
        self.ui.bt_forward_1.setEnabled(False)
        self.ui.bt_reverse_1.setEnabled(False)
        self.ui.bt_halt_1.setEnabled(False)

        self.ui.bt_forward_2.setEnabled(False)
        self.ui.bt_reverse_2.setEnabled(False)
        self.ui.bt_halt_2.setEnabled(False)

        self.ui.slider_1.setEnabled(False)
        self.ui.slider_1.setProperty("value", 0)
        self.ui.slider_2.setEnabled(False)
        self.ui.slider_2.setProperty("value", 0)

        self.ui.position_1.setEnabled(True)
        self.ui.position_1.setText("Position")
        self.ui.velocity_1.setEnabled(True)
        self.ui.velocity_1.setText("Velocity")
        self.ui.status_1.setEnabled(True)
        self.ui.status_1.setText("Status")

        self.ui.position_2.setEnabled(True)
        self.ui.position_2.setText("Position")
        self.ui.velocity_2.setEnabled(True)
        self.ui.velocity_2.setText("Velocity")
        self.ui.status_2.setEnabled(True)
        self.ui.status_2.setText("Status")
    
    def set_jumping(self) -> None:
        self.ui.bt_open_device.clicked.connect(self.open_device)
        
        self.ui.bt_init_motor.clicked.connect(self.initialize_motor)




    def open_device(self) -> None:
        if UsbCan.open_device():
            self.usbcan_0.set_timer(self.ui.bx_rate0.currentText())
            self.usbcan_1.set_timer(self.ui.bx_rate1.currentText())
            
            if self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.ui.bt_open_device.setEnabled(False)
                self.ui.bt_open_device.setText("Device Ready")
                
                self.ui.bt_init_motor.setEnabled(True)

                self.read_canopen_thread = CANopenUpdateThread(self.canopen_pdo, self.canopen_sdo)
                self.read_canopen_thread.start()
            else: pass

        else: pass
    
    # 读取CAN卡数据 进行反馈
    def canopen_pdo(self, node_id):
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
            getattr(self.ui, f"status_{node_id}").setText(status_str)
            
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
            getattr(self.ui, f"velocity_{node_id}").setText(speed_str)
        
        else: pass
    
    def canopen_sdo(self, node_id, status, label, value):
        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value)
        print("canopen_sdo")

    # def canopen_nmt(self, node_id, label):
    #     CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
    #     print("canopen_nmt")

    def initialize_motor(self) -> None:
        # self.init = InitMotorThread()
        # self.motor_1.set_bus_status("enter_pre-operational_state")
        print(self.motor_1.get_bus_status())
        # self.motor_1.set_bus_status("start_remote_node")
        # print(self.motor_1.get_bus_status())
        
        # print(self.motor_1.get_motor_status())
    


    
    
    
    
    










    
    

    
    # # 按钮信号绑定slot函数
    # def set_motor_jumping(self) -> None:
    #     self.ui.check_all.clicked.connect(self.check_motor) # 检查
    #     self.ui.bt_save.clicked.connect(self.save_config) # 保存
    #     self.ui.bt_launch.clicked.connect(self.init_motor) # 生效
    #     self.ui.start.clicked.connect(self.start_pdo) # 开启PDO
    #     self.ui.stop.clicked.connect(self.stop_pdo) # 关闭PDO

    #     self.ui.bt_quick_stop.clicked.connect(self.quick_stop) # 急停
    #     self.ui.bt_quick_stop.setShortcut("space")
    #     self.ui.bt_unlock.clicked.connect(self.release_break) # 掉电
    #     self.ui.bt_enable.clicked.connect(self.enable_servo) # 使能

    #     self.ui.bt_joint_control.clicked.connect(self.joint_control) # 进入单电机控制
    #     self.ui.bt_quit.clicked.connect(self.quit_joint_control) # 退出单电机控制
        
    #     # 10组电机控制按钮
    #     for node_id in Motor.motor_dict:
    #         getattr(self.ui, f"bt_positive_{node_id}").pressed.connect(getattr(self, f"joint_forward_{node_id}"))
    #         getattr(self.ui, f"bt_positive_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
    #         getattr(self.ui, f"bt_negative_{node_id}").pressed.connect(getattr(self, f"joint_reverse_{node_id}"))
    #         getattr(self.ui, f"bt_negative_{node_id}").released.connect(getattr(self, f"joint_stop_{node_id}"))
    
    # # 检查状态
    # def check_motor(self):
    #     def change(status):
    #         if status == True:
    #             self.enable_check_all(False, "进行中...")
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
    #             else: self.enable_open_io(False) # 失效按钮 打开IO
    #         else:
    #             self.enable_check_all(True, "再次检查")
    #             self.check_motor_thread = CheckMotorThread()
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
    #             else: self.enable_open_io(True) # 生效按钮 打开IO
    #     def update(node_id):
    #         getattr(self, f"enable_check_{node_id}")(False, "OK")
    #         getattr(self, f"enable_status_{node_id}")(True)
    #     def status(node_id):
    #         getattr(self.ui, f"servo_{node_id}").setText(getattr(self, f"motor_{node_id}").motor_status)
    #         getattr(self.ui, f"position_{node_id}").setText(str(getattr(self, f"motor_{node_id}").current_position))
    #         getattr(self.ui, f"speed_{node_id}").setText(str(getattr(self, f"motor_{node_id}").current_speed))
    #     def next():
    #         if self.is_admin:
    #             self.enable_choose_mode(True) # 激活 模式选择
    #             self.enable_set_param(True) # 激活 设置参数
    #         else: pass
    #         self.enable_save_param(True) # 激活 保存参数
    #         self.enable_check_all(False, "完成")
    #         # IO的按钮
    #         if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
    #         else: self.enable_open_io(True) # 生效按钮 打开IO
    #     self.check_motor_thread.running_signal.connect(change)
    #     self.check_motor_thread.check_signal.connect(update)
    #     self.check_motor_thread.status_signal.connect(status)
    #     self.check_motor_thread.finish_signal.connect(next)
    #     self.check_motor_thread.start()
    
    # # 保存参数
    # def save_config(self):
    #     if not self.is_admin: Motor.config() # 无权限 直接保存默认参数
    #     else: # 管理员权限
    #         mode = "position_control" if self.ui.r_pos.isChecked() else "speed_control" # 记录模式
    #         acc = self.ui.le_acc.text() if self.ui.le_acc.text() != "" else "1000" # 记录加速度
    #         dec = self.ui.le_dec.text() if self.ui.le_dec.text() != "" else "10000" # 记录减速度
    #         vel = self.ui.le_vel.text() if self.ui.le_vel.text() != "" else "100" # 记录动作速度
    #         position = self.ui.le_position.text() if self.ui.le_position.text() != "" else "50" # 记录动作幅度
    #         inhibit = self.ui.le_inhibit.text() if self.ui.le_inhibit.text() != "" else "500" # 记录禁止时间
    #         # 保存上述参数
    #         Motor.config(mode, int(acc), int(dec), int(vel), int(position), int(inhibit))
    #         # 生效
    #         self.enable_set_param(True, "{}".format(Motor.acceleration), "{}".format(Motor.deceleration), "{}".format(Motor.velocity), "{}".format(Motor.position), "{}".format(Motor.inhibit_time))
    #     self.enable_init_motor(True, "生效") # 激活 生效
    
    # # 初始化电机参数
    # def init_motor(self):
    #     def change(status):
    #         if status == True:
    #             self.enable_init_motor(False, "等待")
    #             self.enable_start_pdo(False)
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
    #             else: self.enable_open_io(False) # 失效按钮 打开IO
    #         else:
    #             self.enable_init_motor(False, "完成")
    #             self.enable_start_pdo(True) # 激活 开启TPDO
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
    #             else: self.enable_open_io(True) # 生效按钮 打开IO
    #     # self.init_motor_thread = InitMotorThread()
    #     self.init_motor_thread.running_signal.connect(change)
    #     self.init_motor_thread.start()
    
    # # 开启PDO通讯
    # def start_pdo(self):
    #     def change(status):
    #         if status == True:
    #             self.enable_start_pdo(False, "启动中")
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
    #             else: self.enable_open_io(False) # 失效按钮 打开IO
    #         else:
    #             self.motor_update.start() # 开启线程
    #             self.enable_close_device(False)
    #             self.enable_start_pdo(False, "已启动")
    #             self.enable_stop_pdo(True, "关闭状态读取")
    #             self.enable_set_param(False) # 失效
    #             self.enable_save_param(False) # 失效
    #             self.enable_choose_mode(False, mode=None) # 失效
    #             ''' 状态控制 '''
    #             self.enable_quick_stop(True) # 生效
    #             self.enable_release_break(True) # 生效
    #             ''' 操作 '''
    #             if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
    #                 self.enable_joint_control(True) # 生效
    #             elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
    #                 self.enable_joint_control(True) # 生效
    #                 self.enable_end_control(True) # 生效
    #             else: pass
    #             # 调用一下解除 安全起见
    #             self.release_break()
    #             # 设置状态flag
    #             self.motor_is_running = True
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
    #             else: self.enable_open_io(True) # 生效按钮 打开IO
    #     self.start_pdo_thread.running_signal.connect(change)
    #     self.start_pdo_thread.start()
    #     self.stop_pdo_thread = StopPDO()
    
    # # 关闭PDO通讯
    # def stop_pdo(self):
    #     def change(status):
    #         if status == True:
    #             self.enable_stop_pdo(False, "关闭中")
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(False) # 失效按钮 关闭IO
    #             else: self.enable_open_io(False) # 失效按钮 打开IO
    #         else:
    #             self.enable_close_device(True)
    #             self.enable_start_pdo(True, "启动状态读取")
    #             self.enable_stop_pdo(False, "已关闭")
    #             if self.is_admin:
    #                 self.enable_set_param(True)
    #                 self.enable_choose_mode(True, mode=None)
    #             self.enable_save_param(True)
    #             ''' 状态控制 '''
    #             self.enable_quick_stop(False) # 失效
    #             self.enable_release_break(False) # 失效
    #             self.enable_enable_servo(False) # 失效
    #             ''' 遥操作 '''
    #             self.enable_joint_control(False) # 失效
    #             for i in range(1, 11):
    #                 getattr(self, f"enable_motor_group_{i}")(False) # 失效
    #             self.enable_end_control(False) # 失效
    #             self.enable_exit(False) # 失效
    #             # 设置状态flag
    #             self.motor_is_running = False
    #             # IO的按钮
    #             if self.io_is_running: self.enable_close_io(True) # 生效按钮 关闭IO
    #             else: self.enable_open_io(True) # 生效按钮 打开IO
    #     self.motor_update.stop()
    #     self.motor_update.wait()
    #     self.motor_update = CANopenUpdateThread(self.update_canopen) # 重新创建线程
    #     self.stop_pdo_thread.running_signal.connect(change)
    #     self.stop_pdo_thread.start()
    #     self.start_pdo_thread = StartPDO()
    
    # # 急停所有电机
    # def quick_stop(self):
    #     # 功能
    #     Motor.quick_stop()
    #     # 开关状态
    #     self.enable_quick_stop(False)
    #     self.enable_release_break(False)
    #     self.enable_enable_servo(True)
    #     # 停止控制
    #     self.quit_joint_control()
    #     self.exit_end_control()
    #     # 开关状态
    #     self.enable_joint_control(False)
    #     self.enable_end_control(False)
    
    # # 解除抱闸 所有电机
    # def release_break(self):
    #     # 功能
    #     Motor.release_brake()
    #     # 开关状态
    #     self.enable_quick_stop(False)
    #     self.enable_release_break(False)
    #     self.enable_enable_servo(True)
    #     # 停止控制
    #     self.quit_joint_control()
    #     self.exit_end_control()
    #     # 开关状态
    #     self.enable_joint_control(False)
    #     self.enable_end_control(False)
    
    # # 使能所有电机
    # def enable_servo(self):
    #     # 开关状态
    #     self.enable_quick_stop(True)
    #     self.enable_release_break(True)
    #     self.enable_enable_servo(False)
    #     # 控制 生效
    #     if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
    #         self.enable_joint_control(True) # 生效
    #     elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
    #         self.enable_joint_control(True) # 生效
    #         self.enable_end_control(True) # 生效
    #     else: pass
    #     # 功能
    #     Motor.enable_servo()
    
    # # 进入单电机控制
    # def joint_control(self):
    #     self.enable_joint_control(False)
    #     self.enable_end_control(False)
    #     self.enable_quit(True)
    #     for node_id in Motor.motor_dict:
    #         if Motor.motor_dict[node_id].motor_is_checked:
    #             getattr(self, f"enable_motor_group_{node_id}")(True)
    #         else: pass
    
    # # 函数工厂 电机的正反转功能
    # def joint_forward_factory(self, i):
    #     # 位置模式
    #     if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
    #         setattr(self, f"joint_{i}", JointControlThread(getattr(self, f"motor_{i}"), True, Motor.position, Motor.velocity))
    #     # 速度模式
    #     elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
    #         setattr(self, f"joint_{i}", JointControlSpeedModeThread(getattr(self, f"motor_{i}"), True, 200))
    #     else: pass
    #     getattr(self, f"joint_{i}").start()
    #     getattr(self.ui, f"bt_negative_{i}").setEnabled(False)
    # def joint_reverse_factory(self, i):
    #     # 位置模式
    #     if Motor.control_mode == protocol.CONTROL_MODE["position_control"]:
    #         setattr(self, f"joint_{i}", JointControlThread(getattr(self, f"motor_{i}"), False, Motor.position, Motor.velocity))
    #     # 速度模式
    #     elif Motor.control_mode == protocol.CONTROL_MODE["speed_control"]:
    #         setattr(self, f"joint_{i}", JointControlSpeedModeThread(getattr(self, f"motor_{i}"), False, 200))
    #     else: pass
    #     getattr(self, f"joint_{i}").start()
    #     getattr(self.ui, f"bt_positive_{i}").setEnabled(False)
    # def joint_stop_factory(self, i):
    #     getattr(self, f"joint_{i}").stop()
    #     getattr(self, f"joint_{i}").wait()
    #     getattr(self.ui, f"bt_positive_{i}").setEnabled(True)
    #     getattr(self.ui, f"bt_negative_{i}").setEnabled(True)
    # for i in range(1,11):
    #     exec(f"def joint_forward_{i}(self): self.joint_forward_factory({i})")
    #     exec(f"def joint_reverse_{i}(self): self.joint_reverse_factory({i})")
    #     exec(f"def joint_stop_{i}(self): self.joint_stop_factory({i})")

    # # 退出单电机控制
    # def quit_joint_control(self):
    #     self.enable_joint_control(True) # 生效
    #     self.enable_end_control(True) # 生效
    #     self.enable_quit(False)
    #     for i in range(1,11):
    #         getattr(self, f"enable_motor_group_{i}")(False)
    
    # # 电机10 归位
    # def homing_pdo(self):
    #     print("\033[0;33mmotor 10 is homing ...\033[0m")
    #     if not self.io.switch_1:
    #         self.motor_10.sdo_write_32("control_mode", protocol.CONTROL_MODE["speed_control"], check=False)
    #         self.motor_10.set_speed(100)
    #         self.motor_10.set_servo_status("servo_enable/start")
    #         while True:
    #             if self.io.switch_1:
    #                 self.motor_10.set_servo_status("quick_stop")
    #                 break
    
    # def homing_sdo(self):
    #     print("\033[0;33mmotor 10 is homing ...\033[0m")
    #     for motor in Motor.motor_dict.values():
    #         motor.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_close"])
    #         time.sleep(0.001)
    #     if not self.io.switch_1:
    #         self.motor_10.sdo_write_32("target_speed", 100)
    #         self.motor_10.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_enable/start"])
    #         time.sleep(0.001)
    #         while True:
    #             if self.io.switch_1:
    #                 self.motor_10.sdo_write_32("control_word", protocol.CONTROL_WORD["quick_stop"])
    #                 time.sleep(0.001)
    #                 break

