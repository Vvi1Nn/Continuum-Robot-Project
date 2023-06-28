# -*- coding:utf-8 -*-


''' gui.py maxon motor v1.0 '''


from PyQt5.QtWidgets import QMainWindow


# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from maxon_motor_epos2.control_panel import Ui_MainWindow as Ui_ControlPanel
from maxon_motor_epos2.usbcan import UsbCan
from maxon_motor_epos2.processor import CanOpenBusProcessor
from maxon_motor_epos2.motor import Motor, MotorInitThread
from maxon_motor_epos2.threads import CANopenUpdateThread, ParamLaunchThread, JointControlLockModeThread





''' 控制界面 '''
class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        
        # CAN卡实例化
        self.usbcan_0 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("0") # 通道0
        self.usbcan_1 = UsbCan.set_device_type("USBCAN2", "0").is_show_log(False)("1") # 通道1
        
        CanOpenBusProcessor.link_device(self.usbcan_0) # 将CANopen总线绑定至CAN卡的通道0
        
        # 电机实例化
        self.motor_1 = Motor(1, [-100000000,100000000], [-12700,12700])
        # self.motor_2 = Motor(2, [-100000000,100000000], [-12700,12700])

        
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

        self.ui.bt_shut_down_2.setEnabled(False)
        self.ui.bt_switch_on_2.setEnabled(False)
        self.ui.bt_enable_operation_2.setEnabled(False)
        self.ui.bt_disable_operation_2.setEnabled(False)
        self.ui.bt_disable_voltage_2.setEnabled(False)
        self.ui.bt_quick_stop_2.setEnabled(False)
        self.ui.bt_fault_reset_2.setEnabled(False)

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

        self.ui.le_acc.setPlaceholderText(str(Motor.profile_acceleration))
        self.ui.le_dec.setPlaceholderText(str(Motor.profile_deceleration))
        self.ui.le_quick.setPlaceholderText(str(Motor.quick_stop_deceleration))
        self.ui.le_inhibit.setPlaceholderText(str(Motor.tpdo_inhibit_time))

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

        self.ui.bt_shut_down.clicked.connect(lambda: self.motor_1.shut_down(is_pdo=True))
        self.ui.bt_switch_on.clicked.connect(lambda: self.motor_1.switch_on(is_pdo=True))
        self.ui.bt_enable_operation.clicked.connect(lambda: self.motor_1.enable_operation(is_pdo=True))
        self.ui.bt_disable_operation.clicked.connect(lambda: self.motor_1.disable_operation(is_pdo=True))
        self.ui.bt_disable_voltage.clicked.connect(lambda: self.motor_1.disable_voltage(is_pdo=True))
        self.ui.bt_quick_stop.clicked.connect(lambda: self.motor_1.quick_stop(is_pdo=True))
        self.ui.bt_fault_reset.clicked.connect(lambda: self.motor_1.fault_reset(is_pdo=True))

        self.ui.bt_shut_down_2.clicked.connect(lambda: self.motor_2.shut_down(is_pdo=True))
        self.ui.bt_switch_on_2.clicked.connect(lambda: self.motor_2.switch_on(is_pdo=True))
        self.ui.bt_enable_operation_2.clicked.connect(lambda: self.motor_2.enable_operation(is_pdo=True))
        self.ui.bt_disable_operation_2.clicked.connect(lambda: self.motor_2.disable_operation(is_pdo=True))
        self.ui.bt_disable_voltage_2.clicked.connect(lambda: self.motor_2.disable_voltage(is_pdo=True))
        self.ui.bt_quick_stop_2.clicked.connect(lambda: self.motor_2.quick_stop(is_pdo=True))
        self.ui.bt_fault_reset_2.clicked.connect(lambda: self.motor_2.fault_reset(is_pdo=True))

        self.ui.bt_save.clicked.connect(self.save_param)
        self.ui.bt_launch.clicked.connect(self.launch_param)

        self.ui.bt_adaptive.clicked.connect(self.switch_to_adaptive)
        self.ui.bt_lock.clicked.connect(self.switch_to_lock)



    ''' 打开设备 '''
    def open_device(self) -> None:
        if UsbCan.open_device():
            self.usbcan_0.set_timer(self.ui.bx_rate0.currentText())
            self.usbcan_1.set_timer(self.ui.bx_rate1.currentText())
            
            if self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.ui.bx_rate0.setEnabled(False)
                self.ui.bx_rate1.setEnabled(False)

                self.ui.bt_open_device.setEnabled(False)
                self.ui.bt_open_device.setText("Device Ready")
                
                self.read_canopen_thread = CANopenUpdateThread(self.canopen_pdo_1, self.canopen_pdo_2)
                self.read_canopen_thread.start()

                self.ui.bt_init_motor.setEnabled(True)
            else: pass

        else: pass
    
    ''' TPDO1 ''' 
    def canopen_pdo_1(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
            # 状态
            status = getattr(self, f"motor_{node_id}").servo_status
            if status == "switch_on_disabled":
                color, status = "#ffff00", "INIT"

                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(True)
                    self.ui.bt_switch_on.setEnabled(False)
                    self.ui.bt_disable_voltage.setEnabled(False)
                    self.ui.bt_quick_stop.setEnabled(False)
                    self.ui.bt_disable_operation.setEnabled(False)
                    self.ui.bt_enable_operation.setEnabled(False)
                    self.ui.bt_fault_reset.setEnabled(False)

                    self.ui.group_motor_1.setEnabled(False)
                elif node_id == 2:
                    # 状态控制按钮
                    self.ui.bt_shut_down_2.setEnabled(True)
                    self.ui.bt_switch_on_2.setEnabled(False)
                    self.ui.bt_disable_voltage_2.setEnabled(False)
                    self.ui.bt_quick_stop_2.setEnabled(False)
                    self.ui.bt_disable_operation_2.setEnabled(False)
                    self.ui.bt_enable_operation_2.setEnabled(False)
                    self.ui.bt_fault_reset_2.setEnabled(False)

                    self.ui.group_motor_2.setEnabled(False)
                else: pass
            elif status == "ready_to_switch_on":
                color, status = "#ffff00","READY"

                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(False)
                    self.ui.bt_switch_on.setEnabled(True)
                    self.ui.bt_disable_voltage.setEnabled(True)
                    self.ui.bt_quick_stop.setEnabled(True)
                    self.ui.bt_disable_operation.setEnabled(False)
                    self.ui.bt_enable_operation.setEnabled(False)
                    self.ui.bt_fault_reset.setEnabled(False)

                    self.ui.group_motor_1.setEnabled(True)
                elif node_id == 2:
                    # 状态控制按钮
                    self.ui.bt_shut_down_2.setEnabled(False)
                    self.ui.bt_switch_on_2.setEnabled(True)
                    self.ui.bt_disable_voltage_2.setEnabled(True)
                    self.ui.bt_quick_stop_2.setEnabled(True)
                    self.ui.bt_disable_operation_2.setEnabled(False)
                    self.ui.bt_enable_operation_2.setEnabled(False)
                    self.ui.bt_fault_reset_2.setEnabled(False)

                    self.ui.group_motor_2.setEnabled(True)
                else: pass
            elif status == "switched_on":
                color, status = "#ffff00","DISABLE"
                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(True)
                    self.ui.bt_switch_on.setEnabled(False)
                    self.ui.bt_disable_voltage.setEnabled(True)
                    self.ui.bt_quick_stop.setEnabled(True)
                    self.ui.bt_disable_operation.setEnabled(False)
                    self.ui.bt_enable_operation.setEnabled(True)
                    self.ui.bt_fault_reset.setEnabled(False)
                elif node_id == 2:
                    # 状态控制按钮
                    self.ui.bt_shut_down_2.setEnabled(True)
                    self.ui.bt_switch_on_2.setEnabled(False)
                    self.ui.bt_disable_voltage_2.setEnabled(True)
                    self.ui.bt_quick_stop_2.setEnabled(True)
                    self.ui.bt_disable_operation_2.setEnabled(False)
                    self.ui.bt_enable_operation_2.setEnabled(True)
                    self.ui.bt_fault_reset_2.setEnabled(False)
                else: pass
            elif status == "operation_enable":
                color, status = "#00ff00", "ENABLE"
                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(True)
                    self.ui.bt_switch_on.setEnabled(False)
                    self.ui.bt_disable_voltage.setEnabled(True)
                    self.ui.bt_quick_stop.setEnabled(True)
                    self.ui.bt_disable_operation.setEnabled(True)
                    self.ui.bt_enable_operation.setEnabled(False)
                    self.ui.bt_fault_reset.setEnabled(False)
                elif node_id == 2:
                    # 状态控制按钮
                    self.ui.bt_shut_down_2.setEnabled(True)
                    self.ui.bt_switch_on_2.setEnabled(False)
                    self.ui.bt_disable_voltage_2.setEnabled(True)
                    self.ui.bt_quick_stop_2.setEnabled(True)
                    self.ui.bt_disable_operation_2.setEnabled(True)
                    self.ui.bt_enable_operation_2.setEnabled(False)
                    self.ui.bt_fault_reset_2.setEnabled(False)
            elif status == "quick_stop_active":
                color, status = "#ff0000","STOP"
                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(False)
                    self.ui.bt_switch_on.setEnabled(False)
                    self.ui.bt_disable_voltage.setEnabled(True)
                    self.ui.bt_quick_stop.setEnabled(False)
                    self.ui.bt_disable_operation.setEnabled(False)
                    self.ui.bt_enable_operation.setEnabled(True)
                    self.ui.bt_fault_reset.setEnabled(False)
                elif node_id == 2:
                    self.ui.bt_shut_down_2.setEnabled(False)
                    self.ui.bt_switch_on_2.setEnabled(False)
                    self.ui.bt_disable_voltage_2.setEnabled(True)
                    self.ui.bt_quick_stop_2.setEnabled(False)
                    self.ui.bt_disable_operation_2.setEnabled(False)
                    self.ui.bt_enable_operation_2.setEnabled(True)
                    self.ui.bt_fault_reset_2.setEnabled(False)
            elif status == "fault_reaction_active" or "fault":
                color, status = "#ff0000","FAULT"
                if node_id == 1:
                    # 状态控制按钮
                    self.ui.bt_shut_down.setEnabled(False)
                    self.ui.bt_switch_on.setEnabled(False)
                    self.ui.bt_disable_voltage.setEnabled(False)
                    self.ui.bt_quick_stop.setEnabled(False)
                    self.ui.bt_disable_operation.setEnabled(False)
                    self.ui.bt_enable_operation.setEnabled(False)
                    self.ui.bt_fault_reset.setEnabled(True)
                elif node_id == 2:
                    # 状态控制按钮
                    self.ui.bt_shut_down_2.setEnabled(False)
                    self.ui.bt_switch_on_2.setEnabled(False)
                    self.ui.bt_disable_voltage_2.setEnabled(False)
                    self.ui.bt_quick_stop_2.setEnabled(False)
                    self.ui.bt_disable_operation_2.setEnabled(False)
                    self.ui.bt_enable_operation_2.setEnabled(False)
                    self.ui.bt_fault_reset_2.setEnabled(True)
                else: pass
            else:
                color, status = "#0000ff","UNKNOWN"
            
            status_str = "<span style=\"color:{};\">{}</span>".format(color, status)
            getattr(self.ui, f"status_{node_id}").setText(status_str)
        
        else: pass

    ''' TPDO2 ''' 
    def canopen_pdo_2(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
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
    
    ''' 初始化电机 '''
    def initialize_motor(self) -> None:
        self.motor_init_thread = MotorInitThread()

        def change(status):
            if status == True:
                self.ui.bt_init_motor.setEnabled(False)
                self.ui.bt_init_motor.setText("Running ...")
            else:
                self.ui.bt_init_motor.setEnabled(True)
                self.ui.bt_init_motor.setText("Initialize Motor")

                self.motor_init_thread = MotorInitThread()
        
        def update(node_id):
            getattr(self.ui, f"motor_check_{node_id}").setEnabled(True)
            getattr(self.ui, f"motor_check_{node_id}").setText("<span style=\"color:#00ff00;\">Motor {} Ready</span>".format(node_id))

        def next():
            self.ui.bt_init_motor.setText("Initialize Finish")
            # # 状态控制按钮
            # self.ui.bt_shut_down.setEnabled(True)
            # self.ui.bt_switch_on.setEnabled(False)
            # self.ui.bt_enable_operation.setEnabled(False)
            # self.ui.bt_disable_operation.setEnabled(False)
            # self.ui.bt_disable_voltage.setEnabled(False)
            # self.ui.bt_quick_stop.setEnabled(False)
            # self.ui.bt_fault_reset.setEnabled(False)
                
            # 参数
            self.ui.type_t.setEnabled(True)
            self.ui.type_t.setChecked(True)

            self.ui.type_s.setEnabled(True)

            self.ui.tx_acc.setEnabled(True)
            self.ui.le_acc.setEnabled(True)

            self.ui.tx_dec.setEnabled(True)
            self.ui.le_dec.setEnabled(True)

            self.ui.tx_quick.setEnabled(True)
            self.ui.le_quick.setEnabled(True)

            self.ui.tx_inhibit.setEnabled(True)
            self.ui.le_inhibit.setEnabled(True)

            self.ui.bt_save.setEnabled(True)

            # 电机
            self.ui.bt_adaptive.setEnabled(True)
            self.mode = "lock"

            self.ui.bt_lock.setEnabled(False)

            self.ui.bt_halt_1.setEnabled(False)
            self.ui.bt_halt_2.setEnabled(False)

            self.ui.slider_1.setEnabled(True)
            self.ui.slider_1.setProperty("value", 0)
            self.ui.slider_2.setEnabled(True)
            self.ui.slider_2.setProperty("value", 0)

            self.ui.bt_forward_1.pressed.connect(self.motor_1_forward)
            self.ui.bt_forward_1.released.connect(self.motor_1_stop)
            self.ui.bt_reverse_1.pressed.connect(self.motor_1_reverse)
            self.ui.bt_reverse_1.released.connect(self.motor_1_stop)

            self.ui.bt_forward_2.pressed.connect(self.motor_2_forward)
            self.ui.bt_forward_2.released.connect(self.motor_2_stop)
            self.ui.bt_reverse_2.pressed.connect(self.motor_2_reverse)
            self.ui.bt_reverse_2.released.connect(self.motor_2_stop)
            
            self.ui.bt_forward_1.setEnabled(True)
            self.ui.bt_reverse_1.setEnabled(True)

            self.ui.bt_forward_2.setEnabled(True)
            self.ui.bt_reverse_2.setEnabled(True)
        
        self.motor_init_thread.running_signal.connect(change)
        self.motor_init_thread.check_signal.connect(update)
        self.motor_init_thread.finish_signal.connect(next)

        self.motor_init_thread.start()
    
    ''' 保存 '''
    def save_param(self):
        Motor.motion_profile_type = "trapezoidal" if self.ui.type_t.isChecked() else "sinusoidal"
        Motor.profile_acceleration = int(self.ui.le_acc.text()) if self.ui.le_acc.text() != "" else Motor.profile_acceleration
        Motor.profile_deceleration = int(self.ui.le_dec.text()) if self.ui.le_dec.text() != "" else Motor.profile_deceleration
        Motor.quick_stop_deceleration = int(self.ui.le_quick.text()) if self.ui.le_quick.text() != "" else Motor.quick_stop_deceleration
        Motor.tpdo_inhibit_time = int(self.ui.le_inhibit.text()) if self.ui.le_inhibit.text() != "" else Motor.tpdo_inhibit_time

        self.ui.le_acc.setPlaceholderText(str(Motor.profile_acceleration))
        self.ui.le_dec.setPlaceholderText(str(Motor.profile_deceleration))
        self.ui.le_quick.setPlaceholderText(str(Motor.quick_stop_deceleration))
        self.ui.le_inhibit.setPlaceholderText(str(Motor.tpdo_inhibit_time))

        self.ui.le_acc.clear()
        self.ui.le_dec.clear()
        self.ui.le_quick.clear()
        self.ui.le_inhibit.clear()

        self.param_launch_thread = ParamLaunchThread(Motor.motion_profile_type, Motor.profile_acceleration, Motor.profile_deceleration, Motor.quick_stop_deceleration, Motor.tpdo_inhibit_time)

        self.ui.bt_launch.setEnabled(True)
        self.ui.bt_launch.setText("Launch")

    ''' 生效 '''
    def launch_param(self):
        def change(status):
            if status:
                self.ui.bt_launch.setEnabled(False)
                self.ui.bt_launch.setText("Waiting ...")
            else: self.ui.bt_launch.setText("Done !")
        
        self.param_launch_thread.running_signal.connect(change)

        self.param_launch_thread.start()
    
    ''' lock模式 '''
    def switch_to_lock(self):
        self.ui.bt_adaptive.setEnabled(True)
        self.mode = "lock"

        self.ui.bt_lock.setEnabled(False)

        self.ui.bt_halt_1.setEnabled(False)
        self.ui.bt_halt_2.setEnabled(False)

        self.ui.slider_1.setEnabled(True)
        self.ui.slider_1.setProperty("value", 0)
        self.ui.slider_2.setEnabled(True)
        self.ui.slider_2.setProperty("value", 0)

        self.ui.bt_forward_1.clicked.disconnect(self.motor_1_forward)
        self.ui.bt_reverse_1.clicked.disconnect(self.motor_1_reverse)

        self.ui.bt_forward_2.clicked.disconnect(self.motor_2_forward)
        self.ui.bt_reverse_2.clicked.disconnect(self.motor_2_reverse)

        self.ui.slider_1.valueChanged.disconnect(self.motor_1_set_speed)
        self.ui.slider_2.valueChanged.disconnect(self.motor_2_set_speed)

        self.ui.bt_forward_1.pressed.connect(self.motor_1_forward)
        self.ui.bt_forward_1.released.connect(self.motor_1_stop)
        self.ui.bt_reverse_1.pressed.connect(self.motor_1_reverse)
        self.ui.bt_reverse_1.released.connect(self.motor_1_stop)

        self.ui.bt_forward_2.pressed.connect(self.motor_2_forward)
        self.ui.bt_forward_2.released.connect(self.motor_2_stop)
        self.ui.bt_reverse_2.pressed.connect(self.motor_2_reverse)
        self.ui.bt_reverse_2.released.connect(self.motor_2_stop)

    ''' adaptive模式 '''
    def switch_to_adaptive(self):
        self.ui.bt_adaptive.setEnabled(False)
        self.mode = "adaptive"

        self.ui.bt_lock.setEnabled(True)

        self.ui.bt_halt_1.setEnabled(False)
        self.ui.bt_halt_2.setEnabled(False)

        self.ui.slider_1.setEnabled(False)
        self.ui.slider_1.setProperty("value", 0)

        self.ui.slider_2.setEnabled(False)
        self.ui.slider_2.setProperty("value", 0)

        self.ui.bt_forward_1.pressed.disconnect(self.motor_1_forward)
        self.ui.bt_forward_1.released.disconnect(self.motor_1_stop)
        self.ui.bt_reverse_1.pressed.disconnect(self.motor_1_reverse)
        self.ui.bt_reverse_1.released.disconnect(self.motor_1_stop)

        self.ui.bt_forward_2.pressed.disconnect(self.motor_2_forward)
        self.ui.bt_forward_2.released.disconnect(self.motor_2_stop)
        self.ui.bt_reverse_2.pressed.disconnect(self.motor_2_reverse)
        self.ui.bt_reverse_2.released.disconnect(self.motor_2_stop)

        self.ui.bt_forward_1.clicked.connect(self.motor_1_forward)
        self.ui.bt_reverse_1.clicked.connect(self.motor_1_reverse)
        self.ui.bt_halt_1.clicked.connect(self.motor_1_stop)

        self.ui.bt_forward_2.clicked.connect(self.motor_2_forward)
        self.ui.bt_reverse_2.clicked.connect(self.motor_2_reverse)
        self.ui.bt_halt_2.clicked.connect(self.motor_2_stop)

        self.ui.slider_1.valueChanged.connect(self.motor_1_set_speed)
        self.ui.slider_2.valueChanged.connect(self.motor_2_set_speed)
    
    ''' 正 '''
    def motor_1_forward(self):
        if self.mode == "lock":
            self.motor_1_thread = JointControlLockModeThread(self.motor_1, int(self.ui.slider_1.value()) * 100, is_forward=True)
            
            self.ui.bt_reverse_1.setEnabled(False)
            self.ui.slider_1.setEnabled(False)

            self.motor_1_thread.start()
        
        elif self.mode == "adaptive":
            self.ui.bt_forward_1.setEnabled(False)
            self.ui.bt_reverse_1.setEnabled(False)
            self.ui.bt_halt_1.setEnabled(True)
            
            self.motor_1_direction = True

            self.motor_1.halt(is_pdo=True)

            self.ui.slider_1.setEnabled(True)
            self.ui.slider_1.setProperty("value", 0)
        
        else: pass

    ''' 反 '''
    def motor_1_reverse(self):
        if self.mode == "lock":
            self.motor_1_thread = JointControlLockModeThread(self.motor_1, int(self.ui.slider_1.value()) * 100, is_forward=False)
            
            self.ui.bt_forward_1.setEnabled(False)
            self.ui.slider_1.setEnabled(False)

            self.motor_1_thread.start()
        
        elif self.mode == "adaptive":
            self.ui.bt_forward_1.setEnabled(False)
            self.ui.bt_reverse_1.setEnabled(False)
            self.ui.bt_halt_1.setEnabled(True)
            
            self.motor_1_direction = False

            self.motor_1.halt(is_pdo=True)

            self.ui.slider_1.setEnabled(True)
            self.ui.slider_1.setProperty("value", 0)
        
        else: pass

    ''' 停止 '''
    def motor_1_stop(self):
        if self.mode == "lock":
            self.motor_1_thread.stop()
            self.motor_1_thread.wait()

            self.ui.bt_forward_1.setEnabled(True)
            self.ui.bt_reverse_1.setEnabled(True)
            self.ui.slider_1.setEnabled(True)
        
        elif self.mode == "adaptive":
            self.ui.slider_1.setEnabled(False)
            self.ui.slider_1.setProperty("value", 0)

            time.sleep(0.05)
            
            self.motor_1.disable_operation(is_pdo=True)

            self.ui.bt_halt_1.setEnabled(False)

            self.ui.bt_forward_1.setEnabled(True)
            self.ui.bt_reverse_1.setEnabled(True)
        
        else: pass

    ''' 速度更新 '''
    def motor_1_set_speed(self):
        if self.motor_1_direction:
            self.motor_1.set_speed(int(self.ui.slider_1.value()) * 100, is_pdo=True)
            self.motor_1.enable_operation(is_pdo=True)
        else:
            self.motor_1.set_speed(- int(self.ui.slider_1.value()) * 100, is_pdo=True)
            self.motor_1.enable_operation(is_pdo=True)

    ''' 正 '''
    def motor_2_forward(self):
        if self.mode == "lock":
            self.motor_2_thread = JointControlLockModeThread(self.motor_2, int(self.ui.slider_2.value()) * 100, is_forward=True)
            
            self.ui.bt_reverse_2.setEnabled(False)
            self.ui.slider_2.setEnabled(False)

            self.motor_2_thread.start()
        
        elif self.mode == "adaptive":
            self.ui.bt_forward_2.setEnabled(False)
            self.ui.bt_reverse_2.setEnabled(False)
            self.ui.bt_halt_2.setEnabled(True)
            
            self.motor_2_direction = True

            self.motor_2.halt(is_pdo=True)

            self.ui.slider_2.setEnabled(True)
            self.ui.slider_2.setProperty("value", 0)
        
        else: pass

    ''' 反 '''
    def motor_2_reverse(self):
        if self.mode == "lock":
            self.motor_2_thread = JointControlLockModeThread(self.motor_2, int(self.ui.slider_2.value()) * 100, is_forward=False)
            
            self.ui.bt_forward_2.setEnabled(False)
            self.ui.slider_2.setEnabled(False)

            self.motor_2_thread.start()
        
        elif self.mode == "adaptive":
            self.ui.bt_forward_2.setEnabled(False)
            self.ui.bt_reverse_2.setEnabled(False)
            self.ui.bt_halt_2.setEnabled(True)
            
            self.motor_2_direction = False

            self.motor_2.halt(is_pdo=True)

            self.ui.slider_2.setEnabled(True)
            self.ui.slider_2.setProperty("value", 0)
        
        else: pass

    ''' 停止 '''
    def motor_2_stop(self):
        if self.mode == "lock":
            self.motor_2_thread.stop()
            self.motor_2_thread.wait()

            self.ui.bt_forward_2.setEnabled(True)
            self.ui.bt_reverse_2.setEnabled(True)
            self.ui.slider_2.setEnabled(True)
        
        elif self.mode == "adaptive":
            self.ui.slider_2.setEnabled(False)
            self.ui.slider_2.setProperty("value", 0)

            time.sleep(0.05)
            
            self.motor_2.disable_operation(is_pdo=True)

            self.ui.bt_halt_2.setEnabled(False)

            self.ui.bt_forward_2.setEnabled(True)
            self.ui.bt_reverse_2.setEnabled(True)
        
        else: pass

    ''' 速度更新 '''
    def motor_2_set_speed(self):
        if self.motor_2_direction:
            self.motor_2.set_speed(int(self.ui.slider_2.value()) * 100)
            self.motor_2.enable_operation(is_pdo=True)
        else:
            self.motor_2.set_speed(- int(self.ui.slider_2.value()) * 100)
            self.motor_2.enable_operation(is_pdo=True)

