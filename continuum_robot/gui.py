# -*- coding:utf-8 -*-


''' gui.py GUI v5.0 '''


from PyQt5.QtWidgets import QMainWindow, QTableWidgetItem
from PyQt5.QtCore import QThread, QRunnable, QThreadPool

from PyQt5.QtGui import QPixmap, QColor

import math

from control_panel import Ui_MainWindow as Ui_ControlPanel
from motor import Motor
from sensor import Sensor
from robot import ContinuumRobot


class ControlPanel(QMainWindow):
    
    class RobotFunctionThread(QThread):
        def __init__(self, fn, *args, **kwargs) -> None:
            super().__init__()
            self.fn = fn
            self.args = args
            self.kwargs = kwargs
        def run(self):
            self.fn(*self.args, **self.kwargs)
    
    def __init__(self) -> None:
        super().__init__()
        
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)

        # self.thread_pool = QThreadPool()

        self.robot = ContinuumRobot()

        self.initParameterTable()
        
        self.connectMenu()
        self.connectCustomSignal()
        self.connectButton()

        ''' 高级测试 '''

        # self.ui.test_12.clicked.connect(lambda: self.robot.rope_move("1", -3, 1, is_relative=True))

        self.ui.teleoperation.clicked.connect(self.robot.teleoperation_thread.start)

        self.ui.bt_open_camera.clicked.connect(self.OpenCamera)
        self.ui.bt_close_camera.clicked.connect(self.robot.CloseCamera)
        
        # self.ui.test_24.clicked.connect(lambda: self.robot.rope_move("4", 10, 1, is_relative=True))
        # self.ui.test_25.clicked.connect(lambda: self.robot.rope_move("4", -10, 1, is_relative=True))


        self.show() # 显示界面
    

    def connectMenu(self) -> None:
        self.ui.control_basic.triggered.connect(lambda: self.ui.pages.setCurrentIndex(0))
        self.ui.control_kinematics.triggered.connect(lambda: self.ui.pages.setCurrentIndex(1))
        self.ui.control_video_stream.triggered.connect(lambda: self.ui.pages.setCurrentIndex(2))
        self.ui.control_motor.triggered.connect(lambda: self.ui.pages.setCurrentIndex(3))
        self.ui.control_valve.triggered.connect(lambda: self.ui.pages.setCurrentIndex(4))
        self.ui.control_test.triggered.connect(lambda: self.ui.pages.setCurrentIndex(5))

        self.ui.settings_parameter_table.triggered.connect(lambda: self.ui.pages.setCurrentIndex(6))

        self.ui.pages.setCurrentIndex(0)
    
    def connectCustomSignal(self):
        self.robot.parameter_changed.connect(self.updateParameterTable)

        self.robot.show_motor_status.connect(self.showMotorStatus)
        self.robot.show_motor_original.connect(self.showMotorOriginal)
        self.robot.show_motor_mode.connect(self.showMotorMode)
        self.robot.show_switch.connect(self.showSwitch)
        self.robot.status_signal.connect(self.showStatus)
        self.robot.show_gripper.connect(self.showGripper)
        self.robot.show_rope.connect(self.show_rope)
        self.robot.show_kinematics.connect(self.showKinematics)

        self.robot.io.show_valve.connect(self.showValve)

        self.robot.show_force.connect(self.showForce)

    def connectButton(self) -> None:
        self.ui.statusBar.setSizeGripEnabled(False)
        self.ui.statusBar.showMessage("Welcome to Continnum Robot Control Panel", 10000)
        
        ''' 打开设备 '''
        self.ui.bt_open_device.setEnabled(True)
        self.ui.bt_open_device.clicked.connect(self.initUsbCan)

        ''' 初始化机器人 '''
        self.ui.bt_init_robot.setEnabled(False)
        self.ui.bt_init_robot.clicked.connect(self.initRobot)

        ''' 界面 '''
        self.ui.control.setEnabled(False)
        self.ui.control_all.setEnabled(False)
        self.ui.status.setEnabled(False)
        self.ui.param.setEnabled(False)

        ''' 电机 状态控制 '''
        for i in range(1,11):
            exec(f"self.ui.shut_down_{i}.clicked.connect(self.robot.motor_{i}.shut_down)")
            exec(f"self.ui.switch_on_{i}.clicked.connect(self.robot.motor_{i}.switch_on)")
            exec(f"self.ui.enable_operation_{i}.clicked.connect(self.robot.motor_{i}.enable_operation)")
            exec(f"self.ui.disable_operation_{i}.clicked.connect(self.robot.motor_{i}.disable_operation)")
            exec(f"self.ui.disable_voltage_{i}.clicked.connect(self.robot.motor_{i}.disable_voltage)")
            exec(f"self.ui.quick_stop_{i}.clicked.connect(self.robot.motor_{i}.quick_stop)")
            exec(f"self.ui.fault_reset_{i}.clicked.connect(self.robot.motor_{i}.fault_reset)")

        ''' 控制 '''
        self.ui.shut_down_11.clicked.connect(lambda: self.robot.setServo("shut_down"))
        self.ui.switch_on_11.clicked.connect(lambda: self.robot.setServo("switch_on"))
        self.ui.enable_operation_11.clicked.connect(lambda: self.robot.setServo("enable_operation"))
        self.ui.disable_operation_11.clicked.connect(lambda: self.robot.setServo("disable_operation"))
        self.ui.disable_voltage_11.clicked.connect(lambda: self.robot.setServo("disable_voltage"))
        self.ui.quick_stop_11.clicked.connect(lambda: self.robot.setServo("quick_stop"))
        self.ui.fault_reset_11.clicked.connect(lambda: self.robot.setServo("fault_reset"))

        ''' 电磁阀 '''
        self.ui.open_valve_1.clicked.connect(self.robot.io.open_valve_1)
        self.ui.open_valve_2.clicked.connect(self.robot.io.open_valve_2)
        self.ui.open_valve_3.clicked.connect(self.robot.io.open_valve_3)
        self.ui.open_valve_4.clicked.connect(self.robot.io.open_valve_4)

        self.ui.close_valve_1.clicked.connect(self.robot.io.close_valve_1)
        self.ui.close_valve_2.clicked.connect(self.robot.io.close_valve_2)
        self.ui.close_valve_3.clicked.connect(self.robot.io.close_valve_3)
        self.ui.close_valve_4.clicked.connect(self.robot.io.close_valve_4)

        ''' 速度 关节 '''
        for node_id in Motor.motor_dict:
            getattr(self.ui, f"speed_forward_{node_id}").pressed.connect(getattr(self, f"speed_forward_{node_id}"))
            getattr(self.ui, f"speed_forward_{node_id}").released.connect(getattr(self, f"speed_stop_{node_id}"))
            getattr(self.ui, f"speed_reverse_{node_id}").pressed.connect(getattr(self, f"speed_reverse_{node_id}"))
            getattr(self.ui, f"speed_reverse_{node_id}").released.connect(getattr(self, f"speed_stop_{node_id}"))

        ''' 
            Gripper 
        '''
        self.ui.calibrate_gripper.clicked.connect(self.calibrateGripper)
        self.ui.homing_gripper.clicked.connect(self.homingGripper)

        '''
            Continuum
        '''
        self.ui.start_adjust.clicked.connect(self.startZeroContinuum)
        self.ui.set_rope_zero.clicked.connect(self.stopZeroContinuum)

        ''' 
            Sensor 
        '''
        self.ui.set_sensor_zero.clicked.connect(self.calibrateForceSensor) # 标定传感器
        '''
            Kinematics
        '''
        self.ui.bt_cali.clicked.connect(self.calibrateKinematics) # 标定运动学

        '''
            Control Inside
        '''
        self.ui.length_forward_in.clicked.connect(lambda: self.moveInsideSection("extend"))
        self.ui.length_back_in.clicked.connect(lambda: self.moveInsideSection("shorten"))
        self.ui.length_stop_in.clicked.connect(self.stopInsideSection)
        
        self.ui.curve_forward_in.pressed.connect(lambda: self.moveInsideSection("curve"))
        self.ui.curve_forward_in.released.connect(self.stopInsideSection)

        self.ui.curve_back_in.pressed.connect(lambda: self.moveInsideSection("straighten"))
        self.ui.curve_back_in.released.connect(self.stopInsideSection)

        self.ui.angle_forward_in.pressed.connect(lambda: self.moveInsideSection("rotate_clockwise"))
        self.ui.angle_forward_in.released.connect(self.stopInsideSection)

        self.ui.angle_back_in.pressed.connect(lambda: self.moveInsideSection("rotate_anticlockwise"))
        self.ui.angle_back_in.released.connect(self.stopInsideSection)

        '''
            Control Midside
        '''
        self.ui.length_forward_mid.clicked.connect(self.robot.ExtendMidsideSection)
        self.ui.length_back_mid.clicked.connect(self.robot.ShortenMidsideSection)
        self.ui.length_stop_mid.clicked.connect(self.robot.StopMidsideSection)
        self.ui.curve_forward_mid.pressed.connect(self.robot.CurveMidsideSection)
        self.ui.curve_forward_mid.released.connect(self.robot.StopMidsideSection)
        self.ui.curve_back_mid.pressed.connect(self.robot.StraightenMidsideSection)
        self.ui.curve_back_mid.released.connect(self.robot.StopMidsideSection)
        self.ui.angle_forward_mid.pressed.connect(self.robot.RotateMidsideSectionClockwise)
        self.ui.angle_forward_mid.released.connect(self.robot.StopMidsideSection)
        self.ui.angle_back_mid.pressed.connect(self.robot.RotateMidsideSectionAntiClockwise)
        self.ui.angle_back_mid.released.connect(self.robot.StopMidsideSection)

        self.ui.length_forward_out.clicked.connect(self.robot.ExtendOutsideSection)
        self.ui.length_back_out.clicked.connect(self.robot.ShortenOutsideSection)
        self.ui.length_stop_out.clicked.connect(self.robot.StopOutsideSection)
        self.ui.curve_forward_out.pressed.connect(self.robot.CurveOutsideSection)
        self.ui.curve_forward_out.released.connect(self.robot.StopOutsideSection)
        self.ui.curve_back_out.pressed.connect(self.robot.StraightenOutsideSection)
        self.ui.curve_back_out.released.connect(self.robot.StopOutsideSection)
        self.ui.angle_forward_out.pressed.connect(self.robot.RotateOutsideSectionClockwise)
        self.ui.angle_forward_out.released.connect(self.robot.StopOutsideSection)
        self.ui.angle_back_out.pressed.connect(self.robot.RotateOutsideSectionAntiClockwise)
        self.ui.angle_back_out.released.connect(self.robot.StopOutsideSection)

    def initParameterTable(self):
        self.ui.tableWidget.setRowCount(len(self.robot.PARAMETER)) # 创建足够数量的行

        row = 0
        for name, value in self.robot.PARAMETER.items(): # 填充
            name_item = QTableWidgetItem(name)
            name_item.setBackground(QColor(252, 175, 62))

            value_item = QTableWidgetItem(str(value["value"]))

            units_item = QTableWidgetItem(self.robot.PARAMETER[name]["units"])
            units_item.setFlags(units_item.flags() & False)

            note_item = QTableWidgetItem(self.robot.PARAMETER[name]["note"])
            note_item.setFlags(note_item.flags() & False)

            self.ui.tableWidget.setVerticalHeaderItem(row, name_item)
            self.ui.tableWidget.setItem(row, 0, value_item)
            self.ui.tableWidget.setItem(row, 1, units_item)
            self.ui.tableWidget.setItem(row, 2, note_item)

            row += 1
        
        def updateParameter(row, column):
            try:
                variable = self.ui.tableWidget.verticalHeaderItem(row).text()
                self.robot.PARAMETER[variable]["value"] = float(self.ui.tableWidget.item(row, column).text())
                print("\033[0;32m[Parameter] {} = {}\033[0m".format(variable, self.robot.PARAMETER[variable]["value"]))
            except: print("No item")

        self.ui.tableWidget.cellChanged.connect(updateParameter)
    
    def updateParameterTable(self, name, value):
        value_item = QTableWidgetItem(str(value))
        for row in range(self.ui.tableWidget.rowCount()):
            if self.ui.tableWidget.verticalHeaderItem(row).text() == name:
                self.ui.tableWidget.setItem(row, 0, value_item)
                break
    def showMotorStatus(self, node_id) -> None:
        status = getattr(self.robot, f"motor_{node_id}").servo_status
        
        if status == "switch_on_disabled":
            color, status = "#ffff00", "INIT"
            
            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(False)

        elif status == "ready_to_switch_on":
            color, status = "#ffff00","READY"

            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(True)

        elif status == "switched_on":
            color, status = "#ffff00","DISABLE"
            
            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(True)

        elif status == "operation_enable":
            color, status = "#00ff00", "ENABLE"

            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(True)

        elif status == "quick_stop_active":
            color, status = "#ff0000","STOP"

            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(False)

        elif status == "fault_reaction_active" or "fault":
            color, status = "#ff0000","FAULT"

            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(True)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(False)
            
        else:
            color, status = "#0000ff","UNKNOWN"

            getattr(self.ui, "shut_down_{}".format(node_id)).setEnabled(True)
            getattr(self.ui, "switch_on_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_voltage_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "quick_stop_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "disable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "enable_operation_{}".format(node_id)).setEnabled(False)
            getattr(self.ui, "fault_reset_{}".format(node_id)).setEnabled(False)

            getattr(self.ui, "tab_motor_{}".format(node_id)).setEnabled(False)
        
        status_str = "<span style=\"color:{};\">{}</span>".format(color, status)
        getattr(self.ui, f"status_{node_id}").setText(status_str)
    def showSwitch(self) -> None:
        if self.robot.io.input_1 or self.robot.io.input_1: pass
        
        warning = "<span style=\"color:#ff0000;\">{}</span>".format("WARNING")
        clear = "<span style=\"color:#00ff00;\">{}</span>".format("CLEAR")

        self.ui.switch_1.setText(warning if self.robot.io.input_1 else clear)
        self.ui.switch_2.setText(warning if self.robot.io.input_2 else clear)
    def showMotorOriginal(self, node_id) -> None:
        # 位置
        position = getattr(self.robot, f"motor_{node_id}").current_position

        max_position = getattr(self.robot, f"motor_{node_id}").max_position
        min_position = getattr(self.robot, f"motor_{node_id}").min_position
        if max_position != None and min_position != None:
            range = max_position - min_position

            if position < max_position and position > min_position:
                if position <= min_position+range*0.1 or position >= max_position-range*0.1: color = "#ff0000"
                elif position > min_position+range*0.1 and position <= min_position+range*0.3 or position < max_position-range*0.1 and position >= max_position-range*0.3: color = "#ffff00"
                else: color = "#00ff00"
            else: color = "#0000ff"
        else: color = "#00ff00"

        position_str = "<span style=\"color:{};\">{}</span>".format(color, position)

        getattr(self.ui, f"current_position_{node_id}").setText(position_str)
        
        # 速度
        speed = getattr(self.robot, f"motor_{node_id}").current_speed

        max_speed = getattr(self.robot, f"motor_{node_id}").max_speed
        min_speed = getattr(self.robot, f"motor_{node_id}").min_speed
        if max_position != None and min_position != None:
            range = max_speed - min_speed

            if speed <= max_speed and speed >= min_speed:
                if speed <= min_speed+range*0.1 or speed >= max_speed-range*0.1: color = "#ff0000"
                elif speed > min_speed+range*0.1 and speed <= min_speed+range*0.3 or speed < max_speed-range*0.1 and speed >= max_speed-range*0.3: color = "#ffff00"
                else: color = "#00ff00"
            else: color = "#0000ff"
        else: color = "#00ff00"

        speed_str = "<span style=\"color:{};\">{}</span>".format(color, speed)

        getattr(self.ui, f"current_velocity_{node_id}").setText(speed_str)
    def showMotorMode(self, node_id) -> None:
        control_mode = getattr(self.robot, f"motor_{node_id}").control_mode

        if control_mode == "position_control": mode_str = "<span style=\"color:#00ff00;\">POSITION</span>"
        elif control_mode == "speed_control": mode_str = "<span style=\"color:#00ff00;\">SPEED</span>"
        else: mode_str = "<span style=\"color:#ff0000;\">None</span>"
        
        getattr(self.ui, f"mode_{node_id}").setText(mode_str)
    def showForce(self, node_id) -> None:
        if node_id in Sensor.sensor_dict.keys():
            force = getattr(self.robot, f"sensor_{node_id}").force

            if force > 0: color = "#0000ff"
            else:
                if abs(force) <= 10: color = "#00ff00"
                else: color = "#ffff00"
            
            force_str = "<span style=\"color:{};\">{}</span>".format(color, round(abs(force), 2))

            getattr(self.ui, f"force_{node_id}").setText(force_str)
    def showValve(self) -> None:
        # 小爪 开启状态较为危险
        open = "<span style=\"color:#ffff00;\">{}</span>".format("OPEN")
        close = "<span style=\"color:#00ff00;\">{}</span>".format("CLOSE")

        if self.robot.io.output_1:
            self.ui.valve_1.setText(open)

            self.ui.open_valve_1.setEnabled(False)
            self.ui.close_valve_1.setEnabled(True)
        else:
            self.ui.valve_1.setText(close)

            self.ui.open_valve_1.setEnabled(True)
            self.ui.close_valve_1.setEnabled(False)
        
        if self.robot.io.output_2:
            self.ui.valve_2.setText(open)

            self.ui.open_valve_2.setEnabled(False)
            self.ui.close_valve_2.setEnabled(True)
        else:
            self.ui.valve_2.setText(close)

            self.ui.open_valve_2.setEnabled(True)
            self.ui.close_valve_2.setEnabled(False)

        if self.robot.io.output_3:
            self.ui.valve_3.setText(open)

            self.ui.open_valve_3.setEnabled(False)
            self.ui.close_valve_3.setEnabled(True)
        else:
            self.ui.valve_3.setText(close)

            self.ui.open_valve_3.setEnabled(True)
            self.ui.close_valve_3.setEnabled(False)

        # 大爪 关闭状态危险
        open = "<span style=\"color:#00ff00;\">{}</span>".format("OPEN")
        close = "<span style=\"color:#ffff00;\">{}</span>".format("CLOSE")

        if self.robot.io.output_4:
            self.ui.valve_4.setText(close)

            self.ui.open_valve_4.setEnabled(True)
            self.ui.close_valve_4.setEnabled(False)
        else:
            self.ui.valve_4.setText(open)

            self.ui.open_valve_4.setEnabled(False)
            self.ui.close_valve_4.setEnabled(True)
    def showStatus(self, message) -> None:
        self.ui.statusBar.showMessage(message, 5000)
    def showGripper(self, is_zero):
        if is_zero:
            color = "#00ff00" if self.robot.gripper_position >= 0 else "#ff0000"

            self.ui.ballscrew.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.gripper_position, 2)))
            self.ui.ballscrew_v.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.gripper_velocity, 2)))
        else:
            self.ui.ballscrew.setText("<span style=\"color:#ff0000;\">No Zero</span>")
            self.ui.ballscrew_v.setText("<span style=\"color:#ff0000;\">No Zero</span>")
    def show_rope(self, is_zero, node_id):
        if is_zero:
            position = getattr(self.robot, f"rope_{node_id}_position")
            velocity = self.robot.rope_velocity[node_id-1]
            
            color = "#00ff00" if position >= 0 else "#ff0000"
            
            getattr(self.ui, "rope_{}".format(node_id)).setText("<span style=\"color:{};\">{}</span>".format(color, round(position, 2)))
            getattr(self.ui, "rope_v_{}".format(node_id)).setText("<span style=\"color:{};\">{}</span>".format(color, round(velocity, 2)))
        else:
            getattr(self.ui, "rope_{}".format(node_id)).setText("<span style=\"color:#ff0000;\">No Zero</span>")
            getattr(self.ui, "rope_v_{}".format(node_id)).setText("<span style=\"color:#ff0000;\">No Zero</span>")
    def showKinematics(self):
        # outside
        self.ui.length_1_o.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_outside_length[0], 2)))
        self.ui.length_2_o.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_outside_length[1], 2)))
        self.ui.length_3_o.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_outside_length[2], 2)))
        self.ui.length_out.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_length[2], 2)))
        self.ui.curvature_out.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_curvature[2], 5)))
        self.ui.angle_out.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_rotation_angle[2]/math.pi*180, 2)))
        self.ui.x_out_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_coordinate[0], 2)))
        self.ui.y_out_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_coordinate[1], 2)))
        self.ui.z_out_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_coordinate[2], 2)))
        self.ui.x_out_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_world_coordinate[0], 2)))
        self.ui.y_out_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_world_coordinate[1], 2)))
        self.ui.z_out_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.outside_world_coordinate[2], 2)))

        # midside
        self.ui.length_1_m.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_midside_length[0], 2)))
        self.ui.length_2_m.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_midside_length[1], 2)))
        self.ui.length_3_m.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_midside_length[2], 2)))
        self.ui.length_4_m.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_midside_length[3], 2)))
        self.ui.length_5_m.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_midside_length[4], 2)))
        self.ui.length_6_m.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_midside_length[5], 2)))
        self.ui.length_mid.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_length[1], 2)))
        self.ui.curvature_mid.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_curvature[1], 5)))
        self.ui.angle_mid.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_rotation_angle[1]/math.pi*180, 2)))
        self.ui.x_mid_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_coordinate[0], 2)))
        self.ui.y_mid_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_coordinate[1], 2)))
        self.ui.z_mid_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_coordinate[2], 2)))
        self.ui.x_mid_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_world_coordinate[0], 2)))
        self.ui.y_mid_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_world_coordinate[1], 2)))
        self.ui.z_mid_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.midside_world_coordinate[2], 2)))

        # inside
        self.ui.length_1_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[0], 2)))
        self.ui.length_2_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[1], 2)))
        self.ui.length_3_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[2], 2)))
        self.ui.length_4_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[3], 2)))
        self.ui.length_5_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[4], 2)))
        self.ui.length_6_i.setText("<span style=\"color:#ffff00;\">{}</span>".format(round(self.robot.rope_inside_length[5], 2)))
        self.ui.length_7_i.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_inside_length[6], 2)))
        self.ui.length_8_i.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_inside_length[7], 2)))
        self.ui.length_9_i.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.rope_inside_length[8], 2)))
        self.ui.length_in.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_length[0], 2)))
        self.ui.curvature_in.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_curvature[0], 5)))
        self.ui.angle_in.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.backbone_rotation_angle[0]/math.pi*180, 2)))
        self.ui.x_in_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_coordinate[0], 2)))
        self.ui.y_in_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_coordinate[1], 2)))
        self.ui.z_in_local.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_coordinate[2], 2)))
        self.ui.x_in_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_world_coordinate[0], 2)))
        self.ui.y_in_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_world_coordinate[1], 2)))
        self.ui.z_in_world.setText("<span style=\"color:#00ff00;\">{}</span>".format(round(self.robot.inside_world_coordinate[2], 2)))

        # # rope
        # for id in range(1,10):
        #     exec("self.ui.rope_{}.setText('<span style=\"color:#00ff00;\">{}</span>')".format(id, round(self.robot.rope_total_length[id-1], 2)))


    ''' 初始化设备 '''
    def initUsbCan(self):
        if self.robot.initUsbCan():
            self.update_status_thread = self.RobotFunctionThread(self.robot.updateStatus)
            self.update_force_thread = self.RobotFunctionThread(self.robot.updateForce)

            self.update_status_thread.start()
            self.update_force_thread.start()

            self.ui.bt_open_device.setEnabled(False)

            self.ui.bt_init_robot.setEnabled(True)

            self.showStatus("Open Device !!!")
        else: self.showStatus("Open Device Failed")
        
    ''' 初始化机器人 '''
    def initRobot(self) -> None:
        def change(status):
            if status:
                self.ui.bt_init_robot.setEnabled(False)
                self.showStatus("Initializing Robot ...")
            else:
                self.ui.bt_init_robot.setEnabled(True)
                self.showStatus("Something is wrong in the progress of Initializing Robot, please try again.")
        def next():
            self.ui.bt_init_robot.setEnabled(False)
            self.showStatus("Robot is ready, Control is launch !!!")

            self.ui.control.setEnabled(True)
            self.ui.control_all.setEnabled(True)
            self.ui.status.setEnabled(True)
            self.ui.param.setEnabled(True)

            self.ui.homing_gripper.setEnabled(True)
            self.ui.calibrate_gripper.setEnabled(True)
            self.ui.start_adjust.setEnabled(True)
            self.ui.set_sensor_zero.setEnabled(True)
            self.ui.bt_cali.setEnabled(True)
            self.ui.inside_length.setEnabled(True)
            self.ui.midside_length.setEnabled(True)
            self.ui.outside_length.setEnabled(True)

            self.ui.inside_length.setPlaceholderText(str(self.robot.PARAMETER["kinematics_control_inside_s_min"]["value"]))
            self.ui.midside_length.setPlaceholderText(str(self.robot.PARAMETER["kinematics_control_midside_s_min"]["value"]))
            self.ui.outside_length.setPlaceholderText(str(self.robot.PARAMETER["kinematics_control_outside_s_min"]["value"]))

            self.ui.length_forward_in.setEnabled(True)
            self.ui.length_back_in.setEnabled(True)
            self.ui.curve_forward_in.setEnabled(True)
            self.ui.curve_back_in.setEnabled(True)
            self.ui.angle_forward_in.setEnabled(True)
            self.ui.angle_back_in.setEnabled(True)
            self.ui.length_forward_mid.setEnabled(True)
            self.ui.length_back_mid.setEnabled(True)
            self.ui.curve_forward_mid.setEnabled(True)
            self.ui.curve_back_mid.setEnabled(True)
            self.ui.angle_forward_mid.setEnabled(True)
            self.ui.angle_back_mid.setEnabled(True)
            self.ui.length_forward_out.setEnabled(True)
            self.ui.length_back_out.setEnabled(True)
            self.ui.curve_forward_out.setEnabled(True)
            self.ui.curve_back_out.setEnabled(True)
            self.ui.angle_forward_out.setEnabled(True)
            self.ui.angle_back_out.setEnabled(True)

            self.ui.teleoperation.setEnabled(True)

        self.robot.robot_init_start.connect(change)
        self.robot.robot_init_end.connect(next)
        
        self.init_robot_thread = self.RobotFunctionThread(self.robot.initRobot)
        self.init_robot_thread.start()

    ''' 夹爪归零 '''
    def homingGripper(self):
        def start():
            self.ui.homing_gripper.setEnabled(False)
            self.showStatus("Ballscrew is backing to zero ...")
        def end():
            self.ui.homing_gripper.setEnabled(True)
            self.showStatus("Ballscrew is backed to zero !")

        self.robot.gripper_homing_start.connect(start)
        self.robot.gripper_homing_end.connect(end)

        self.gripper_homing_thread = self.RobotFunctionThread(self.robot.homingGripper)
        self.gripper_homing_thread.start()

    ''' 标定夹爪 '''
    def calibrateGripper(self):
        def start():
            self.ui.calibrate_gripper.setEnabled(False)
            self.showStatus("Ballscrew is being setting zero ...")
        def end():
            self.ui.calibrate_gripper.setEnabled(True)
            self.showStatus("Ballscrew is set zero !")
            self.ui.ballscrew.setText("<span style=\"color:#00ff00;\">Zero</span>")
        
        self.robot.gripper_calibration_start.connect(start)
        self.robot.gripper_calibration_end.connect(end)

        self.gripper_calibration_thread = self.RobotFunctionThread(self.robot.calibrateGripper)
        self.gripper_calibration_thread.start()
    
    ''' 调整连续体 '''
    def startZeroContinuum(self):
        def start():
            self.ui.start_adjust.setEnabled(False)
            self.ui.set_rope_zero.setEnabled(True)
            self.showStatus("All ropes are being adapting force ...")
        def end():
            self.ui.start_adjust.setEnabled(True)
            self.ui.set_rope_zero.setEnabled(False)
            self.showStatus("All ropes are set zero !")

            for i in range(1,10):
                getattr(self.ui, f"rope_{i}").setText("<span style=\"color:#00ff00;\">Zero</span>")
        
        self.robot.continuum_calibration_start.connect(start)
        self.robot.continuum_calibration_end.connect(end)

        self.continuum_zero_thread = self.RobotFunctionThread(self.robot.zeroContinuum)
        self.continuum_zero_thread.start()
    def stopZeroContinuum(self):
        self.robot.isCalibrateContinuum = False
    
    ''' 标定传感器 '''
    def calibrateForceSensor(self):
        def start():
            self.ui.set_sensor_zero.setEnabled(False)
            self.showStatus("All sensors are being adapting force ...")
        def end():
            self.ui.set_sensor_zero.setEnabled(True)
            self.showStatus("All sensors are set zero !")

        self.robot.sensor_calibration_start.connect(start)
        self.robot.sensor_calibration_end.connect(end)

        self.sensor_calibration_thread = self.RobotFunctionThread(self.robot.calibrateForceSensor)
        self.sensor_calibration_thread.start()



    def speed_forward_factory(self, node_id):
        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)
        
        speed = int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value())
        self.robot.joint_speed([node_id], speed)
    for node_id in range(1,11):
        exec(f"def speed_forward_{node_id}(self): self.speed_forward_factory({node_id})")
    def speed_reverse_factory(self, node_id):
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)

        speed = - int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value())
        self.robot.joint_speed([node_id], speed)
    for node_id in range(1,11):
        exec(f"def speed_reverse_{node_id}(self): self.speed_reverse_factory({node_id})")
    def speed_stop_factory(self, node_id):
        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(True)

        self.robot.joint_speed_stop()
    for node_id in range(1,11):
        exec(f"def speed_stop_{node_id}(self): self.speed_stop_factory({node_id})")

    
   
    ''' 标定运动学 '''
    def calibrateKinematics(self):
        bl_o = float(self.ui.outside_length.text()) if self.ui.outside_length.text() != "" else float(self.ui.outside_length.placeholderText())
        bl_m = float(self.ui.midside_length.text()) if self.ui.midside_length.text() != "" else float(self.ui.midside_length.placeholderText())
        bl_i = float(self.ui.inside_length.text()) if self.ui.inside_length.text() != "" else float(self.ui.inside_length.placeholderText())
        self.robot.calibrateContinuum(bl_o, bl_m, bl_i)

    ''' 相机 '''
    def OpenCamera(self):
        def update(image):
            self.ui.label_video.setPixmap(QPixmap.fromImage(image)) # 往显示视频的Label里显示QImage
        def clear():
            self.ui.label_video.clear() # 清除label组件上的图片
            self.ui.label_video.setText("No Video Stream")
        self.ui.label_video.setScaledContents(True) # 自适应
        self.robot.OpenCamera(update_slot=update, clear_slot=clear)


    ''' 运动控制inside '''
    def moveInsideSection(self, action: str):
        def start():
            self.ui.length_forward_in.setEnabled(False)
            self.ui.length_back_in.setEnabled(False)
            if action == "extend" or "shorten": self.ui.length_stop_in.setEnabled(True)
            # if action != "curve": self.ui.curve_forward_in.setEnabled(False)
            # if action != "straighten": self.ui.curve_back_in.setEnabled(False)
            # if action != "rotate_clockwise": self.ui.angle_forward_in.setEnabled(False)
            # if action != "rotate_anticlockwise": self.ui.angle_back_in.setEnabled(False)

            self.ui.length_forward_mid.setEnabled(False)
            self.ui.length_back_mid.setEnabled(False)
            self.ui.curve_forward_mid.setEnabled(False)
            self.ui.curve_back_mid.setEnabled(False)
            self.ui.angle_forward_mid.setEnabled(False)
            self.ui.angle_back_mid.setEnabled(False)

            self.ui.length_forward_out.setEnabled(False)
            self.ui.length_back_out.setEnabled(False)
            self.ui.curve_forward_out.setEnabled(False)
            self.ui.curve_back_out.setEnabled(False)
            self.ui.angle_forward_out.setEnabled(False)
            self.ui.angle_back_out.setEnabled(False)
        def end():
            self.ui.length_forward_in.setEnabled(True)
            self.ui.length_back_in.setEnabled(True)
            self.ui.length_stop_in.setEnabled(False)
            self.ui.curve_forward_in.setEnabled(True)
            self.ui.curve_back_in.setEnabled(True)
            self.ui.angle_forward_in.setEnabled(True)
            self.ui.angle_back_in.setEnabled(True)

            self.ui.length_forward_mid.setEnabled(True)
            self.ui.length_back_mid.setEnabled(True)
            self.ui.curve_forward_mid.setEnabled(True)
            self.ui.curve_back_mid.setEnabled(True)
            self.ui.angle_forward_mid.setEnabled(True)
            self.ui.angle_back_mid.setEnabled(True)

            self.ui.length_forward_out.setEnabled(True)
            self.ui.length_back_out.setEnabled(True)
            self.ui.curve_forward_out.setEnabled(True)
            self.ui.curve_back_out.setEnabled(True)
            self.ui.angle_forward_out.setEnabled(True)
            self.ui.angle_back_out.setEnabled(True)
    
        self.robot.continuum_move_start.connect(start)
        self.robot.continuum_move_end.connect(end)

        if action == "extend": s_d = abs(self.robot.PARAMETER["kinematics_control_inside_s_d"]["value"])
        elif action == "shorten": s_d = -abs(self.robot.PARAMETER["kinematics_control_inside_s_d"]["value"])
        else: s_d = 0

        if action == "curve": kappa_d = abs(self.robot.PARAMETER["kinematics_control_inside_kappa_d"]["value"])
        elif action == "straighten": kappa_d = -abs(self.robot.PARAMETER["kinematics_control_inside_kappa_d"]["value"])
        else: kappa_d = 0

        if action == "rotate_clockwise": phi_d = abs(self.robot.PARAMETER["kinematics_control_inside_phi_d"]["value"])
        elif action == "rotate_anticlockwise": phi_d = -abs(self.robot.PARAMETER["kinematics_control_inside_phi_d"]["value"])
        else: phi_d = 0

        self.move_inside_thread = self.RobotFunctionThread(self.robot.controlContinuum, "inside", s_d, kappa_d, phi_d)
        self.move_inside_thread.start()
    def stopInsideSection(self):
        self.robot.isControl = False

