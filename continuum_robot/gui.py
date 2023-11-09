# -*- coding:utf-8 -*-


''' gui.py GUI v5.0 '''


import typing
from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QObject, QThread, pyqtSignal, QMutex

from PyQt5.QtGui import QPixmap


# 添加模块路径
import sys, os, time, math
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


from continuum_robot.control_panel import Ui_MainWindow as Ui_ControlPanel

from continuum_robot.motor import Motor
from continuum_robot.io import IoModule
from continuum_robot.sensor import Sensor

from continuum_robot.robot import ContinuumRobot




class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)

        self.robot = ContinuumRobot()

        self.initParameterTable()
        
        self.connectMenu()
        self.connectUserSignal()
        self.signal_connect_slot()

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

        self.ui.settings_force_sensor_calibration.triggered.connect(lambda: self.ui.pages.setCurrentIndex(6))
        self.ui.settings_continuum_calibration.triggered.connect(lambda: self.ui.pages.setCurrentIndex(7))
        self.ui.settings_parameter_table.triggered.connect(lambda: self.ui.pages.setCurrentIndex(8))

        self.ui.pages.setCurrentIndex(0)
    
    def connectUserSignal(self):
        self.robot.show_motor_status.connect(self.show_motor_status)
        self.robot.show_motor_original.connect(self.show_motor_original)
        self.robot.show_motor_mode.connect(self.show_motor_mode)
        self.robot.show_switch.connect(self.show_switch)
        self.robot.status_signal.connect(self.show_status)
        self.robot.show_gripper.connect(self.show_gripper)
        self.robot.show_rope.connect(self.show_rope)
        self.robot.show_kinematics.connect(self.show_kinematics)

        self.robot.io.show_valve.connect(self.show_valve)

        self.robot.show_force.connect(self.show_force)

    def signal_connect_slot(self) -> None:
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
        self.ui.shut_down_1.clicked.connect(lambda: self.robot.motor_1.shut_down(is_pdo=True))
        self.ui.shut_down_2.clicked.connect(lambda: self.robot.motor_2.shut_down(is_pdo=True))
        self.ui.shut_down_3.clicked.connect(lambda: self.robot.motor_3.shut_down(is_pdo=True))
        self.ui.shut_down_4.clicked.connect(lambda: self.robot.motor_4.shut_down(is_pdo=True))
        self.ui.shut_down_5.clicked.connect(lambda: self.robot.motor_5.shut_down(is_pdo=True))
        self.ui.shut_down_6.clicked.connect(lambda: self.robot.motor_6.shut_down(is_pdo=True))
        self.ui.shut_down_7.clicked.connect(lambda: self.robot.motor_7.shut_down(is_pdo=True))
        self.ui.shut_down_8.clicked.connect(lambda: self.robot.motor_8.shut_down(is_pdo=True))
        self.ui.shut_down_9.clicked.connect(lambda: self.robot.motor_9.shut_down(is_pdo=True))
        self.ui.shut_down_10.clicked.connect(lambda: self.robot.motor_10.shut_down(is_pdo=True))

        self.ui.switch_on_1.clicked.connect(lambda: self.robot.motor_1.switch_on(is_pdo=True))
        self.ui.switch_on_2.clicked.connect(lambda: self.robot.motor_2.switch_on(is_pdo=True))
        self.ui.switch_on_3.clicked.connect(lambda: self.robot.motor_3.switch_on(is_pdo=True))
        self.ui.switch_on_4.clicked.connect(lambda: self.robot.motor_4.switch_on(is_pdo=True))
        self.ui.switch_on_5.clicked.connect(lambda: self.robot.motor_5.switch_on(is_pdo=True))
        self.ui.switch_on_6.clicked.connect(lambda: self.robot.motor_6.switch_on(is_pdo=True))
        self.ui.switch_on_7.clicked.connect(lambda: self.robot.motor_7.switch_on(is_pdo=True))
        self.ui.switch_on_8.clicked.connect(lambda: self.robot.motor_8.switch_on(is_pdo=True))
        self.ui.switch_on_9.clicked.connect(lambda: self.robot.motor_9.switch_on(is_pdo=True))
        self.ui.switch_on_10.clicked.connect(lambda: self.robot.motor_10.switch_on(is_pdo=True))

        self.ui.enable_operation_1.clicked.connect(lambda: self.robot.motor_1.enable_operation(is_pdo=True))
        self.ui.enable_operation_2.clicked.connect(lambda: self.robot.motor_2.enable_operation(is_pdo=True))
        self.ui.enable_operation_3.clicked.connect(lambda: self.robot.motor_3.enable_operation(is_pdo=True))
        self.ui.enable_operation_4.clicked.connect(lambda: self.robot.motor_4.enable_operation(is_pdo=True))
        self.ui.enable_operation_5.clicked.connect(lambda: self.robot.motor_5.enable_operation(is_pdo=True))
        self.ui.enable_operation_6.clicked.connect(lambda: self.robot.motor_6.enable_operation(is_pdo=True))
        self.ui.enable_operation_7.clicked.connect(lambda: self.robot.motor_7.enable_operation(is_pdo=True))
        self.ui.enable_operation_8.clicked.connect(lambda: self.robot.motor_8.enable_operation(is_pdo=True))
        self.ui.enable_operation_9.clicked.connect(lambda: self.robot.motor_9.enable_operation(is_pdo=True))
        self.ui.enable_operation_10.clicked.connect(lambda: self.robot.motor_10.enable_operation(is_pdo=True))

        self.ui.disable_operation_1.clicked.connect(lambda: self.robot.motor_1.disable_operation(is_pdo=True))
        self.ui.disable_operation_2.clicked.connect(lambda: self.robot.motor_2.disable_operation(is_pdo=True))
        self.ui.disable_operation_3.clicked.connect(lambda: self.robot.motor_3.disable_operation(is_pdo=True))
        self.ui.disable_operation_4.clicked.connect(lambda: self.robot.motor_4.disable_operation(is_pdo=True))
        self.ui.disable_operation_5.clicked.connect(lambda: self.robot.motor_5.disable_operation(is_pdo=True))
        self.ui.disable_operation_6.clicked.connect(lambda: self.robot.motor_6.disable_operation(is_pdo=True))
        self.ui.disable_operation_7.clicked.connect(lambda: self.robot.motor_7.disable_operation(is_pdo=True))
        self.ui.disable_operation_8.clicked.connect(lambda: self.robot.motor_8.disable_operation(is_pdo=True))
        self.ui.disable_operation_9.clicked.connect(lambda: self.robot.motor_9.disable_operation(is_pdo=True))
        self.ui.disable_operation_10.clicked.connect(lambda: self.robot.motor_10.disable_operation(is_pdo=True))

        self.ui.disable_voltage_1.clicked.connect(lambda: self.robot.motor_1.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_2.clicked.connect(lambda: self.robot.motor_2.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_3.clicked.connect(lambda: self.robot.motor_3.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_4.clicked.connect(lambda: self.robot.motor_4.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_5.clicked.connect(lambda: self.robot.motor_5.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_6.clicked.connect(lambda: self.robot.motor_6.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_7.clicked.connect(lambda: self.robot.motor_7.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_8.clicked.connect(lambda: self.robot.motor_8.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_9.clicked.connect(lambda: self.robot.motor_9.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_10.clicked.connect(lambda: self.robot.motor_10.disable_voltage(is_pdo=True))

        self.ui.quick_stop_1.clicked.connect(lambda: self.robot.motor_1.quick_stop(is_pdo=True))
        self.ui.quick_stop_2.clicked.connect(lambda: self.robot.motor_2.quick_stop(is_pdo=True))
        self.ui.quick_stop_3.clicked.connect(lambda: self.robot.motor_3.quick_stop(is_pdo=True))
        self.ui.quick_stop_4.clicked.connect(lambda: self.robot.motor_4.quick_stop(is_pdo=True))
        self.ui.quick_stop_5.clicked.connect(lambda: self.robot.motor_5.quick_stop(is_pdo=True))
        self.ui.quick_stop_6.clicked.connect(lambda: self.robot.motor_6.quick_stop(is_pdo=True))
        self.ui.quick_stop_7.clicked.connect(lambda: self.robot.motor_7.quick_stop(is_pdo=True))
        self.ui.quick_stop_8.clicked.connect(lambda: self.robot.motor_8.quick_stop(is_pdo=True))
        self.ui.quick_stop_9.clicked.connect(lambda: self.robot.motor_9.quick_stop(is_pdo=True))
        self.ui.quick_stop_10.clicked.connect(lambda: self.robot.motor_10.quick_stop(is_pdo=True))

        self.ui.fault_reset_1.clicked.connect(lambda: self.robot.motor_1.quick_stop(is_pdo=True))
        self.ui.fault_reset_2.clicked.connect(lambda: self.robot.motor_2.quick_stop(is_pdo=True))
        self.ui.fault_reset_3.clicked.connect(lambda: self.robot.motor_3.quick_stop(is_pdo=True))
        self.ui.fault_reset_4.clicked.connect(lambda: self.robot.motor_4.quick_stop(is_pdo=True))
        self.ui.fault_reset_5.clicked.connect(lambda: self.robot.motor_5.quick_stop(is_pdo=True))
        self.ui.fault_reset_6.clicked.connect(lambda: self.robot.motor_6.quick_stop(is_pdo=True))
        self.ui.fault_reset_7.clicked.connect(lambda: self.robot.motor_7.quick_stop(is_pdo=True))
        self.ui.fault_reset_8.clicked.connect(lambda: self.robot.motor_8.quick_stop(is_pdo=True))
        self.ui.fault_reset_9.clicked.connect(lambda: self.robot.motor_9.quick_stop(is_pdo=True))
        self.ui.fault_reset_10.clicked.connect(lambda: self.robot.motor_10.quick_stop(is_pdo=True))

        # for node_id in range(1, 11):
        #     getattr(self.ui, "shut_down_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "switch_on_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "enable_operation_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "disable_operation_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "disable_voltage_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "quick_stop_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "fault_reset_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))

        ''' 控制 '''
        self.ui.shut_down_11.clicked.connect(self.robot.shut_down_all)
        self.ui.switch_on_11.clicked.connect(self.robot.switch_on_all)
        self.ui.enable_operation_11.clicked.connect(self.robot.enable_operation_all)
        self.ui.disable_operation_11.clicked.connect(self.robot.disable_operation_all)
        self.ui.disable_voltage_11.clicked.connect(self.robot.disable_voltage_all)
        self.ui.quick_stop_11.clicked.connect(self.robot.quick_stop_all)
        self.ui.fault_reset_11.clicked.connect(self.robot.fault_reset_all)

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

        ''' 线 适应 调0 '''
        self.ui.start_adjust.clicked.connect(self.rope_force_adapt)
        self.ui.set_rope_zero.clicked.connect(self.rope_set_zero)

        ''' 力 调0 '''
        self.ui.set_sensor_zero.clicked.connect(self.calibrateForceSensor)


        self.ui.bt_cali.clicked.connect(self.calibrateContinuum)

        # self.ui.length_forward_in.clicked.connect(self.robot.ExtendInsideSection)
        # self.ui.length_back_in.clicked.connect(self.robot.ShortenInsideSection)
        self.ui.length_forward_in.clicked.connect(self.extendInsideSection)
        self.ui.length_back_in.clicked.connect(self.shortenInsideSection)
        self.ui.length_stop_in.clicked.connect(self.robot.StopInsideSection)
        # self.ui.curve_forward_in.pressed.connect(self.robot.CurveInsideSection)
        self.ui.curve_forward_in.pressed.connect(self.curveInsideSection)
        self.ui.curve_forward_in.released.connect(self.robot.StopInsideSection)
        # self.ui.curve_back_in.pressed.connect(self.robot.StraightenInsideSection)
        self.ui.curve_back_in.pressed.connect(self.straightenInsideSection)
        self.ui.curve_back_in.released.connect(self.robot.StopInsideSection)
        # self.ui.angle_forward_in.pressed.connect(self.robot.RotateInsideSectionClockwise)
        self.ui.angle_forward_in.pressed.connect(self.rotateInsideSectionClockwise)
        self.ui.angle_forward_in.released.connect(self.robot.StopInsideSection)
        # self.ui.angle_back_in.pressed.connect(self.robot.RotateInsideSectionAntiClockwise)
        self.ui.angle_back_in.pressed.connect(self.rotateInsideSectionAntiClockwise)
        self.ui.angle_back_in.released.connect(self.robot.StopInsideSection)

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


    ''' 显示 电机状态 '''
    def show_motor_status(self, node_id) -> None:
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

    ''' 显示 开关 '''
    def show_switch(self) -> None:
        if self.robot.io.input_1 or self.robot.io.input_1: pass
        
        warning = "<span style=\"color:#ff0000;\">{}</span>".format("WARNING")
        clear = "<span style=\"color:#00ff00;\">{}</span>".format("CLEAR")

        self.ui.switch_1.setText(warning if self.robot.io.input_1 else clear)
        self.ui.switch_2.setText(warning if self.robot.io.input_2 else clear)

    ''' 显示 原始位置速度 '''
    def show_motor_original(self, node_id) -> None:
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
    
    ''' 显示 控制模式 '''
    def show_motor_mode(self, node_id) -> None:
        control_mode = getattr(self.robot, f"motor_{node_id}").control_mode

        if control_mode == "position_control": mode_str = "<span style=\"color:#00ff00;\">POSITION</span>"
        elif control_mode == "speed_control": mode_str = "<span style=\"color:#00ff00;\">SPEED</span>"
        
        getattr(self.ui, f"mode_{node_id}").setText(mode_str)
    
    ''' 显示 力 '''
    def show_force(self, node_id) -> None:
        if node_id in Sensor.sensor_dict.keys():
            force = getattr(self.robot, f"sensor_{node_id}").force

            if force > 0: color = "#0000ff"
            else:
                if abs(force) <= 10: color = "#00ff00"
                else: color = "#ffff00"
            
            force_str = "<span style=\"color:{};\">{}</span>".format(color, round(abs(force), 2))

            getattr(self.ui, f"force_{node_id}").setText(force_str)

    ''' 显示 电磁阀 '''
    def show_valve(self) -> None:
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
    
    ''' 显示 程序状态 '''
    def show_status(self, message) -> None:
        self.ui.statusBar.showMessage(message, 5000)
    
    ''' 显示 夹爪 '''
    def show_gripper(self, is_zero):
        if is_zero:
            color = "#00ff00" if self.robot.gripper_position >= 0 else "#ff0000"

            self.ui.ballscrew.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.gripper_position, 2)))
            self.ui.ballscrew_v.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.gripper_velocity, 2)))
        else:
            self.ui.ballscrew.setText("<span style=\"color:#ff0000;\">No Zero</span>")
            self.ui.ballscrew_v.setText("<span style=\"color:#ff0000;\">No Zero</span>")
    
    ''' 显示 绳 '''
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

    ''' 显示 运动学参数 '''
    def show_kinematics(self):
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
        class UpdateStatus(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.updateStatus()
        class UpdateForce(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.updateForce()

        if self.robot.initUsbCan():
            self.update_status_thread = UpdateStatus(self.robot)
            self.update_force_thread = UpdateForce(self.robot)

            self.update_status_thread.start()
            self.update_force_thread.start()

            self.ui.bt_open_device.setEnabled(False)

            self.ui.bt_init_robot.setEnabled(True)

            self.show_status("Open Device !!!")
        else: self.show_status("Open Device Failed")
        
    ''' 初始化机器人 '''
    def initRobot(self) -> None:
        def change(status):
            if status:
                self.ui.bt_init_robot.setEnabled(False)
                self.show_status("Initializing Robot ...")
            else:
                self.ui.bt_init_robot.setEnabled(True)
                self.show_status("Something is wrong in the progress of Initializing Robot, please try again.")
        def next():
            self.ui.bt_init_robot.setEnabled(False)
            self.show_status("Robot is ready, Control is launch !!!")

            self.ui.control.setEnabled(True)
            self.ui.control_all.setEnabled(True)
            self.ui.status.setEnabled(True)
            self.ui.param.setEnabled(True)

            self.ui.calibrate_gripper.setEnabled(True) # 调零
            self.ui.start_adjust.setEnabled(True) # 调零
            self.ui.set_sensor_zero.setEnabled(True)

        self.robot.robot_init_start.connect(change)
        self.robot.robot_init_end.connect(next)
        
        class InitRobot(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.initRobot()
        class SendSensorRequest(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.sendSensorRequest()

        self.init_robot_thread = InitRobot(self.robot)
        self.send_sensor_request = SendSensorRequest(self.robot)

        self.init_robot_thread.start()
        self.send_sensor_request.start()

    ''' 标定夹爪 '''
    def calibrateGripper(self):
        def start():
            self.ui.calibrate_gripper.setEnabled(False)
            self.show_status("Ballscrew is being setting zero ...")
        def end():
            self.ui.calibrate_gripper.setEnabled(True)
            self.show_status("Ballscrew is set zero !")
            self.ui.ballscrew.setText("<span style=\"color:#00ff00;\">Zero</span>")
        
        self.robot.gripper_calibration_start.connect(start)
        self.robot.gripper_calibration_end.connect(end)

        class GripperCalibration(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.calibrateGripper()

        self.gripper_calibration_thread = GripperCalibration(self.robot)
        self.gripper_calibration_thread.start()
    
    ''' 线 调零 '''
    def rope_force_adapt(self):
        def start():
            self.ui.start_adjust.setEnabled(False)
            self.ui.set_rope_zero.setEnabled(True)
            self.show_status("All ropes are being adapting force ...")

        def finish():
            self.ui.start_adjust.setEnabled(True)
            self.ui.set_rope_zero.setEnabled(False)
            self.show_status("All ropes are set zero !")

            for i in range(1,10):
                getattr(self.ui, f"rope_{i}").setText("<span style=\"color:#00ff00;\">Zero</span>")

        i_f = float(self.ui.inside_force.text()) if self.ui.inside_force.text() != "" \
            else float(self.ui.inside_force.placeholderText())
        m_f = float(self.ui.midside_force.text()) if self.ui.midside_force.text() != "" \
            else float(self.ui.midside_force.placeholderText())
        o_f = float(self.ui.outside_force.text()) if self.ui.outside_force.text() != "" \
            else float(self.ui.outside_force.placeholderText())
        
        i_pid = (float(self.ui.inside_p.text()) if self.ui.inside_p.text() != "" \
                    else float(self.ui.inside_p.placeholderText()), 
                 float(self.ui.inside_i.text()) if self.ui.inside_i.text() != "" \
                    else float(self.ui.inside_i.placeholderText()), 
                 float(self.ui.inside_d.text()) if self.ui.inside_d.text() != "" \
                    else float(self.ui.inside_d.placeholderText()))
        m_pid = (float(self.ui.midside_p.text()) if self.ui.midside_p.text() != "" \
                    else float(self.ui.midside_p.placeholderText()), 
                 float(self.ui.midside_i.text()) if self.ui.midside_i.text() != "" \
                    else float(self.ui.midside_i.placeholderText()), 
                 float(self.ui.midside_d.text()) if self.ui.midside_d.text() != "" \
                    else float(self.ui.midside_d.placeholderText()))
        o_pid = (float(self.ui.outside_p.text()) if self.ui.outside_p.text() != "" \
                    else float(self.ui.midside_p.placeholderText()), 
                 float(self.ui.outside_i.text()) if self.ui.outside_i.text() != "" \
                    else float(self.ui.outside_i.placeholderText()), 
                 float(self.ui.outside_d.text()) if self.ui.outside_d.text() != "" \
                    else float(self.ui.outside_d.placeholderText()))
        
        self.robot.rope_force_adapt(i_f, m_f, o_f, i_pid, m_pid, o_pid, start, finish)
    def rope_set_zero(self):
        self.robot.rope_set_zero()
    
    ''' 力 调零 '''
    def calibrateForceSensor(self):
        def start():
            self.ui.set_sensor_zero.setEnabled(False)
            self.show_status("All sensors are being adapting force ...")

        def finish():
            self.ui.set_sensor_zero.setEnabled(True)
            self.show_status("All sensors are set zero !")
        
        force_list = []
        for i in range(1,10):
            box = getattr(self.ui, f"force_ref_{i}")
            force_list.append(float(box.text()) if box.text() != "" else float(box.placeholderText()))
        
        self.robot.calibrateForceSensor(force_list, 100, start, finish)


    ''' 电机 速度 正 '''
    def speed_forward_factory(self, node_id):
        # setattr(self, f"joint_{node_id}", JointControlSpeedModeThread(getattr(self.robot, "motor_{}".format(node_id)), int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value()), is_forward=True))
        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)
        
        speed = int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value())
        self.robot.joint_speed([node_id], speed)

        # getattr(self, f"joint_{node_id}").start()
    for node_id in range(1,11):
        exec(f"def speed_forward_{node_id}(self): self.speed_forward_factory({node_id})")
    
    ''' 电机 速度 反 '''
    def speed_reverse_factory(self, node_id):
        # setattr(self, f"joint_{node_id}", JointControlSpeedModeThread(getattr(self.robot, "motor_{}".format(node_id)), int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value()), is_forward=False))
        
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)

        speed = - int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value())
        self.robot.joint_speed([node_id], speed)

        # getattr(self, f"joint_{node_id}").start()
    for node_id in range(1,11):
        exec(f"def speed_reverse_{node_id}(self): self.speed_reverse_factory({node_id})")

    ''' 电机 速度 停 '''
    def speed_stop_factory(self, node_id):
        # getattr(self, f"joint_{node_id}").stop()
        # getattr(self, f"joint_{node_id}").wait()

        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(True)

        self.robot.joint_speed_stop()
    for node_id in range(1,11):
        exec(f"def speed_stop_{node_id}(self): self.speed_stop_factory({node_id})")

    ''' 夹爪归零 '''
    def homingGripper(self):
        def start():
            self.ui.homing_gripper.setEnabled(False)
            self.show_status("Ballscrew is backing to zero ...")
        def end():
            self.ui.homing_gripper.setEnabled(True)
            self.show_status("Ballscrew is backed to zero !")

        self.robot.gripper_homing_start.connect(start)
        self.robot.gripper_homing_end.connect(end)

        class GripperCalibration(QThread):
            def __init__(self, robot: ContinuumRobot) -> None:
                super().__init__()
                self.robot = robot
            def run(self):
                self.robot.homingGripper()

        self.gripper_homing_thread = GripperCalibration(self.robot)
        self.gripper_homing_thread.start()
   
    ''' 标定 '''
    def calibrateContinuum(self):
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




    def extendInsideSection(self):
        self.robot.moveInsideSection(s_d=float(self.ui.s_d.text()) if self.ui.s_d.text()!="" else 0.0,
                                     kappa_d=0.0,
                                     phi_d=0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def shortenInsideSection(self):
        self.robot.moveInsideSection(s_d=-abs(float(self.ui.s_d.text())) if self.ui.s_d.text()!="" else 0.0,
                                     kappa_d=0.0,
                                     phi_d=0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def curveInsideSection(self):
        self.robot.moveInsideSection(s_d=0.0,
                                     kappa_d=float(self.ui.kappa_d.text()) if self.ui.kappa_d.text()!="" else 0.0,
                                     phi_d=0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def straightenInsideSection(self):
        self.robot.moveInsideSection(s_d=0.0,
                                     kappa_d=-abs(float(self.ui.kappa_d.text())) if self.ui.kappa_d.text()!="" else 0.0,
                                     phi_d=0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def rotateInsideSectionClockwise(self):
        self.robot.moveInsideSection(s_d=0.0,
                                     kappa_d=0.0,
                                     phi_d=float(self.ui.phi_d.text()) if self.ui.phi_d.text()!="" else 0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def rotateInsideSectionAntiClockwise(self):
        self.robot.moveInsideSection(s_d=0.0,
                                     kappa_d=0.0,
                                     phi_d=-abs(float(self.ui.phi_d.text())) if self.ui.phi_d.text()!="" else 0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    def moveInsideSection(self):
        self.robot.moveInsideSection(s_d=float(self.ui.s_d.text()) if self.ui.s_d.text()!="" else 0.0,
                                     kappa_d=float(self.ui.kappa_d.text()) if self.ui.kappa_d.text()!="" else 0.0,
                                     phi_d=float(self.ui.phi_d.text()) if self.ui.phi_d.text()!="" else 0.0,
                                     ref=(float(self.ui.ref_inside.text()), float(self.ui.ref_midside.text()), float(self.ui.ref_outside.text())),
                                     kp=(float(self.ui.kp_inside.text()), float(self.ui.kp_midside.text()), float(self.ui.kp_outside.text())),
                                     ki=(float(self.ui.ki_inside.text()), float(self.ui.ki_midside.text()), float(self.ui.ki_outside.text())),
                                     kd=(float(self.ui.kd_inside.text()), float(self.ui.kd_midside.text()), float(self.ui.kd_outside.text())))
    
    def initParameterTable(self):
        for i in range(self.ui.tableWidget.rowCount()):
            try: self.ui.tableWidget.item(i, 0).setText(str(self.robot.PARAMETER[self.ui.tableWidget.verticalHeaderItem(i).text()]))
            except: print("No item")
        
        def updateParameter(row, column):
            try:
                variable = self.ui.tableWidget.verticalHeaderItem(row).text()
                self.robot.PARAMETER[variable] = float(self.ui.tableWidget.item(row, column).text())
                print("\033[0;32m[Parameter] {} = {}\033[0m".format(variable, self.robot.PARAMETER[variable]))
            except: print("No item")

        self.ui.tableWidget.cellChanged.connect(updateParameter)
