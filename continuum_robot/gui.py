# -*- coding:utf-8 -*-


''' gui.py GUI v4.4 '''


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal, QMutex

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
        
        self.robot.read_canopen_thread.show_motor_status.connect(self.show_motor_status)
        self.robot.read_canopen_thread.show_motor_original.connect(self.show_motor_original)
        self.robot.read_canopen_thread.show_motor_mode.connect(self.show_motor_mode)
        self.robot.read_canopen_thread.show_switch.connect(self.show_switch)
        self.robot.read_canopen_thread.status_signal.connect(self.show_status)
        self.robot.read_canopen_thread.show_ballscrew.connect(self.show_ballscrew)
        self.robot.read_canopen_thread.show_rope.connect(self.show_rope)
        self.robot.read_canopen_thread.show_kinematics.connect(self.show_kinematics)

        self.robot.io.show_valve.connect(self.show_valve)

        self.robot.read_sensor_thread.show_force.connect(self.show_force)
        
        self.signal_connect_slot()

        ''' 高级测试 '''
        self.ui.stretch_inside.pressed.connect(self.stretch_inside)
        self.ui.stretch_inside.released.connect(self.stop_inside)
        self.ui.release_inside.pressed.connect(self.release_inside)
        self.ui.release_inside.released.connect(self.stop_inside)

        self.ui.stretch_midside.pressed.connect(self.stretch_midside)
        self.ui.stretch_midside.released.connect(self.stop_midside)
        self.ui.release_midside.pressed.connect(self.release_midside)
        self.ui.release_midside.released.connect(self.stop_midside)

        self.ui.stretch_outside.pressed.connect(self.stretch_outside)
        self.ui.stretch_outside.released.connect(self.stop_outside)
        self.ui.release_outside.pressed.connect(self.release_outside)
        self.ui.release_outside.released.connect(self.stop_outside)

        # self.ui.test_4.clicked.connect(lambda: self.robot.ballscrew_move(221, 10, is_close=False, is_relative=False))
        # self.ui.test_5.clicked.connect(lambda: self.robot.ballscrew_move(237, 10, is_close=False, is_relative=False))
        # self.ui.test_6.clicked.connect(lambda: self.robot.ballscrew_move(348, 10, is_close=False, is_relative=False))
        # self.ui.test_7.clicked.connect(lambda: self.robot.ballscrew_move(358, 10, is_close=False, is_relative=False))

        # self.ui.test_8.clicked.connect(self.force_test)
        # self.ui.test_9.clicked.connect(self.force_test_stop)
        

        self.ui.test_1.clicked.connect(self.robot.ExtendInsideSection)
        self.ui.test_2.clicked.connect(self.robot.ShortenInsideSection)
        self.ui.test_3.clicked.connect(self.robot.StopInsideSection)
        self.ui.test_4.pressed.connect(self.robot.CurveInsideSection)
        self.ui.test_4.released.connect(self.robot.StopInsideSection)
        self.ui.test_5.pressed.connect(self.robot.StraightenInsideSection)
        self.ui.test_5.released.connect(self.robot.StopInsideSection)
        self.ui.test_6.pressed.connect(self.robot.RotateInsideSectionClockwise)
        self.ui.test_6.released.connect(self.robot.StopInsideSection)
        self.ui.test_7.pressed.connect(self.robot.RotateInsideSectionAntiClockwise)
        self.ui.test_7.released.connect(self.robot.StopInsideSection)

        self.ui.test_8.clicked.connect(self.robot.ExtendMidsideSection)
        self.ui.test_9.clicked.connect(self.robot.ShortenMidsideSection)
        self.ui.test_10.clicked.connect(self.robot.StopMidsideSection)
        self.ui.test_11.pressed.connect(self.robot.CurveMidsideSection)
        self.ui.test_11.released.connect(self.robot.StopMidsideSection)
        self.ui.test_12.pressed.connect(self.robot.StraightenMidsideSection)
        self.ui.test_12.released.connect(self.robot.StopMidsideSection)
        self.ui.test_13.pressed.connect(self.robot.RotateMidsideSectionClockwise)
        self.ui.test_13.released.connect(self.robot.StopMidsideSection)
        self.ui.test_14.pressed.connect(self.robot.RotateMidsideSectionAntiClockwise)
        self.ui.test_14.released.connect(self.robot.StopMidsideSection)

        self.ui.test_15.clicked.connect(self.robot.ExtendOutsideSection)
        self.ui.test_16.clicked.connect(self.robot.ShortenOutsideSection)
        self.ui.test_17.clicked.connect(self.robot.StopOutsideSection)
        self.ui.test_18.pressed.connect(self.robot.CurveOutsideSection)
        self.ui.test_18.released.connect(self.robot.StopOutsideSection)
        self.ui.test_19.pressed.connect(self.robot.StraightenOutsideSection)
        self.ui.test_19.released.connect(self.robot.StopOutsideSection)
        self.ui.test_20.pressed.connect(self.robot.RotateOutsideSectionClockwise)
        self.ui.test_20.released.connect(self.robot.StopOutsideSection)
        self.ui.test_21.pressed.connect(self.robot.RotateOutsideSectionAntiClockwise)
        self.ui.test_21.released.connect(self.robot.StopOutsideSection)

        # self.ui.test_11.clicked.connect(lambda: self.robot.rope_move("1", 3, 1, is_relative=True))
        # self.ui.test_12.clicked.connect(lambda: self.robot.rope_move("1", -3, 1, is_relative=True))

        self.ui.pushButton.pressed.connect(self.robot.config_space_multi)
        self.ui.pushButton.released.connect(self.robot.config_space_stop)
        self.ui.teleoperation.clicked.connect(self.robot.teleoperation_thread.start)

        self.ui.bt_open_camera.clicked.connect(self.OpenCamera)
        self.ui.bt_close_camera.clicked.connect(self.robot.CloseCamera)


        self.show() # 显示界面
    

    ''' Jumping '''
    def signal_connect_slot(self) -> None:
        ''' 菜单 '''
        self.ui.control_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.set_zero_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.kinematics_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.video_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(4))

        self.ui.statusBar.setSizeGripEnabled(False)
        self.ui.statusBar.showMessage("Welcome to Continnum Robot Control Panel", 10000)
        
        ''' 打开设备 '''
        self.ui.bt_open_device.setEnabled(True)
        self.ui.bt_open_device.setText("Open Device")
        self.ui.bt_open_device.clicked.connect(self.InitUSBCAN)

        ''' 初始化机器人 '''
        self.ui.bt_init_robot.setEnabled(False)
        self.ui.bt_init_robot.setText("Initialize Robot")
        self.ui.bt_init_robot.clicked.connect(self.InitRobot)

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

        ''' 滚珠 调0 归0 '''
        self.ui.set_ballscrew_zero.clicked.connect(self.GripperCalibration)
        self.ui.go_zero.clicked.connect(self.ballscrew_go_zero)

        ''' 线 适应 调0 '''
        self.ui.start_adjust.clicked.connect(self.rope_force_adapt)
        self.ui.set_rope_zero.clicked.connect(self.rope_set_zero)

        ''' 力 调0 '''
        self.ui.set_sensor_zero.clicked.connect(self.ForceSensorCalibration)

        ''' 伸 缩 '''
        self.ui.extension.clicked.connect(self.robot.forward)
        self.ui.retraction.clicked.connect(self.robot.back)

        self.ui.bt_cali.clicked.connect(self.ContinuumCalibration)

        self.ui.curve_forward_out.pressed.connect(lambda: self.robot.config_space("outside", "curve"))
        self.ui.curve_forward_out.released.connect(self.robot.config_space_stop)
        self.ui.curve_back_out.pressed.connect(lambda: self.robot.config_space("outside", "straighten"))
        self.ui.curve_back_out.released.connect(self.robot.config_space_stop)
        self.ui.angle_forward_out.pressed.connect(lambda: self.robot.config_space("outside", "rotate_clockwise"))
        self.ui.angle_forward_out.released.connect(self.robot.config_space_stop)
        self.ui.angle_back_out.pressed.connect(lambda: self.robot.config_space("outside", "rotate_anticlockwise"))
        self.ui.angle_back_out.released.connect(self.robot.config_space_stop)
        self.ui.reset_out.clicked.connect(lambda: self.robot.config_space("outside", "reset"))

        self.ui.curve_forward_mid.pressed.connect(lambda: self.robot.config_space("midside", "curve"))
        self.ui.curve_forward_mid.released.connect(self.robot.config_space_stop)
        self.ui.curve_back_mid.pressed.connect(lambda: self.robot.config_space("midside", "straighten"))
        self.ui.curve_back_mid.released.connect(self.robot.config_space_stop)
        self.ui.angle_forward_mid.pressed.connect(lambda: self.robot.config_space("midside", "rotate_clockwise"))
        self.ui.angle_forward_mid.released.connect(self.robot.config_space_stop)
        self.ui.angle_back_mid.pressed.connect(lambda: self.robot.config_space("midside", "rotate_anticlockwise"))
        self.ui.angle_back_mid.released.connect(self.robot.config_space_stop)
        self.ui.reset_mid.clicked.connect(lambda: self.robot.config_space("midside", "reset"))

        self.ui.curve_forward_in.pressed.connect(lambda: self.robot.config_space("inside", "curve"))
        self.ui.curve_forward_in.released.connect(self.robot.config_space_stop)
        self.ui.curve_back_in.pressed.connect(lambda: self.robot.config_space("inside", "straighten"))
        self.ui.curve_back_in.released.connect(self.robot.config_space_stop)
        self.ui.angle_forward_in.pressed.connect(lambda: self.robot.config_space("inside", "rotate_clockwise"))
        self.ui.angle_forward_in.released.connect(self.robot.config_space_stop)
        self.ui.angle_back_in.pressed.connect(lambda: self.robot.config_space("inside", "rotate_anticlockwise"))
        self.ui.angle_back_in.released.connect(self.robot.config_space_stop)
        self.ui.reset_in.clicked.connect(lambda: self.robot.config_space("inside", "reset"))


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
    
    ''' 显示 滚珠丝杠 '''
    def show_ballscrew(self, is_zero):
        if is_zero:
            color = "#00ff00" if self.robot.ballscrew_position >= 0 else "#ff0000"

            self.ui.ballscrew.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.ballscrew_position, 2)))
            self.ui.ballscrew_v.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.robot.ballscrew_velocity, 2)))
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


    ''' 打开设备 '''
    def InitUSBCAN(self) -> None:
        if self.robot.InitUSBCAN():
            self.ui.bt_open_device.setEnabled(False)

            self.ui.bt_init_robot.setEnabled(True)

            self.show_status("Open Device !!!")
        else: self.show_status("Open Device Failed")
    
    ''' 初始化机器人 '''
    def InitRobot(self) -> None:
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

            self.ui.set_ballscrew_zero.setEnabled(True) # 调零
            self.ui.start_adjust.setEnabled(True) # 调零
            self.ui.set_sensor_zero.setEnabled(True)
        
        self.robot.InitRobot(1, change, next)

    ''' 丝杠 调零 '''
    def GripperCalibration(self):
        def start():
            self.ui.ballscrew_set_zero.setEnabled(False)
            self.show_status("Ballscrew is being setting zero ...")
        
        def finish():
            self.ui.ballscrew_set_zero.setEnabled(True)
            self.show_status("Ballscrew is set zero !")
            self.ui.ballscrew.setText("<span style=\"color:#00ff00;\">Zero</span>")
        
        distance = float(self.ui.forward_distance.text()) if self.ui.forward_distance.text() != "" \
            else float(self.ui.forward_distance.placeholderText())
        velocity = int(self.ui.forward_velocity.text()) if self.ui.forward_velocity.text() != "" \
            else int(self.ui.forward_velocity.placeholderText())
        speed = int(self.ui.backward_speed.text()) if self.ui.backward_speed.text() != "" \
            else int(self.ui.backward_speed.placeholderText())
        
        self.robot.GripperCalibration(distance, velocity, speed, start, finish)
    
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
    def ForceSensorCalibration(self):
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
        
        self.robot.ForceSensorCalibration(force_list, 100, start, finish)


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

    ''' 电机10 归零 '''
    def ballscrew_go_zero(self):
        def start():
            self.ui.go_zero.setEnabled(False)
            self.show_status("Ballscrew is backing to zero ...")
        
        def finish():
            self.ui.go_zero.setEnabled(True)
            self.show_status("Ballscrew is backed to zero !")
        
        self.robot.ballscrew_go_zero(300, start, finish)
   
    
    ''' 缩放 内段 线 '''
    def stretch_inside(self):
        self.stretch_inside_thread = StretchInsideThread(-50, self.robot.motor_7, self.robot.motor_8, self.robot.motor_9)

        self.force_test_thread_1 = JointForceFollow(self.robot.motor_1, self.robot.sensor_1, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_2 = JointForceFollow(self.robot.motor_2, self.robot.sensor_2, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_3 = JointForceFollow(self.robot.motor_3, self.robot.sensor_3, force_ref=5, kp=10, ki=0, kd=0)
        
        self.force_test_thread_4 = JointForceFollow(self.robot.motor_4, self.robot.sensor_4, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_5 = JointForceFollow(self.robot.motor_5, self.robot.sensor_5, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_6 = JointForceFollow(self.robot.motor_6, self.robot.sensor_6, force_ref=5, kp=10, ki=0, kd=0)

        self.stretch_inside_thread.start()
        self.force_test_thread_1.start()
        self.force_test_thread_2.start()
        self.force_test_thread_3.start()
        self.force_test_thread_4.start()
        self.force_test_thread_5.start()
        self.force_test_thread_6.start()
    def release_inside(self):
        self.stretch_inside_thread = StretchInsideThread(50, self.robot.motor_7, self.robot.motor_8, self.robot.motor_9)
        self.stretch_inside_thread.start()
    def stop_inside(self):
        self.stretch_inside_thread.stop()
        self.force_test_thread_1.stop()
        self.force_test_thread_2.stop()
        self.force_test_thread_3.stop()
        self.force_test_thread_4.stop()
        self.force_test_thread_5.stop()
        self.force_test_thread_6.stop()
        self.stretch_inside_thread.wait()
        self.force_test_thread_1.wait()
        self.force_test_thread_2.wait()
        self.force_test_thread_3.wait()
        self.force_test_thread_4.wait()
        self.force_test_thread_5.wait()
        self.force_test_thread_6.wait()


    ''' 缩放 中段 线 '''
    def stretch_midside(self):
        self.stretch_midside_thread = StretchMidsideThread(-50, self.robot.motor_4, self.robot.motor_5, self.robot.motor_6)
        
        self.force_test_thread_1 = JointForceFollow(self.robot.motor_1, self.robot.sensor_1, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_2 = JointForceFollow(self.robot.motor_2, self.robot.sensor_2, force_ref=5, kp=10, ki=0, kd=0)
        self.force_test_thread_3 = JointForceFollow(self.robot.motor_3, self.robot.sensor_3, force_ref=5, kp=10, ki=0, kd=0)
        
        self.stretch_midside_thread.start()
        self.force_test_thread_1.start()
        self.force_test_thread_2.start()
        self.force_test_thread_3.start()
    def release_midside(self):
        self.stretch_midside_thread = StretchMidsideThread(50, self.robot.motor_4, self.robot.motor_5, self.robot.motor_6)
        
        self.stretch_midside_thread.start()
    def stop_midside(self):
        self.stretch_midside_thread.stop()
        self.force_test_thread_1.stop()
        self.force_test_thread_2.stop()
        self.force_test_thread_3.stop()
        self.stretch_midside_thread.wait()
        self.force_test_thread_1.wait()
        self.force_test_thread_2.wait()
        self.force_test_thread_3.wait()
    
    ''' 缩放 外段 线 '''
    def stretch_outside(self):
        self.stretch_outide_thread = StretchOutsideThread(-50, 
            self.robot.motor_1, self.robot.motor_2, self.robot.motor_3)
        self.stretch_outide_thread.start()
    def release_outside(self):
        self.stretch_outide_thread = StretchOutsideThread(50, 
            self.robot.motor_1, self.robot.motor_2, self.robot.motor_3)
        self.stretch_outide_thread.start()
    def stop_outside(self):
        self.stretch_outide_thread.stop()
        
        self.stretch_outide_thread.wait()
    
    ''' 标定 '''
    def ContinuumCalibration(self):
        bl_o = float(self.ui.outside_length.text()) if self.ui.outside_length.text() != "" else float(self.ui.outside_length.placeholderText())
        bl_m = float(self.ui.midside_length.text()) if self.ui.midside_length.text() != "" else float(self.ui.midside_length.placeholderText())
        bl_i = float(self.ui.inside_length.text()) if self.ui.inside_length.text() != "" else float(self.ui.inside_length.placeholderText())
        self.robot.ContinuumCalibration(bl_o, bl_m, bl_i)
    
    
    
    def force_test(self):
        self.force_test_thread_1 = JointForceFollow(self.motor_1, self.sensor_1, force_ref=5, kp=5, ki=0, kd=0)
        self.force_test_thread_2 = JointForceFollow(self.motor_2, self.sensor_2, force_ref=5, kp=5, ki=0, kd=0)
        self.force_test_thread_3 = JointForceFollow(self.motor_3, self.sensor_3, force_ref=5, kp=5, ki=0, kd=0)
        
        self.force_test_thread_4 = JointForceFollow(self.motor_4, self.sensor_4, force_ref=5, kp=5, ki=0, kd=0)
        self.force_test_thread_5 = JointForceFollow(self.motor_5, self.sensor_5, force_ref=5, kp=5, ki=0, kd=0)
        self.force_test_thread_6 = JointForceFollow(self.motor_6, self.sensor_6, force_ref=5, kp=5, ki=0, kd=0)
        
        # self.force_test_thread_7 = JointForceFollow(self.motor_7, self.sensor_7, force_ref=10, kp=5, ki=0, kd=1)
        # self.force_test_thread_8 = JointForceFollow(self.motor_8, self.sensor_8, force_ref=10, kp=5, ki=0, kd=1)
        # self.force_test_thread_9 = JointForceFollow(self.motor_9, self.sensor_9, force_ref=10, kp=5, ki=0, kd=1)
        
        self.force_test_thread_1.start()
        self.force_test_thread_2.start()
        self.force_test_thread_3.start()
        self.force_test_thread_4.start()
        self.force_test_thread_5.start()
        self.force_test_thread_6.start()
        # self.force_test_thread_7.start()
        # self.force_test_thread_8.start()
        # self.force_test_thread_9.start()
    def force_test_stop(self):
        self.force_test_thread_1.stop()
        self.force_test_thread_2.stop()
        self.force_test_thread_3.stop()
        self.force_test_thread_4.stop()
        self.force_test_thread_5.stop()
        self.force_test_thread_6.stop()
        # self.force_test_thread_7.stop()
        # self.force_test_thread_8.stop()
        # self.force_test_thread_9.stop()

        self.force_test_thread_1.wait()
        self.force_test_thread_2.wait()
        self.force_test_thread_3.wait()
        self.force_test_thread_4.wait()
        self.force_test_thread_5.wait()
        self.force_test_thread_6.wait()
        # self.force_test_thread_7.wait()
        # self.force_test_thread_8.wait()
        # self.force_test_thread_9.wait()

    ''' 相机 '''
    def OpenCamera(self):
        def update(image):
            self.ui.label_video.setPixmap(QPixmap.fromImage(image)) # 往显示视频的Label里显示QImage
        def clear():
            self.ui.label_video.clear() # 清除label组件上的图片
            self.ui.label_video.setText("No Video Stream")
        self.ui.label_video.setScaledContents(True) # 自适应
        self.robot.OpenCamera(update_slot=update, clear_slot=clear)



'''
    测试
'''
class StretchInsideThread(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, speed: int, 
                #  motor_1: Motor, 
                #  motor_2: Motor, 
                #  motor_3: Motor, 
                #  motor_4: Motor, 
                #  motor_5: Motor, 
                #  motor_6: Motor, 
                 motor_7: Motor, 
                 motor_8: Motor, 
                 motor_9: Motor, 
                 /, *, start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        # self.__motor_1 = motor_1
        # self.__motor_2 = motor_2
        # self.__motor_3 = motor_3

        # self.__motor_4 = motor_4
        # self.__motor_5 = motor_5
        # self.__motor_6 = motor_6

        self.__motor_7 = motor_7
        self.__motor_8 = motor_8
        self.__motor_9 = motor_9

        self.__speed = speed

        if start_signal != None and finish_signal != None:
            self.__start_signal.connect(start_signal)
            self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()

        # while not self.__motor_1.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_2.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_3.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_4.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_5.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_6.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_7.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_8.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_9.set_control_mode("speed_control", check=False): time.sleep(0.1)

        # self.__motor_1.set_speed(self.__speed, is_pdo=True)
        # self.__motor_2.set_speed(self.__speed, is_pdo=True)
        # self.__motor_3.set_speed(self.__speed, is_pdo=True)
        # self.__motor_4.set_speed(self.__speed, is_pdo=True)
        # self.__motor_5.set_speed(self.__speed, is_pdo=True)
        # self.__motor_6.set_speed(self.__speed, is_pdo=True)
        self.__motor_7.set_speed(self.__speed, is_pdo=True)
        self.__motor_8.set_speed(self.__speed, is_pdo=True)
        self.__motor_9.set_speed(self.__speed, is_pdo=True)

        # self.__motor_1.halt(is_pdo=True)
        # self.__motor_2.halt(is_pdo=True)
        # self.__motor_3.halt(is_pdo=True)
        # self.__motor_4.halt(is_pdo=True)
        # self.__motor_5.halt(is_pdo=True)
        # self.__motor_6.halt(is_pdo=True)
        self.__motor_7.halt(is_pdo=True)
        self.__motor_8.halt(is_pdo=True)
        self.__motor_9.halt(is_pdo=True)
        
        while not self.__is_stop:
            # self.__motor_1.enable_operation(is_pdo=True)
            # self.__motor_2.enable_operation(is_pdo=True)
            # self.__motor_3.enable_operation(is_pdo=True)
            # self.__motor_4.enable_operation(is_pdo=True)
            # self.__motor_5.enable_operation(is_pdo=True)
            # self.__motor_6.enable_operation(is_pdo=True)
            self.__motor_7.enable_operation(is_pdo=True)
            self.__motor_8.enable_operation(is_pdo=True)
            self.__motor_9.enable_operation(is_pdo=True)
    
    def stop(self):
        self.__is_stop = True

        # self.__motor_1.set_speed(0, is_pdo=True)
        # self.__motor_2.set_speed(0, is_pdo=True)
        # self.__motor_3.set_speed(0, is_pdo=True)
        # self.__motor_4.set_speed(0, is_pdo=True)
        # self.__motor_5.set_speed(0, is_pdo=True)
        # self.__motor_6.set_speed(0, is_pdo=True)
        self.__motor_7.set_speed(0, is_pdo=True)
        self.__motor_8.set_speed(0, is_pdo=True)
        self.__motor_9.set_speed(0, is_pdo=True)

        # self.__motor_1.disable_operation(is_pdo=True)
        # self.__motor_2.disable_operation(is_pdo=True)
        # self.__motor_3.disable_operation(is_pdo=True)
        # self.__motor_4.disable_operation(is_pdo=True)
        # self.__motor_5.disable_operation(is_pdo=True)
        # self.__motor_6.disable_operation(is_pdo=True)
        self.__motor_7.disable_operation(is_pdo=True)
        self.__motor_8.disable_operation(is_pdo=True)
        self.__motor_9.disable_operation(is_pdo=True)

class StretchMidsideThread(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, speed: int, 
                 motor_1: Motor, 
                 motor_2: Motor, 
                 motor_3: Motor, 
                #  motor_4: Motor, 
                #  motor_5: Motor, 
                #  motor_6: Motor,
                 /, *, start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.__motor_1 = motor_1
        self.__motor_2 = motor_2
        self.__motor_3 = motor_3

        # self.__motor_4 = motor_4
        # self.__motor_5 = motor_5
        # self.__motor_6 = motor_6

        self.__speed = speed

        if start_signal != None and finish_signal != None:
            self.__start_signal.connect(start_signal)
            self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()

        while not self.__motor_1.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_2.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_3.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_4.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_5.set_control_mode("speed_control", check=False): time.sleep(0.1)
        # while not self.__motor_6.set_control_mode("speed_control", check=False): time.sleep(0.1)

        self.__motor_1.set_speed(self.__speed, is_pdo=True)
        self.__motor_2.set_speed(self.__speed, is_pdo=True)
        self.__motor_3.set_speed(self.__speed, is_pdo=True)
        # self.__motor_4.set_speed(self.__speed, is_pdo=True)
        # self.__motor_5.set_speed(self.__speed, is_pdo=True)
        # self.__motor_6.set_speed(self.__speed, is_pdo=True)

        self.__motor_1.halt(is_pdo=True)
        self.__motor_2.halt(is_pdo=True)
        self.__motor_3.halt(is_pdo=True)
        # self.__motor_4.halt(is_pdo=True)
        # self.__motor_5.halt(is_pdo=True)
        # self.__motor_6.halt(is_pdo=True)
        
        while not self.__is_stop:
            self.__motor_1.enable_operation(is_pdo=True)
            self.__motor_2.enable_operation(is_pdo=True)
            self.__motor_3.enable_operation(is_pdo=True)
            # self.__motor_4.enable_operation(is_pdo=True)
            # self.__motor_5.enable_operation(is_pdo=True)
            # self.__motor_6.enable_operation(is_pdo=True)
    
    def stop(self):
        self.__is_stop = True

        self.__motor_1.set_speed(0, is_pdo=True)
        self.__motor_2.set_speed(0, is_pdo=True)
        self.__motor_3.set_speed(0, is_pdo=True)
        # self.__motor_4.set_speed(0, is_pdo=True)
        # self.__motor_5.set_speed(0, is_pdo=True)
        # self.__motor_6.set_speed(0, is_pdo=True)

        self.__motor_1.disable_operation(is_pdo=True)
        self.__motor_2.disable_operation(is_pdo=True)
        self.__motor_3.disable_operation(is_pdo=True)
        # self.__motor_4.disable_operation(is_pdo=True)
        # self.__motor_5.disable_operation(is_pdo=True)
        # self.__motor_6.disable_operation(is_pdo=True)

class StretchOutsideThread(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, speed: int, 
                 motor_1: Motor, 
                 motor_2: Motor, 
                 motor_3: Motor, 
                 /, *, start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.__motor_1 = motor_1
        self.__motor_2 = motor_2
        self.__motor_3 = motor_3

        self.__speed = speed

        if start_signal != None and finish_signal != None:
            self.__start_signal.connect(start_signal)
            self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()

        while not self.__motor_1.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_2.set_control_mode("speed_control", check=False): time.sleep(0.1)
        while not self.__motor_3.set_control_mode("speed_control", check=False): time.sleep(0.1)

        self.__motor_1.set_speed(self.__speed, is_pdo=True)
        self.__motor_2.set_speed(self.__speed, is_pdo=True)
        self.__motor_3.set_speed(self.__speed, is_pdo=True)

        self.__motor_1.halt(is_pdo=True)
        self.__motor_2.halt(is_pdo=True)
        self.__motor_3.halt(is_pdo=True)

        while not self.__is_stop:
            self.__motor_1.enable_operation(is_pdo=True)
            self.__motor_2.enable_operation(is_pdo=True)
            self.__motor_3.enable_operation(is_pdo=True)
    
    def stop(self):
        self.__is_stop = True

        self.__motor_1.set_speed(0, is_pdo=True)
        self.__motor_2.set_speed(0, is_pdo=True)
        self.__motor_3.set_speed(0, is_pdo=True)

        self.__motor_1.disable_operation(is_pdo=True)
        self.__motor_2.disable_operation(is_pdo=True)
        self.__motor_3.disable_operation(is_pdo=True)



class JointForceFollow(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, motor: Motor, sensor: Sensor, 
                 /, *, force_ref: int, kp: int, ki: int, kd: int, start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.__motor = motor
        self.__sensor = sensor

        self.__force_ref = force_ref
        
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd

        self.__current_error = 0
        self.__last_error = 0
        self.__error_integral = 0

        if start_signal != None and finish_signal != None:
            self.__start_signal.connect(start_signal)
            self.__finish_signal.connect(finish_signal)
    
    def run(self):
        print("im in !!")
        self.__start_signal.emit()

        self.__motor.set_control_mode("speed_control", check=False)
        
        self.__motor.halt(is_pdo=True)

        self.__motor.set_speed(0, is_pdo=True)

        self.__motor.enable_operation(is_pdo=True)

        while not self.__is_stop:
            if self.__sensor.force < 0:
                self.__last_error = self.__current_error
                
                self.__current_error = abs(self.__sensor.force) - self.__force_ref

                kp_out = self.__current_error * self.__kp
                
                self.__error_integral += self.__current_error * self.__ki

                kd_out = (self.__current_error - self.__last_error) * self.__kd

                speed = kp_out + self.__error_integral + kd_out

                print("force = {} speed = {}".format(round(self.__sensor.force, 2), int(speed)))

                self.__motor.set_speed(int(speed), is_pdo=True, log=False)
                # self.__motor.enable_operation(is_pdo=True)
            

    
    def stop(self):
        self.__is_stop = True

        self.__motor.set_speed(0, is_pdo=True)

        self.__motor.disable_operation(is_pdo=True)
