# -*- coding:utf-8 -*-


''' robot.py continuum robot v2.0 '''


from PyQt5.QtCore import QThread, pyqtSignal, QObject

from PyQt5 import QtGui
from PyQt5.QtGui import QImage

from math import *
import numpy as np
import pygame

import cv2

import time, json

from usbcan import UsbCan
from processor import CanOpenBusProcessor
from motor import Motor
from continuum_robot.io import IoModule
from sensor import Sensor


class ContinuumRobot(QObject):
    BALLSCREW_RATIO = 5120
    ROPE_RATIO = 12536.512440
    VELOCITY_RATIO = 440

    PARAMETER = {
        "gripper_calibration_forward_distance": 30,
        "gripper_calibration_forward_velocity": 200,
        "gripper_calibration_backward_velocity": 50,
        "gripper_homing_velocity": 300,
        "sensor_sampling_frequency": 20,
        "sensor_calibration_reference_force_1": 8.0,
        "sensor_calibration_reference_force_2": 8.0,
        "sensor_calibration_reference_force_3": 8.0,
        "sensor_calibration_reference_force_4": 8.0,
        "sensor_calibration_reference_force_5": 8.0,
        "sensor_calibration_reference_force_6": 8.0,
        "sensor_calibration_reference_force_7": 8.0,
        "sensor_calibration_reference_force_8": 8.0,
        "sensor_calibration_reference_force_9": 8.0,
        "sensor_calibration_init_count_1": 100,
        "sensor_calibration_init_count_2": 100,
        "sensor_calibration_init_count_3": 100,
        "sensor_calibration_init_count_4": 100,
        "sensor_calibration_init_count_5": 100,
        "sensor_calibration_init_count_6": 100,
        "sensor_calibration_init_count_7": 100,
        "sensor_calibration_init_count_8": 100,
        "sensor_calibration_init_count_9": 100,
    }

    show_motor_status = pyqtSignal(int)
    show_motor_original = pyqtSignal(int)
    show_motor_mode = pyqtSignal(int)
    show_switch = pyqtSignal()
    status_signal = pyqtSignal(str)
    show_gripper = pyqtSignal(bool)
    show_rope = pyqtSignal(bool, int)
    show_kinematics = pyqtSignal()

    show_force = pyqtSignal(int)

    robot_init_start = pyqtSignal(bool)
    robot_init_end = pyqtSignal()

    gripper_calibration_start = pyqtSignal()
    gripper_calibration_end = pyqtSignal()

    gripper_homing_start = pyqtSignal()
    gripper_homing_end = pyqtSignal()
    
    sensor_calibration_start = pyqtSignal()
    sensor_calibration_end = pyqtSignal()
    
    def __init__(self) -> None:
        QObject.__init__(self)

        # with open('robot_parameter.json', 'r') as file: self.PARAMETER = json.load(file) # 加载参数

        self.usbcan_0 = UsbCan.setDeviceType(type="USBCAN2", index="0").isShowLog(False)("0")
        self.usbcan_1 = UsbCan.setDeviceType(type="USBCAN2", index="0").isShowLog(False)("1")

        self.usbcan_0.setTimer("250K")
        self.usbcan_1.setTimer("1000K")

        self.usbcan_0_is_start = False
        self.usbcan_1_is_start = False

        CanOpenBusProcessor.linkDevice(self.usbcan_0)
        Sensor.linkDevice(self.usbcan_1)

        self.motor_1 = Motor(1, speed_range=[-400,400])
        self.motor_2 = Motor(2, speed_range=[-400,400])
        self.motor_3 = Motor(3, speed_range=[-400,400])
        self.motor_4 = Motor(4, speed_range=[-400,400])
        self.motor_5 = Motor(5, speed_range=[-400,400])
        self.motor_6 = Motor(6, speed_range=[-400,400])
        self.motor_7 = Motor(7, speed_range=[-400,400])
        self.motor_8 = Motor(8, speed_range=[-400,400])
        self.motor_9 = Motor(9, speed_range=[-400,400])
        self.motor_10 = Motor(10, speed_range=[-400,400])

        # self.MOTOR, self.SENSOR = {}, {}
        # for i in range(1,11):
        #     self.MOTOR[i] = Motor(i, speed_range=[-400, 400])
        #     self.SENSOR[i] = Sensor(i)

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

        self.io = IoModule(11)

        self.camera = None

        self.gripper_calibration = False
        self.gripper_position = None # mm
        self.gripper_velocity = None # mm/s

        self.rope_is_set_zero = False

        self.rope_1_position = None # mm
        self.rope_2_position = None
        self.rope_3_position = None
        self.rope_4_position = None
        self.rope_5_position = None
        self.rope_6_position = None
        self.rope_7_position = None
        self.rope_8_position = None
        self.rope_9_position = None

        self.rope_1_velocity = None # mm/s
        self.rope_2_velocity = None
        self.rope_3_velocity = None
        self.rope_4_velocity = None
        self.rope_5_velocity = None
        self.rope_6_velocity = None
        self.rope_7_velocity = None
        self.rope_8_velocity = None
        self.rope_9_velocity = None

        self.teleoperation_thread = TeleOperation(self)

        self.backbone_init_length = [92, 95.5, 96] # in mid out
        self.backbone_length = [None, None, None] # in mid out
        # self.backbone_length_d = [0, 0, 0]

        self.backbone_curvature = [None, None, None] # in mid out
        # self.backbone_curvature_d = [0, 0, 0]

        self.backbone_rotation_angle = [None, None, None] # in mid out
        # self.backbone_rotation_angle_d = [0, 0, 0]

        self.backbone_section_num = [11, 11, 11]
        self.backbone_d = 2.6

        self.continuum_calibration = False
        self.rope_zero_position = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_init_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_total_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_velocity = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_outside_length = [None, None, None] # 123
        self.rope_midside_length = [None, None, None, None, None, None] # 123456
        self.rope_inside_length = [None, None, None, None, None, None, None, None, None] # 123456789

        self.outside_coordinate = (None, None, None)
        self.midside_coordinate = (None, None, None)
        self.inside_coordinate = (None, None, None)
        self.outside_world_coordinate = (None, None, None)
        self.midside_world_coordinate = (None, None, None)
        self.inside_world_coordinate = (None, None, None)

    def updateStatus(self):
        print("CANopen Update Thread Started")
        self.status_signal.emit("CANopen Update Thread Started !")
        
        while True:
            ret = CanOpenBusProcessor.device.readBuffer(1, wait_time=0)
            
            if ret != None:
                [num, msg] = ret
                for i in range(num):
                    # TPDO1
                    if msg[i].ID > 0x180 and msg[i].ID < 0x200:
                        node_id = msg[i].ID - 0x180
                        
                        # 电机的ID
                        if node_id in Motor.motor_dict.keys():
                            status = self.__hex_list_to_int([msg[i].Data[0]]) # 状态字
                            
                            for key in Motor.STATUS_WORD: # 遍历字典关键字
                                for r in Motor.STATUS_WORD[key]: # 在每一个关键字对应的列表中 核对数值
                                    if status == r:
                                        Motor.motor_dict[node_id].servo_status = key # 更新电机的伺服状态
                                        break
                            
                            self.show_motor_status.emit(node_id)
                        
                        # IO模块的ID
                        elif node_id in IoModule.io_dict.keys():
                            data_low = bin(msg[i].Data[0])[2:] # 首先转换为bin 去除0b
                            data_low = '0' * (8 - len(data_low)) + data_low # 头部补齐

                            data_high = bin(msg[i].Data[1])[2:]
                            data_high = '0' * (8 - len(data_high)) + data_high

                            data = data_high + data_low # 拼接

                            for i, c in enumerate(data):
                                setattr(IoModule.io_dict[node_id], f"input_{16-i}", False if c == "0" else True)

                            self.show_switch.emit()
                    
                    # TPDO2
                    elif msg[i].ID > 0x280 and msg[i].ID < 0x300:
                        node_id = msg[i].ID - 0x280
                        
                        if node_id in Motor.motor_dict.keys():
                            position = self.__hex_list_to_int([msg[i].Data[j] for j in range(0,4)]) # 当前位置
                            speed = self.__hex_list_to_int([msg[i].Data[j] for j in range(4,8)]) # 当前速度

                            Motor.motor_dict[node_id].current_position = position
                            Motor.motor_dict[node_id].current_speed = speed

                            self.show_motor_original.emit(node_id)

                            if node_id == 10:
                                if self.gripper_calibration:
                                    self.gripper_position = (self.motor_10.zero_position - self.motor_10.current_position) / 5120
                                    self.gripper_velocity = self.motor_10.current_speed * 440 / 5120

                                self.show_gripper.emit(self.gripper_calibration)
                            
                            else:
                                if self.rope_is_set_zero:
                                    position = (getattr(self, f"motor_{node_id}").current_position - getattr(self, f"motor_{node_id}").zero_position) / 12536.512440
                                    setattr(self, f"rope_{node_id}_position", position)

                                    velocity = getattr(self, f"motor_{node_id}").current_speed * 440 / 12536.512440
                                    setattr(self, f"rope_{node_id}_velocity", velocity)
                                
                                self.rope_velocity[node_id-1] = speed * self.VELOCITY_RATIO / self.ROPE_RATIO
                                
                                self.show_rope.emit(self.rope_is_set_zero, node_id)

                    # TPDO4
                    elif msg[i].ID > 0x480 and msg[i].ID < 0x500:
                        node_id = msg[i].ID - 0x480
                        
                        if node_id in Motor.motor_dict.keys():
                            control_mode = self.__hex_list_to_int([msg[i].Data[0]])

                            for key in Motor.CONTROL_MODE:
                                if control_mode == Motor.CONTROL_MODE[key]:
                                    Motor.motor_dict[node_id].control_mode = key
                                    break

                            self.show_motor_mode.emit(node_id)
                    
                    # SDO
                    elif msg[i].ID > 0x580 and msg[i].ID < 0x600:
                        node_id = msg[i].ID - 0x580
                        
                        command =  msg[i].Data[0]
                        if command == CanOpenBusProcessor.CMD_R["read_16"] or CanOpenBusProcessor.CMD_R["read_32"] or CanOpenBusProcessor.CMD_R["read_8"]: status = True
                        elif command == CanOpenBusProcessor.CMD_R["write"]: status = True
                        elif command == CanOpenBusProcessor.CMD_R["error"]: status = False
                        else: status = None
                        
                        index = self.__match_index(msg[i].Data[1], msg[i].Data[2], msg[i].Data[3])
                        for key in CanOpenBusProcessor.OD: # 遍历字典关键字
                            if index == CanOpenBusProcessor.OD[key]: # 在每一个关键字对应的列表中 核对数值
                                label = key
                                break
                        else: label = ""
                        
                        value_list = [msg[i].Data[j] for j in range(4,8)]
                        
                        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value_list)

                        print("\033[0;34m[SDO {}] {}\033[0m".format(node_id, CanOpenBusProcessor.node_dict[node_id].sdo_feedback))
                        self.status_signal.emit("[Node {}] Get SDO response, object is {}, status is {}, value is {}".format(node_id, label, status, hex(self.__hex_list_to_int(value_list))))
                    
                    # NMT
                    elif msg[i].ID > 0x700 and msg[i].ID < 0x780:
                        node_id = msg[i].ID - 0x700

                        for key in CanOpenBusProcessor.NMT_STATUS:
                            if msg[0].Data[0] & 0b01111111 == CanOpenBusProcessor.NMT_STATUS[key]:
                                label = key
                                break
                        else: label = ""
                        
                        CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
                        
                        print("\033[0;34m[NMT {}] {}\033[0m".format(node_id, CanOpenBusProcessor.node_dict[node_id].nmt_feedback))
                        self.status_signal.emit("[Node {}] Get NMT response, bus status is {}".format(node_id, label))

            ''' 运动学参数 '''
            if self.continuum_calibration:
                for i in range(1,10):
                    self.rope_total_length[i-1] = self.rope_init_length[i-1] + (getattr(self, f"motor_{i}").current_position - self.rope_zero_position[i-1]) / self.ROPE_RATIO
                    # self.robot.rope_velocity[i-1] = getattr(self.robot, f"motor_{i}").current_speed * self.robot.VELOCITY_RATIO / self.robot.ROPE_RATIO
                
                # inside
                for i in range(6,9):
                    self.rope_inside_length[i] = self.rope_total_length[i]
                
                s_in, kappa_in, phi_in = self.transferSpaceActuator2Config(self.rope_inside_length[6], self.rope_inside_length[7], self.rope_inside_length[8])
                
                self.backbone_length[0] = s_in
                self.backbone_curvature[0] = kappa_in
                self.backbone_rotation_angle[0] = phi_in

                l_1, l_2, l_3, l_4, l_5, l_6, l_7, l_8, l_9 = self.transferSpaceConfig2Actuator("inside", s_in, kappa_in, phi_in)
                for i in range(0,6):
                    exec("self.robot.rope_inside_length[{}] = l_{}".format(i, i+1))

                self.inside_coordinate = self.transferSpaceConfig2Task("inside")
                trans_in = self.transferSpaceConfig2Task("inside", is_matrix=True)
                
                # midside
                for i in range(3,6):
                    self.rope_midside_length[i] = self.rope_total_length[i] - self.rope_inside_length[i]

                s_mid, kappa_mid, phi_mid = self.transferSpaceActuator2Config(self.rope_midside_length[3], self.rope_midside_length[4], self.rope_midside_length[5])
                self.backbone_length[1] = s_mid
                self.backbone_curvature[1] = kappa_mid
                self.backbone_rotation_angle[1] = phi_mid

                l_1, l_2, l_3, l_4, l_5, l_6 = self.transferSpaceConfig2Actuator("midside", s_mid, kappa_mid, phi_mid)
                for i in range(0,3):
                    exec("self.rope_midside_length[{}] = l_{}".format(i, i+1))

                self.midside_coordinate = self.transferSpaceConfig2Task("midside")
                trans_mid = self.transferSpaceConfig2Task("midside", is_matrix=True)

                # outside
                for i in range(0,3):
                    self.rope_outside_length[i] = self.rope_total_length[i] - self.rope_inside_length[i] - self.rope_midside_length[i]

                s_out, kappa_out, phi_out = self.transferSpaceActuator2Config(self.rope_outside_length[0], self.rope_outside_length[1], self.rope_outside_length[2])
                self.backbone_length[2] = s_out
                self.backbone_curvature[2] = kappa_out
                self.backbone_rotation_angle[2] = phi_out
                
                self.outside_coordinate = self.transferSpaceConfig2Task("outside")
                trans_out = self.transferSpaceConfig2Task("outside", is_matrix=True)

                # world
                trans_out_to_mid =  np.array([
                    [cos(-40*pi/180), -sin(-40*pi/180), 0, 0], 
                    [sin(-40*pi/180), cos(-40*pi/180),  0, 0],
                    [0,               0,                1, 0],
                    [0,               0,                0, 1],
                ])
                trans_mid_to_in =  np.array([
                    [cos(-40*pi/180), -sin(-40*pi/180), 0, 0], 
                    [sin(-40*pi/180), cos(-40*pi/180),  0, 0],
                    [0,               0,                1, 0],
                    [0,               0,                0, 1],
                ])
                trans_in_to_world =  np.array([
                    [-cos(80*pi/180), sin(80*pi/180), 0,  0], 
                    [sin(80*pi/180), cos(80*pi/180),  0,  0],
                    [0,               0,               -1, 0],
                    [0,               0,               0,  1],
                ])
                p_out_world = np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(np.matmul(trans_in_to_world, trans_in), trans_mid_to_in), trans_mid), trans_out_to_mid), trans_out), np.array([[0, 0, 0, 1]]).T)
                p_mid_world = np.matmul(np.matmul(np.matmul(np.matmul(trans_in_to_world, trans_in), trans_mid_to_in), trans_mid), np.array([[0, 0, 0, 1]]).T)
                p_in_world = np.matmul(np.matmul(trans_in_to_world, trans_in), np.array([[0, 0, 0, 1]]).T)
                self.outside_world_coordinate = (p_out_world[0,0], p_out_world[1,0], p_out_world[2,0])
                self.midside_world_coordinate = (p_mid_world[0,0], p_mid_world[1,0], p_mid_world[2,0])
                self.inside_world_coordinate = (p_in_world[0,0], p_in_world[1,0], p_in_world[2,0])

                self.show_kinematics.emit()
                
        print("CANopen Update Thread Stopped")
        self.status_signal.emit("CANopen Update Thread Stopped.")

    def updateForce(self):
        print("Update Force Thread Started")
        
        while True:
            start_time = time.time()

            for sensor in Sensor.sensor_dict.values(): send_success = sensor.send_request()

            ret = Sensor.device.readBuffer(10, wait_time=0)
            if ret != None:
                [num, msg] = ret
                
                for i in range(num):
                    if msg[i].Data[0] == Sensor.msg[0] \
                        and msg[i].Data[1] == Sensor.msg[1] \
                        and msg[i].Data[6] == Sensor.msg[2] \
                        and msg[i].Data[7] == Sensor.msg[3]:
                        if msg[i].ID in Sensor.sensor_dict.keys():
                            Sensor.sensor_dict[msg[i].ID].original_data = self.__hex_list_to_float([msg[i].Data[j] for j in range(2, 6)])

                            Sensor.sensor_dict[msg[i].ID].force = (self.__hex_list_to_float([msg[i].Data[j] for j in range(2, 6)]) - Sensor.sensor_dict[msg[i].ID].zero) / 2
                        
                            self.show_force.emit(msg[i].ID)
            
            time.sleep(max(1 / self.PARAMETER["sensor_sampling_frequency"] - (time.time() - start_time), 0))
        
        print("Update Force Thread Stopped")
    
    def initUsbCan(self) -> bool:
        if UsbCan.openDevice():
            if not self.usbcan_0_is_start and self.usbcan_0.initChannel() and self.usbcan_0.startChannel(): self.usbcan_0_is_start = True
            if not self.usbcan_1_is_start and self.usbcan_1.initChannel() and self.usbcan_1.startChannel(): self.usbcan_1_is_start = True
        return self.usbcan_0_is_start and self.usbcan_1_is_start
   
    def initRobot(self, times=1):
        def initMotor() -> bool:
            motor_init_count = 0
            for node_id in Motor.motor_dict:
                print("====================INIT MOTOR====================")
                if Motor.motor_dict[node_id].check_bus_status(log=False) \
                and Motor.motor_dict[node_id].start_pdo(log=True, check=False):
                        time.sleep(0.01)
                        if Motor.motor_dict[node_id].check_servo_status(log=True):
                            if Motor.motor_dict[node_id].set_control_mode("position_control") \
                            and Motor.motor_dict[node_id].set_tpdo_inhibit_time(100, channel="2") \
                            and Motor.motor_dict[node_id].set_profile_acceleration(10) \
                            and Motor.motor_dict[node_id].set_profile_deceleration(10):
                                motor_init_count += 1
                                continue
                print("===============INIT MOTOR SHUT DOWN===============")
                return False
            return motor_init_count == len(Motor.motor_dict)

        def initIO() -> bool:
            io_init_count = 0
            for node_id in IoModule.io_dict:
                print("====================INIT IO====================")
                if IoModule.io_dict[node_id].check_bus_status() \
                and IoModule.io_dict[node_id].initialize_device(log=True) \
                and IoModule.io_dict[node_id].start_device(log=True):
                    if IoModule.io_dict[node_id].close_valve_1() \
                    and IoModule.io_dict[node_id].close_valve_2() \
                    and IoModule.io_dict[node_id].close_valve_3() \
                    and IoModule.io_dict[node_id].open_valve_4():
                        io_init_count += 1
                        continue
                print("===============INIT IO SHUT DOWN===============")
                return False
            return io_init_count == len(IoModule.io_dict)
        
        self.robot_init_start.emit(True)
        while times != 0:
            if initMotor() and initIO():
                self.robot_init_end.emit()
                break
            else: times -= 1
        else: self.robot_init_start.emit(False)
    
    def shut_down_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.shut_down(is_pdo=True): time.sleep(0.001)
    def switch_on_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.switch_on(is_pdo=True): time.sleep(0.001)
    def enable_operation_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.enable_operation(is_pdo=True): time.sleep(0.001)
    def disable_operation_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.disable_operation(is_pdo=True): time.sleep(0.001)
    def disable_voltage_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.disable_voltage(is_pdo=True): time.sleep(0.001)
    def fault_reset_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.fault_reset(is_pdo=True): time.sleep(0.001)
    def quick_stop_all(self):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            while not motor.quick_stop(is_pdo=True): time.sleep(0.001)

    def joint_speed(self, motor: list, speed: int):
        self.joint_speed_thread = JointSpeed(motor, speed, robot=self)
        self.joint_speed_thread.start()
    def joint_speed_stop(self):
        self.joint_speed_thread.stop()
        self.joint_speed_thread.wait()


    ''' 力传感器 零位标定
    '''
    def calibrateForceSensor(self):
        # self.motor_1.set_control_mode("speed_control", check=False)
        # self.motor_2.set_control_mode("speed_control", check=False)
        # self.motor_3.set_control_mode("speed_control", check=False)
        # self.motor_4.set_control_mode("speed_control", check=False)
        # self.motor_5.set_control_mode("speed_control", check=False)
        # self.motor_6.set_control_mode("speed_control", check=False)
        # self.motor_7.set_control_mode("speed_control", check=False)
        # self.motor_8.set_control_mode("speed_control", check=False)
        # self.motor_9.set_control_mode("speed_control", check=False)

        # self.motor_1.set_speed(0, is_pdo=True)
        # self.motor_2.set_speed(0, is_pdo=True)
        # self.motor_3.set_speed(0, is_pdo=True)
        # self.motor_4.set_speed(0, is_pdo=True)
        # self.motor_5.set_speed(0, is_pdo=True)
        # self.motor_6.set_speed(0, is_pdo=True)
        # self.motor_7.set_speed(0, is_pdo=True)
        # self.motor_8.set_speed(0, is_pdo=True)
        # self.motor_9.set_speed(0, is_pdo=True)

        self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")

        self.sensor_calibration_start.emit()

        for i in range(9,0,-1):
            motor = getattr(self, f"motor_{i}")
            sensor = getattr(self, f"sensor_{i}")
            force_ref = -abs(self.PARAMETER["sensor_calibration_reference_force_{}".format(i)])

            # motor.enable_operation(is_pdo=True)

            while True:
                force_error = force_ref - sensor.force
                if abs(force_error) < 0.01:
                    motor.halt(is_pdo=True)
                    break
                elif abs(force_error) < 0.1: motor.set_speed(- 10 if force_error < 0 else 10, is_pdo=True, log=False)
                elif abs(force_error) < 2: motor.set_speed(int(force_error * 20), is_pdo=True, log=False)
                elif abs(force_error) < 4: motor.set_speed(int(force_error * 25), is_pdo=True, log=False)
                else: motor.set_speed(- 100 if force_error < 0 else 100, is_pdo=True, log=False)
            
            motor.set_speed(10, is_pdo=True)
            motor.enable_operation(is_pdo=True)

            while True:
                last_force = sensor.original_data
                time.sleep(1)
                current_force = sensor.original_data
                if abs(last_force - current_force) < 0.1:
                    motor.halt(is_pdo=True)
                    motor.set_speed(0, is_pdo=True)
                    break
            
            sensor.set_zero(self.PARAMETER["sensor_calibration_init_count_{}".format(i)])

            motor.enable_operation(is_pdo=True)
            while True:
                force_error = -5 - sensor.force

                if abs(force_error) < 0.01:
                    motor.disable_operation(is_pdo=True)
                    motor.set_speed(0, is_pdo=True)
                    break
                elif abs(force_error) < 0.1: motor.set_speed(- 10 if force_error < 0 else 10, is_pdo=True, log=False)
                elif abs(force_error) < 2: motor.set_speed(int(force_error * 20), is_pdo=True, log=False)
                elif abs(force_error) < 4: motor.set_speed(int(force_error * 25), is_pdo=True, log=False)
                else: motor.set_speed(- 100 if force_error < 0 else 100, is_pdo=True, log=False)
        
        self.sensor_calibration_end.emit()

    def rope_force_adapt(self, i_f: int, m_f: int, o_f: int, i_pid: tuple, m_pid: tuple, o_pid: tuple, start, finish) -> None:
        self.rope_force_adapt_thread = ContinuumAttitudeAdjust(self, 
                                                               i_f=i_f, m_f=m_f, o_f=o_f, 
                                                               i_pid=i_pid, m_pid=m_pid, o_pid=o_pid, 
                                                               start_signal=start, finish_signal=finish)
        self.rope_force_adapt_thread.start()
    
    def rope_set_zero(self) -> None:
        self.rope_force_adapt_thread.stop()
        self.rope_force_adapt_thread.wait()

    ''' 夹爪 归零
    手爪运动至极限位置
    speed: 运动速度 > 0
    start: 开始信号
    finish: 结束信号
    '''
    def homingGripper(self):
        self.gripper_homing_start.emit()
        
        self.setGripper("open")
        if not self.io.input_1:
            self.motor_10.set_control_mode("speed_control", check=False)
            self.motor_10.set_speed(abs(int(self.PARAMETER["gripper_homing_velocity"])), is_pdo=True)
            self.motor_10.halt(is_pdo=True)
            self.motor_10.enable_operation(is_pdo=True)
        else:
            self.gripper_homing_end.emit()
            return

        while True:
            if self.io.input_1:
                self.motor_10.set_speed(0, is_pdo=True)
                self.motor_10.disable_operation(is_pdo=True)
                self.gripper_homing_end.emit()
                break

    ''' 夹爪 调零
    确定手爪处于极限位置时电机10的位置 该位置作为电机10的参考零点
    思路: 先让手爪前进一段距离(位置模式) 再缓慢后退寻找接近开关(速度模式)
    '''
    def calibrateGripper(self):
        self.gripper_calibration = False

        self.gripper_calibration_start.emit()
        
        self.setGripper("open")

        self.motor_10.set_control_mode("position_control", check=False)
        time.sleep(0.01)

        d = abs(float(self.PARAMETER["gripper_calibration_forward_distance"]))
        v = abs(int(self.PARAMETER["gripper_calibration_forward_velocity"]))
        duration = (d * 5120) / (v * 440) + 0.5

        self.motor_10.set_position(- int(d * 5120), velocity=v, is_pdo=True)

        self.motor_10.ready(is_pdo=True)
        self.motor_10.action(is_immediate=True, is_relative=True, is_pdo=True)

        time.sleep(duration)
        
        if not self.io.input_1:
            self.motor_10.set_control_mode("speed_control")
            time.sleep(0.01)
        
            self.motor_10.set_speed(abs(int(self.PARAMETER["gripper_calibration_backward_velocity"])), is_pdo=True)

            self.motor_10.halt(is_pdo=True)

            self.motor_10.enable_operation(is_pdo=True)

        while True:
            if self.io.input_1:
                self.motor_10.set_speed(0, is_pdo=True)
                self.motor_10.disable_operation(is_pdo=True)

                time.sleep(0.01)

                self.motor_10.zero_position = self.motor_10.current_position

                self.gripper_calibration = True
                self.gripper_position = 0

                self.gripper_calibration_end.emit()

                break
            time.sleep(0.001)

    def initGripperActuator(self, control_mode: str):
        if control_mode == "speed":
            self.motor_10.set_control_mode("speed_control")
            self.motor_10.set_speed(0, is_pdo=True)
            self.motor_10.enable_operation(is_pdo=True)
        elif control_mode == "position":
            self.motor_10.set_control_mode("position_control")
    ''' 手爪移动到绝对位置点 以后极限点为原点 向前为正方向
    point: 位置点坐标 >=0 <=358 mm
    velocity: 运动速度 >0 mm/s
    is_wait: 添加延时 等待运动结束后再进行下一步操作
    '''
    def moveGripperAbsolute(self, point: float, velocity: float, /, *, is_close=False, is_wait=True):
        if self.gripper_calibration:
            if is_close: self.io.close_valve_4()
            else: self.io.open_valve_4()
            
            target_position = int(round(self.motor_10.zero_position - abs(point) * self.BALLSCREW_RATIO, 0))
            profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

            self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
            self.motor_10.ready(is_pdo=True)
            self.motor_10.action(is_immediate=False, is_relative=False, is_pdo=True)

            if is_wait:
                duration_time = abs(target_position - self.motor_10.current_position) / (profile_velocity * self.VELOCITY_RATIO) + 1
                time.sleep(duration_time)
    ''' 手爪在当前位置下移动相对距离
    distance: 移动距离 mm 有正负区别
    velocity: 运动速度 >0 mm/s
    is_wait: 添加延时 等待运动结束后再进行下一步操作
    '''
    def moveGripperRelative(self, distance: float, velocity: float, /, *, is_close=False, is_wait=True):
        if self.gripper_calibration:
            if is_close: self.io.close_valve_4()
            else: self.io.open_valve_4()
            
            target_position = int(round(distance * self.BALLSCREW_RATIO, 0))
            profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

            self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
            self.motor_10.ready(is_pdo=True)
            self.motor_10.action(is_immediate=False, is_relative=True, is_pdo=True)

            if is_wait:
                duration_time = abs(target_position) / (profile_velocity * self.VELOCITY_RATIO) + 1
                time.sleep(duration_time)
    def moveGripperSpeed(self, speed: float):
        if self.gripper_calibration:
            target_speed = int(round(self.BALLSCREW_RATIO * (- speed) / self.VELOCITY_RATIO, 0))
            self.motor_10.set_speed(target_speed, is_pdo=True)
    def stopGripper(self):
        if self.gripper_calibration:
            self.motor_10.disable_operation(is_pdo=True)
            self.motor_10.set_speed(0, is_pdo=True)
    def setGripper(self, status: str):
        if status == "open":
            if self.io.output_4:
                success = self.io.set_output(False, "4")
                time.sleep(0.2)
            else: success = True
        elif status == "close":
            if not self.io.output_4:
                success = self.io.set_output(True, "4")
                time.sleep(0.4)
            else: success = True
        else: success = False
        return success
    def setAnchor(self, number: str, status: str):
        if status == "open":
            if not getattr(self.io, "output_{}".format(number)):
                success = self.io.set_output(True, number)
                time.sleep(0.1)
            else: success = True
        elif status == "close":
            if getattr(self.io, "output_{}".format(number)):
                success = self.io.set_output(False, number)
                time.sleep(0.1)
            else: success = True
        else: success = False
        return success


    # def rope_move_abs(self, rope: str, /, *, point: float, velocity: float) -> None:
    #     duration_time = []

    #     for node_id in rope:
    #         target_position = int(round(getattr(self, f"motor_{node_id}").zero_position + abs(point) * self.ROPE_RATIO, 0))
    #         profile_velocity = int(round(self.ROPE_RATIO * abs(velocity) / self.VELOCITY_RATIO, 0))
    #         getattr(self, f"motor_{node_id}").set_position(target_position, velocity=profile_velocity, is_pdo=True)
    #         getattr(self, f"motor_{node_id}").ready(is_pdo=True)
            
    #         t = abs(target_position - getattr(self, f"motor_{node_id}").current_position) / (profile_velocity * self.VELOCITY_RATIO) + 0.1
    #         duration_time.append(t)

    #     duration_time.sort(reverse=True)
    #     delay = duration_time[0]
        
    #     for node_id in rope:
    #         getattr(self, f"motor_{node_id}").action(is_immediate=False, is_relative=False, is_pdo=True)
        
    #     time.sleep(delay)
    # def rope_move_rel(self, rope: str, /, *, distance: float, velocity: float, is_wait=True) -> None:
    #     target_position = int(round(distance * self.ROPE_RATIO, 0))
    #     profile_velocity = int(round(self.ROPE_RATIO * abs(velocity) / self.VELOCITY_RATIO, 0))
        
    #     for node_id in rope:
    #         getattr(self, f"motor_{node_id}").set_position(target_position, velocity=profile_velocity, is_pdo=True)
    #         getattr(self, f"motor_{node_id}").ready(is_pdo=True)

    #     for node_id in rope:
    #         getattr(self, f"motor_{node_id}").action(is_immediate=False, is_relative=True, is_pdo=True)

    #     if is_wait:
    #         duration_time = abs(target_position) / (profile_velocity * self.VELOCITY_RATIO) + 0.1
    #         time.sleep(duration_time)
    # def rope_ready_position(self, *args: str):
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").set_control_mode("position_control")
    def rope_move_abs_new(self, *args: tuple):
        id_list = []
        t_list = []
        for tuple in args:
            id, tar_l, vel = tuple
            id_list.append(id)
            
            tar_pos = int(round(self.rope_zero_position[int(id)-1] + (tar_l - self.rope_init_length[int(id)-1]) * self.ROPE_RATIO, 0))
            pro_vel = int(round(self.ROPE_RATIO * abs(vel) / self.VELOCITY_RATIO, 0))
            if pro_vel == 0: pro_vel = 1
            
            t = abs(tar_pos - getattr(self, f"motor_{id}").current_position) / (pro_vel * self.VELOCITY_RATIO) + 0.05
            t_list.append(t)

            getattr(self, f"motor_{id}").set_position(tar_pos, velocity=pro_vel, is_pdo=True)
            getattr(self, f"motor_{id}").ready(is_pdo=True)
        
        t_list.sort(reverse=True)
        delay = t_list[0]
        for id in id_list:
            getattr(self, f"motor_{id}").action(is_immediate=False, is_relative=False, is_pdo=True)
        time.sleep(delay)
    def rope_move_rel_new(self, *args: tuple):
        id_list = []
        t_list = []
        for tuple in args:
            id, pos, vel = tuple
            id_list.append(id)
            
            tar_pos = int(round(pos * self.ROPE_RATIO, 0))
            pro_vel = int(round(self.ROPE_RATIO * abs(vel) / self.VELOCITY_RATIO, 0))
            if pro_vel == 0: pro_vel = 1
            
            t = abs(tar_pos) / (pro_vel * self.VELOCITY_RATIO)
            t_list.append(t)

            getattr(self, f"motor_{id}").set_position(tar_pos, velocity=pro_vel, is_pdo=True)
            getattr(self, f"motor_{id}").ready(is_pdo=True)
        
        t_list.sort(reverse=True)
        delay = t_list[0]
        for id in id_list:
            getattr(self, f"motor_{id}").action(is_immediate=False, is_relative=True, is_pdo=True)
        time.sleep(delay)
    # ''' 对控制肌腱伸缩的电机进行速度模式的准备工作
    # 设置速度模式 -> 设置目标速度: 0 -> 设置控制状态: enable operation
    # 输入: 字符串 电机ID "1"~"9"
    # '''
    # def rope_ready_speed(self, *args: str):
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").set_control_mode("speed_control")
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").enable_operation(is_pdo=True)
    # def rope_stop_speed(self, *args: str):
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").disable_operation(is_pdo=True)
    #     for node_id in args:
    #         getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True) 
    # def rope_move_speed(self, *args: tuple):
    #     for tuple in args:
    #         node_id, speed = tuple
    #         target_speed = int(round(self.ROPE_RATIO * speed / self.VELOCITY_RATIO, 0))
    #         getattr(self, f"motor_{node_id}").set_speed(target_speed, is_pdo=True)
    # def rope_move(self, rope: str, distance: float, velocity: float, /, *, is_relative: bool) -> None:
    #     self.rope_move_thread = RopeMove(rope, distance, velocity, is_relative=is_relative, robot=self)
    #     self.rope_move_thread.start()
    
    def initTendonActuator(self, control_mode: str, *args: str):
        if control_mode == "speed":
            for node_id in args:
                getattr(self, f"motor_{node_id}").set_control_mode("speed_control")
            for node_id in args:
                getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
            for node_id in args:
                getattr(self, f"motor_{node_id}").enable_operation(is_pdo=True)
        elif control_mode == "position":
            for node_id in args:
                getattr(self, f"motor_{node_id}").set_control_mode("position_control")
    ''' 对控制肌腱伸缩的电机进行速度模式的运动
    输入: 元组 (电机ID, 速度)
    电机ID: "1" ~ "9"
    速度: mm/s 有正负区别
    '''
    def moveTendonSpeed(self, *args: tuple):
        for tuple in args:
            node_id, speed = tuple
            target_speed = int(round(self.ROPE_RATIO * speed / self.VELOCITY_RATIO, 0))
            getattr(self, f"motor_{node_id}").set_speed(target_speed, is_pdo=True)
    ''' 对控制肌腱伸缩的电机进行速度模式的后处理
    设置控制状态: disable operation -> 设置目标速度: 0
    输入: 字符串 电机ID "1"~"9"
    '''
    def stopTendon(self, *args: str):
        for node_id in args:
            getattr(self, f"motor_{node_id}").disable_operation(is_pdo=True)
        for node_id in args:
            getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)


    ''' Inside '''
    def moveInsideSection(self, s_d: float, kappa_d: float, phi_d: float, ref=(-2.0, -3.0, -5.0), kp=(0.2, 0.3, 0.5), ki=(0, 0, 0), kd=(0, 0, 0)):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=s_d, kappa_d=kappa_d, phi_d=phi_d, ref=ref, kp=kp, ki=ki, kd=kd)
        self.inside_section_move_thread.start()
    
    def ExtendInsideSection(self, s_d, ref=(-2,-3,-5), kp=(0.2, 0.3, 0.5), ki=(0, 0, 0), kd=(0, 0, 0)):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=s_d, kappa_d=0, phi_d=0, ref=ref, kp=kp, ki=ki, kd=kd)
        self.inside_section_move_thread.start()
    def ShortenInsideSection(self, s_d, ref=(-2,-3,-5), kp=(0.2, 0.3, 0.5), ki=(0, 0, 0), kd=(0, 0, 0)):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=-abs(s_d), kappa_d=0, phi_d=0, ref=ref, kp=kp, ki=ki, kd=kd)
        self.inside_section_move_thread.start()
    def CurveInsideSection(self):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0.001, phi_d=0)
        self.inside_section_move_thread.start()
    def StraightenInsideSection(self):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=-0.001, phi_d=0)
        self.inside_section_move_thread.start()
    def RotateInsideSectionClockwise(self):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=0.25)
        self.inside_section_move_thread.start()
    def RotateInsideSectionAntiClockwise(self):
        self.inside_section_move_thread = InsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=-0.25)
        self.inside_section_move_thread.start()
    def StopInsideSection(self):
        self.inside_section_move_thread.stop()
        self.inside_section_move_thread.wait()

    ''' Midside '''
    def ExtendMidsideSection(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=10, kappa_d=0, phi_d=0)
        self.midside_section_move_thread.start()
    def ShortenMidsideSection(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=-10, kappa_d=0, phi_d=0)
        self.midside_section_move_thread.start()
    def CurveMidsideSection(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0.001, phi_d=0)
        self.midside_section_move_thread.start()
    def StraightenMidsideSection(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=-0.001, phi_d=0)
        self.midside_section_move_thread.start()
    def RotateMidsideSectionClockwise(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=0.25)
        self.midside_section_move_thread.start()
    def RotateMidsideSectionAntiClockwise(self):
        self.midside_section_move_thread = MidsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=-0.25)
        self.midside_section_move_thread.start()
    def StopMidsideSection(self):
        self.midside_section_move_thread.stop()
        self.midside_section_move_thread.wait()

    ''' Outside '''
    def ExtendOutsideSection(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=5, kappa_d=0, phi_d=0)
        self.outside_section_move_thread.start()
    def ShortenOutsideSection(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=-5, kappa_d=0, phi_d=0)
        self.outside_section_move_thread.start()
    def CurveOutsideSection(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0.002, phi_d=0)
        self.outside_section_move_thread.start()
    def StraightenOutsideSection(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=-0.002, phi_d=0)
        self.outside_section_move_thread.start()
    def RotateOutsideSectionClockwise(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=0.5)
        self.outside_section_move_thread.start()
    def RotateOutsideSectionAntiClockwise(self):
        self.outside_section_move_thread = OutsideSectionConfigurationSpaceJacobianControl(self, s_d=0, kappa_d=0, phi_d=-0.5)
        self.outside_section_move_thread.start()
    def StopOutsideSection(self):
        self.outside_section_move_thread.stop()
        self.outside_section_move_thread.wait()


    ''' 标定 初始状态 曲率0 长度已知 '''
    def calibrateContinuum(self, bl_o: float, bl_m: float, bl_i: float):
        self.rope_zero_position = [getattr(self, f"motor_{i}").current_position for i in range(1,10)]

        self.rope_init_length = [bl_o+bl_m+bl_i, bl_o+bl_m+bl_i, bl_o+bl_m+bl_i, 
                                 bl_m+bl_i,      bl_m+bl_i,      bl_m+bl_i, 
                                 bl_i,           bl_i,           bl_i]
        
        self.rope_inside_length = [bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i] # 123456789
        self.rope_midside_length = [bl_m, bl_m, bl_m, bl_m, bl_m, bl_m] # 123456
        self.rope_outside_length = [bl_o, bl_o, bl_o] # 123

        self.backbone_init_length = [bl_i, bl_m, bl_o] # in mid out
        self.backbone_length = [bl_i, bl_m, bl_o] # in mid out
        self.backbone_curvature = [0, 0, 0] # in mid out
        self.backbone_rotation_angle = [0, 0, 0] # in mid out

        self.continuum_calibration = True

    ''' 正运动学 '''
    def transferSpaceActuator2Config(self, l_1: float, l_2: float, l_3: float):
        param_1 = sqrt(pow(l_1,2)+pow(l_2,2)+pow(l_3,2)-l_1*l_2-l_2*l_3-l_1*l_3)
        param_2 = l_1+l_2+l_3
        param_3 = l_3+l_2-2*l_1
        param_4 = l_2-l_3

        kappa = (2*param_1)/(self.backbone_d*param_2)

        if param_4 == 0:
            if param_3 > 0: phi = pi/2
            elif param_3 == 0: phi = 0
            elif param_3 < 0: phi = 3*pi/2
        elif param_4 > 0:
            if param_3 >= 0: phi = atan(1/sqrt(3)*param_3/param_4)
            else: phi = atan(1/sqrt(3)*param_3/param_4) + 2*pi
        elif param_4 < 0: phi = atan(1/sqrt(3)*param_3/param_4) + pi

        if param_1 == 0: s = param_2 / 3
        else: s = self.backbone_section_num[2]*self.backbone_d*param_2/param_1*asin(param_1/(3*self.backbone_section_num[2]*self.backbone_d))

        return s, kappa, phi
    
    ''' 正运动学 '''
    def transferSpaceConfig2Task(self, section: str, /, *, is_matrix=False):
        if section == "outside":
            length = -self.backbone_length[2] # z轴向内 论文是向外
            kappa = self.backbone_curvature[2]
            phi = self.backbone_rotation_angle[2]
        elif section == "midside":
            length = -self.backbone_length[1] # z轴向内 论文是向外
            kappa = self.backbone_curvature[1]
            phi = self.backbone_rotation_angle[1]
        elif section == "inside":
            length = -self.backbone_length[0] # z轴向内 论文是向外
            kappa = self.backbone_curvature[0]
            phi = self.backbone_rotation_angle[0]
        else: return 0, 0, 0
        if kappa != 0:
            transform = np.array([
                [cos(phi)*cos(phi*length), -sin(phi), cos(phi)*sin(kappa*length), cos(phi)*(1-cos(kappa*length))/kappa], 
                [sin(phi)*cos(phi*length), cos(phi),  sin(phi)*sin(kappa*length), sin(phi)*(1-cos(kappa*length))/kappa],
                [-sin(kappa*length),       0,         cos(kappa*length),          sin(kappa*length)/kappa],
                [0,                        0,         0,                          1]
            ])
        else:
            transform = np.array([
                [cos(phi)*cos(phi*length), 0, 0, 0     ], 
                [sin(phi)*cos(phi*length), 1, 0, 0     ],
                [0,                        0, 1, length],
                [0,                        0, 0, 1     ],
            ])
        
        if not is_matrix:
            coordinate = np.array([[0, 0, 0, 1]]).T
            position = np.matmul(transform, coordinate)
            x = position[0,0]
            y = position[1,0]
            z = position[2,0]
            return x, y, z
        else: return transform

    ''' 逆运动学 '''
    def transferSpaceTask2Config(self, position: tuple):
        x, y, z = position

        if z > 172 or z <= 0: return

        if x > 0 and y == 0: phi = 0
        elif x > 0 and y > 0: phi = atan(y/x)
        elif x == 0 and y > 0: phi = pi/2
        elif x < 0 and y > 0: phi = atan(y/x)+pi
        elif x < 0 and y == 0: phi = pi
        elif x < 0 and y < 0: phi = atan(y/x)+pi
        elif x == 0 and y < 0: phi = 3*pi/2
        elif x > 0 and y < 0: phi = atan(y/x)+2*pi

        kappa = 2*sqrt(pow(x,2)+pow(y,2))/(pow(x,2)+pow(y,2)+pow(z,2))

        theta = acos(1-kappa*sqrt(pow(x,2)+pow(y,2))) if z > 0 else 2*pi-acos(1-kappa*sqrt(pow(x,2)+pow(y,2)))
        s = 1/kappa*theta

        return s, kappa, phi

    ''' 逆运动学 '''
    def transferSpaceConfig2Actuator(self, section: str, s: float, kappa: float, phi: float):
        def transform(n, s, kappa, phi):
            param_1 = 2*n*sin((kappa*s)/(2*n))
            if kappa != 0:
                l_1 = param_1*(1/kappa-self.backbone_d*sin(phi))
                l_2 = param_1*(1/kappa+self.backbone_d*sin(phi+pi/3))
                l_3 = param_1*(1/kappa-self.backbone_d*cos(phi+pi/6))
            else:
                l_1, l_2, l_3 = s, s, s
            return l_1, l_2, l_3

        if section == "outside":
            n = self.backbone_section_num[2]
            l_1, l_2, l_3 = transform(n, s, kappa, phi)
            return l_1, l_2, l_3
        elif section == "midside":
            n = self.backbone_section_num[1]
            l_4, l_5, l_6 = transform(n, s, kappa, phi)
            l_1, l_2, l_3 = transform(n, s, kappa, phi+40/180*pi)
            return l_1, l_2, l_3, l_4, l_5, l_6
        elif section == "inside":
            n = self.backbone_section_num[0]
            l_7, l_8, l_9 = transform(n, s, kappa, phi)
            l_4, l_5, l_6 = transform(n, s, kappa, phi+40/180*pi)
            l_1, l_2, l_3 = transform(n, s, kappa, phi+80/180*pi)
            return l_1, l_2, l_3, l_4, l_5, l_6, l_7, l_8, l_9

    ''' 正雅可比 '''
    def transferSpaceActuator2ConfigJacobian(self, section: str, s_d_1: float, s_d_2: float, s_d_3: float):
        s_d = np.array([[s_d_1, s_d_2, s_d_3]]).T

        def jacobian(n, s, kappa, phi):
            if kappa != 0:
                param_1 = cos((kappa * s) / (2 * n))
                param_2 = sin((kappa * s) / (2 * n))
                param_3 = 2 * n / pow(kappa, 2)
                param_4 = 2 * n * self.backbone_d
                param_5 = 1 / kappa - self.backbone_d * sin(phi)
                param_6 = 1 / kappa + self.backbone_d * sin(pi/3 + phi)
                param_7 = 1 / kappa - self.backbone_d * cos(pi/6 + phi)
                
                transform = np.array([
                    [s*param_1*param_5 - param_3*param_2, -param_4*param_2*cos(phi),     kappa*param_1*param_5],
                    [s*param_1*param_6 - param_3*param_2, param_4*param_2*cos(pi/3+phi), kappa*param_1*param_6],
                    [s*param_1*param_7 - param_3*param_2, param_4*param_2*sin(pi/6+phi), kappa*param_1*param_7],
                ])
            else:
                transform = np.array([
                    [-s*self.backbone_d*sin(phi),      0, 1],
                    [s*self.backbone_d*sin(pi/3+phi),  0, 1],
                    [-s*self.backbone_d*cos(pi/6+phi), 0, 1],
                ])
            return np.linalg.inv(transform)

        if section == "inside": i = 0
        elif section == "midside": i = 1
        elif section == "outside": i = 2
        n, s, kappa, phi = self.backbone_section_num[i], self.backbone_length[i], self.backbone_curvature[i], self.backbone_rotation_angle[i]

        config_d = np.matmul(jacobian(n, s, kappa, phi), s_d)

        kappa_d, phi_d, l_d = config_d[0, 0], config_d[1, 0], config_d[2, 0]

        return kappa_d, phi_d, l_d

    ''' 逆雅可比 '''
    def transferSpaceConfig2ActuatorJacobian(self, section: str, s_d: float, kappa_d: float, phi_d: float):
        config_d = np.array([[kappa_d, phi_d, s_d]]).T

        def jacobian(n, s, kappa, phi):
            if kappa != 0:
                param_1 = cos((kappa * s) / (2 * n))
                param_2 = sin((kappa * s) / (2 * n))
                param_3 = 2 * n / pow(kappa, 2)
                param_4 = 2 * n * self.backbone_d
                param_5 = 1 / kappa - self.backbone_d * sin(phi)
                param_6 = 1 / kappa + self.backbone_d * sin(pi/3 + phi)
                param_7 = 1 / kappa - self.backbone_d * cos(pi/6 + phi)
                
                transform = np.array([
                    [s*param_1*param_5 - param_3*param_2, -param_4*param_2*cos(phi),     kappa*param_1*param_5],
                    [s*param_1*param_6 - param_3*param_2, param_4*param_2*cos(pi/3+phi), kappa*param_1*param_6],
                    [s*param_1*param_7 - param_3*param_2, param_4*param_2*sin(pi/6+phi), kappa*param_1*param_7],
                ])
            else:
                transform = np.array([
                    [-s*self.backbone_d*sin(phi),      0, 1],
                    [s*self.backbone_d*sin(pi/3+phi),  0, 1],
                    [-s*self.backbone_d*cos(pi/6+phi), 0, 1],
                ])
            return transform
        
        if section == "outside":
            n, s, kappa, phi = self.backbone_section_num[2], self.backbone_length[2], self.backbone_curvature[2], self.backbone_rotation_angle[2]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_1_d, s_2_d, s_3_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]
            return s_1_d, s_2_d, s_3_d
        
        elif section == "midside":
            n, s, kappa, phi = self.backbone_section_num[1], self.backbone_length[1], self.backbone_curvature[1], self.backbone_rotation_angle[1]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_4_d, s_5_d, s_6_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]
            
            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_1_d, s_2_d, s_3_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]

            return s_1_d, s_2_d, s_3_d, s_4_d, s_5_d, s_6_d
        
        elif section == "inside":
            n, s, kappa, phi = self.backbone_section_num[0], self.backbone_length[0], self.backbone_curvature[0], self.backbone_rotation_angle[0]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_7_d, s_8_d, s_9_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]

            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_4_d, s_5_d, s_6_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]

            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            s_1_d, s_2_d, s_3_d = actuator_d[0, 0], actuator_d[1, 0], actuator_d[2, 0]

            return s_1_d, s_2_d, s_3_d, s_4_d, s_5_d, s_6_d, s_7_d, s_8_d, s_9_d
        
        else: return 0, 0, 0, 0, 0, 0, 0, 0, 0

    ''' 逆雅可比 '''
    def transferSpaceTask2ConfigJacobian(self, section: str, spatial_velocity: tuple, /, *, is_matrix=False):
        if section == "outside":
            s = self.backbone_length[2]
            kappa = self.backbone_curvature[2]
            phi = self.backbone_rotation_angle[2]
        elif section == "midside":
            s = self.backbone_length[1]
            kappa = self.backbone_curvature[1]
            phi = self.backbone_rotation_angle[1]
        elif section == "inside":
            s = self.backbone_length[0]
            kappa = self.backbone_curvature[0]
            phi = self.backbone_rotation_angle[0]
        else: return 0, 0, 0

        if kappa != 0:
            transform = np.array([
                [cos(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
                [sin(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
                [-(sin(kappa*s)-kappa*s)/pow(kappa,2),   0, 1              ],
                [-s*sin(phi),                            0, -kappa*sin(phi)],
                [s*cos(phi),                             0, kappa*cos(phi) ],
                [0,                                      1, 0              ],
            ])
        else:
            transform = np.array([
                [-pow(s,2)*cos(phi)/2, 0, 0],
                [-pow(s,2)*sin(phi)/2, 0, 0],
                [0,                    0, 1],
                [-s*sin(phi),          0, 0],
                [s*cos(phi),           0, 0],
                [0,                    1, 0],
            ])

        if not is_matrix:
            x_d = spatial_velocity[0]
            y_d = spatial_velocity[1]
            z_d = spatial_velocity[2]
            x_w = spatial_velocity[3]
            y_w = spatial_velocity[4]
            z_w = spatial_velocity[5]

            spatial_velocity_vector = np.array([[x_d, y_d, z_d, x_w, y_w, z_w]]).T

            config_vector = np.matmul(np.linalg.pinv(transform), spatial_velocity_vector)

            kappa_d = config_vector[0, 0]
            phi_d = config_vector[1, 0]
            s_d = config_vector[2, 0]

            return kappa_d, phi_d, s_d
        else: return transform


    
    ''' 打开相机 '''
    def OpenCamera(self, update_slot, clear_slot):
        self.video_stream_thread = Camera(self, update_slot=update_slot, clear_slot=clear_slot)
        self.video_stream_thread.start()
    ''' 关闭相机 '''
    def CloseCamera(self):
        self.video_stream_thread.stop()
        self.video_stream_thread.wait()



    @staticmethod
    def __hex_list_to_int(data_list) -> int:
        data_str = ""
        for i in range(len(data_list)):
            data_bin = bin(data_list[i])[2:] # 首先转换为bin 去除0b
            data_bin = '0' * (8 - len(data_bin)) + data_bin # 头部补齐
            data_str = data_bin + data_str # 拼接
        # 首位是0 正数
        if int(data_str[0]) == 0: return int(data_str, 2)
        # 首位是1 负数
        else: return - ((int(data_str, 2) ^ 0xFFFFFFFF) + 1)
    
    @staticmethod
    def __match_index(index_low, index_high, subindex) -> list:
        # 低位 int转hex
        index_low_str = hex(index_low)[2:].upper()
        index_low_str = (2 - len(index_low_str)) * "0" + index_low_str
        # 高位 int转hex
        index_high_str = hex(index_high)[2:].upper()
        index_high_str = (2 - len(index_high_str)) * "0" + index_high_str
        # 合并 转int
        index = int(index_high_str + index_low_str, 16)
        # 返回列表的形式 以供比较
        return [index, subindex]

    @staticmethod
    def __hex_list_to_float(data_list: list) -> float:
        data_str = ""
        for i in range(len(data_list)):
            data_bin = bin(data_list[i])[2:] # 首先转换为bin 去除0b
            data_bin = '0' * (8 - len(data_bin)) + data_bin # 头部补齐
            data_str = data_bin + data_str # 拼接
        # 确定符号
        if data_str[0] == "0": sign = 1
        else: sign = -1
        # 确定整数
        power = 2 ** (int(data_str[1:9], 2) - 127)
        # 确定小数
        decimal = 1
        for i, num in enumerate(data_str[9:]):
            if num == "1": decimal = 2 ** (- (i + 1)) + decimal
        # 结果
        return sign * power * decimal



''' 连续体 调整 '''
class ContinuumAttitudeAdjust(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot, /, *, 
                 i_f: int, m_f: int, o_f: int, 
                 i_pid: tuple, m_pid: tuple, o_pid: tuple, 
                 start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.robot = robot

        self.__inside_reference_force = - abs(i_f)
        self.__midside_reference_force = - abs(m_f)
        self.__outside_reference_force = - abs(o_f)
        
        self.__inside_kp = abs(i_pid[0])
        self.__inside_ki = abs(i_pid[1])
        self.__inside_kd = abs(i_pid[2])
        
        self.__midside_kp = abs(m_pid[0])
        self.__midside_ki = abs(m_pid[1])
        self.__midside_kd = abs(m_pid[2])

        self.__outside_kp = abs(o_pid[0])
        self.__outside_ki = abs(o_pid[1])
        self.__outside_kd = abs(o_pid[2])

        # last_error  current_error  error_integral
        self.__error_1 = [0, 0, 0]
        self.__error_2 = [0, 0, 0]
        self.__error_3 = [0, 0, 0]
        self.__error_4 = [0, 0, 0]
        self.__error_5 = [0, 0, 0]
        self.__error_6 = [0, 0, 0]
        self.__error_7 = [0, 0, 0]
        self.__error_8 = [0, 0, 0]
        self.__error_9 = [0, 0, 0]

        if start_signal != None and finish_signal != None:
            self.__start_signal.connect(start_signal)
            self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()

        self.robot.rope_is_set_zero = False

        self.open_valve()
        
        self.force_follow()

        self.close_valve()

        self.set_zero()

        self.__finish_signal.emit()

    def open_valve(self):
        self.robot.io.open_valve_3()
        self.robot.io.open_valve_2()
        self.robot.io.open_valve_1()

    def close_valve(self):
        self.robot.io.close_valve_1()
        self.robot.io.close_valve_2()
        self.robot.io.close_valve_3()

    def force_follow(self):
        self.robot.motor_1.set_control_mode("speed_control", check=False)
        self.robot.motor_2.set_control_mode("speed_control", check=False)
        self.robot.motor_3.set_control_mode("speed_control", check=False)
        self.robot.motor_4.set_control_mode("speed_control", check=False)
        self.robot.motor_5.set_control_mode("speed_control", check=False)
        self.robot.motor_6.set_control_mode("speed_control", check=False)
        self.robot.motor_7.set_control_mode("speed_control", check=False)
        self.robot.motor_8.set_control_mode("speed_control", check=False)
        self.robot.motor_9.set_control_mode("speed_control", check=False)

        self.robot.motor_1.set_speed(0, is_pdo=True)
        self.robot.motor_2.set_speed(0, is_pdo=True)
        self.robot.motor_3.set_speed(0, is_pdo=True)
        self.robot.motor_4.set_speed(0, is_pdo=True)
        self.robot.motor_5.set_speed(0, is_pdo=True)
        self.robot.motor_6.set_speed(0, is_pdo=True)
        self.robot.motor_7.set_speed(0, is_pdo=True)
        self.robot.motor_8.set_speed(0, is_pdo=True)
        self.robot.motor_9.set_speed(0, is_pdo=True)

        self.robot.motor_1.halt(is_pdo=True)
        self.robot.motor_2.halt(is_pdo=True)
        self.robot.motor_3.halt(is_pdo=True)
        self.robot.motor_4.halt(is_pdo=True)
        self.robot.motor_5.halt(is_pdo=True)
        self.robot.motor_6.halt(is_pdo=True)
        self.robot.motor_7.halt(is_pdo=True)
        self.robot.motor_8.halt(is_pdo=True)
        self.robot.motor_9.halt(is_pdo=True)

        self.robot.motor_1.enable_operation(is_pdo=True)
        self.robot.motor_2.enable_operation(is_pdo=True)
        self.robot.motor_3.enable_operation(is_pdo=True)
        self.robot.motor_4.enable_operation(is_pdo=True)
        self.robot.motor_5.enable_operation(is_pdo=True)
        self.robot.motor_6.enable_operation(is_pdo=True)
        self.robot.motor_7.enable_operation(is_pdo=True)
        self.robot.motor_8.enable_operation(is_pdo=True)
        self.robot.motor_9.enable_operation(is_pdo=True)

        while not self.__is_stop:
            self.__error_1[0] = self.__error_1[1]
            self.__error_2[0] = self.__error_2[1]
            self.__error_3[0] = self.__error_3[1]
            self.__error_4[0] = self.__error_4[1]
            self.__error_5[0] = self.__error_5[1]
            self.__error_6[0] = self.__error_6[1]
            self.__error_7[0] = self.__error_7[1]
            self.__error_8[0] = self.__error_8[1]
            self.__error_9[0] = self.__error_9[1]

            self.__error_1[2] += self.__error_1[1]
            self.__error_2[2] += self.__error_2[1]
            self.__error_3[2] += self.__error_3[1]
            self.__error_4[2] += self.__error_4[1]
            self.__error_5[2] += self.__error_5[1]
            self.__error_6[2] += self.__error_6[1]
            self.__error_7[2] += self.__error_7[1]
            self.__error_8[2] += self.__error_8[1]
            self.__error_9[2] += self.__error_9[1]

            self.__error_1[1] = self.__outside_reference_force - self.robot.sensor_1.force
            self.__error_2[1] = self.__outside_reference_force - self.robot.sensor_2.force
            self.__error_3[1] = self.__outside_reference_force - self.robot.sensor_3.force

            self.__error_4[1] = self.__midside_reference_force - self.robot.sensor_4.force
            self.__error_5[1] = self.__midside_reference_force - self.robot.sensor_5.force
            self.__error_6[1] = self.__midside_reference_force - self.robot.sensor_6.force

            self.__error_7[1] = self.__inside_reference_force - self.robot.sensor_7.force
            self.__error_8[1] = self.__inside_reference_force - self.robot.sensor_8.force
            self.__error_9[1] = self.__inside_reference_force - self.robot.sensor_9.force

            speed_1 = self.__outside_kp * self.__error_1[1] + self.__outside_ki * self.__error_1[2] + self.__outside_kd * (self.__error_1[1] - self.__error_1[0])
            speed_2 = self.__outside_kp * self.__error_2[1] + self.__outside_ki * self.__error_2[2] + self.__outside_kd * (self.__error_2[1] - self.__error_2[0])
            speed_3 = self.__outside_kp * self.__error_3[1] + self.__outside_ki * self.__error_3[2] + self.__outside_kd * (self.__error_3[1] - self.__error_3[0])

            speed_4 = self.__midside_kp * self.__error_4[1] + self.__midside_ki * self.__error_4[2] + self.__midside_kd * (self.__error_4[1] - self.__error_4[0])
            speed_5 = self.__midside_kp * self.__error_5[1] + self.__midside_ki * self.__error_5[2] + self.__midside_kd * (self.__error_5[1] - self.__error_5[0])
            speed_6 = self.__midside_kp * self.__error_6[1] + self.__midside_ki * self.__error_6[2] + self.__midside_kd * (self.__error_6[1] - self.__error_6[0])

            speed_7 = self.__inside_kp * self.__error_7[1] + self.__inside_ki * self.__error_7[2] + self.__inside_kd * (self.__error_7[1] - self.__error_7[0])
            speed_8 = self.__inside_kp * self.__error_8[1] + self.__inside_ki * self.__error_8[2] + self.__inside_kd * (self.__error_8[1] - self.__error_8[0])
            speed_9 = self.__inside_kp * self.__error_9[1] + self.__inside_ki * self.__error_9[2] + self.__inside_kd * (self.__error_9[1] - self.__error_9[0])

            print("==================================================")
            print(int(speed_1), int(speed_2), int(speed_3))
            print(int(speed_4), int(speed_5), int(speed_6))
            print(int(speed_7), int(speed_8), int(speed_9))

            self.robot.motor_1.set_speed(int(speed_1), is_pdo=True, log=False)
            self.robot.motor_2.set_speed(int(speed_2), is_pdo=True, log=False)
            self.robot.motor_3.set_speed(int(speed_3), is_pdo=True, log=False)
            self.robot.motor_4.set_speed(int(speed_4), is_pdo=True, log=False)
            self.robot.motor_5.set_speed(int(speed_5), is_pdo=True, log=False)
            self.robot.motor_6.set_speed(int(speed_6), is_pdo=True, log=False)
            self.robot.motor_7.set_speed(int(speed_7), is_pdo=True, log=False)
            self.robot.motor_8.set_speed(int(speed_8), is_pdo=True, log=False)
            self.robot.motor_9.set_speed(int(speed_9), is_pdo=True, log=False)
        
        self.robot.motor_1.set_speed(0, is_pdo=True)
        self.robot.motor_2.set_speed(0, is_pdo=True)
        self.robot.motor_3.set_speed(0, is_pdo=True)
        self.robot.motor_4.set_speed(0, is_pdo=True)
        self.robot.motor_5.set_speed(0, is_pdo=True)
        self.robot.motor_6.set_speed(0, is_pdo=True)
        self.robot.motor_7.set_speed(0, is_pdo=True)
        self.robot.motor_8.set_speed(0, is_pdo=True)
        self.robot.motor_9.set_speed(0, is_pdo=True)

        self.robot.motor_1.disable_operation(is_pdo=True)
        self.robot.motor_2.disable_operation(is_pdo=True)
        self.robot.motor_3.disable_operation(is_pdo=True)
        self.robot.motor_4.disable_operation(is_pdo=True)
        self.robot.motor_5.disable_operation(is_pdo=True)
        self.robot.motor_6.disable_operation(is_pdo=True)
        self.robot.motor_7.disable_operation(is_pdo=True)
        self.robot.motor_8.disable_operation(is_pdo=True)
        self.robot.motor_9.disable_operation(is_pdo=True)

    def set_zero(self):
        self.robot.motor_1.zero_position = self.robot.motor_1.current_position
        self.robot.motor_2.zero_position = self.robot.motor_2.current_position
        self.robot.motor_3.zero_position = self.robot.motor_3.current_position
        self.robot.motor_4.zero_position = self.robot.motor_4.current_position
        self.robot.motor_5.zero_position = self.robot.motor_5.current_position
        self.robot.motor_6.zero_position = self.robot.motor_6.current_position
        self.robot.motor_7.zero_position = self.robot.motor_7.current_position
        self.robot.motor_8.zero_position = self.robot.motor_8.current_position
        self.robot.motor_9.zero_position = self.robot.motor_9.current_position

        self.robot.rope_is_set_zero = True
    
    def stop(self):
        self.__is_stop = True

''' 电机 速度模式 '''
class JointSpeed(QThread):
    def __init__(self, motor: list, speed: int, /, *, robot: ContinuumRobot) -> None:
        super().__init__()

        self.__is_stop = False

        self.__motor = motor
        self.__speed = speed

        self.robot = robot
    
    def run(self):
        for node_id in self.__motor:
            if node_id in Motor.motor_dict.keys():
                getattr(self.robot, f"motor_{node_id}").set_control_mode("speed_control", check=False)
                getattr(self.robot, f"motor_{node_id}").set_speed(self.__speed, is_pdo=True)
                getattr(self.robot, f"motor_{node_id}").halt(is_pdo=True)
        
        while not self.__is_stop:
            for node_id in self.__motor:
                if node_id in Motor.motor_dict.keys():
                    if getattr(self.robot, f"motor_{node_id}").is_in_range():
                        getattr(self.robot, f"motor_{node_id}").enable_operation(is_pdo=True)
                    else:
                        if getattr(self.robot, f"motor_{node_id}").current_position > getattr(self.robot, f"motor_{node_id}").max_position:
                            if self.__speed > 0: getattr(self.robot, f"motor_{node_id}").halt(is_pdo=True)
                            else: getattr(self.robot, f"motor_{node_id}").enable_operation(is_pdo=True)
                        else:
                            if not self.__speed > 0: getattr(self.robot, f"motor_{node_id}").halt(is_pdo=True)
                            else: getattr(self.robot, f"motor_{node_id}").enable_operation(is_pdo=True)
    
    def stop(self):
        self.__is_stop = True
        for node_id in self.__motor:
            if node_id in Motor.motor_dict.keys():
                getattr(self.robot, f"motor_{node_id}").set_speed(0, is_pdo=True)
                getattr(self.robot, f"motor_{node_id}").disable_operation(is_pdo=True)
                getattr(self.robot, f"motor_{node_id}").set_control_mode("position_control", check=False)



''' 遥操作 '''
class TeleOperation(QThread):
    button_signal_0 = pyqtSignal(int)
    button_signal_1 = pyqtSignal(int)
    button_signal_2 = pyqtSignal(int)
    button_signal_3 = pyqtSignal(int)
    button_signal_4 = pyqtSignal(int)
    button_signal_5 = pyqtSignal(int)
    button_signal_6 = pyqtSignal(int)
    button_signal_7 = pyqtSignal(int)
    button_signal_8 = pyqtSignal(int)
    button_signal_9 = pyqtSignal(int)
    button_signal_10 = pyqtSignal(int)
    button_signal_11 = pyqtSignal(int)
    axis_signal_0 = pyqtSignal(float)
    axis_signal_1 = pyqtSignal(float)
    axis_signal_2 = pyqtSignal(float)
    axis_signal_3 = pyqtSignal(float)
    hat_signal_0 = pyqtSignal(tuple)
    
    def __init__(self, robot: ContinuumRobot) -> None:
        super().__init__()
        
        self.robot = robot

        self.__is_joystick = False
        self.__is_stop = False
        
    def __setup(self):
        self.__is_stop = False

        pygame.init() # 初始化模块

        # 界面
        self.screen = pygame.display.set_mode([800, 500])
        # self.screen = pygame.display.set_mode((240, 180))
        pygame.display.set_caption("遥操作界面")
        self.screen.fill((255, 255, 255))

        self.font = pygame.font.SysFont(None, 40) # 字体

        self.clock = pygame.time.Clock() # 时钟

        # 等待joystick插入
        while not self.__is_joystick:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit()
            if pygame.joystick.get_count() == 1:
                self.__is_joystick = True
                self.joystick = pygame.joystick.Joystick(0)
            # 显示
            self.screen.fill((255,255,255))
            self.__print("No Joystick!", x=10, y=10)
            # 刷新
            pygame.display.flip()
            self.clock.tick(20)

    def __print(self, text, /, *, x, y):
        textBitmap = self.font.render(text, True, (0,0,0))
        self.screen.blit(textBitmap, [x, y])
    
    def run(self):
        self.__setup() # 初始化

        self.robot.rope_ready_speed("1","2","3","4","5","6","7","8","9")

        while not self.__is_stop:
            # 检测
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.__is_stop = True
            
            # 显示
            self.screen.fill((255,255,255))
            x_current, y_current = 10, 10
            self.__print("Joystick name: {}".format(self.joystick.get_name()), x=x_current, y=y_current)
            
            button_num = self.joystick.get_numbuttons()
            axis_num = self.joystick.get_numaxes()
            hat_num = self.joystick.get_numhats()
            
            for i in range(button_num):
                getattr(self, f"button_signal_{i}").emit(self.joystick.get_button(i))
                y_current += 40
                self.__print("Button {}:  {}".format(i, True if self.joystick.get_button(i) else False), x=x_current, y=y_current)
            x_current, y_current = 310, 10
            
            for i in range(axis_num):
                getattr(self, f"axis_signal_{i}").emit(self.joystick.get_axis(i))
                y_current += 40
                self.__print("Axis {}:  {:.6f}".format(i, self.joystick.get_axis(i)), x=x_current, y=y_current)
            x_current, y_current = 610, 10
            
            for i in range(hat_num):
                y_current += 40
                self.__print("Hat {}:  {}".format(i, str(self.joystick.get_hat(i))), x=x_current, y=y_current)
            
            # 操作
            kappa_d = self.joystick.get_axis(1) * 0.001 if abs(self.joystick.get_axis(1)) > 0.01 else 0
            phi_d = self.joystick.get_axis(2) * 0.25 if abs(self.joystick.get_axis(2)) > 0.01 else 0

            if self.joystick.get_button(2) and not self.joystick.get_button(3): # out
                if kappa_d >= 0 and self.robot.backbone_curvature[2] > 0.01: kappa_d = 0
                elif kappa_d < 0 and self.robot.backbone_curvature[2] < 0.00001: kappa_d = 0
                print("outside ", kappa_d, phi_d)
                l_1_d, l_2_d, l_3_d = self.robot.transferSpaceConfig2ActuatorJacobian("outside", 0, kappa_d, phi_d)
                self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d))
            elif self.joystick.get_button(3) and not self.joystick.get_button(2): # mid
                if kappa_d >= 0 and self.robot.backbone_curvature[1] > 0.01: kappa_d = 0
                elif kappa_d < 0 and self.robot.backbone_curvature[1] < 0.00001: kappa_d = 0
                print("midside ", kappa_d, phi_d)
                l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", 0, kappa_d, phi_d)
                self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d), ("4", l_4_d), ("5", l_5_d), ("6", l_6_d))
            else:
                self.robot.motor_1.set_speed(0, is_pdo=True)
                self.robot.motor_2.set_speed(0, is_pdo=True)
                self.robot.motor_3.set_speed(0, is_pdo=True)
                self.robot.motor_4.set_speed(0, is_pdo=True)
                self.robot.motor_5.set_speed(0, is_pdo=True)
                self.robot.motor_6.set_speed(0, is_pdo=True)
                self.robot.motor_7.set_speed(0, is_pdo=True)
                self.robot.motor_8.set_speed(0, is_pdo=True)
                self.robot.motor_9.set_speed(0, is_pdo=True)
            
            # 刷新
            pygame.display.flip()
            self.clock.tick(20)

        self.robot.rope_stop_speed("1","2","3","4","5","6","7","8","9")
        pygame.quit()

    def stop(self):
        self.__is_stop = True



''' 内节 运动学控制 '''
# class InsideSectionConfigurationSpaceJacobianControl(QThread):
#     action = pyqtSignal()
#     shutdown = pyqtSignal()
    
#     def __init__(self, robot: ContinuumRobot, /, *, s_d: float, kappa_d: float, phi_d: float, ref=(-2,-3,-5), kp=(0.2, 0.3, 0.5), ki=(0, 0, 0), kd=(0, 0, 0)) -> None:
#         super().__init__()

#         self.robot = robot

#         self.__s_d = s_d
#         self.__kappa_d = kappa_d
#         self.__phi_d = phi_d

#         self.__is_stop = False

#         self.__min_point = 348
#         self.__max_point = 358

#         self.__min_length = self.robot.backbone_init_length[0]
#         self.__max_length = 160

#         self.__min_curvature = 0.00001
#         self.__max_curvature = 0.03

#         self.__ref = ref
#         self.__kp = kp
#         self.__ki = ki
#         self.__kd = kd
    
#     def run(self):
#         self.action.emit()
        
#         # ref_in, ref_mid, ref_out = -2.0, -3.0, -5.0
#         ref_in, ref_mid, ref_out = self.__ref[0], self.__ref[1], self.__ref[2]
#         reference_force = np.array([[ref_out, ref_out, ref_out,
#                                      ref_mid, ref_mid, ref_mid,
#                                      ref_in,  ref_in,  ref_in]]).T
        
#         # kp_inside, kp_midside, kp_outside = 0.2, 0.3, 0.5
#         kp_inside, kp_midside, kp_outside = self.__kp[0], self.__kp[1], self.__kp[2]
#         kp = np.array([[kp_outside, kp_outside, kp_outside,
#                         kp_midside, kp_midside, kp_midside,
#                         kp_inside,  kp_inside,  kp_inside]]).T
#         # ki_inside, ki_midside, ki_outside = 0.0, 0.0, 0.0
#         ki_inside, ki_midside, ki_outside = self.__ki[0], self.__ki[1], self.__ki[2]
#         ki = np.array([[ki_outside, ki_outside, ki_outside,
#                         ki_midside, ki_midside, ki_midside,
#                         ki_inside,  ki_inside,  ki_inside]]).T
#         # kd_inside, kd_midside, kd_outside = 0.0, 0.0, 0.0
#         kd_inside, kd_midside, kd_outside = self.__kd[0], self.__kd[1], self.__kd[2]
#         kd = np.array([[kd_outside, kd_outside, kd_outside,
#                         kd_midside, kd_midside, kd_midside,
#                         kd_inside,  kd_inside,  kd_inside]]).T

#         previous_error = np.array([[0.0] * 9]).T
#         integral_error = np.array([[0.0] * 9]).T
#         l_d_compensation = np.array([[0.0] * 9]).T

#         if self.robot.continuum_calibration:
#             self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")
#             self.robot.initGripperActuator("speed")

#             while not self.__is_stop:
#                 ''' 曲率 '''
#                 # 曲率在范围内
#                 if self.__min_curvature <= self.robot.backbone_curvature[0] <= self.__max_curvature: kappa_d = self.__kappa_d
#                 else: # 曲率超出范围
#                     if (self.robot.backbone_curvature[0] < self.__min_curvature and self.__kappa_d > 0) or (self.robot.backbone_curvature[0] > self.__max_curvature and self.__kappa_d < 0): kappa_d = self.__kappa_d
#                     else: kappa_d = 0

#                 ''' 角度 '''
#                 phi_d = self.__phi_d
                
#                 ''' 判断backbone行为 '''
#                 # backbone固定
#                 if self.__s_d == 0:
#                     ''' 爪 '''
#                     self.robot.setGripper("open")
#                     self.robot.setAnchor("1", "close")
#                     self.robot.setAnchor("2", "close")
#                     self.robot.setAnchor("3", "close")
#                     ''' 长度 '''
#                     s_d = 0
#                     ''' 计算 '''
#                     l_1_d, l_2_d, l_3_d, \
#                         l_4_d, l_5_d, l_6_d, \
#                             l_7_d, l_8_d, l_9_d = self.robot.transferSpaceConfig2ActuatorJacobian("inside", s_d, kappa_d, phi_d)
#                     ''' 运动 '''
#                     self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
#                                                ("4",l_4_d),("5",l_5_d),("6",l_6_d),
#                                                ("7",l_7_d),("8",l_8_d),("9",l_9_d))
#                 else:
#                     if self.__min_point <= self.robot.gripper_position <= self.__max_point: # 大爪在运动范围内
#                         start_time = time.time()
#                         if (self.__s_d < 0 and self.robot.backbone_length[0] >= self.__min_length) or \
#                             (self.__s_d > 0 and self.robot.backbone_length[0] <= self.__max_length): # backbone不超过极限
#                             ''' 爪 '''
#                             self.robot.setGripper("close")
#                             self.robot.setAnchor("3", "open")
#                             self.robot.setAnchor("2", "open")
#                             self.robot.setAnchor("1", "open")
#                             ''' 长度 '''
#                             s_d = self.__s_d
#                         else: # backbone超过极限
#                             self.robot.stopGripper() # 大爪停止运动
#                             if self.robot.io.output_1: # 如果爪1处于开启
#                                 self.robot.stopTendon("1","2","3","4","5","6","7","8","9") # 先停止tendon运动
#                                 self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
#                             ''' 爪 '''
#                             self.robot.setGripper("open")
#                             self.robot.setAnchor("1", "close")
#                             self.robot.setAnchor("2", "close")
#                             self.robot.setAnchor("3", "close")
#                             ''' 长度 '''
#                             s_d = 0

#                             self.stop() # 跳出循环
                        
#                         ''' 雅可比 '''
#                         l_1_d, l_2_d, l_3_d, \
#                             l_4_d, l_5_d, l_6_d, \
#                                 l_7_d, l_8_d, l_9_d = self.robot.transferSpaceConfig2ActuatorJacobian("inside", s_d, kappa_d, phi_d)
#                         l_d = np.array([[l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d, l_7_d, l_8_d, l_9_d]]).T
#                         ''' PID '''
#                         current_force = np.array([[getattr(self.robot, f"sensor_{i}").force for i in range(1,10)]]).T
#                         # ref_in -= (self.robot.backbone_length[0] - self.robot.backbone_init_length[0]) * 3 / 70
#                         # ref_mid -= (self.robot.backbone_length[0] - self.robot.backbone_init_length[0]) * 3 / 70
#                         # ref_out -= (self.robot.backbone_length[0] - self.robot.backbone_init_length[0]) * 3 / 70
#                         # print(ref_in, ref_mid, ref_out)
#                         # reference_force = np.array([[ref_out, ref_out, ref_out,
#                         #                             ref_mid, ref_mid, ref_mid,
#                         #                             ref_in,  ref_in,  ref_in]]).T
#                         current_error = reference_force - current_force
#                         integral_error = np.maximum(np.minimum(integral_error + current_error, np.array([[10.0] * 9]).T), np.array([[-10.0] * 9]).T)
#                         l_d_compensation = current_error * kp + integral_error * ki + (current_error - previous_error) * kd
#                         previous_error = current_error

#                         if s_d > 0: l_d = np.maximum(l_d + l_d_compensation, np.array([[0.0] * 9]).T)
#                         else: l_d = np.minimum(l_d + l_d_compensation, np.array([[0.0] * 9]).T)
#                         # l_1_d += l_d_compensation[0, 0]
#                         # l_2_d += l_d_compensation[1, 0]
#                         # l_3_d += l_d_compensation[2, 0]
#                         # l_4_d += l_d_compensation[3, 0]
#                         # l_5_d += l_d_compensation[4, 0]
#                         # l_6_d += l_d_compensation[5, 0]
#                         # l_7_d += l_d_compensation[6, 0]
#                         # l_8_d += l_d_compensation[7, 0]
#                         # l_9_d += l_d_compensation[8, 0]
#                         ''' 运动 '''
#                         self.robot.moveTendonSpeed(("1",l_d[0, 0]),("2",l_d[1, 0]),("3",l_d[2, 0]),
#                                                     ("4",l_d[3, 0]),("5",l_d[4, 0]),("6",l_d[5, 0]),
#                                                     ("7",l_d[6, 0]),("8",l_d[7, 0]),("9",l_d[8, 0])) # tendon运动
#                         self.robot.moveGripperSpeed(s_d) # 大爪运动
#                         end_time = time.time()
#                         time.sleep(max(0.05 - (end_time - start_time), 0))
#                     else: # 大爪超出运动范围
#                         self.robot.stopGripper() # 大爪停止运动
#                         self.robot.stopTendon("1","2","3","4","5","6","7","8","9") # tendon停止运动

#                         self.robot.setAnchor("1", "close") # 关闭小爪
#                         self.robot.setAnchor("2", "close")
#                         self.robot.setAnchor("3", "close")
#                         self.robot.setGripper("open") # 打开大爪

#                         self.robot.initGripperActuator("position") # 大爪设置为位置模式

#                         if self.__s_d < 0: self.robot.moveGripperAbsolute(self.__max_point, 10, is_close=False, is_wait=True)
#                         else: self.robot.moveGripperAbsolute(self.__min_point, 10, is_close=False, is_wait=True)

#                         # self.robot.initTendonActuator("position","1","2","3","4","5","6","7","8","9")
#                         # for i in range(9,0,-1): # 拉紧
#                         #     if getattr(self.robot, f"sensor_{i}").force > - 0.25:
#                         #         while getattr(self.robot, f"sensor_{i}").force > - 0.5:
#                         #             self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)

#                         self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
#                         self.robot.initGripperActuator("speed") # 大爪速度模式

#                         previous_error = np.array([[0.0] * 9]).T
#                         integral_error = np.array([[0.0] * 9]).T
#         else: print("\033[0;31mPlease Calibration First ...\033[0m")

#         self.robot.stopTendon("1","2","3","4","5","6","7","8","9")
#         self.robot.stopGripper()

#         self.robot.setAnchor("1", "close")
#         self.robot.setAnchor("2", "close")
#         self.robot.setAnchor("3", "close")
#         self.robot.setGripper("open")

#         # 拉紧
#         self.robot.initTendonActuator("position","1","2","3","4","5","6","7","8","9")
#         for i in range(9,0,-1):
#             if getattr(self.robot, f"sensor_{i}").force > - 0.25:
#                 while getattr(self.robot, f"sensor_{i}").force > - 0.5:
#                     self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)

#         self.shutdown.emit()
    
#     def stop(self):
#         self.__is_stop = True

class InsideSectionConfigurationSpaceJacobianControl(QThread):
    action = pyqtSignal()
    shutdown = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot, /, *, s_d: float, kappa_d: float, phi_d: float, ref=(-2,-3,-5), kp=(0.2, 0.3, 0.5), ki=(0, 0, 0), kd=(0, 0, 0)) -> None:
        super().__init__()

        self.robot = robot

        self.__s_d = s_d
        self.__kappa_d = kappa_d
        self.__phi_d = phi_d

        self.__is_stop = False

        self.__min_point = 348
        self.__max_point = 358

        self.__min_length = self.robot.backbone_init_length[0]
        self.__max_length = 160

        self.__min_curvature = 0.0002
        self.__max_curvature = 0.03

        self.__ref = ref
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
    
    def run(self):
        self.action.emit()
        
        self.moveInside()

        self.shutdown.emit()

    def moveInside(self):
        # ref_in, ref_mid, ref_out = -2.0, -3.0, -5.0
        ref_in, ref_mid, ref_out = self.__ref[0], self.__ref[1], self.__ref[2]
        reference_force = np.array([[ref_out, ref_out, ref_out,
                                     ref_mid, ref_mid, ref_mid,
                                     ref_in,  ref_in,  ref_in]]).T
        
        # kp_inside, kp_midside, kp_outside = 0.2, 0.3, 0.5
        kp_inside, kp_midside, kp_outside = self.__kp[0], self.__kp[1], self.__kp[2]
        kp = np.array([[kp_outside, kp_outside, kp_outside,
                        kp_midside, kp_midside, kp_midside,
                        kp_inside,  kp_inside,  kp_inside]]).T
        # ki_inside, ki_midside, ki_outside = 0.0, 0.0, 0.0
        ki_inside, ki_midside, ki_outside = self.__ki[0], self.__ki[1], self.__ki[2]
        ki = np.array([[ki_outside, ki_outside, ki_outside,
                        ki_midside, ki_midside, ki_midside,
                        ki_inside,  ki_inside,  ki_inside]]).T
        # kd_inside, kd_midside, kd_outside = 0.0, 0.0, 0.0
        kd_inside, kd_midside, kd_outside = self.__kd[0], self.__kd[1], self.__kd[2]
        kd = np.array([[kd_outside, kd_outside, kd_outside,
                        kd_midside, kd_midside, kd_midside,
                        kd_inside,  kd_inside,  kd_inside]]).T

        previous_error = np.array([[0.0] * 9]).T
        integral_error = np.array([[0.0] * 9]).T

        if self.robot.continuum_calibration:
            self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")
            self.robot.initGripperActuator("speed")

            while not self.__is_stop:
                start_time = time.time()

                # 曲率在范围内
                if self.__min_curvature <= self.robot.backbone_curvature[0] <= self.__max_curvature: kappa_d = self.__kappa_d
                else: # 曲率超出范围
                    if (self.robot.backbone_curvature[0] < self.__min_curvature and self.__kappa_d > 0) or (self.robot.backbone_curvature[0] > self.__max_curvature and self.__kappa_d < 0): kappa_d = self.__kappa_d
                    else: kappa_d = 0

                phi_d = self.__phi_d
                
                if self.__s_d == 0: # backbone固定
                    self.robot.setGripper("open")
                    self.robot.setAnchor("1", "close")
                    self.robot.setAnchor("2", "close")
                    self.robot.setAnchor("3", "close")
                    
                    s_d = 0
                else: # backbone运动
                    if self.__min_point <= self.robot.gripper_position <= self.__max_point: # 大爪在运动范围内
                        if (self.__s_d < 0 and self.robot.backbone_length[0] >= self.__min_length) or \
                            (self.__s_d > 0 and self.robot.backbone_length[0] <= self.__max_length): # backbone不超过极限
                            
                            self.robot.setGripper("close")
                            self.robot.setAnchor("3", "open")
                            self.robot.setAnchor("2", "open")
                            self.robot.setAnchor("1", "open")

                            s_d = self.__s_d
                        else: # backbone超过极限
                            self.robot.stopGripper() # 大爪停止运动
                            if self.robot.io.output_1: # 如果爪1处于开启
                                self.robot.stopTendon("1","2","3","4","5","6","7","8","9") # 先停止tendon运动
                                self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
                            
                            self.robot.setGripper("open")
                            self.robot.setAnchor("1", "close")
                            self.robot.setAnchor("2", "close")
                            self.robot.setAnchor("3", "close")
                            
                            s_d = 0

                            break # 跳出循环
                    else: # 大爪超出运动范围
                        self.robot.stopGripper() # 大爪停止运动
                        self.robot.stopTendon("1","2","3","4","5","6","7","8","9") # tendon停止运动

                        self.robot.setAnchor("1", "close") # 关闭小爪
                        self.robot.setAnchor("2", "close")
                        self.robot.setAnchor("3", "close")
                        self.robot.setGripper("open") # 打开大爪

                        self.robot.initGripperActuator("position") # 大爪设置为位置模式

                        if self.__s_d < 0: self.robot.moveGripperAbsolute(self.__max_point, 20, is_close=False, is_wait=True)
                        else: self.robot.moveGripperAbsolute(self.__min_point, 20, is_close=False, is_wait=True)

                        self.robot.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
                        self.robot.initGripperActuator("speed") # 大爪速度模式

                        previous_error = np.array([[0.0] * 9]).T # 新一轮控制 清零
                        integral_error = np.array([[0.0] * 9]).T

                        continue
                
                ''' 雅可比 '''
                l_1_d, l_2_d, l_3_d, \
                    l_4_d, l_5_d, l_6_d, \
                        l_7_d, l_8_d, l_9_d = self.robot.transferSpaceConfig2ActuatorJacobian("inside", s_d, kappa_d, phi_d)
                l_d = np.array([[l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d, l_7_d, l_8_d, l_9_d]]).T
                
                ''' PID '''
                current_force = np.array([[getattr(self.robot, f"sensor_{i}").force for i in range(1,10)]]).T
                current_error = reference_force - current_force
                integral_error = np.maximum(np.minimum(integral_error + current_error, np.array([[10.0] * 9]).T), np.array([[-10.0] * 9]).T)
                l_d_compensation = current_error * kp + integral_error * ki + (current_error - previous_error) * kd
                previous_error = current_error

                if s_d > 0: l_d = np.maximum(l_d + l_d_compensation, np.array([[0.0] * 9]).T)
                elif s_d < 0: l_d = np.minimum(l_d + l_d_compensation, np.array([[0.0] * 9]).T)
                # else: l_d += l_d_compensation
                
                ''' 运动 '''
                self.robot.moveTendonSpeed(("1",l_d[0, 0]),("2",l_d[1, 0]),("3",l_d[2, 0]),
                                            ("4",l_d[3, 0]),("5",l_d[4, 0]),("6",l_d[5, 0]),
                                            ("7",l_d[6, 0]),("8",l_d[7, 0]),("9",l_d[8, 0])) # tendon运动
                self.robot.moveGripperSpeed(s_d) # 大爪运动

                time.sleep(max(0.05 - (time.time() - start_time), 0))
        else: print("\033[0;31mPlease Calibration First ...\033[0m")

        self.robot.stopTendon("1","2","3","4","5","6","7","8","9")
        self.robot.stopGripper()

        self.robot.setAnchor("1", "close")
        self.robot.setAnchor("2", "close")
        self.robot.setAnchor("3", "close")
        self.robot.setGripper("open")

        # 拉紧
        self.robot.initTendonActuator("position","1","2","3","4","5","6","7","8","9")
        for i in range(9,0,-1):
            if getattr(self.robot, f"sensor_{i}").force > - 0.25:
                while getattr(self.robot, f"sensor_{i}").force > - 0.5:
                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)
    
    def moveMidside(self): ...

    def moveOutside(self): ...

    def stop(self):
        self.__is_stop = True
    


''' 中节 运动学控制 '''
class MidsideSectionConfigurationSpaceJacobianControl(QThread):
    action = pyqtSignal()
    shutdown = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot, /, *, s_d: float, kappa_d: float, phi_d: float) -> None:
        super().__init__()

        self.robot = robot

        self.__s_d = s_d
        self.__kappa_d = kappa_d
        self.__phi_d = phi_d

        self.__is_stop = False

        self.__min_point = 227
        self.__max_point = 237

        self.__min_length = 51
        self.__max_length = 169

        self.__min_curvature = 0.0001
        self.__max_curvature = 0.04
    
    def run(self):
        self.action.emit()

        if self.robot.continuum_calibration:
            self.robot.initTendonActuator("speed","1","2","3","4","5","6")
            self.robot.initGripperActuator("speed")

            while not self.__is_stop:
                ''' 曲率 '''
                # 曲率在范围内
                if self.__min_curvature <= self.robot.backbone_curvature[1] <= self.__max_curvature:
                    kappa_d = self.__kappa_d
                # 曲率超出范围
                else:
                    if self.robot.backbone_curvature[1] < self.__min_curvature and self.__kappa_d >= 0:
                        kappa_d = self.__kappa_d
                    elif self.robot.backbone_curvature[1] > self.__max_curvature and self.__kappa_d <= 0:
                        kappa_d = self.__kappa_d
                    else: kappa_d = 0

                ''' 角度 '''
                phi_d = self.__phi_d
                
                ''' 判断backbone行为 '''
                # backbone固定
                if self.__s_d == 0:
                    ''' 爪 '''
                    self.robot.setGripper("open")
                    self.robot.setAnchor("1", "close")
                    self.robot.setAnchor("2", "close")
                    self.robot.setAnchor("3", "close")
                    ''' 长度 '''
                    s_d = 0
                    ''' 计算 '''
                    l_1_d, l_2_d, l_3_d, \
                        l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", s_d, kappa_d, phi_d)
                    ''' 运动 '''
                    self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
                                               ("4",l_4_d),("5",l_5_d),("6",l_6_d))
                # backbone缩短
                elif self.__s_d < 0:
                    ''' 判断大爪范围 '''
                    # 大爪在运动范围内
                    if self.__min_point <= self.robot.gripper_position <= self.__max_point + 0.2:
                        # backbone不超过最短极限 正常运动
                        if self.robot.backbone_length[1] >= self.__min_length:
                            ''' 爪 '''
                            self.robot.setGripper("close")
                            self.robot.setAnchor("3", "open")
                            self.robot.setAnchor("2", "open")
                            ''' 长度 '''
                            s_d = self.__s_d
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d, \
                                l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
                                                       ("4",l_4_d),("5",l_5_d),("6",l_6_d)) # tendon运动
                            self.robot.moveGripperSpeed(s_d) # 大爪运动
                        # backbone超过最短极限
                        else:
                            # 大爪停止运动
                            self.robot.stopGripper()
                            # 如果爪2处于开启
                            if self.robot.io.output_2:
                                # 先停止tendon运动
                                self.robot.stopTendon("1","2","3","4","5","6")
                                # tendon速度模式
                                self.robot.initTendonActuator("speed","1","2","3","4","5","6")
                            ''' 爪 '''
                            self.robot.setGripper("open")
                            self.robot.setAnchor("1", "close")
                            self.robot.setAnchor("2", "close")
                            self.robot.setAnchor("3", "close")
                            ''' 长度 '''
                            s_d = 0
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d, \
                                l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
                                                       ("4",l_4_d),("5",l_5_d),("6",l_6_d)) # tendon运动
                    # 大爪超出运动范围
                    else:
                        # 大爪停止运动
                        self.robot.stopGripper()
                        # tendon停止运动
                        self.robot.stopTendon("1","2","3","4","5","6")
                        # 关闭小爪
                        self.robot.setAnchor("1", "close")
                        self.robot.setAnchor("2", "close")
                        self.robot.setAnchor("3", "close")
                        # 打开大爪
                        self.robot.setGripper("open")
                        # 大爪设置为位置模式
                        self.robot.initGripperActuator("position")
                        # 大爪运动至min点 等待
                        self.robot.moveGripperAbsolute(self.__max_point, 10, is_close=False, is_wait=True)
                        # 拉紧
                        self.robot.initTendonActuator("position","1","2","3","4","5","6")
                        for i in range(6,0,-1):
                            if getattr(self.robot, f"sensor_{i}").force > - 0.25:
                                while getattr(self.robot, f"sensor_{i}").force > - 0.5:
                                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)
                        # tendon速度模式
                        self.robot.initTendonActuator("speed","1","2","3","4","5","6")
                        # 大爪速度模式
                        self.robot.initGripperActuator("speed")
                # backbone伸长
                else:
                    ''' 判断大爪范围 '''
                    # 大爪在运动范围内
                    if self.__min_point - 0.2 <= self.robot.gripper_position <= self.__max_point:
                        # backbone不超过最长极限 正常运动
                        if self.robot.backbone_length[1] <= self.__max_length:
                            ''' 爪 '''
                            self.robot.setGripper("close")
                            self.robot.setAnchor("3", "open")
                            self.robot.setAnchor("2", "open")
                            ''' 长度 '''
                            s_d = self.__s_d
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d, \
                                l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
                                                       ("4",l_4_d),("5",l_5_d),("6",l_6_d)) # tendon运动
                            self.robot.moveGripperSpeed(s_d) # 大爪运动
                        # backbone超过最长极限
                        else:
                            # 大爪停止运动
                            # self.robot.moveGripperSpeed(0)
                            self.robot.stopGripper()
                            # 如果爪2处于开启
                            if self.robot.io.output_2:
                                # 先停止tendon运动
                                self.robot.stopTendon("1","2","3","4","5","6")
                                # tendon速度模式
                                self.robot.initTendonActuator("speed","1","2","3","4","5","6")
                            ''' 爪 '''
                            self.robot.setGripper("open")
                            self.robot.setAnchor("1", "close")
                            self.robot.setAnchor("2", "close")
                            self.robot.setAnchor("3", "close")
                            ''' 长度 '''
                            s_d = 0
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d, \
                                l_4_d, l_5_d, l_6_d = self.robot.transferSpaceConfig2ActuatorJacobian("midside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d),
                                                       ("4",l_4_d),("5",l_5_d),("6",l_6_d)) # tendon运动
                    # 大爪超出运动范围
                    else:
                        # 大爪停止运动
                        self.robot.stopGripper()
                        # tendon停止运动
                        self.robot.stopTendon("1","2","3","4","5","6")
                        # 关闭小爪
                        self.robot.setAnchor("1", "close")
                        self.robot.setAnchor("2", "close")
                        self.robot.setAnchor("3", "close")
                        # 打开大爪
                        self.robot.setGripper("open")
                        # 大爪设置为位置模式
                        self.robot.initGripperActuator("position")
                        # 大爪运动至min点 等待
                        self.robot.moveGripperAbsolute(self.__min_point, 10, is_close=False, is_wait=True)
                        # 拉紧
                        self.robot.initTendonActuator("position","1","2","3","4","5","6")
                        for i in range(6,0,-1):
                            if getattr(self.robot, f"sensor_{i}").force > - 0.25:
                                while getattr(self.robot, f"sensor_{i}").force > - 0.5:
                                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)
                        # tendon速度模式
                        self.robot.initTendonActuator("speed","1","2","3","4","5","6")
                        # 大爪速度模式
                        self.robot.initGripperActuator("speed")
                        
        
        else: print("\033[0;31mPlease Calibration First ...\033[0m")

        self.robot.stopTendon("1","2","3","4","5","6")
        self.robot.stopGripper()

        self.robot.setAnchor("1", "close")
        self.robot.setAnchor("2", "close")
        self.robot.setAnchor("3", "close")
        self.robot.setGripper("open")

        # 拉紧
        self.robot.initTendonActuator("position","1","2","3","4","5","6")
        for i in range(6,0,-1):
            if getattr(self.robot, f"sensor_{i}").force > - 0.25:
                while getattr(self.robot, f"sensor_{i}").force > - 0.5:
                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)

        self.shutdown.emit()
    
    def stop(self):
        self.__is_stop = True


''' 外节 运动学控制 '''
class OutsideSectionConfigurationSpaceJacobianControl(QThread):
    action = pyqtSignal()
    shutdown = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot, /, *, s_d: float, kappa_d: float, phi_d: float) -> None:
        super().__init__()

        self.robot = robot

        self.__s_d = s_d
        self.__kappa_d = kappa_d
        self.__phi_d = phi_d

        self.__is_stop = False

        self.__min_point = 100
        self.__max_point = 110

        self.__min_length = 53
        self.__max_length = 170 #153

        self.__min_curvature = 0.0001
        self.__max_curvature = 0.05
    
    def run(self):
        self.action.emit()

        if self.robot.continuum_calibration:
            self.robot.initTendonActuator("speed","1","2","3")
            self.robot.initGripperActuator("speed")

            while not self.__is_stop:
                ''' 曲率 '''
                # 曲率在范围内
                if self.__min_curvature <= self.robot.backbone_curvature[2] <= self.__max_curvature:
                    kappa_d = self.__kappa_d
                # 曲率超出范围
                else:
                    if self.robot.backbone_curvature[2] < self.__min_curvature and self.__kappa_d >= 0:
                        kappa_d = self.__kappa_d
                    elif self.robot.backbone_curvature[2] > self.__max_curvature and self.__kappa_d <= 0:
                        kappa_d = self.__kappa_d
                    else: kappa_d = 0

                ''' 角度 '''
                phi_d = self.__phi_d
                
                ''' 判断backbone行为 '''
                # backbone固定
                if self.__s_d == 0:
                    ''' 爪 '''
                    self.robot.setGripper("open")
                    self.robot.setAnchor("1", "close")
                    self.robot.setAnchor("2", "close")
                    self.robot.setAnchor("3", "close")
                    ''' 长度 '''
                    s_d = 0
                    ''' 计算 '''
                    l_1_d, l_2_d, l_3_d  = self.robot.transferSpaceConfig2ActuatorJacobian("outside", s_d, kappa_d, phi_d)
                    ''' 运动 '''
                    self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d))
                # backbone缩短
                elif self.__s_d < 0:
                    ''' 判断大爪范围 '''
                    # 大爪在运动范围内
                    if self.__min_point <= self.robot.gripper_position <= self.__max_point + 0.2:
                        # backbone不超过最短极限 正常运动
                        if self.robot.backbone_length[2] >= self.__min_length:
                            ''' 爪 '''
                            self.robot.setGripper("close")
                            self.robot.setAnchor("3", "open")
                            ''' 长度 '''
                            s_d = self.__s_d
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d = self.robot.transferSpaceConfig2ActuatorJacobian("outside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d)) # tendon运动
                            self.robot.moveGripperSpeed(s_d) # 大爪运动
                        # backbone超过最长极限
                        else:
                            # 大爪停止运动
                            self.robot.stopGripper()
                            # 如果爪3处于开启
                            if self.robot.io.output_3:
                                # 先停止tendon运动
                                self.robot.stopTendon("1","2","3")
                                # tendon速度模式
                                self.robot.initTendonActuator("speed","1","2","3")
                            ''' 爪 '''
                            self.robot.setGripper("open")
                            self.robot.setAnchor("1", "close")
                            self.robot.setAnchor("2", "close")
                            self.robot.setAnchor("3", "close")
                            ''' 长度 '''
                            s_d = 0
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d = self.robot.transferSpaceConfig2ActuatorJacobian("outside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d)) # tendon运动
                    # 大爪超出运动范围
                    else:
                        # 大爪停止运动
                        self.robot.stopGripper()
                        # tendon停止运动
                        self.robot.stopTendon("1","2","3")
                        # 关闭小爪
                        self.robot.setAnchor("1", "close")
                        self.robot.setAnchor("2", "close")
                        self.robot.setAnchor("3", "close")
                        # 打开大爪
                        self.robot.setGripper("open")
                        # 大爪设置为位置模式
                        self.robot.initGripperActuator("position")
                        # 大爪运动至min点 等待
                        self.robot.moveGripperAbsolute(self.__max_point, 10, is_close=False, is_wait=True)
                        # 拉紧
                        self.robot.initTendonActuator("position","1","2","3")
                        for i in range(3,0,-1):
                            if getattr(self.robot, f"sensor_{i}").force > - 0.5:
                                while getattr(self.robot, f"sensor_{i}").force > - 1:
                                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)
                        # tendon速度模式
                        self.robot.initTendonActuator("speed","1","2","3")
                        # 大爪速度模式
                        self.robot.initGripperActuator("speed")
                # backbone伸长
                else:
                    ''' 判断大爪范围 '''
                    # 大爪在运动范围内
                    if self.__min_point - 0.2 <= self.robot.gripper_position <= self.__max_point:
                        # backbone不超过最长极限 正常运动
                        if self.robot.backbone_length[2] <= self.__max_length:
                            ''' 爪 '''
                            self.robot.setGripper("close")
                            self.robot.setAnchor("3", "open")
                            ''' 长度 '''
                            s_d = self.__s_d
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d = self.robot.transferSpaceConfig2ActuatorJacobian("outside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d)) # tendon运动
                            self.robot.moveGripperSpeed(s_d) # 大爪运动
                        # backbone超过最长极限
                        else:
                            # 大爪停止运动
                            self.robot.stopGripper()
                            # 如果爪3处于开启
                            if self.robot.io.output_3:
                                # 先停止tendon运动
                                self.robot.stopTendon("1","2","3")
                                # tendon速度模式
                                self.robot.initTendonActuator("speed","1","2","3")
                            ''' 爪 '''
                            self.robot.setGripper("open")
                            self.robot.setAnchor("1", "close")
                            self.robot.setAnchor("2", "close")
                            self.robot.setAnchor("3", "close")
                            ''' 长度 '''
                            s_d = 0
                            ''' 计算 '''
                            l_1_d, l_2_d, l_3_d = self.robot.transferSpaceConfig2ActuatorJacobian("outside", s_d, kappa_d, phi_d)
                            ''' 运动 '''
                            self.robot.moveTendonSpeed(("1",l_1_d),("2",l_2_d),("3",l_3_d)) # tendon运动
                    # 大爪超出运动范围
                    else:
                        # 大爪停止运动
                        self.robot.stopGripper()
                        # tendon停止运动
                        self.robot.stopTendon("1","2","3")
                        # 关闭小爪
                        self.robot.setAnchor("1", "close")
                        self.robot.setAnchor("2", "close")
                        self.robot.setAnchor("3", "close")
                        # 打开大爪
                        self.robot.setGripper("open")
                        # 大爪设置为位置模式
                        self.robot.initGripperActuator("position")
                        # 大爪运动至min点 等待
                        self.robot.moveGripperAbsolute(self.__min_point, 10, is_close=False, is_wait=True)
                        # 拉紧
                        self.robot.initTendonActuator("position","1","2","3")
                        for i in range(3,0,-1):
                            if getattr(self.robot, f"sensor_{i}").force > - 0.5:
                                while getattr(self.robot, f"sensor_{i}").force > - 1:
                                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)
                        # tendon速度模式
                        self.robot.initTendonActuator("speed","1","2","3")
                        # 大爪速度模式
                        self.robot.initGripperActuator("speed")
                        
        
        else: print("\033[0;31mPlease Calibration First ...\033[0m")

        self.robot.stopTendon("1","2","3")
        self.robot.stopGripper()

        self.robot.setAnchor("1", "close")
        self.robot.setAnchor("2", "close")
        self.robot.setAnchor("3", "close")
        self.robot.setGripper("open")

        # 拉紧
        self.robot.initTendonActuator("position","1","2","3")
        for i in range(3,0,-1):
            if getattr(self.robot, f"sensor_{i}").force > - 0.5:
                while getattr(self.robot, f"sensor_{i}").force > - 1:
                    self.robot.rope_move_rel(str(i), distance=-0.2, velocity=10)

        self.shutdown.emit()
    
    def stop(self):
        self.__is_stop = True


''' 内窥相机 '''
class Camera(QThread):
    update_img = pyqtSignal(QImage)
    clear_img = pyqtSignal()

    def __init__(self, robot: ContinuumRobot, /, *, update_slot=None, clear_slot=None):
        super().__init__()

        self.robot = robot

        if update_slot != None: self.update_img.connect(update_slot)
        if clear_slot != None: self.clear_img.connect(clear_slot)

        self.__is_stop = False

    def run(self):
        self.robot.camera = cv2.VideoCapture(0, cv2.CAP_V4L2)
        
        w, h = 640, 360
        self.robot.camera.set(3, w)
        self.robot.camera.set(4, h)

        while not self.__is_stop:
            success, img = self.robot.camera.read()
            mirrow = cv2.flip(img, 1)
            width, height = mirrow.shape[:2] # 行:宽 列:高

            # 显示图片
            image_show = cv2.cvtColor(mirrow, cv2.COLOR_BGR2RGB) # opencv读的通道是BGR,要转成RGB

            showImage = QtGui.QImage(image_show.data, height, width, QImage.Format_RGB888)
            self.update_img.emit(showImage)
        
        self.robot.camera.release()
        self.clear_img.emit()

    def stop(self):
        self.__is_stop = True
