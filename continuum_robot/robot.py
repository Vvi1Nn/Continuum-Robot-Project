# -*- coding:utf-8 -*-


''' robot.py continuum robot v2.3 '''


from PyQt5.QtCore import QThread, pyqtSignal, QObject

from PyQt5 import QtGui
from PyQt5.QtGui import QImage

from math import *
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
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
        "gripper_calibration_forward_distance": {"value": 30, "units": "mm", "note": "0~50"},
        "gripper_calibration_forward_velocity": {"value": 200, "units": "puu/s", "note": "0~200"},
        "gripper_calibration_backward_velocity": {"value": 50, "units": "puu/s", "note": "0~50"},
        "gripper_homing_velocity": {"value": 300, "units": "puu/s", "note": "0~300"},
        "sensor_sampling_frequency": {"value": 20, "units": "Hz", "note": "10~20"},
        "control_frequency": {"value": 20, "units": "Hz", "note": "10~20"},
        "sensor_calibration_ref_force_1": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_2": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_3": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_4": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_5": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_6": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_7": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_8": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_ref_force_9": {"value": 8.0, "units": "N", "note": ""},
        "sensor_calibration_init_count_1": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_2": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_3": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_4": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_5": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_6": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_7": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_8": {"value": 100, "units": "times", "note": ""},
        "sensor_calibration_init_count_9": {"value": 100, "units": "times", "note": ""},
        "continnum_calibration_ref_force_1": {"value": 5.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_2": {"value": 5.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_3": {"value": 5.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_4": {"value": 3.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_5": {"value": 3.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_6": {"value": 3.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_7": {"value": 2.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_8": {"value": 2.0, "units": "N", "note": ""},
        "continnum_calibration_ref_force_9": {"value": 2.0, "units": "N", "note": ""},
        "continnum_calibration_kp_1": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_2": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_3": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_4": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_5": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_6": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_7": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_8": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kp_9": {"value": 10.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_1": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_2": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_3": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_4": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_5": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_6": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_7": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_8": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_ki_9": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_1": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_2": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_3": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_4": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_5": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_6": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_7": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_8": {"value": 0.0, "units": "puu/(s*N)", "note": ""},
        "continnum_calibration_kd_9": {"value": 0.0, "units": "puu/(s*N)", "note": ""},

        "kinematics_control_inside_s_d": {"value": 5.0, "units": "mm/s", "note": ""},
        "kinematics_control_midside_s_d": {"value": 5.0, "units": "mm/s", "note": ""},
        "kinematics_control_outside_s_d": {"value": 5.0, "units": "mm/s", "note": ""},
        "kinematics_control_inside_kappa_d": {"value": 0.001, "units": "1/(mm*s)", "note": ""},
        "kinematics_control_midside_kappa_d": {"value": 0.001, "units": "1/(mm*s)", "note": ""},
        "kinematics_control_outside_kappa_d": {"value": 0.001, "units": "1/(mm*s)", "note": ""},
        "kinematics_control_inside_phi_d": {"value": 0.25, "units": "rad/s", "note": ""},
        "kinematics_control_midside_phi_d": {"value": 0.25, "units": "rad/s", "note": ""},
        "kinematics_control_outside_phi_d": {"value": 0.25, "units": "rad/s", "note": ""},

        "kinematics_control_inside_s_min": {"value": None, "units": "mm", "note": ""},
        "kinematics_control_inside_s_max": {"value": 160.0, "units": "mm", "note": ""},
        "kinematics_control_midside_s_min": {"value": None, "units": "mm", "note": ""},
        "kinematics_control_midside_s_max": {"value": 170.0, "units": "mm", "note": ""},
        "kinematics_control_outside_s_min": {"value": None, "units": "mm", "note": ""},
        "kinematics_control_outside_s_max": {"value": 170.0, "units": "mm", "note": ""},

        "kinematics_control_inside_kappa_min": {"value": 0.0002, "units": "1/mm", "note": ""},
        "kinematics_control_inside_kappa_max": {"value": 0.03, "units": "1/mm", "note": ""},
        "kinematics_control_midside_kappa_min": {"value": 0.0001, "units": "1/mm", "note": ""},
        "kinematics_control_midside_kappa_max": {"value": 0.04, "units": "1/mm", "note": ""},
        "kinematics_control_outside_kappa_min": {"value": 0.0001, "units": "1/mm", "note": ""},
        "kinematics_control_outside_kappa_max": {"value": 0.05, "units": "1/mm", "note": ""},

        "kinematics_control_inside_min_point": {"value": 348.0, "units": "mm", "note": ""},
        "kinematics_control_inside_max_point": {"value": 358.0, "units": "mm", "note": ""},
        "kinematics_control_midside_min_point": {"value": 227.0, "units": "mm", "note": ""},
        "kinematics_control_midside_max_point": {"value": 237.0, "units": "mm", "note": ""},
        "kinematics_control_outside_min_point": {"value": 100.0, "units": "mm", "note": ""},
        "kinematics_control_outside_max_point": {"value": 110.0, "units": "mm", "note": ""},

        "kinematics_control_inside_ref_force_1": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_2": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_3": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_4": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_5": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_6": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_7": {"value": 2.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_8": {"value": 2.0, "units": "N", "note": ""},
        "kinematics_control_inside_ref_force_9": {"value": 2.0, "units": "N", "note": ""},
        "kinematics_control_inside_kp_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_7": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_8": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kp_9": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_7": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_8": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_ki_9": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_7": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_8": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_inside_kd_9": {"value": 0.0, "units": "mm/(s*N)", "note": ""},

        "kinematics_control_midside_ref_force_1": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_midside_ref_force_2": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_midside_ref_force_3": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_midside_ref_force_4": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_midside_ref_force_5": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_midside_ref_force_6": {"value": 3.0, "units": "N", "note": ""},
        "kinematics_control_midside_kp_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kp_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kp_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kp_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kp_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kp_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_ki_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_4": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_5": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_midside_kd_6": {"value": 0.0, "units": "mm/(s*N)", "note": ""},

        "kinematics_control_outside_ref_force_1": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_outside_ref_force_2": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_outside_ref_force_3": {"value": 5.0, "units": "N", "note": ""},
        "kinematics_control_outside_kp_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_kp_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_kp_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_ki_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_ki_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_ki_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_kd_1": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_kd_2": {"value": 0.0, "units": "mm/(s*N)", "note": ""},
        "kinematics_control_outside_kd_3": {"value": 0.0, "units": "mm/(s*N)", "note": ""},

        "tighten_ref_force_1": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_2": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_3": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_4": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_5": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_6": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_7": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_8": {"value": 0.2, "units": "N", "note": "0~0.5"},
        "tighten_ref_force_9": {"value": 0.2, "units": "N", "note": "0~0.5"},
    }
    parameter_changed = pyqtSignal(str, float)
    def setParameter(self, name, value):
        if name in self.PARAMETER and self.PARAMETER[name]["value"] != value:
            self.PARAMETER[name]["value"] = float(value)
            self.parameter_changed.emit(name, float(value))

    show_motor_status = pyqtSignal(int)
    show_motor_original = pyqtSignal(int)
    show_motor_mode = pyqtSignal(int)
    show_switch = pyqtSignal()
    status_signal = pyqtSignal(str)
    show_gripper = pyqtSignal(bool)
    show_tendon = pyqtSignal(bool, int)
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

    continuum_calibration_start = pyqtSignal()
    continuum_calibration_end = pyqtSignal()

    continuum_move_start = pyqtSignal(str)
    continuum_move_end = pyqtSignal(str)
    
    def __init__(self) -> None:
        QObject.__init__(self)

        with open('robot_parameter.json', 'r') as file: self.PARAMETER = json.load(file) # 加载参数

        self.USBCAN_0 = UsbCan.setDeviceType(type="USBCAN2", index="0").isShowLog(False)("0")
        self.USBCAN_1 = UsbCan.setDeviceType(type="USBCAN2", index="0").isShowLog(False)("1")

        self.USBCAN_0.setTimer("250K")
        self.USBCAN_1.setTimer("1000K")

        self.USBCAN_0_is_start = False
        self.USBCAN_1_is_start = False

        CanOpenBusProcessor.linkDevice(self.USBCAN_0)
        Sensor.linkDevice(self.USBCAN_1)

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

        self.isSample = False
        self.TIME_STAMP, self.FORCE = {}, {}
        for i in range(1,11):
            self.TIME_STAMP[i] = []
            self.FORCE[i] = []

        self.io = IoModule(11)

        self.camera = None

        self.gripper_calibration = False
        self.gripper_position = None # mm
        self.gripper_velocity = None # mm/s

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
        self.tendon_zero_position = [None, None, None, None, None, None, None, None, None] # 123456789
        self.tendon_init_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.tendon_total_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.tendon_velocity = [None, None, None, None, None, None, None, None, None] # 123456789
        self.tendon_outside_length = [None, None, None] # 123
        self.tendon_midside_length = [None, None, None, None, None, None] # 123456
        self.tendon_inside_length = [None, None, None, None, None, None, None, None, None] # 123456789

        self.outside_coordinate = (None, None, None)
        self.midside_coordinate = (None, None, None)
        self.inside_coordinate = (None, None, None)
        self.outside_world_coordinate = (None, None, None)
        self.midside_world_coordinate = (None, None, None)
        self.inside_world_coordinate = (None, None, None)

        self.isMoving = False


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
                                if self.gripper_calibration: self.gripper_position = (self.motor_10.zero_position - self.motor_10.current_position) / 5120
                                self.gripper_velocity = self.motor_10.current_speed * 440 / 5120
                                self.show_gripper.emit(self.gripper_calibration)
                            else:
                                self.tendon_velocity[node_id-1] = speed * self.VELOCITY_RATIO / self.ROPE_RATIO
                                self.show_tendon.emit(self.continuum_calibration, node_id)

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
                    self.tendon_total_length[i-1] = self.tendon_init_length[i-1] + (getattr(self, f"motor_{i}").current_position - self.tendon_zero_position[i-1]) / self.ROPE_RATIO
                    # self.robot.tendon_velocity[i-1] = getattr(self.robot, f"motor_{i}").current_speed * self.robot.VELOCITY_RATIO / self.robot.ROPE_RATIO
                
                # inside
                for i in range(6,9):
                    self.tendon_inside_length[i] = self.tendon_total_length[i]
                
                s_in, kappa_in, phi_in = self.transferSpaceActuator2Config(self.tendon_inside_length[6], self.tendon_inside_length[7], self.tendon_inside_length[8])
                
                self.backbone_length[0] = s_in
                self.backbone_curvature[0] = kappa_in
                self.backbone_rotation_angle[0] = phi_in

                l_1, l_2, l_3, l_4, l_5, l_6, l_7, l_8, l_9 = self.transferSpaceConfig2Actuator("inside", s_in, kappa_in, phi_in)
                for i in range(0,6):
                    exec("self.tendon_inside_length[{}] = l_{}".format(i, i+1))

                self.inside_coordinate = self.transferSpaceConfig2Task("inside")
                trans_in = self.transferSpaceConfig2Task("inside", is_matrix=True)
                
                # midside
                for i in range(3,6):
                    self.tendon_midside_length[i] = self.tendon_total_length[i] - self.tendon_inside_length[i]

                s_mid, kappa_mid, phi_mid = self.transferSpaceActuator2Config(self.tendon_midside_length[3], self.tendon_midside_length[4], self.tendon_midside_length[5])
                self.backbone_length[1] = s_mid
                self.backbone_curvature[1] = kappa_mid
                self.backbone_rotation_angle[1] = phi_mid

                l_1, l_2, l_3, l_4, l_5, l_6 = self.transferSpaceConfig2Actuator("midside", s_mid, kappa_mid, phi_mid)
                for i in range(0,3):
                    exec("self.tendon_midside_length[{}] = l_{}".format(i, i+1))

                self.midside_coordinate = self.transferSpaceConfig2Task("midside")
                trans_mid = self.transferSpaceConfig2Task("midside", is_matrix=True)

                # outside
                for i in range(0,3):
                    self.tendon_outside_length[i] = self.tendon_total_length[i] - self.tendon_inside_length[i] - self.tendon_midside_length[i]

                s_out, kappa_out, phi_out = self.transferSpaceActuator2Config(self.tendon_outside_length[0], self.tendon_outside_length[1], self.tendon_outside_length[2])
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

            if self.isSample:
                self.TIME_STAMP[1].append(time.time())
                self.FORCE[1].append(self.sensor_1.force)
                if len(self.FORCE[1]) > 200:
                    self.TIME_STAMP[1].pop(0)
                    self.FORCE[1].pop(0)
            
            time.sleep(max(1 / self.PARAMETER["sensor_sampling_frequency"]["value"] - (time.time() - start_time), 0))
        
        print("Update Force Thread Stopped")
    
    def initUsbCan(self) -> bool:
        if UsbCan.openDevice():
            if not self.USBCAN_0_is_start and self.USBCAN_0.initChannel() and self.USBCAN_0.startChannel(): self.USBCAN_0_is_start = True
            if not self.USBCAN_1_is_start and self.USBCAN_1.initChannel() and self.USBCAN_1.startChannel(): self.USBCAN_1_is_start = True
        return self.USBCAN_0_is_start and self.USBCAN_1_is_start
   
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
    
    def setServo(self, status: str):
        for motor in Motor.motor_dict.values():
            time.sleep(0.001)
            exec("while not motor.{}(is_pdo=True): time.sleep(0.001)".format(status))


    def joint_speed(self, motor: list, speed: int):
        self.joint_speed_thread = JointSpeed(motor, speed, robot=self)
        self.joint_speed_thread.start()
    def joint_speed_stop(self):
        self.joint_speed_thread.stop()
        self.joint_speed_thread.wait()


    ''' 力传感器 零位标定
    '''
    def calibrateForceSensor(self):
        self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")

        self.sensor_calibration_start.emit()

        for i in range(9,0,-1):
            motor = getattr(self, f"motor_{i}")
            sensor = getattr(self, f"sensor_{i}")
            force_ref = -abs(self.PARAMETER["sensor_calibration_ref_force_{}".format(i)]["value"])

            while True:
                force_error = force_ref - sensor.force
                # if abs(force_error) < 0.01:
                #     motor.halt(is_pdo=True)
                #     break
                if sensor.force <= force_ref:
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
                time.sleep(0.8)
                current_force = sensor.original_data
                if abs(last_force - current_force) < 0.1:
                    motor.halt(is_pdo=True)
                    motor.set_speed(0, is_pdo=True)
                    break
            
            sensor.set_zero(self.PARAMETER["sensor_calibration_init_count_{}".format(i)]["value"])
            print(i, sensor.zero)

            motor.enable_operation(is_pdo=True)
            while True:
                force_error = -5 - sensor.force

                # if abs(force_error) < 0.01:
                #     motor.disable_operation(is_pdo=True)
                #     motor.set_speed(0, is_pdo=True)
                #     break
                if sensor.force <= -5:
                    motor.disable_operation(is_pdo=True)
                    motor.set_speed(0, is_pdo=True)
                    break
                elif abs(force_error) < 0.1: motor.set_speed(- 10 if force_error < 0 else 10, is_pdo=True, log=False)
                elif abs(force_error) < 2: motor.set_speed(int(force_error * 20), is_pdo=True, log=False)
                elif abs(force_error) < 4: motor.set_speed(int(force_error * 25), is_pdo=True, log=False)
                else: motor.set_speed(- 100 if force_error < 0 else 100, is_pdo=True, log=False)
        
        self.sensor_calibration_end.emit()

    ''' 调整连续体 '''
    def zeroContinuum(self):
        self.continuum_calibration_start.emit()

        self.isCalibrateContinuum = True

        self.setGripper("open")
        self.setAnchor("3", "open")
        self.setAnchor("2", "open")
        self.setAnchor("1", "open")
        
        self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")

        reference_force = np.array([[-abs(self.PARAMETER["continnum_calibration_ref_force_{}".format(i)]["value"]) for i in range(1,10)]]).T
        kp = np.array([[self.PARAMETER["continnum_calibration_kp_{}".format(i)]["value"] for i in range(1,10)]]).T
        ki = np.array([[self.PARAMETER["continnum_calibration_ki_{}".format(i)]["value"] for i in range(1,10)]]).T
        kd = np.array([[self.PARAMETER["continnum_calibration_kd_{}".format(i)]["value"] for i in range(1,10)]]).T
        previous_error = np.array([[0.0] * 9]).T
        integral_error = np.array([[0.0] * 9]).T
        integral_error_max = np.array([[10.0] * 9]).T
        integral_error_min = np.array([[-10.0] * 9]).T
         
        while self.isCalibrateContinuum:
            current_force = np.array([[getattr(self, f"sensor_{i}").force for i in range(1,10)]]).T
            current_error = reference_force - current_force
            integral_error = np.maximum(np.minimum(integral_error + current_error, integral_error_max), integral_error_min)
            speed = current_error * kp + integral_error * ki + (current_error - previous_error) * kd
            previous_error = current_error

            print("==================================================")
            print(int(speed[0, 0]), int(speed[1, 0]), int(speed[2, 0]))
            print(int(speed[3, 0]), int(speed[4, 0]), int(speed[5, 0]))
            print(int(speed[6, 0]), int(speed[7, 0]), int(speed[8, 0]))

            for i in range(1,10):
                getattr(self, f"motor_{i}").set_speed(int(speed[i-1, 0]), is_pdo=True, log=False)
        
        self.stopTendon("1","2","3","4","5","6","7","8","9")

        self.setAnchor("1", "close")
        self.setAnchor("2", "close")
        self.setAnchor("3", "close")

        for i in range(1,10):
            getattr(self, f"motor_{i}").setZeroPosition()

        self.continuum_calibration_end.emit()
    
    ''' 夹爪 归零
    手爪运动至极限位置
    speed: 运动速度 > 0
    start: 开始信号
    finish: 结束信号
    '''
    def homingGripper(self):
        self.gripper_homing_start.emit()
        
        if not self.gripper_calibration:
            self.setGripper("open")
            if not self.io.input_1:
                self.motor_10.set_control_mode("speed_control", check=False)
                self.motor_10.set_speed(abs(int(self.PARAMETER["gripper_homing_velocity"]["value"])), is_pdo=True)
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
                time.sleep(0.001)
        else:
            self.initGripperActuator("position")
            self.moveGripperAbsolute(0, 25, is_close=False, is_wait=True)
            self.stopGripper()
            self.gripper_homing_end.emit()

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

        d = abs(float(self.PARAMETER["gripper_calibration_forward_distance"]["value"]))
        v = abs(int(self.PARAMETER["gripper_calibration_forward_velocity"]["value"]))
        duration = (d * 5120) / (v * 440) + 0.5

        self.motor_10.set_position(- int(d * 5120), velocity=v, is_pdo=True)

        self.motor_10.ready(is_pdo=True)
        self.motor_10.action(is_immediate=True, is_relative=True, is_pdo=True)

        time.sleep(duration)
        
        if not self.io.input_1:
            self.motor_10.set_control_mode("speed_control")
            time.sleep(0.01)
        
            self.motor_10.set_speed(abs(int(self.PARAMETER["gripper_calibration_backward_velocity"]["value"])), is_pdo=True)

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
            if is_close: self.setGripper("close")
            else: self.setGripper("open")
            
            target_position = int(round(self.motor_10.zero_position - abs(point) * self.BALLSCREW_RATIO, 0))
            profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

            self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
            self.motor_10.ready(is_pdo=True)
            self.motor_10.action(is_immediate=False, is_relative=False, is_pdo=True)

            if is_wait:
                while self.motor_10.current_position != target_position: time.sleep(0.001)
                # duration_time = abs(target_position - self.motor_10.current_position) / (profile_velocity * self.VELOCITY_RATIO)
                # time.sleep(duration_time + 0.2)
    ''' 手爪在当前位置下移动相对距离
    distance: 移动距离 mm 有正负区别
    velocity: 运动速度 >0 mm/s
    is_wait: 添加延时 等待运动结束后再进行下一步操作
    '''
    def moveGripperRelative(self, distance: float, velocity: float, /, *, is_close=False, is_wait=True):
        if self.gripper_calibration:
            if is_close: self.setGripper("close")
            else: self.setGripper("open")
            
            target_position = int(round(distance * self.BALLSCREW_RATIO, 0))
            profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

            self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
            self.motor_10.ready(is_pdo=True)
            self.motor_10.action(is_immediate=False, is_relative=True, is_pdo=True)

            if is_wait:
                duration_time = abs(target_position) / (profile_velocity * self.VELOCITY_RATIO)
                time.sleep(duration_time + 0.2)
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

   
    ''' 肌腱初始化
    ''' 
    def initTendonActuator(self, control_mode: str, *args: str):
        if control_mode == "speed":
            for node_id in args: getattr(self, f"motor_{node_id}").set_control_mode("speed_control")
            for node_id in args: getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
            for node_id in args: getattr(self, f"motor_{node_id}").enable_operation(is_pdo=True)
        elif control_mode == "position":
            for node_id in args: getattr(self, f"motor_{node_id}").set_control_mode("position_control")
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
        for node_id in args: getattr(self, f"motor_{node_id}").disable_operation(is_pdo=True)
        for node_id in args: getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
    def moveTendonAbsolute(self, *args: tuple):
        id_list = []
        t_list = []
        for tuple in args:
            id, tar_l, vel = tuple
            id_list.append(id)
            
            tar_pos = int(round(self.tendon_zero_position[int(id)-1] + (tar_l - self.tendon_init_length[int(id)-1]) * self.ROPE_RATIO, 0))
            pro_vel = max(int(round(self.ROPE_RATIO * abs(vel) / self.VELOCITY_RATIO, 0)), 1)

            t_list.append(abs(tar_pos - getattr(self, f"motor_{id}").current_position) / (pro_vel * self.VELOCITY_RATIO) + 0.05)

            getattr(self, f"motor_{id}").set_position(tar_pos, velocity=pro_vel, is_pdo=True)
            getattr(self, f"motor_{id}").ready(is_pdo=True)
        
        t_list.sort(reverse=True)
        for id in id_list: getattr(self, f"motor_{id}").action(is_immediate=False, is_relative=False, is_pdo=True)
        time.sleep(t_list[0])

        for id in id_list: getattr(self, f"motor_{id}").disable_operation()
    def moveTendonRelative(self, *args: tuple):
        id_list = []
        t_list = []
        for tuple in args:
            id, pos, vel = tuple
            id_list.append(id)
            
            tar_pos = int(round(pos * self.ROPE_RATIO, 0))
            pro_vel = max(int(round(self.ROPE_RATIO * abs(vel) / self.VELOCITY_RATIO, 0)), 1)

            t_list.append(abs(tar_pos) / (pro_vel * self.VELOCITY_RATIO))

            getattr(self, f"motor_{id}").set_position(tar_pos, velocity=pro_vel, is_pdo=True)
            getattr(self, f"motor_{id}").ready(is_pdo=True)
        
        t_list.sort(reverse=True)
        for id in id_list: getattr(self, f"motor_{id}").action(is_immediate=False, is_relative=True, is_pdo=True)
        time.sleep(t_list[0])
        
        for id in id_list: getattr(self, f"motor_{id}").disable_operation()
    

    ''' 标定运动学 '''
    def calibrateContinuum(self, bl_o: float, bl_m: float, bl_i: float):
        self.tendon_zero_position = [getattr(self, f"motor_{i}").current_position for i in range(1,10)]

        self.tendon_init_length = [bl_o+bl_m+bl_i, bl_o+bl_m+bl_i, bl_o+bl_m+bl_i, 
                                 bl_m+bl_i,      bl_m+bl_i,      bl_m+bl_i, 
                                 bl_i,           bl_i,           bl_i]
        
        self.tendon_inside_length = [bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i, bl_i] # 123456789
        self.tendon_midside_length = [bl_m, bl_m, bl_m, bl_m, bl_m, bl_m] # 123456
        self.tendon_outside_length = [bl_o, bl_o, bl_o] # 123

        self.backbone_init_length = [bl_i, bl_m, bl_o] # in mid out
        self.backbone_length = [bl_i, bl_m, bl_o] # in mid out
        self.backbone_curvature = [0, 0, 0] # in mid out
        self.backbone_rotation_angle = [0, 0, 0] # in mid out

        self.setParameter("kinematics_control_inside_s_min", bl_i)
        self.setParameter("kinematics_control_midside_s_min", bl_m)
        self.setParameter("kinematics_control_outside_s_min", bl_o)

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
    
    
    ''' 运动学控制 '''
    def controlContinuum(self, section: str, s_d: float, kappa_d: float, phi_d: float):
        self.continuum_move_start.emit(section)

        if not self.continuum_calibration:
            print("\033[0;31mNo Calibration\033[0m")
            self.continuum_move_end.emit(section)
            return

        self.isControl = True
        self.isMoving = True

        if section == "inside": tendon_count = 9
        elif section == "midside": tendon_count = 6
        elif section == "outside": tendon_count = 3

        reference_force = np.array([[-abs(self.PARAMETER[f"kinematics_control_{section}_ref_force_{i}"]["value"]) for i in range(1, tendon_count+1)]]).T
        kp = np.array([[self.PARAMETER[f"kinematics_control_{section}_kp_{i}"]["value"] for i in range(1, tendon_count+1)]]).T
        ki = np.array([[self.PARAMETER[f"kinematics_control_{section}_ki_{i}"]["value"] for i in range(1, tendon_count+1)]]).T
        kd = np.array([[self.PARAMETER[f"kinematics_control_{section}_kd_{i}"]["value"] for i in range(1, tendon_count+1)]]).T
        previous_error = np.array([[0.0] * tendon_count]).T
        integral_error = np.array([[0.0] * tendon_count]).T

        min_length = self.PARAMETER[f"kinematics_control_{section}_s_min"]["value"]
        max_length = self.PARAMETER[f"kinematics_control_{section}_s_max"]["value"]
        min_curvature = self.PARAMETER[f"kinematics_control_{section}_kappa_min"]["value"]
        max_curvature = self.PARAMETER[f"kinematics_control_{section}_kappa_max"]["value"]
        min_point = self.PARAMETER[f"kinematics_control_{section}_min_point"]["value"]
        max_point = self.PARAMETER[f"kinematics_control_{section}_max_point"]["value"]

        try:
            if section == "inside": self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9")
            elif section == "midside": self.initTendonActuator("speed","1","2","3","4","5","6")
            elif section == "outside": self.initTendonActuator("speed","1","2","3")
            self.initGripperActuator("speed")

            while self.isControl:
                start_time = time.time()
                
                if section == "inside": current_backbone_curvature = self.backbone_curvature[0]
                elif section == "midside": current_backbone_curvature = self.backbone_curvature[1]
                elif section == "outside": current_backbone_curvature = self.backbone_curvature[2]

                if min_curvature <= current_backbone_curvature <= max_curvature: kappa_d = kappa_d # 曲率在范围内
                else: # 曲率超出范围
                    if (current_backbone_curvature < min_curvature and kappa_d > 0) or (current_backbone_curvature > max_curvature and kappa_d < 0): kappa_d = kappa_d
                    else: kappa_d = 0
                
                if s_d == 0: # backbone固定
                    self.setGripper("open")
                    self.setAnchor("1", "close")
                    self.setAnchor("2", "close")
                    self.setAnchor("3", "close")
                else: # backbone运动
                    if min_point <= self.gripper_position <= max_point: # 大爪在运动范围内
                        if section == "inside": current_backbone_length = self.backbone_length[0]
                        elif section == "midside": current_backbone_length = self.backbone_length[1]
                        elif section == "outside": current_backbone_length = self.backbone_length[2]

                        if (s_d < 0 and current_backbone_length >= min_length) or \
                        (s_d > 0 and current_backbone_length <= max_length): # backbone不超过极限
                            self.setGripper("close")
                            if section == "inside" or section == "midside" or section == "outside": self.setAnchor("3", "open")
                            if section == "inside" or section == "midside": self.setAnchor("2", "open")
                            if section == "inside": self.setAnchor("1", "open")
                        else: # backbone超过极限
                            self.stopGripper() # 大爪停止运动
                            if self.io.output_1: # 如果爪1处于开启
                                if section == "inside":
                                    self.stopTendon("1","2","3","4","5","6","7","8","9") # 先停止tendon运动
                                    self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
                                elif section == "midside":
                                    self.stopTendon("1","2","3","4","5","6")
                                    self.initTendonActuator("speed","1","2","3","4","5","6")
                                elif section == "outside":
                                    self.stopTendon("1","2","3")
                                    self.initTendonActuator("speed","1","2","3")
                                
                            self.setGripper("open")
                            self.setAnchor("1", "close")
                            self.setAnchor("2", "close")
                            self.setAnchor("3", "close")
                            
                            s_d = 0

                            break # 跳出循环
                    else: # 大爪超出运动范围
                        self.stopGripper() # 大爪停止运动
                        if section == "inside": self.stopTendon("1","2","3","4","5","6","7","8","9") # tendon停止运动
                        elif section == "midside": self.stopTendon("1","2","3","4","5","6")
                        elif section == "outside": self.stopTendon("1","2","3")

                        self.setAnchor("1", "close") # 关闭小爪
                        self.setAnchor("2", "close")
                        self.setAnchor("3", "close")
                        self.setGripper("open") # 打开大爪

                        self.initGripperActuator("position") # 大爪设置为位置模式

                        if s_d < 0: self.moveGripperAbsolute(max_point, 20, is_close=False, is_wait=True)
                        else: self.moveGripperAbsolute(min_point, 20, is_close=False, is_wait=True)

                        if section == "inside": self.initTendonActuator("speed","1","2","3","4","5","6","7","8","9") # tendon速度模式
                        elif section == "midside": self.initTendonActuator("speed","1","2","3","4","5","6")
                        elif section == "outside": self.initTendonActuator("speed","1","2","3")
                        self.initGripperActuator("speed") # 大爪速度模式

                        previous_error = np.array([[0.0] * tendon_count]).T # 新一轮控制 清零
                        integral_error = np.array([[0.0] * tendon_count]).T

                        continue
                
                ''' 雅可比 '''
                ret = self.transferSpaceConfig2ActuatorJacobian(section, s_d, kappa_d, phi_d)
                l_d = np.array([[l_i_d for l_i_d in ret]]).T
                
                ''' PID '''
                current_force = np.array([[getattr(self, f"sensor_{i}").force for i in range(1, tendon_count+1)]]).T
                current_error = reference_force - current_force
                integral_error = np.maximum(np.minimum(integral_error + current_error, np.array([[10.0] * tendon_count]).T), np.array([[-10.0] * tendon_count]).T)
                l_d_compensation = current_error * kp + integral_error * ki + (current_error - previous_error) * kd
                previous_error = current_error

                if s_d > 0: l_d = np.maximum(l_d + l_d_compensation, np.array([[0.0] * tendon_count]).T)
                elif s_d < 0: l_d = np.minimum(l_d + l_d_compensation, np.array([[0.0] * tendon_count]).T)
                # else: l_d += l_d_compensation
                
                ''' 运动 '''
                if section == "inside":
                    self.moveTendonSpeed(("1",l_d[0, 0]),("2",l_d[1, 0]),("3",l_d[2, 0]),
                                        ("4",l_d[3, 0]),("5",l_d[4, 0]),("6",l_d[5, 0]),
                                        ("7",l_d[6, 0]),("8",l_d[7, 0]),("9",l_d[8, 0])) # tendon运动
                elif section == "midside":
                    self.moveTendonSpeed(("1",l_d[0, 0]),("2",l_d[1, 0]),("3",l_d[2, 0]),
                                        ("4",l_d[3, 0]),("5",l_d[4, 0]),("6",l_d[5, 0])) # tendon运动
                elif section == "outside":
                    self.moveTendonSpeed(("1",l_d[0, 0]),("2",l_d[1, 0]),("3",l_d[2, 0])) # tendon运动
                self.moveGripperSpeed(s_d) # 大爪运动

                time.sleep(max(1 / self.PARAMETER["control_frequency"]["value"] - (time.time() - start_time), 0))
        except Exception as e:
            print(e)
            print("Control Fail")
            
        # 停
        if section == "inside": self.stopTendon("1","2","3","4","5","6","7","8","9")
        elif section == "midside": self.stopTendon("1","2","3","4","5","6")
        elif section == "outside": self.stopTendon("1","2","3")
        self.stopGripper()

        # 爪
        self.setAnchor("1", "close")
        self.setAnchor("2", "close")
        self.setAnchor("3", "close")
        self.setGripper("open")

        # 紧
        if section == "inside": self.initTendonActuator("position","1","2","3","4","5","6","7","8","9")
        elif section == "midside": self.initTendonActuator("position","1","2","3","4","5","6")
        elif section == "outside": self.initTendonActuator("position","1","2","3")
        for i in range(tendon_count, 0, -1):
            # if getattr(self, f"sensor_{i}").force > -abs(self.PARAMETER[f"tighten_ref_force_{i}"]):
            while getattr(self, f"sensor_{i}").force > -abs(self.PARAMETER[f"tighten_ref_force_{i}"]["value"]): self.moveTendonRelative((i, -0.1, 10))
        
        self.continuum_move_end.emit(section)

    
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


    def plotParameter(self):
        self.isSample = True
        def animate(i):
            plt.cla()
            plt.plot(self.TIME_STAMP[1], self.FORCE[1])
            plt.tight_layout()
        ani = FuncAnimation(plt.gcf(), animate, interval=10)
        plt.tight_layout()
        plt.show()
        self.isSample = False


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
                    if getattr(self.robot, f"motor_{node_id}").isInRange():
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
