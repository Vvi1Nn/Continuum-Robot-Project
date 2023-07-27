# -*- coding:utf-8 -*-


''' robot.py continuum robot v1.0 '''


from PyQt5.QtCore import QThread, pyqtSignal

from math import *
import numpy as np

# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from continuum_robot.usbcan import UsbCan
from continuum_robot.processor import CanOpenBusProcessor
from continuum_robot.motor import Motor
from continuum_robot.io import IoModule
from continuum_robot.sensor import Sensor
from continuum_robot.joystick import Joystick


class ContinuumRobot():
    BALLSCREW_RATIO = 5120
    ROPE_RATIO = 12536.512440
    VELOCITY_RATIO = 440
    
    def __init__(self) -> None:
        self.usbcan_0 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("0")
        self.usbcan_1 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("1")

        self.usbcan_0.set_timer("250K")
        self.usbcan_1.set_timer("1000K")

        self.usbcan_0_is_start = False
        self.usbcan_1_is_start = False

        CanOpenBusProcessor.link_device(self.usbcan_0)
        Sensor.link_device(self.usbcan_1)

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

        self.ballscrew_is_set_zero = False

        self.ballscrew_position = None # mm
        self.ballscrew_velocity = None # mm/s

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

        self.read_canopen_thread = CANopenUpdate(self)
        
        self.read_sensor_thread = SensorResolve()

        self.joystick_thread = Joystick()

        self.backbone_init_length = [168, 170, 172] # in mid out
        self.backbone_length = [None, None, None] # in mid out
        # self.backbone_length_d = [0, 0, 0]

        self.backbone_curvature = [None, None, None] # in mid out
        # self.backbone_curvature_d = [0, 0, 0]

        self.backbone_rotation_angle = [None, None, None] # in mid out
        # self.backbone_rotation_angle_d = [0, 0, 0]

        self.backbone_section_num = [9, 10, 10]
        self.backbone_d = 2.6

        # self.outside_calibration = [None, None, None] # 1 2 3
        # self.midside_calibration = [None, None, None] # 4 5 6
        # self.inside_calibration = [None, None, None] # 7 8 9

        self.is_calibration = False
        self.rope_zero_position = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_init_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_total_length = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_velocity = [None, None, None, None, None, None, None, None, None] # 123456789
        self.rope_outside_length = [None, None, None] # 123
        self.rope_midside_length = [None, None, None, None, None, None] # 123456
        self.rope_inside_length = [None, None, None, None, None, None, None, None, None] # 123456789

        # self.outside_length = [None, None, None] # 1 2 3
        # self.midside_length = [None, None, None] # 4 5 6
        # self.inside_length = [None, None, None] # 7 8 9

        self.outside_coordinate = (None, None, None)
        self.midside_coordinate = (None, None, None)
        self.inside_coordinate = (None, None, None)

    def open_device(self) -> bool:
        if UsbCan.open_device():
            
            if not self.usbcan_0_is_start and self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.read_canopen_thread.start()
                self.usbcan_0_is_start = True

            if not self.usbcan_1_is_start and self.usbcan_1.init_can() and self.usbcan_1.start_can():
                self.read_sensor_thread.start()
                self.usbcan_1_is_start = True
        
        return self.usbcan_0_is_start and self.usbcan_1_is_start
    
    def initialize_robot(self, times: int, running_signal, finish_signal) -> None:
        self.init_robot_thread = RobotInit(times=times, running_signal=running_signal, finish_signal=finish_signal)
        self.send_request_thread = SensorRequest()

        self.init_robot_thread.start()
        self.send_request_thread.start()
    
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

    def ballscrew_set_zero(self, distance: int, velocity: int, speed: int, start, finish) -> None:
        self.ballscrew_set_zero_thread = BallScrewSetZero(distance, velocity, speed, robot=self, start_signal=start, finish_signal=finish)
        self.ballscrew_set_zero_thread.start()

    def rope_force_adapt(self, i_f: int, m_f: int, o_f: int, i_pid: tuple, m_pid: tuple, o_pid: tuple, start, finish) -> None:
        self.rope_force_adapt_thread = ContinuumAttitudeAdjust(self, 
                                                               i_f=i_f, m_f=m_f, o_f=o_f, 
                                                               i_pid=i_pid, m_pid=m_pid, o_pid=o_pid, 
                                                               start_signal=start, finish_signal=finish)
        self.rope_force_adapt_thread.start()
    
    def rope_set_zero(self) -> None:
        self.rope_force_adapt_thread.stop()
        self.rope_force_adapt_thread.wait()

    def ballscrew_go_zero(self, speed: int, start, finish) -> None:
        self.ballscrew_go_zero_thread = BallScrewGoZero(speed, robot=self, start_signal=start, finish_signal=finish)
        self.ballscrew_go_zero_thread.start()
    
    def ballscrew_move_abs(self, point: float, /, *, velocity: float, is_wait=True) -> None:
        target_position = int(round(self.motor_10.zero_position - abs(point) * self.BALLSCREW_RATIO, 0))
        profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

        self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
        self.motor_10.ready(is_pdo=True)
        self.motor_10.action(is_immediate=False, is_relative=False, is_pdo=True)

        if is_wait:
            duration_time = abs(target_position - self.motor_10.current_position) / (profile_velocity * self.VELOCITY_RATIO) + 1
            time.sleep(duration_time)
    
    def ballscrew_move_rel(self, distance: float, /, *, velocity: float, is_wait=True) -> None:
        target_position = int(round(distance * self.BALLSCREW_RATIO, 0))
        profile_velocity = int(round(self.BALLSCREW_RATIO * velocity / self.VELOCITY_RATIO, 0))

        self.motor_10.set_position(target_position, velocity=profile_velocity, is_pdo=True)
        self.motor_10.ready(is_pdo=True)
        self.motor_10.action(is_immediate=False, is_relative=True, is_pdo=True)

        if is_wait:
            duration_time = abs(target_position) / (profile_velocity * self.VELOCITY_RATIO) + 1
            time.sleep(duration_time)

    def ballscrew_move(self, distance: float, velocity: float, /, *, is_close=False, is_relative=False) -> None:
        self.ballscrew_move_thread = BallScrewMove(distance, velocity, is_close=is_close, is_relative=is_relative, robot=self)
        self.ballscrew_move_thread.start()


    def rope_move_abs(self, rope: str, /, *, point: float, velocity: float) -> None:
        duration_time = []

        for node_id in rope:
            target_position = int(round(getattr(self, f"motor_{node_id}").zero_position + abs(point) * self.ROPE_RATIO, 0))
            profile_velocity = int(round(self.ROPE_RATIO * abs(velocity) / self.VELOCITY_RATIO, 0))
            getattr(self, f"motor_{node_id}").set_position(target_position, velocity=profile_velocity, is_pdo=True)
            getattr(self, f"motor_{node_id}").ready(is_pdo=True)
            
            t = abs(target_position - getattr(self, f"motor_{node_id}").current_position) / (profile_velocity * self.VELOCITY_RATIO) + 0.1
            duration_time.append(t)

        duration_time.sort(reverse=True)
        delay = duration_time[0]
        
        for node_id in rope:
            getattr(self, f"motor_{node_id}").action(is_immediate=False, is_relative=False, is_pdo=True)
        
        time.sleep(delay)
    
    def rope_move_rel(self, rope: str, /, *, distance: float, velocity: float, is_wait=True) -> None:
        target_position = int(round(distance * self.ROPE_RATIO, 0))
        profile_velocity = int(round(self.ROPE_RATIO * abs(velocity) / self.VELOCITY_RATIO, 0))
        
        for node_id in rope:
            getattr(self, f"motor_{node_id}").set_position(target_position, velocity=profile_velocity, is_pdo=True)
            getattr(self, f"motor_{node_id}").ready(is_pdo=True)

        for node_id in rope:
            getattr(self, f"motor_{node_id}").action(is_immediate=False, is_relative=True, is_pdo=True)

        if is_wait:
            duration_time = abs(target_position) / (profile_velocity * self.VELOCITY_RATIO) + 0.1
            time.sleep(duration_time)

    def rope_ready_position(self, *args: str):
        for node_id in args:
            getattr(self, f"motor_{node_id}").set_control_mode("position_control")
    
    def rope_move_abs_new(self, *args: tuple):
        id_list = []
        t_list = []
        for tuple in args:
            id, tar_l, vel = tuple
            id_list.append(id)
            
            tar_pos = int(round(self.rope_zero_position[int(id)-1] + (tar_l - self.rope_init_length[int(id)-1]) * self.ROPE_RATIO, 0))
            pro_vel = int(round(self.ROPE_RATIO * abs(vel) / self.VELOCITY_RATIO, 0))
            
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
            
            t = abs(tar_pos) / (pro_vel * self.VELOCITY_RATIO)
            t_list.append(t)

            getattr(self, f"motor_{id}").set_position(tar_pos, velocity=pro_vel, is_pdo=True)
            getattr(self, f"motor_{id}").ready(is_pdo=True)
        
        t_list.sort(reverse=True)
        delay = t_list[0]
        for id in id_list:
            getattr(self, f"motor_{id}").action(is_immediate=False, is_relative=True, is_pdo=True)
        time.sleep(delay)

    def rope_ready_speed(self, *args: str):
        for node_id in args:
            getattr(self, f"motor_{node_id}").set_control_mode("speed_control")
        for node_id in args:
            getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
        for node_id in args:
            getattr(self, f"motor_{node_id}").enable_operation(is_pdo=True)
    
    def rope_stop_speed(self, *args: str):
        for node_id in args:
            getattr(self, f"motor_{node_id}").disable_operation(is_pdo=True)
        for node_id in args:
            getattr(self, f"motor_{node_id}").set_speed(0, is_pdo=True)
    
    def rope_move_speed(self, *args: tuple):
        for tuple in args:
            node_id, speed = tuple
            target_speed = int(round(self.ROPE_RATIO * speed / self.VELOCITY_RATIO, 0))
            getattr(self, f"motor_{node_id}").set_speed(target_speed, is_pdo=True)

    def rope_move(self, rope: str, distance: float, velocity: float, /, *, is_relative: bool) -> None:
        self.rope_move_thread = RopeMove(rope, distance, velocity, is_relative=is_relative, robot=self)
        self.rope_move_thread.start()


    def forward(self):
        self.test_thread = Forward(self)
        self.test_thread.start()

    def force_set_zero(self, force_list: list, num, start, finish):
        self.force_zero_thread = ForceSetZero(force_list, num=num, robot=self, start_signal=start, finish_signal=finish)
        self.force_zero_thread.start()
    
    def back(self):
        self.back_thread = Back(robot=self)
        self.back_thread.start()

    def kinematics_test(self):
        self.kinematics_thread = Kinematics(self)
        self.kinematics_thread.start()
    def kinematics_test_stop(self):
        self.kinematics_thread.stop()
        self.kinematics_thread.wait()

    ''' 标定 初始状态 曲率0 长度已知 '''
    def calibration(self, bl_o: float, bl_m: float, bl_i: float):
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

        self.is_calibration = True

    ''' 正运动学 '''
    def actuator_to_config(self, l_1: float, l_2: float, l_3: float):
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
    def config_to_task(self, length: float, kappa: float, phi: float):
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
        
        coordinate = np.array([[0, 0, 0, 1]]).T

        position = np.matmul(transform, coordinate)
        x = position[0,0]
        y = position[1,0]
        z = position[2,0]

        return x, y, z

    ''' 逆运动学 '''
    def task_to_config(self, position: tuple):
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
    def config_to_actuator(self, section: str, s: float, kappa: float, phi: float):
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

    ''' 逆雅可比 '''
    def config_to_actuator_jacobian(self, section: str, s_d: float, kappa_d: float, phi_d: float):
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
            n = self.backbone_section_num[2]
            s = self.backbone_length[2]
            kappa = self.backbone_curvature[2]
            phi = self.backbone_rotation_angle[2]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_1_d = actuator_d[0, 0]
            l_2_d = actuator_d[1, 0]
            l_3_d = actuator_d[2, 0]
            return l_1_d, l_2_d, l_3_d
        
        elif section == "midside":
            n = self.backbone_section_num[1]
            s = self.backbone_length[1]
            kappa = self.backbone_curvature[1]
            phi = self.backbone_rotation_angle[1]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_4_d = actuator_d[0, 0]
            l_5_d = actuator_d[1, 0]
            l_6_d = actuator_d[2, 0]
            
            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_1_d = actuator_d[0, 0]
            l_2_d = actuator_d[1, 0]
            l_3_d = actuator_d[2, 0]

            return l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d
        
        elif section == "inside":
            n = self.backbone_section_num[0]
            s = self.backbone_length[0]
            kappa = self.backbone_curvature[0]
            phi = self.backbone_rotation_angle[0]

            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_7_d = actuator_d[0, 0]
            l_8_d = actuator_d[1, 0]
            l_9_d = actuator_d[2, 0]

            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_4_d = actuator_d[0, 0]
            l_5_d = actuator_d[1, 0]
            l_6_d = actuator_d[2, 0]

            phi += 40/180*pi
            actuator_d = np.matmul(jacobian(n, s, kappa, phi), config_d)
            l_1_d = actuator_d[0, 0]
            l_2_d = actuator_d[1, 0]
            l_3_d = actuator_d[2, 0]

            return l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d, l_7_d, l_8_d, l_9_d
        
        else: return 0

    ''' 逆雅可比 '''
    def task_to_config_jacobian(self, section: str, spatial_velocity: tuple):
        x_d = spatial_velocity[0]
        y_d = spatial_velocity[1]
        z_d = spatial_velocity[2]
        x_w = spatial_velocity[3]
        y_w = spatial_velocity[4]
        z_w = spatial_velocity[5]

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

        transform = np.array([
            [cos(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
            [sin(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
            [-(sin(kappa*s)-kappa*s)/pow(kappa,2),   0, 1              ],
            [-s*sin(phi),                            0, -kappa*sin(phi)],
            [s*cos(phi),                             0, kappa*cos(phi) ],
            [0,                                      1, 0              ],
        ])

        spatial_velocity_vector = np.array([[x_d, y_d, z_d, x_w, y_w, z_w]]).T

        config_vector = np.matmul(np.linalg.pinv(transform), spatial_velocity_vector)

        kappa_d = config_vector[0, 0]
        phi_d = config_vector[1, 0]
        s_d = config_vector[2, 0]

        return kappa_d, phi_d, s_d


    ''' configuration space '''
    def config_space(self, section: str, operation: str):
        if operation == "curve":
            self.config_space_thread = SingleSectionKinematics(self, section, config_d=(0, 0.001, 0))
        elif operation == "straighten":
            self.config_space_thread = SingleSectionKinematics(self, section, config_d=(0, -0.001, 0))
        elif operation == "rotate_clockwise":
            self.config_space_thread = SingleSectionKinematics(self, section, config_d=(0, 0, 0.25))
        elif operation == "rotate_anticlockwise":
            self.config_space_thread = SingleSectionKinematics(self, section, config_d=(0, 0, -0.25))
        elif operation == "reset":
            if section == "outside":
                self.config_space_thread = SingleSectionKinematics(self, "outside", config=(self.backbone_init_length[2], 0, 0))
            elif section == "midside":
                self.config_space_thread = SingleSectionKinematics(self, "midside", config=(self.backbone_init_length[1], 0, 0))
            elif section == "inside":
                self.config_space_thread = SingleSectionKinematics(self, "inside", config=(self.backbone_init_length[0], 0, 0))
        
        try: self.config_space_thread.start()
        except: print("\033[0;31m[Error] config_space_thread is not created\033[0m")
    def config_space_stop(self):
        try:
            self.config_space_thread.stop()
            self.config_space_thread.wait()
        except: print("\033[0;31m[Error] config_space_thread is not created\033[0m")



''' CANopen 接收 数据处理 '''
class CANopenUpdate(QThread):
    show_motor_status = pyqtSignal(int)
    show_motor_original = pyqtSignal(int)
    show_motor_mode = pyqtSignal(int)

    show_switch = pyqtSignal()

    status_signal = pyqtSignal(str)

    show_ballscrew = pyqtSignal(bool)
    show_rope = pyqtSignal(bool, int)

    show_kinematics = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot) -> None:
        super().__init__()

        self.robot = robot

        self.__is_stop = False
    
    def run(self):
        print("CANopen Update Thread Started")
        self.status_signal.emit("CANopen Update Thread Started !")
        
        while not self.__is_stop:
            ret = CanOpenBusProcessor.device.read_buffer(1, wait_time=0)
            
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
                                if self.robot.ballscrew_is_set_zero:
                                    self.robot.ballscrew_position = (self.robot.motor_10.zero_position - self.robot.motor_10.current_position) / 5120
                                    self.robot.ballscrew_velocity = self.robot.motor_10.current_speed * 440 / 5120

                                self.show_ballscrew.emit(self.robot.ballscrew_is_set_zero)
                            
                            else:
                                if self.robot.rope_is_set_zero:
                                    position = (getattr(self.robot, f"motor_{node_id}").current_position - getattr(self.robot, f"motor_{node_id}").zero_position) / 12536.512440
                                    setattr(self.robot, f"rope_{node_id}_position", position)

                                    # velocity = getattr(self.robot, f"motor_{node_id}").current_speed * 440 / 12536.512440
                                    # setattr(self.robot, f"rope_{node_id}_velocity", velocity)
                                
                                if self.robot.is_calibration:
                                    self.robot.rope_velocity[node_id-1] = speed * self.robot.VELOCITY_RATIO / self.robot.ROPE_RATIO
                                
                                self.show_rope.emit(self.robot.rope_is_set_zero, node_id)

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
                        
                        # print("OLD", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)

                        # wait_time = 1
                        # time_stamp = time.time()
                        # while CanOpenBusProcessor.node_dict[node_id].sdo_feedback[0] and time.time() - time_stamp < wait_time: time.sleep(0.1)
                        
                        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value_list)

                        # print("[SDO NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].sdo_feedback)
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

                        # print("old  ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        
                        CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
                        
                        # print("[NMT NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        print("\033[0;34m[NMT {}] {}\033[0m".format(node_id, CanOpenBusProcessor.node_dict[node_id].nmt_feedback))
                        self.status_signal.emit("[Node {}] Get NMT response, bus status is {}".format(node_id, label))
                   
            if self.robot.is_calibration:
                for i in range(1,10):
                    self.robot.rope_total_length[i-1] = self.robot.rope_init_length[i-1] + (getattr(self.robot, f"motor_{i}").current_position - self.robot.rope_zero_position[i-1]) / self.robot.ROPE_RATIO
                
                # inside
                for i in range(6,9):
                    self.robot.rope_inside_length[i] = self.robot.rope_total_length[i]
                
                s_in, kappa_in, phi_in = self.robot.actuator_to_config(self.robot.rope_inside_length[6], self.robot.rope_inside_length[7], self.robot.rope_inside_length[8])
                
                self.robot.backbone_length[0] = s_in
                self.robot.backbone_curvature[0] = kappa_in
                self.robot.backbone_rotation_angle[0] = phi_in

                # l_4, l_5, l_6 = self.robot.config_to_actuator(s_in, kappa_in, phi_in+40/180*pi)
                # l_1, l_2, l_3 = self.robot.config_to_actuator(s_in, kappa_in, phi_in+80/180*pi)
                l_1, l_2, l_3, l_4, l_5, l_6, l_7, l_8, l_9 = self.robot.config_to_actuator("inside", s_in, kappa_in, phi_in)
                for i in range(0,6):
                    exec("self.robot.rope_inside_length[{}] = l_{}".format(i, i+1))

                self.robot.inside_coordinate = self.robot.config_to_task(s_in, kappa_in, phi_in)
                
                # midside
                for i in range(3,6):
                    self.robot.rope_midside_length[i] = self.robot.rope_total_length[i] - self.robot.rope_inside_length[i]

                s_mid, kappa_mid, phi_mid = self.robot.actuator_to_config(self.robot.rope_midside_length[3], self.robot.rope_midside_length[4], self.robot.rope_midside_length[5])
                self.robot.backbone_length[1] = s_mid
                self.robot.backbone_curvature[1] = kappa_mid
                self.robot.backbone_rotation_angle[1] = phi_mid

                # l_1, l_2, l_3 = self.robot.config_to_actuator(s_mid, kappa_mid, phi_mid+40/180*pi)
                l_1, l_2, l_3, l_4, l_5, l_6 = self.robot.config_to_actuator("midside", s_mid, kappa_mid, phi_mid)
                for i in range(0,3):
                    exec("self.robot.rope_midside_length[{}] = l_{}".format(i, i+1))

                self.robot.midside_coordinate = self.robot.config_to_task(s_mid, kappa_mid, phi_mid)

                # outside
                for i in range(0,3):
                    self.robot.rope_outside_length[i] = self.robot.rope_total_length[i] - self.robot.rope_inside_length[i] - self.robot.rope_midside_length[i]

                s_out, kappa_out, phi_out = self.robot.actuator_to_config(self.robot.rope_outside_length[0], self.robot.rope_outside_length[1], self.robot.rope_outside_length[2])
                self.robot.backbone_length[2] = s_out
                self.robot.backbone_curvature[2] = kappa_out
                self.robot.backbone_rotation_angle[2] = phi_out
                
                self.robot.outside_coordinate = self.robot.config_to_task(s_out, kappa_out, phi_out)


                self.show_kinematics.emit()
                

        print("CANopen Update Thread Stopped")
        self.status_signal.emit("CANopen Update Thread Stopped.")
    
    def stop(self):
        self.__is_stop = True


    ''' hex列表转换为int '''
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
    
    ''' 匹配地址 '''
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

''' 传感器 解析数据 '''
class SensorResolve(QThread):
    show_force = pyqtSignal(int)
    
    def __init__(self) -> None:
        super().__init__()

        self.__is_stop = False
    
    def run(self):
        print("Sensor Resolve Thread Started")
        
        while not self.__is_stop:
            ret = Sensor.device.read_buffer(1, wait_time=0)

            if ret != None:
                [num, msg] = ret
                
                for i in range(num):
                    if msg[i].Data[0] == Sensor.msg[0] \
                        and msg[i].Data[1] == Sensor.msg[1] \
                        and msg[i].Data[6] == Sensor.msg[2] \
                        and msg[i].Data[7] == Sensor.msg[3]:
                        
                        # Sensor
                        if msg[i].ID in Sensor.sensor_dict.keys():
                            Sensor.sensor_dict[msg[i].ID].original_data = self.__hex_list_to_float([msg[i].Data[j] for j in range(2, 6)])

                            Sensor.sensor_dict[msg[i].ID].force = (self.__hex_list_to_float([msg[i].Data[j] for j in range(2, 6)]) - Sensor.sensor_dict[msg[i].ID].zero) / 2
                        
                            self.show_force.emit(msg[i].ID)
        
        print("Sensor Resolve Thread Stopped")

    def stop(self):
        self.__is_stop = True

        print("Stopping Sensor Resolve Thread")


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

''' 初始化 '''
class RobotInit(QThread):
    __running_signal = pyqtSignal(bool)
    __finish_signal = pyqtSignal()
    
    def __init__(self, /, *, times=1, running_signal=None, finish_signal=None) -> None:
        super().__init__()

        self.__motor_init_count = 0

        self.__io_init_count = 0

        self.__sensor_init_count = 0

        self.__times = times

        if running_signal != None: self.__running_signal.connect(running_signal)
        if finish_signal != None: self.__finish_signal.connect(finish_signal)
    
    def __init_motor(self) -> bool:
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
                            
                            self.__motor_init_count += 1
                            continue
            
            print("===============INIT MOTOR SHUT DOWN===============")
            return False
        
        return self.__motor_init_count == len(Motor.motor_dict)

    def __init_io(self) -> bool:
        for node_id in IoModule.io_dict:
            print("====================INIT IO====================")

            if IoModule.io_dict[node_id].check_bus_status() \
            and IoModule.io_dict[node_id].initialize_device(log=True) \
            and IoModule.io_dict[node_id].start_device(log=True):
                
                if IoModule.io_dict[node_id].close_valve_1() \
                and IoModule.io_dict[node_id].close_valve_2() \
                and IoModule.io_dict[node_id].close_valve_3() \
                and IoModule.io_dict[node_id].open_valve_4():
                    
                    self.__io_init_count += 1
                    continue
            
            print("===============INIT IO SHUT DOWN===============")
            return False
        
        return self.__io_init_count == len(IoModule.io_dict)
    
    def __init_sensor(self) -> bool:

        return self.__sensor_init_count != len(Sensor.sensor_dict)

    def run(self):
        self.__running_signal.emit(True)

        while self.__times != 0:
            if self.__init_motor() and self.__init_io() and self.__init_sensor():
                self.__finish_signal.emit()
                break
            else: self.__times -= 1
        
        else: self.__running_signal.emit(False)

''' 发送 读取请求 '''
class SensorRequest(QThread):
    def __init__(self) -> None:
        super().__init__()

        self.__is_stop = False
    
    def run(self):
        # clear_success = Sensor.device.clear_buffer()

        print("Sensor Request Sending")
        
        while not self.__is_stop:
            for sensor in Sensor.sensor_dict.values():
                send_success = sensor.send_request()

        print("Sensor Request Thread Stopped")

    def stop(self):
        self.__is_stop = True

        print("Stopping Sensor Request Thread")

''' 滚珠丝杠 调零 '''
class BallScrewSetZero(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, distance=50, velocity=200, speed=50, /, *, robot: ContinuumRobot, start_signal, finish_signal) -> None:
        super().__init__()

        self.__is_stop = False

        self.__distance = distance
        self.__velocity = velocity
        self.__speed = speed

        self.robot = robot

        self.__start_signal.connect(start_signal)
        self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.robot.ballscrew_is_set_zero = False
        self.__start_signal.emit()
        
        self.robot.io.open_valve_4()
        time.sleep(1)

        if self.__distance != 0:
            self.robot.motor_10.set_control_mode("position_control", check=False)
            time.sleep(0.01)

            d = abs(self.__distance)
            v = abs(self.__velocity)
            duration = (d * 5120) / (v * 440) + 0.5

            self.robot.motor_10.set_position(- int(d * 5120), velocity=v, is_pdo=True)

            self.robot.motor_10.ready(is_pdo=True)
            self.robot.motor_10.action(is_immediate=True, is_relative=True, is_pdo=True)

            duration = (d * 5120) / (v * 440) + 0.5
            time.sleep(duration)
        
        if not self.robot.io.input_1:
            self.robot.motor_10.set_control_mode("speed_control")
            time.sleep(0.01)
        
            self.robot.motor_10.set_speed(abs(self.__speed), is_pdo=True)

            self.robot.motor_10.halt(is_pdo=True)

            self.robot.motor_10.enable_operation(is_pdo=True)
        else: pass

        while not self.__is_stop:
            if self.robot.io.input_1:
                self.robot.motor_10.halt(is_pdo=True)

                time.sleep(0.5)

                self.robot.motor_10.zero_position = self.robot.motor_10.current_position
                self.robot.ballscrew_is_set_zero = True

                self.__finish_signal.emit()

                break
    
    def stop(self):
        self.__is_stop = True

        self.robot.motor_10.set_speed(0, is_pdo=True)
        self.robot.motor_10.disable_operation(is_pdo=True)

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

''' 滚珠丝杠 归零 '''
class BallScrewGoZero(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, speed=300, /, *, robot: ContinuumRobot, start_signal, finish_signal) -> None:
        super().__init__()

        self.__is_stop = False

        self.robot = robot

        self.__speed = speed

        self.__start_signal.connect(start_signal)
        self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()
        
        self.robot.io.open_valve_4()
        time.sleep(1)
        
        if not self.robot.io.input_1:
            self.robot.motor_10.set_control_mode("speed_control", check=False)
        
            self.robot.motor_10.set_speed(self.__speed, is_pdo=True)

            self.robot.motor_10.halt(is_pdo=True)

            self.robot.motor_10.enable_operation(is_pdo=True)
        else:
            self.__finish_signal.emit()
            return

        while not self.__is_stop:
            if self.robot.io.input_1:
                self.robot.motor_10.halt(is_pdo=True)

                self.__finish_signal.emit()

                break
    
    def stop(self):
        self.__is_stop = True

        self.robot.motor_10.set_speed(0, is_pdo=True)
        self.robot.motor_10.disable_operation(is_pdo=True)

''' 滚珠丝杠 移动 '''
class BallScrewMove(QThread):
    def __init__(self, distance: float, velocity: float, /, *, is_relative=False, is_close=False, robot: ContinuumRobot) -> None:
        super().__init__()

        self.__distance = distance
        self.__velocity = velocity
        self.__is_relative = is_relative
        self.__is_close = is_close

        self.robot = robot
    
    def run(self):
        if self.__is_close: self.robot.io.close_valve_4()
        else: self.robot.io.open_valve_4()

        self.robot.motor_10.set_control_mode("position_control", check=False)

        if self.__is_relative: self.robot.ballscrew_move_rel(self.__distance, velocity=self.__velocity)
        else: self.robot.ballscrew_move_abs(self.__distance, velocity=self.__velocity)

''' 线 移动 '''
class RopeMove(QThread):
    def __init__(self, rope: str, distance: float, velocity: float, /, *, is_relative=False, robot: ContinuumRobot) -> None:
        super().__init__()

        self.__rope = rope
        self.__dis = distance
        self.__vel = velocity
        self.__is_rel = is_relative
        self.robot = robot
    
    def run(self):
        for node_id in self.__rope:
            getattr(self.robot, f"motor_{node_id}").set_control_mode("position_control", check=False)
        
        if self.__is_rel: self.robot.rope_move_rel(self.__rope, distance=self.__dis, velocity=self.__vel)
        else: self.robot.rope_move_abs(self.__rope, point=self.__dis, velocity=self.__vel)

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

''' 传感器 调零 '''
class ForceSetZero(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()

    def __init__(self, force_ref_list: list, /, *, num=100, robot: ContinuumRobot, start_signal, finish_signal) -> None:
        super().__init__()

        self.__is_stop = False

        self.robot = robot

        self.__start_signal.connect(start_signal)
        self.__finish_signal.connect(finish_signal)
        
        self.ref_1 = - abs(force_ref_list[0])
        self.ref_2 = - abs(force_ref_list[1])
        self.ref_3 = - abs(force_ref_list[2])
        self.ref_4 = - abs(force_ref_list[3])
        self.ref_5 = - abs(force_ref_list[4])
        self.ref_6 = - abs(force_ref_list[5])
        self.ref_7 = - abs(force_ref_list[6])
        self.ref_8 = - abs(force_ref_list[7])
        self.ref_9 = - abs(force_ref_list[8])

        self.__num = num
    
    def run(self):
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

        self.__start_signal.emit()

        for i in range(9,0,-1):
            motor = getattr(self.robot, f"motor_{i}")
            sensor = getattr(self.robot, f"sensor_{i}")
            force_ref = getattr(self, f"ref_{i}")

            motor.enable_operation(is_pdo=True)

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
            
            sensor.set_zero(self.__num)

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
        
        self.__finish_signal.emit()


''' 伸 '''
class Forward(QThread):
    def __init__(self, robot: ContinuumRobot) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.robot = robot

        self.__inside_start = 348
        self.__inside_end = 358

        self.__midside_start = 227
        self.__midside_end = 237

        self.__outside_start = 100
        self.__midside_end = 110
    
    # def run(self):
    #     self.robot.io.open_valve_4()
    #     self.robot.io.open_valve_3()
    #     self.robot.io.open_valve_2()
        
    #     self.robot.motor_1.set_control_mode("speed_control", check=False)
    #     self.robot.motor_2.set_control_mode("speed_control", check=False)
    #     self.robot.motor_3.set_control_mode("speed_control", check=False)
    #     self.robot.motor_4.set_control_mode("speed_control", check=False)
    #     self.robot.motor_5.set_control_mode("speed_control", check=False)
    #     self.robot.motor_6.set_control_mode("speed_control", check=False)
    #     self.robot.motor_7.set_control_mode("speed_control", check=False)
    #     self.robot.motor_8.set_control_mode("speed_control", check=False)
    #     self.robot.motor_9.set_control_mode("speed_control", check=False)
    #     self.robot.motor_10.set_control_mode("position_control", check=False)

    #     self.robot.motor_1.set_speed(0, is_pdo=True)
    #     self.robot.motor_2.set_speed(0, is_pdo=True)
    #     self.robot.motor_3.set_speed(0, is_pdo=True)
    #     self.robot.motor_4.set_speed(0, is_pdo=True)
    #     self.robot.motor_5.set_speed(0, is_pdo=True)
    #     self.robot.motor_6.set_speed(0, is_pdo=True)
    #     self.robot.motor_7.set_speed(0, is_pdo=True)
    #     self.robot.motor_8.set_speed(0, is_pdo=True)
    #     self.robot.motor_9.set_speed(0, is_pdo=True)

    #     self.robot.motor_1.enable_operation(is_pdo=True)
    #     self.robot.motor_2.enable_operation(is_pdo=True)
    #     self.robot.motor_3.enable_operation(is_pdo=True)
    #     self.robot.motor_4.enable_operation(is_pdo=True)
    #     self.robot.motor_5.enable_operation(is_pdo=True)
    #     self.robot.motor_6.enable_operation(is_pdo=True)
    #     self.robot.motor_7.enable_operation(is_pdo=True)
    #     self.robot.motor_8.enable_operation(is_pdo=True)
    #     self.robot.motor_9.enable_operation(is_pdo=True)

    #     self.robot.ballscrew_move_abs(348, velocity=20)

    #     times = 13
    #     while not self.__is_stop and times != 0:
    #         self.robot.ballscrew_move_abs(348, velocity=10)

    #         self.robot.io.close_valve_4()
    #         self.robot.io.open_valve_1()

    #         self.robot.ballscrew_move_abs(358, velocity=2.5, is_wait=False)
    #         time.sleep(0.5)

    #         self.robot.motor_1.enable_operation(is_pdo=True)
    #         self.robot.motor_2.enable_operation(is_pdo=True)
    #         self.robot.motor_3.enable_operation(is_pdo=True)
    #         self.robot.motor_4.enable_operation(is_pdo=True)
    #         self.robot.motor_5.enable_operation(is_pdo=True)
    #         self.robot.motor_6.enable_operation(is_pdo=True)
    #         self.robot.motor_7.enable_operation(is_pdo=True)
    #         self.robot.motor_8.enable_operation(is_pdo=True)
    #         self.robot.motor_9.enable_operation(is_pdo=True)

    #         o_f = -7
    #         m_f = -6
    #         i_f = -5

    #         kp = 100
            
    #         while True:
    #             self.robot.motor_1.set_speed(int((o_f - self.robot.sensor_1.force) * kp), is_pdo=True)
    #             self.robot.motor_2.set_speed(int((o_f - self.robot.sensor_2.force) * kp), is_pdo=True)
    #             self.robot.motor_3.set_speed(int((o_f - self.robot.sensor_3.force) * kp), is_pdo=True)
    #             self.robot.motor_4.set_speed(int((m_f - self.robot.sensor_4.force) * kp), is_pdo=True)
    #             self.robot.motor_5.set_speed(int((m_f - self.robot.sensor_5.force) * kp), is_pdo=True)
    #             self.robot.motor_6.set_speed(int((m_f - self.robot.sensor_6.force) * kp), is_pdo=True)
    #             self.robot.motor_7.set_speed(int((i_f - self.robot.sensor_7.force) * kp), is_pdo=True)
    #             self.robot.motor_8.set_speed(int((i_f - self.robot.sensor_8.force) * kp), is_pdo=True)
    #             self.robot.motor_9.set_speed(int((i_f - self.robot.sensor_9.force) * kp), is_pdo=True)

    #             if abs(self.robot.ballscrew_position - 358) < 0.01:
    #                 self.robot.motor_1.set_speed(0, is_pdo=True)
    #                 self.robot.motor_2.set_speed(0, is_pdo=True)
    #                 self.robot.motor_3.set_speed(0, is_pdo=True)
    #                 self.robot.motor_4.set_speed(0, is_pdo=True)
    #                 self.robot.motor_5.set_speed(0, is_pdo=True)
    #                 self.robot.motor_6.set_speed(0, is_pdo=True)
    #                 self.robot.motor_7.set_speed(0, is_pdo=True)
    #                 self.robot.motor_8.set_speed(0, is_pdo=True)
    #                 self.robot.motor_9.set_speed(0, is_pdo=True)
    #                 break
                
    #         self.robot.motor_1.halt(is_pdo=True)
    #         self.robot.motor_2.halt(is_pdo=True)
    #         self.robot.motor_3.halt(is_pdo=True)
    #         self.robot.motor_4.halt(is_pdo=True)
    #         self.robot.motor_5.halt(is_pdo=True)
    #         self.robot.motor_6.halt(is_pdo=True)
    #         self.robot.motor_7.halt(is_pdo=True)
    #         self.robot.motor_8.halt(is_pdo=True)
    #         self.robot.motor_9.halt(is_pdo=True)

    #         self.robot.io.close_valve_1()
    #         self.robot.io.open_valve_4()
            
    #         times -= 1
        
    #     self.robot.motor_1.disable_operation(is_pdo=True)
    #     self.robot.motor_2.disable_operation(is_pdo=True)
    #     self.robot.motor_3.disable_operation(is_pdo=True)
    #     self.robot.motor_4.disable_operation(is_pdo=True)
    #     self.robot.motor_5.disable_operation(is_pdo=True)
    #     self.robot.motor_6.disable_operation(is_pdo=True)
    #     self.robot.motor_7.disable_operation(is_pdo=True)
    #     self.robot.motor_8.disable_operation(is_pdo=True)
    #     self.robot.motor_9.disable_operation(is_pdo=True)

    #     self.robot.ballscrew_move_abs(227, velocity=20)
    #     self.robot.io.open_valve_1()

    #     times = 13
    #     while not self.__is_stop and times != 0:
    #         self.robot.ballscrew_move_abs(227, velocity=10)

    #         self.robot.io.close_valve_4()
    #         self.robot.io.open_valve_2()

    #         self.robot.ballscrew_move_abs(237, velocity=2.5, is_wait=False)
    #         time.sleep(0.1)

    #         self.robot.motor_1.enable_operation(is_pdo=True)
    #         self.robot.motor_2.enable_operation(is_pdo=True)
    #         self.robot.motor_3.enable_operation(is_pdo=True)
    #         self.robot.motor_4.enable_operation(is_pdo=True)
    #         self.robot.motor_5.enable_operation(is_pdo=True)
    #         self.robot.motor_6.enable_operation(is_pdo=True)

    #         o_f = -4
    #         m_f = -4
    #         i_f = -4

    #         kp = 100
            
    #         while True:
    #             self.robot.motor_1.set_speed(int((o_f - self.robot.sensor_1.force) * kp), is_pdo=True)
    #             self.robot.motor_2.set_speed(int((o_f - self.robot.sensor_2.force) * kp), is_pdo=True)
    #             self.robot.motor_3.set_speed(int((o_f - self.robot.sensor_3.force) * kp), is_pdo=True)
    #             self.robot.motor_4.set_speed(int((m_f - self.robot.sensor_4.force) * kp), is_pdo=True)
    #             self.robot.motor_5.set_speed(int((m_f - self.robot.sensor_5.force) * kp), is_pdo=True)
    #             self.robot.motor_6.set_speed(int((m_f - self.robot.sensor_6.force) * kp), is_pdo=True)

    #             if abs(self.robot.ballscrew_position - 237) < 0.01: break
                
    #         self.robot.motor_1.halt(is_pdo=True)
    #         self.robot.motor_2.halt(is_pdo=True)
    #         self.robot.motor_3.halt(is_pdo=True)
    #         self.robot.motor_4.halt(is_pdo=True)
    #         self.robot.motor_5.halt(is_pdo=True)
    #         self.robot.motor_6.halt(is_pdo=True)

    #         self.robot.io.close_valve_2()
    #         self.robot.io.open_valve_4()
            
    #         times -= 1

    #     self.robot.motor_1.disable_operation(is_pdo=True)
    #     self.robot.motor_2.disable_operation(is_pdo=True)
    #     self.robot.motor_3.disable_operation(is_pdo=True)
    #     self.robot.motor_4.disable_operation(is_pdo=True)
    #     self.robot.motor_5.disable_operation(is_pdo=True)
    #     self.robot.motor_6.disable_operation(is_pdo=True)

    #     self.robot.ballscrew_move_abs(100, velocity=20)
    #     self.robot.io.open_valve_2()

    #     times = 0
    #     while not self.__is_stop and times != 0:
    #         self.robot.ballscrew_move_abs(100, velocity=10)

    #         self.robot.io.close_valve_4()
    #         self.robot.io.open_valve_3()

    #         self.robot.ballscrew_move_abs(110, velocity=2.5, is_wait=False)
    #         time.sleep(0.5)

    #         self.robot.motor_1.enable_operation(is_pdo=True)
    #         self.robot.motor_2.enable_operation(is_pdo=True)
    #         self.robot.motor_3.enable_operation(is_pdo=True)

    #         o_f = -4
    #         m_f = -4
    #         i_f = -4

    #         kp = 100
            
    #         while True:
    #             self.robot.motor_1.set_speed(int((o_f - self.robot.sensor_1.force) * kp), is_pdo=True)
    #             self.robot.motor_2.set_speed(int((o_f - self.robot.sensor_2.force) * kp), is_pdo=True)
    #             self.robot.motor_3.set_speed(int((o_f - self.robot.sensor_3.force) * kp), is_pdo=True)

    #             if abs(self.robot.ballscrew_position - 110) < 0.01: break
                
    #         self.robot.motor_1.halt(is_pdo=True)
    #         self.robot.motor_2.halt(is_pdo=True)
    #         self.robot.motor_3.halt(is_pdo=True)

    #         self.robot.io.close_valve_3()
    #         self.robot.io.open_valve_4()
            
    #         times -= 1

    #     self.robot.motor_1.disable_operation(is_pdo=True)
    #     self.robot.motor_2.disable_operation(is_pdo=True)
    #     self.robot.motor_3.disable_operation(is_pdo=True)

    def run(self):
        self.robot.io.open_valve_4()
        
        self.robot.motor_1.set_control_mode("position_control", check=False)
        self.robot.motor_2.set_control_mode("position_control", check=False)
        self.robot.motor_3.set_control_mode("position_control", check=False)
        self.robot.motor_4.set_control_mode("position_control", check=False)
        self.robot.motor_5.set_control_mode("position_control", check=False)
        self.robot.motor_6.set_control_mode("position_control", check=False)
        self.robot.motor_7.set_control_mode("position_control", check=False)
        self.robot.motor_8.set_control_mode("position_control", check=False)
        self.robot.motor_9.set_control_mode("position_control", check=False)
        self.robot.motor_10.set_control_mode("position_control", check=False)

        self.robot.ballscrew_move_abs(348, velocity=20)
        self.robot.io.open_valve_3()
        self.robot.io.open_valve_2()

        times = 15
        while not self.__is_stop and times != 0:
            self.robot.io.close_valve_4()
            self.robot.io.open_valve_1()

            self.robot.ballscrew_move_abs(358, velocity=5, is_wait=False)

            self.robot.rope_move_rel("123456789", distance=10, velocity=5)

            self.robot.io.close_valve_1()
            self.robot.io.open_valve_4()

            for i in range(9,0,-1):
                while getattr(self.robot, f"sensor_{i}").force > - 1.5:
                    self.robot.rope_move_rel(str(i), distance=-0.5, velocity=10)

            self.robot.ballscrew_move_abs(348, velocity=10)

            times -= 1
        
        self.robot.io.open_valve_1()
        
        self.robot.ballscrew_move_abs(227, velocity=20)

        times = 14
        while not self.__is_stop and times != 0:
            self.robot.io.close_valve_4()
            self.robot.io.open_valve_2()

            self.robot.ballscrew_move_abs(237, velocity=5, is_wait=False)

            self.robot.rope_move_rel("123456", distance=10, velocity=5)

            self.robot.io.close_valve_2()
            self.robot.io.open_valve_4()

            for i in range(6,0,-1):
                while getattr(self.robot, f"sensor_{i}").force > - 1.5:
                    self.robot.rope_move_rel(str(i), distance=-0.5, velocity=10)

            self.robot.ballscrew_move_abs(227, velocity=10)

            times -= 1
        
        self.robot.io.open_valve_2()

        self.robot.ballscrew_move_abs(100, velocity=20)

        times = 13
        while not self.__is_stop and times != 0:
            self.robot.io.close_valve_4()
            self.robot.io.open_valve_3()

            self.robot.ballscrew_move_abs(110, velocity=5, is_wait=False)

            self.robot.rope_move_rel("123", distance=10, velocity=5)

            self.robot.io.close_valve_3()
            self.robot.io.open_valve_4()

            for i in range(3,0,-1):
                while getattr(self.robot, f"sensor_{i}").force > - 1.5:
                    self.robot.rope_move_rel(str(i), distance=-0.5, velocity=10)

            self.robot.ballscrew_move_abs(100, velocity=10)

            times -= 1
        
        self.robot.io.close_valve_1()
        self.robot.io.close_valve_2()
        self.robot.io.close_valve_3()


    def stop(self):
        self.__is_stop = True

''' 缩 '''
class Back(QThread):
    def __init__(self, robot: ContinuumRobot) -> None:
        super().__init__()

        self.__is_stop = False

        self.robot = robot
    
    def run(self):
        self.robot.io.open_valve_4()
        
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

        self.robot.motor_1.enable_operation(is_pdo=True)
        self.robot.motor_2.enable_operation(is_pdo=True)
        self.robot.motor_3.enable_operation(is_pdo=True)
        self.robot.io.open_valve_3()
        
        flag_1, flag_2, flag_3 = False, False, False
        self.robot.motor_1.set_speed(-100, is_pdo=True)
        self.robot.motor_2.set_speed(-100, is_pdo=True)
        self.robot.motor_3.set_speed(-100, is_pdo=True)
        time.sleep(0.5)
        while True:
            if self.robot.sensor_1.force < -7:
                self.robot.motor_1.set_speed(0, is_pdo=True)
                flag_1 = True

            if self.robot.sensor_2.force < -7:
                self.robot.motor_2.set_speed(0, is_pdo=True)
                flag_2 = True
            
            if self.robot.sensor_3.force < -7:
                self.robot.motor_3.set_speed(0, is_pdo=True)
                flag_3 = True
            
            if flag_1 and flag_2 and flag_3: break
        
        self.robot.motor_4.enable_operation(is_pdo=True)
        self.robot.motor_5.enable_operation(is_pdo=True)
        self.robot.motor_6.enable_operation(is_pdo=True)
        self.robot.io.open_valve_2()
        
        flag_4, flag_5, flag_6 = False, False, False
        self.robot.motor_4.set_speed(-100, is_pdo=True)
        self.robot.motor_5.set_speed(-100, is_pdo=True)
        self.robot.motor_6.set_speed(-100, is_pdo=True)
        time.sleep(0.5)
        while True:
            if self.robot.sensor_4.force < -7:
                self.robot.motor_4.set_speed(0, is_pdo=True)
                flag_4 = True

            if self.robot.sensor_5.force < -7:
                self.robot.motor_5.set_speed(0, is_pdo=True)
                flag_5 = True

            if self.robot.sensor_6.force < -7:
                self.robot.motor_6.set_speed(0, is_pdo=True)
                flag_6 = True

            self.robot.motor_1.set_speed(int((-4 - self.robot.sensor_1.force) * 50), is_pdo=True)
            self.robot.motor_2.set_speed(int((-4 - self.robot.sensor_2.force) * 50), is_pdo=True)
            self.robot.motor_3.set_speed(int((-4 - self.robot.sensor_3.force) * 50), is_pdo=True)

            if flag_4 and flag_5 and flag_6:
                self.robot.motor_1.set_speed(0, is_pdo=True)
                self.robot.motor_2.set_speed(0, is_pdo=True)
                self.robot.motor_3.set_speed(0, is_pdo=True)
                break
        
        self.robot.motor_7.enable_operation(is_pdo=True)
        self.robot.motor_8.enable_operation(is_pdo=True)
        self.robot.motor_9.enable_operation(is_pdo=True)
        self.robot.io.open_valve_1()

        flag_7, flag_8, flag_9 = False, False, False
        self.robot.motor_7.set_speed(-100, is_pdo=True)
        self.robot.motor_8.set_speed(-100, is_pdo=True)
        self.robot.motor_9.set_speed(-100, is_pdo=True)
        time.sleep(0.5)
        while True:
            if self.robot.sensor_7.force < -7:
                self.robot.motor_7.set_speed(0, is_pdo=True)
                flag_7 = True

            if self.robot.sensor_8.force < -7:
                self.robot.motor_8.set_speed(0, is_pdo=True)
                flag_8 = True

            if self.robot.sensor_9.force < -7:
                self.robot.motor_9.set_speed(0, is_pdo=True)
                flag_9 = True

            self.robot.motor_1.set_speed(int((-4 - self.robot.sensor_1.force) * 50), is_pdo=True)
            self.robot.motor_2.set_speed(int((-4 - self.robot.sensor_2.force) * 50), is_pdo=True)
            self.robot.motor_3.set_speed(int((-4 - self.robot.sensor_3.force) * 50), is_pdo=True)
            self.robot.motor_4.set_speed(int((-4 - self.robot.sensor_4.force) * 50), is_pdo=True)
            self.robot.motor_5.set_speed(int((-4 - self.robot.sensor_5.force) * 50), is_pdo=True)
            self.robot.motor_6.set_speed(int((-4 - self.robot.sensor_6.force) * 50), is_pdo=True)

            if flag_7 and flag_8 and flag_9:
                self.robot.motor_1.set_speed(0, is_pdo=True)
                self.robot.motor_2.set_speed(0, is_pdo=True)
                self.robot.motor_3.set_speed(0, is_pdo=True)
                self.robot.motor_4.set_speed(0, is_pdo=True)
                self.robot.motor_5.set_speed(0, is_pdo=True)
                self.robot.motor_6.set_speed(0, is_pdo=True)
                break
        
        self.robot.motor_1.disable_operation(is_pdo=True)
        self.robot.motor_2.disable_operation(is_pdo=True)
        self.robot.motor_3.disable_operation(is_pdo=True)
        self.robot.motor_4.disable_operation(is_pdo=True)
        self.robot.motor_5.disable_operation(is_pdo=True)
        self.robot.motor_6.disable_operation(is_pdo=True)
        self.robot.motor_7.disable_operation(is_pdo=True)
        self.robot.motor_8.disable_operation(is_pdo=True)
        self.robot.motor_9.disable_operation(is_pdo=True)

        self.robot.io.close_valve_1()
        self.robot.io.close_valve_2()
        self.robot.io.close_valve_3()
    
    def stop(self):
        self.__is_stop = True






class SingleSectionKinematics(QThread):
    action = pyqtSignal()
    shutdown = pyqtSignal()
    
    def __init__(self, robot: ContinuumRobot, section=None, /, *, config_d=None, config=None) -> None:
        super().__init__()

        self.robot = robot
        self.__section = section

        if config_d != None:
            self.__s_d = config_d[0]
            self.__kappa_d = config_d[1]
            self.__phi_d = config_d[2]
            self.__mode = "config_d"
        elif config != None:
            self.__s = config[0]
            self.__kappa = config[1]
            self.__phi = config[2]
            self.__mode = "config"
        else: self.__mode = None

        self.__is_stop = False
    
    def run(self):
        self.action.emit()

        if self.__section == "outside":
            if self.__mode == "config_d":
                self.robot.rope_ready_speed("1","2","3")
                
                while not self.__is_stop:
                    l_1_d, l_2_d, l_3_d = self.robot.config_to_actuator_jacobian(self.__section, self.__s_d, self.__kappa_d, self.__phi_d)
                    self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d))
                
                self.robot.rope_stop_speed("1","2","3")

            elif self.__mode == "config":
                self.robot.rope_ready_position("1","2","3")
                
                l_1, l_2, l_3 = self.robot.config_to_actuator("outside", self.__s, self.__kappa, self.__phi)
                
                for i in range(3):
                    exec("l_{}_tar = l_{} + self.robot.rope_midside_length[{}] + self.robot.rope_inside_length[{}]".format(i+1, i+1, i, i))
                for i in range(3):
                    exec("i = {}".format(i))
                l_1_tar = l_1 + self.robot.rope_midside_length[0] + self.robot.rope_inside_length[0]
                l_2_tar = l_2 + self.robot.rope_midside_length[1] + self.robot.rope_inside_length[1]
                l_3_tar = l_3 + self.robot.rope_midside_length[2] + self.robot.rope_inside_length[2]

                l_1_cur = self.robot.rope_total_length[0]
                l_2_cur = self.robot.rope_total_length[1]
                l_3_cur = self.robot.rope_total_length[2]

                l_1_delta = abs(l_1_tar - l_1_cur)
                l_2_delta = abs(l_2_tar - l_2_cur)
                l_3_delta = abs(l_3_tar - l_3_cur)
                if l_1_delta >= l_2_delta and l_1_delta >= l_3_delta:
                    l_1_d = 1
                    l_2_d = l_1_d / l_1_delta * l_2_delta
                    l_3_d = l_1_d / l_1_delta * l_3_delta
                elif l_2_delta >= l_1_delta and l_2_delta >= l_3_delta:
                    l_2_d = 1
                    l_1_d = l_2_d / l_2_delta * l_1_delta
                    l_3_d = l_2_d / l_2_delta * l_3_delta
                elif l_3_delta >= l_1_delta and l_3_delta >= l_2_delta:
                    l_3_d = 1
                    l_1_d = l_3_d / l_3_delta * l_1_delta
                    l_2_d = l_3_d / l_3_delta * l_2_delta
                
                self.robot.rope_move_abs_new(("1", l_1_tar, l_1_d), ("2", l_2_tar, l_2_d), ("3", l_3_tar, l_3_d))
        
        elif self.__section == "midside":
            if self.__mode == "config_d":
                self.robot.rope_ready_speed("1","2","3","4","5","6")

                while not self.__is_stop:
                    l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d = self.robot.config_to_actuator_jacobian(self.__section, self.__s_d, self.__kappa_d, self.__phi_d)
                    self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d), ("4", l_4_d), ("5", l_5_d), ("6", l_6_d))
                
                self.robot.rope_stop_speed("1","2","3","4","5","6")
            
            elif self.__mode == "config":
                self.robot.rope_ready_position("1","2","3","4","5","6")
                
                l_1, l_2, l_3, l_4, l_5, l_6 = self.robot.config_to_actuator("midside", self.__s, self.__kappa, self.__phi)
                
                l_1_tar = l_1 + self.robot.rope_inside_length[0] + self.robot.rope_outside_length[0]
                l_2_tar = l_2 + self.robot.rope_inside_length[1] + self.robot.rope_outside_length[1]
                l_3_tar = l_3 + self.robot.rope_inside_length[2] + self.robot.rope_outside_length[2]
                l_4_tar = l_4 + self.robot.rope_inside_length[3]
                l_5_tar = l_5 + self.robot.rope_inside_length[4]
                l_6_tar = l_6 + self.robot.rope_inside_length[5]

                l_1_cur = self.robot.rope_total_length[0]
                l_2_cur = self.robot.rope_total_length[1]
                l_3_cur = self.robot.rope_total_length[2]
                l_4_cur = self.robot.rope_total_length[3]
                l_5_cur = self.robot.rope_total_length[4]
                l_6_cur = self.robot.rope_total_length[5]

                l_1_delta = abs(l_1_tar - l_1_cur)
                l_2_delta = abs(l_2_tar - l_2_cur)
                l_3_delta = abs(l_3_tar - l_3_cur)
                l_4_delta = abs(l_4_tar - l_4_cur)
                l_5_delta = abs(l_5_tar - l_5_cur)
                l_6_delta = abs(l_6_tar - l_6_cur)
                if 5*l_1_delta >= l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta:
                    l_1_d = 1
                    k = l_1_d / l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                elif 5*l_2_delta >= l_1_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta:
                    l_2_d = 1
                    k = l_2_d / l_2_delta
                    l_1_d = k * l_1_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                elif 5*l_3_delta >= l_1_delta + l_2_delta + l_4_delta + l_5_delta + l_6_delta:
                    l_3_d = 1
                    k = l_3_d / l_3_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                elif 5*l_4_delta >= l_1_delta + l_2_delta + l_3_delta + l_5_delta + l_6_delta:
                    l_4_d = 1
                    k = l_4_d / l_4_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                elif 5*l_5_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_6_delta:
                    l_5_d = 1
                    k = l_5_d / l_5_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_6_d = k * l_6_delta
                elif 5*l_6_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_5_delta:
                    l_6_d = 1
                    k = l_6_d / l_6_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                
                self.robot.rope_move_abs_new(("1", l_1_tar, l_1_d), ("2", l_2_tar, l_2_d), ("3", l_3_tar, l_3_d), ("4", l_4_tar, l_4_d), ("5", l_5_tar, l_5_d), ("6", l_6_tar, l_6_d))
        
        elif self.__section == "inside":
            if self.__mode == "config_d":
                self.robot.rope_ready_speed("1","2","3","4","5","6","7","8","9")
                
                while not self.__is_stop:
                    l_1_d, l_2_d, l_3_d, l_4_d, l_5_d, l_6_d, l_7_d, l_8_d, l_9_d = self.robot.config_to_actuator_jacobian("inside", self.__s_d, self.__kappa_d, self.__phi_d)
                    self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d), ("4", l_4_d), ("5", l_5_d), ("6", l_6_d), ("7", l_7_d), ("8", l_8_d), ("9", l_9_d))
                
                self.robot.rope_stop_speed("1","2","3","4","5","6","7","8","9")

            elif self.__mode == "config":
                self.robot.rope_ready_position("1","2","3","4","5","6","7","8","9")
                
                l_1, l_2, l_3, l_4, l_5, l_6, l_7, l_8, l_9 = self.robot.config_to_actuator("inside", self.__s, self.__kappa, self.__phi)
                
                l_1_tar = l_1 + self.robot.rope_midside_length[0] + self.robot.rope_outside_length[0]
                l_2_tar = l_2 + self.robot.rope_midside_length[1] + self.robot.rope_outside_length[1]
                l_3_tar = l_3 + self.robot.rope_midside_length[2] + self.robot.rope_outside_length[2]
                l_4_tar = l_4 + self.robot.rope_midside_length[3]
                l_5_tar = l_5 + self.robot.rope_midside_length[4]
                l_6_tar = l_6 + self.robot.rope_midside_length[5]
                l_7_tar = l_7
                l_8_tar = l_8
                l_9_tar = l_9

                l_1_cur = self.robot.rope_total_length[0]
                l_2_cur = self.robot.rope_total_length[1]
                l_3_cur = self.robot.rope_total_length[2]
                l_4_cur = self.robot.rope_total_length[3]
                l_5_cur = self.robot.rope_total_length[4]
                l_6_cur = self.robot.rope_total_length[5]
                l_7_cur = self.robot.rope_total_length[6]
                l_8_cur = self.robot.rope_total_length[7]
                l_9_cur = self.robot.rope_total_length[8]

                l_1_delta = abs(l_1_tar - l_1_cur)
                l_2_delta = abs(l_2_tar - l_2_cur)
                l_3_delta = abs(l_3_tar - l_3_cur)
                l_4_delta = abs(l_4_tar - l_4_cur)
                l_5_delta = abs(l_5_tar - l_5_cur)
                l_6_delta = abs(l_6_tar - l_6_cur)
                l_7_delta = abs(l_7_tar - l_7_cur)
                l_8_delta = abs(l_8_tar - l_8_cur)
                l_9_delta = abs(l_9_tar - l_9_cur)
                if 8*l_1_delta >= l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_1_d = 1
                    k = l_1_d / l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_2_delta >= l_1_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_2_d = 1
                    k = l_2_d / l_2_delta
                    l_1_d = k * l_1_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_3_delta >= l_1_delta + l_2_delta + l_4_delta + l_5_delta + l_6_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_3_d = 1
                    k = l_3_d / l_3_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_4_delta >= l_1_delta + l_2_delta + l_3_delta + l_5_delta + l_6_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_4_d = 1
                    k = l_4_d / l_4_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_5_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_6_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_5_d = 1
                    k = l_5_d / l_5_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_6_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_7_delta + l_8_delta + l_9_delta:
                    l_6_d = 1
                    k = l_6_d / l_6_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_7_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta + l_8_delta + l_9_delta:
                    l_7_d = 1
                    k = l_7_d / l_7_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_8_d = k * l_8_delta
                    l_9_d = k * l_9_delta
                elif 8*l_8_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta + l_7_delta + l_9_delta:
                    l_8_d = 1
                    k = l_8_d / l_8_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_9_d = k * l_9_delta
                elif 8*l_9_delta >= l_1_delta + l_2_delta + l_3_delta + l_4_delta + l_5_delta + l_6_delta + l_7_delta + l_8_delta:
                    l_9_d = 1
                    k = l_9_d / l_9_delta
                    l_1_d = k * l_1_delta
                    l_2_d = k * l_2_delta
                    l_3_d = k * l_3_delta
                    l_4_d = k * l_4_delta
                    l_5_d = k * l_5_delta
                    l_6_d = k * l_6_delta
                    l_7_d = k * l_7_delta
                    l_8_d = k * l_8_delta
                
                self.robot.rope_move_abs_new(("1", l_1_tar, l_1_d), ("2", l_2_tar, l_2_d), ("3", l_3_tar, l_3_d), ("4", l_4_tar, l_4_d), ("5", l_5_tar, l_5_d), ("6", l_6_tar, l_6_d), ("7", l_7_tar, l_7_d), ("8", l_8_tar, l_8_d), ("9", l_9_tar, l_9_d))
        
        self.shutdown.emit()
    
    def stop(self):
        self.__is_stop = True

class Kinematics(QThread):
    def __init__(self, robot: ContinuumRobot) -> None:
        super().__init__()

        self.robot = robot

        self.n = 10
        self.d = 2.5 # mm
        self.l = 172 # mm

        self.__is_stop = False

        self.test = (30, 30, 160)
    
    def config_to_task(self, kappa, length, phi):
        if kappa != 0:
            transform = np.array([
                [cos(phi)*cos(phi*length), -sin(phi), cos(phi)*sin(kappa*length), cos(phi)*(1-cos(kappa*length))/kappa], 
                [sin(phi)*cos(phi*length), cos(phi),  sin(phi)*sin(kappa*length), sin(phi)*(1-cos(kappa*length))/kappa],
                [-sin(kappa*length),       0,         cos(kappa*length),          sin(kappa*length)/kappa             ],
                [0,                        0,         0,                          1                                   ],
            ])
        else:
            transform = np.array([
                [cos(phi)*cos(phi*length), -sin(phi), cos(phi)*sin(kappa*length), 0     ], 
                [sin(phi)*cos(phi*length), cos(phi),  sin(phi)*sin(kappa*length), 0     ],
                [-sin(kappa*length),       0,         cos(kappa*length),          length],
                [0,                        0,         0,                          1     ],
            ])
        
        coordinate = np.array([[0, 0, 0, 1]]).T
        
        # print(transform)
        print(np.matmul(transform, coordinate))
    
    def config_to_actuator(self, kappa_dot, phi_dot, l_dot):
        # 第3节
        # l = self.robot.backbone_length[2]
        # kappa = self.robot.backbone_curvature[2]
        # phi = self.robot.backbone_rotation_angle[2]
        times = 300
        l = 172
        kappa = 0.005
        phi = 0
        while times != 0:
            param_1 = cos((kappa * l) / (2 * self.n))
            param_2 = sin((kappa * l) / (2 * self.n))
            param_3 = 2 * self.n / pow(kappa, 2)
            param_4 = 2 * self.n * self.d
            param_5 = 1 / kappa - self.d * sin(phi)
            param_6 = 1 / kappa + self.d * sin(pi/3 + phi)
            param_7 = 1 / kappa - self.d * cos(pi/6 + phi)
            
            transform = np.array([
                [l*param_1*param_5 - param_3*param_2, -param_4*param_2*cos(phi),     kappa*param_1*param_5],
                [l*param_1*param_6 - param_3*param_2, param_4*param_2*cos(pi/3+phi), kappa*param_1*param_6],
                [l*param_1*param_7 - param_3*param_2, param_4*param_2*sin(pi/6+phi), kappa*param_1*param_7]
            ])

            config_dot = np.array([[kappa_dot, phi_dot, l_dot]]).T

            actuator_dot = np.matmul(transform, config_dot)
            print("l_1_dot =", actuator_dot[0, 0])
            print("l_2_dot =", actuator_dot[1, 0])
            print("l_3_dot =", actuator_dot[2, 0])

            phi += 0.01
            times -= 1


        return actuator_dot
    
    def config_space_single(self, coordinate: tuple):
        x, y, z = coordinate

        if z > self.l or z <= 0: return

        if x > 0 and y == 0: phi = 0
        elif x > 0 and y > 0: phi = atan(y / x)
        elif x == 0 and y > 0: phi = pi / 2
        elif x < 0 and y > 0: phi = atan(y / x) + pi
        elif x < 0 and y == 0: phi = pi
        elif x < 0 and y < 0: phi = atan(y / x) + pi
        elif x == 0 and y < 0: phi = 1.5 * pi
        elif x > 0 and y < 0: phi = atan(y / x) + 2 * pi
        else: phi = None

        kappa = 2 * sqrt(pow(x,2)+pow(y,2)) / (pow(x,2)+pow(y,2)+pow(z,2))

        # theta = acos(1 - kappa * sqrt(pow(x,2)+pow(y,2))) if z > 0 else 2*pi - acos(1 - kappa * sqrt(pow(x,2)+pow(y,2)))

        print("phi =", phi)
        print("kappa =", kappa)

        if kappa != 0:
            l_1 = 2 * self.n * sin(kappa * self.l / 2 / self.n) * (1 / kappa - self.d * sin(phi))
            l_2 = 2 * self.n * sin(kappa * self.l / 2 / self.n) * (1 / kappa + self.d * sin(phi + pi/3))
            l_3 = 2 * self.n * sin(kappa * self.l / 2 / self.n) * (1 / kappa - self.d * cos(phi + pi/6))
        else:
            l_1 = z
            l_2 = z
            l_3 = z

        print("l_1 =", l_1, "delta =", l_1-self.l)
        print("l_2 =", l_2, "delta =", l_2-self.l)
        print("l_3 =", l_3, "delta =", l_3-self.l)

        self.actuator_to_config(l_1, l_2, l_3)

        return l_1, l_2, l_3
    
    def actuator_to_config(self, l_1, l_2, l_3):
        param_1 = sqrt(pow(l_1,2)+pow(l_2,2)+pow(l_3,2)-l_1*l_2-l_2*l_3-l_1*l_3)
        param_2 = l_1+l_2+l_3
        param_3 = l_3+l_2-2*l_1
        param_4 = l_2-l_3

        kappa = (2*param_1)/(self.d*param_2)
        if param_4 == 0:
            if param_3 > 0: phi = pi/2
            elif param_3 == 0: phi = 0
            elif param_3 < 0: phi = 3*pi/2
        elif param_4 > 0: phi = atan(1/sqrt(3)*param_3/param_4)
        elif param_4 < 0: phi = atan(1/sqrt(3)*param_3/param_4) + pi
        if param_1 == 0: s = param_2 / 3
        else: s = self.n*self.d*param_2/param_1*asin(param_1/(3*self.n*self.d))

        print("kappa =", kappa)
        print("phi =", phi)
        print("s =", s)

        return s, kappa, phi
    
    def config_to_actuator_jacobian(self, s_d: float, kappa_d: float, phi_d: float):
        n = 10
        d = 2.5

        s = 172
        kappa = 0.005
        phi = pi/2

        if kappa != 0:
            param_1 = cos((kappa * s) / (2 * n))
            param_2 = sin((kappa * s) / (2 * n))
            param_3 = 2 * n / pow(kappa, 2)
            param_4 = 2 * n * d
            param_5 = 1 / kappa - d * sin(phi)
            param_6 = 1 / kappa + d * sin(pi/3 + phi)
            param_7 = 1 / kappa - d * cos(pi/6 + phi)
            
            transform = np.array([
                [s*param_1*param_5 - param_3*param_2, -param_4*param_2*cos(phi),     kappa*param_1*param_5],
                [s*param_1*param_6 - param_3*param_2, param_4*param_2*cos(pi/3+phi), kappa*param_1*param_6],
                [s*param_1*param_7 - param_3*param_2, param_4*param_2*sin(pi/6+phi), kappa*param_1*param_7]
            ])
        else:
            transform = np.array([
                [-s*d*sin(phi),     0, 1],
                [s*d*sin(pi/3+phi), 0, 1],
                [s*d*sin(pi/6+phi), 0, 1],
            ])

        config_d = np.array([[kappa_d, phi_d, s_d]]).T

        actuator_d = np.matmul(transform, config_d)

        l_1_d = actuator_d[0, 0]
        l_2_d = actuator_d[1, 0]
        l_3_d = actuator_d[2, 0]

        return l_1_d, l_2_d, l_3_d

    def task_to_config_jacobian(self, spatial_velocity: tuple):
        x_d = spatial_velocity[0]
        y_d = spatial_velocity[1]
        z_d = spatial_velocity[2]
        x_w = spatial_velocity[3]
        y_w = spatial_velocity[4]
        z_w = spatial_velocity[5]

        s = 172
        kappa = 0.005
        phi = pi/2

        transform = np.array([
            [cos(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
            [sin(phi)*(cos(kappa*s)-1)/pow(kappa,2), 0, 0              ],
            [-(sin(kappa*s)-kappa*s)/pow(kappa,2),   0, 1              ],
            [-s*sin(phi),                            0, -kappa*sin(phi)],
            [s*cos(phi),                             0, kappa*cos(phi) ],
            [0,                                      1, 0              ],
        ])
        print(np.linalg.pinv(transform))

        spatial_velocity_vector = np.array([[x_d, y_d, z_d, x_w, y_w, z_w]]).T

        config_vector = np.matmul(np.linalg.pinv(transform), spatial_velocity_vector)

        kappa_d = config_vector[0, 0]
        phi_d = config_vector[1, 0]
        s_d = config_vector[2, 0]

        return kappa_d, phi_d, s_d
    
    # ''' 逆运动学 '''
    # def run(self):
    #     rope_1_current = self.l
    #     rope_2_current = self.l
    #     rope_3_current = self.l

    #     print("========== MOVE ==========")
    #     target_coordinate = self.test
    #     rope_1, rope_2, rope_3 = self.config_space_single(target_coordinate)

    #     delta_1 = rope_1 - rope_1_current
    #     delta_2 = rope_2 - rope_2_current
    #     delta_3 = rope_3 - rope_3_current

    #     self.robot.rope_move_rel("1", distance=delta_1, velocity=1, is_wait=False)
    #     self.robot.rope_move_rel("2", distance=delta_2, velocity=1*rope_2/rope_1, is_wait=False)
    #     self.robot.rope_move_rel("3", distance=delta_3, velocity=1*rope_3/rope_1, is_wait=False)

    #     rope_1_current = rope_1
    #     rope_2_current = rope_2
    #     rope_3_current = rope_3

    #     time.sleep(1)

    #     print("========== ZERO ==========")
    #     rope_1, rope_2, rope_3 = self.l, self.l, self.l

    #     delta_1 = rope_1 - rope_1_current
    #     delta_2 = rope_2 - rope_2_current
    #     delta_3 = rope_3 - rope_3_current

    #     self.robot.rope_move_rel("1", distance=delta_1, velocity=1, is_wait=False)
    #     self.robot.rope_move_rel("2", distance=delta_2, velocity=1*rope_2/rope_1, is_wait=False)
    #     self.robot.rope_move_rel("3", distance=delta_3, velocity=1*rope_3/rope_1, is_wait=False)

    #     rope_1_current = rope_1
    #     rope_2_current = rope_2
    #     rope_3_current = rope_3

    ''' 雅可比 '''
    def run(self):
        self.robot.motor_1.set_control_mode("speed_control")
        self.robot.motor_2.set_control_mode("speed_control")
        self.robot.motor_3.set_control_mode("speed_control")

        self.robot.motor_1.halt(is_pdo=True)
        self.robot.motor_2.halt(is_pdo=True)
        self.robot.motor_3.halt(is_pdo=True)

        self.robot.motor_1.set_speed(0, is_pdo=True)
        self.robot.motor_2.set_speed(0, is_pdo=True)
        self.robot.motor_3.set_speed(0, is_pdo=True)

        self.robot.motor_1.enable_operation(is_pdo=True)
        self.robot.motor_2.enable_operation(is_pdo=True)
        self.robot.motor_3.enable_operation(is_pdo=True)

        while not self.__is_stop:
            l_1_d, l_2_d, l_3_d = self.robot.config_to_actuator_jacobian(0, 0, 0.5)

            self.robot.rope_move_speed(("1", l_1_d), ("2", l_2_d), ("3", l_3_d))
        
        self.robot.motor_1.disable_operation(is_pdo=True)
        self.robot.motor_2.disable_operation(is_pdo=True)
        self.robot.motor_3.disable_operation(is_pdo=True)

        self.robot.motor_1.set_speed(0, is_pdo=True)
        self.robot.motor_2.set_speed(0, is_pdo=True)
        self.robot.motor_3.set_speed(0, is_pdo=True)
    
    def stop(self):
        self.__is_stop = True


if __name__ == "__main__":
    kin = Kinematics(None)
    # kin.config_space_single(kin.test)

    # kin.config_to_task(0.00514259,172,0.78539816)

    # kin.config_to_actuator(0.00000001, 0, 0)

    # kin.actuator_to_config(170,172,170)
    # kin.actuator_to_config(170,170,172)
    # kin.actuator_to_config(172,170,172)
    # kin.actuator_to_config(172,172,170)

    kappa_d, phi_d, s_d = kin.task_to_config_jacobian((1,1,-1,0,0,0))
    print("kappa_d =", kappa_d)
    print("phi_d =", phi_d)
    print("s_d =", s_d)
    l_1_d, l_2_d, l_3_d = kin.config_to_actuator_jacobian(s_d, kappa_d, phi_d)
    print("l_1_d =", l_1_d)
    print("l_2_d =", l_2_d)
    print("l_3_d =", l_3_d)