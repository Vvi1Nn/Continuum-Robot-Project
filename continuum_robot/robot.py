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



class ContinuumRobot():
    BALLSCREW_RATIO = 5120
    ROPE_RATIO = 12536.512440
    VELOCITY_RATIO = 440
    
    def __init__(self, /, *, 
                 update_output_status_slot_function, 
                 pdo_1_slot_function, 
                 pdo_2_slot_function, 
                 pdo_4_slot_function, 
                 status_signal, 
                 update_signal, 
                 ) -> None:
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

        self.io = IoModule(11, update_output_status_slot_function=update_output_status_slot_function)

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

        self.__read_canopen_thread = CANopenUpdate(pdo_1_slot_function=pdo_1_slot_function, pdo_2_slot_function=pdo_2_slot_function, pdo_4_slot_function=pdo_4_slot_function, status_signal=status_signal)
        self.__read_sensor_thread = SensorResolve(update_signal=update_signal)

    def open_device(self) -> bool:
        if UsbCan.open_device():
            
            if not self.usbcan_0_is_start and self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.__read_canopen_thread.start()
                self.usbcan_0_is_start = True

            if not self.usbcan_1_is_start and self.usbcan_1.init_can() and self.usbcan_1.start_can():
                self.__read_sensor_thread.start()
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


''' CANopen 接收 数据处理 '''
class CANopenUpdate(QThread):
    __pdo_1_update_signal = pyqtSignal(int)
    __pdo_2_update_signal = pyqtSignal(int)
    __pdo_4_update_signal = pyqtSignal(int)

    __status_signal = pyqtSignal(str)
    
    def __init__(self, /, *, pdo_1_slot_function=None, pdo_2_slot_function=None, pdo_4_slot_function=None, status_signal=None) -> None:
        super().__init__()
        self.__is_stop = False

        if pdo_1_slot_function != None: self.__pdo_1_update_signal.connect(pdo_1_slot_function)
        if pdo_2_slot_function != None: self.__pdo_2_update_signal.connect(pdo_2_slot_function)
        if pdo_4_slot_function != None: self.__pdo_4_update_signal.connect(pdo_4_slot_function)

        if status_signal != None: self.__status_signal.connect(status_signal)
    
    def run(self):
        print("CANopen Update Thread Started")
        self.__status_signal.emit("CANopen Update Thread Started !")
        
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
                        
                        # IO模块的ID
                        elif node_id in IoModule.io_dict.keys():
                            data_low = bin(msg[i].Data[0])[2:] # 首先转换为bin 去除0b
                            data_low = '0' * (8 - len(data_low)) + data_low # 头部补齐

                            data_high = bin(msg[i].Data[1])[2:]
                            data_high = '0' * (8 - len(data_high)) + data_high

                            data = data_high + data_low # 拼接

                            for i, c in enumerate(data):
                                setattr(IoModule.io_dict[node_id], f"input_{16-i}", False if c == "0" else True)
                        # 其他的ID
                        else: pass

                        self.__pdo_1_update_signal.emit(node_id)
                    
                    # TPDO2
                    elif msg[i].ID > 0x280 and msg[i].ID < 0x300:
                        node_id = msg[i].ID - 0x280
                        
                        # 电机的ID
                        if node_id in Motor.motor_dict.keys():
                            position = self.__hex_list_to_int([msg[i].Data[j] for j in range(0,4)]) # 当前位置
                            speed = self.__hex_list_to_int([msg[i].Data[j] for j in range(4,8)]) # 当前速度

                            Motor.motor_dict[node_id].current_position = position
                            Motor.motor_dict[node_id].current_speed = speed
                        
                        # 其他的ID
                        else: pass

                        self.__pdo_2_update_signal.emit(node_id)
                    
                    # TPDO4
                    elif msg[i].ID > 0x480 and msg[i].ID < 0x500:
                        node_id = msg[i].ID - 0x480
                        
                        # 电机的ID
                        if node_id in Motor.motor_dict.keys():
                            control_mode = self.__hex_list_to_int([msg[i].Data[0]])

                            for key in Motor.CONTROL_MODE:
                                if control_mode == Motor.CONTROL_MODE[key]:
                                    Motor.motor_dict[node_id].control_mode = key
                                    break
                        
                        # 其他的ID
                        else: pass

                        self.__pdo_4_update_signal.emit(node_id)
                    
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
                            else: pass
                        else: label = ""
                        
                        value_list = [msg[i].Data[j] for j in range(4,8)]
                        
                        # print("OLD", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)

                        # wait_time = 1
                        # time_stamp = time.time()
                        # while CanOpenBusProcessor.node_dict[node_id].sdo_feedback[0] and time.time() - time_stamp < wait_time: time.sleep(0.1)
                        
                        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value_list)

                        # print("[SDO NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].sdo_feedback)
                        print("\033[0;34m[SDO {}] {}\033[0m".format(node_id, CanOpenBusProcessor.node_dict[node_id].sdo_feedback))
                        self.__status_signal.emit("[Node {}] Get SDO response, object is {}, status is {}, value is {}".format(node_id, label, status, hex(self.__hex_list_to_int(value_list))))
                    
                    # NMT
                    elif msg[i].ID > 0x700 and msg[i].ID < 0x780:
                        node_id = msg[i].ID - 0x700

                        for key in CanOpenBusProcessor.NMT_STATUS:
                            if msg[0].Data[0] & 0b01111111 == CanOpenBusProcessor.NMT_STATUS[key]:
                                label = key
                                break
                            else: pass
                        else: label = ""

                        # print("old  ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        
                        CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
                        
                        # print("[NMT NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        print("\033[0;34m[NMT {}] {}\033[0m".format(node_id, CanOpenBusProcessor.node_dict[node_id].nmt_feedback))
                        self.__status_signal.emit("[Node {}] Get NMT response, bus status is {}".format(node_id, label))
                    
                    # 其他
                    else: pass

        print("CANopen Update Thread Stopped")
        self.__status_signal.emit("CANopen Update Thread Stopped.")
    
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
    __update_signal = pyqtSignal(int)
    
    def __init__(self, /, *, update_signal=None) -> None:
        super().__init__()

        self.__is_stop = False

        if update_signal != None: self.__update_signal.connect(update_signal)
    
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
                        
                            self.__update_signal.emit(msg[i].ID)
        
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

class Kinematics(QThread):
    def __init__(self, robot: ContinuumRobot) -> None:
        super().__init__()

        self.robot = robot

        self.n = 10
        self.d = 2.5 # mm
        self.l = 172 # mm

        self.test = (50, 50, 150)
    
    def config_to_task(self, kappa, length, phi):
        transform = np.array([
            [cos(phi)*cos(phi*length), -sin(phi), cos(phi)*sin(kappa*length), cos(phi)*(1-cos(kappa*length))/kappa], 
            [sin(phi)*cos(phi*length), cos(phi),  sin(phi)*sin(kappa*length), sin(phi)*(1-cos(kappa*length))/kappa],
            [-sin(kappa*length),       0,         cos(kappa*length),          sin(kappa*length)/kappa],
            [0,                        0,         0,                          1]
            ])
        
        coordinate = np.array([[0], [0], [0], [1]])
        
        # print(transform)
        print(np.matmul(transform, coordinate))
    
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

        return l_1, l_2, l_3
    
    def run(self):
        rope_1_current = self.l
        rope_2_current = self.l
        rope_3_current = self.l

        print("========== MOVE ==========")
        target_coordinate = self.test
        rope_1, rope_2, rope_3 = self.config_space_single(target_coordinate)

        delta_1 = rope_1 - rope_1_current
        delta_2 = rope_2 - rope_2_current
        delta_3 = rope_3 - rope_3_current

        self.robot.rope_move_rel("1", distance=delta_1, velocity=1, is_wait=False)
        self.robot.rope_move_rel("2", distance=delta_2, velocity=1*rope_2/rope_1, is_wait=False)
        self.robot.rope_move_rel("3", distance=delta_3, velocity=1*rope_3/rope_1, is_wait=False)

        rope_1_current = rope_1
        rope_2_current = rope_2
        rope_3_current = rope_3

        time.sleep(1)

        print("========== ZERO ==========")
        rope_1, rope_2, rope_3 = self.l, self.l, self.l

        delta_1 = rope_1 - rope_1_current
        delta_2 = rope_2 - rope_2_current
        delta_3 = rope_3 - rope_3_current

        self.robot.rope_move_rel("1", distance=delta_1, velocity=1, is_wait=False)
        self.robot.rope_move_rel("2", distance=delta_2, velocity=1*rope_2/rope_1, is_wait=False)
        self.robot.rope_move_rel("3", distance=delta_3, velocity=1*rope_3/rope_1, is_wait=False)

        rope_1_current = rope_1
        rope_2_current = rope_2
        rope_3_current = rope_3


    
if __name__ == "__main__":
    kin = Kinematics(None)
    kin.config_space_single(kin.test)

    kin.config_to_task(0.00514259,172,0.78539816)