# -*- coding:utf-8 -*-


''' gui.py GUI v4.3 '''


from PyQt5.QtWidgets import QMainWindow
from PyQt5.QtCore import QThread, pyqtSignal, QMutex


# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from continuum_robot.control_panel import Ui_MainWindow as Ui_ControlPanel

from continuum_robot.usbcan import UsbCan
from continuum_robot.processor import CanOpenBusProcessor
from continuum_robot.motor import Motor
from continuum_robot.io import IoModule
from continuum_robot.sensor import Sensor


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

                        print("[SDO NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].sdo_feedback)
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
                        
                        print("[NMT NEW {}] ".format(node_id), CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
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

''' 解析数据 '''
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



''' 初始化机器人 '''
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
                        and Motor.motor_dict[node_id].set_profile_acceleration(100) \
                        and Motor.motor_dict[node_id].set_profile_deceleration(100):
                            
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
    
    def __init__(self, distance=50, velocity=200, speed=50, /, *, motor: Motor, io: IoModule, start_signal, finish_signal) -> None:
        super().__init__()

        self.__is_stop = False

        self.__distance = distance
        self.__velocity = velocity
        self.__speed = speed

        self.__motor = motor
        self.__io = io

        self.__start_signal.connect(start_signal)
        self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()
        
        self.__io.open_valve_4()
        time.sleep(1)

        if self.__distance != 0:
            self.__motor.set_control_mode("position_control", check=False)
            time.sleep(0.01)

            d = abs(self.__distance)
            v = abs(self.__velocity)
            duration = (d * 5120) / (v * 440) + 0.5

            self.__motor.set_position(- int(d * 5120), velocity=v, is_pdo=True)

            self.__motor.ready(is_pdo=True)
            self.__motor.action(is_immediate=True, is_relative=True, is_pdo=True)

            duration = (d * 5120) / (v * 440) + 0.5
            time.sleep(duration)
        
        if not self.__io.input_1:
            self.__motor.set_control_mode("speed_control")
            time.sleep(0.01)
        
            self.__motor.set_speed(abs(self.__speed), is_pdo=True)

            self.__motor.halt(is_pdo=True)

            self.__motor.enable_operation(is_pdo=True)
        else: pass

        while not self.__is_stop:
            if self.__io.input_1:
                self.__motor.halt(is_pdo=True)

                time.sleep(0.5)

                self.__motor.zero_position = self.__motor.current_position

                self.__finish_signal.emit()

                break
    
    def stop(self):
        self.__is_stop = True

        self.__motor.set_speed(0, is_pdo=True)
        self.__motor.disable_operation(is_pdo=True)


''' 连续体 调整 '''
class ContinuumAttitudeAdjust(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, 
                 m1: Motor, m2: Motor, m3: Motor, m4: Motor, m5: Motor, m6: Motor, m7: Motor, m8: Motor, m9: Motor, 
                 s1: Sensor, s2: Sensor, s3: Sensor, s4: Sensor, s5: Sensor, s6: Sensor, s7: Sensor, s8: Sensor, s9: Sensor, 
                 io: IoModule, 
                 /, *, i_f: int, m_f: int, o_f: int, 
                 i_pid: tuple, m_pid: tuple, o_pid: tuple, 
                 start_signal=None, finish_signal=None) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.__motor_1 = m1
        self.__motor_2 = m2
        self.__motor_3 = m3
        self.__motor_4 = m4
        self.__motor_5 = m5
        self.__motor_6 = m6
        self.__motor_7 = m7
        self.__motor_8 = m8
        self.__motor_9 = m9

        self.__sensor_1 = s1
        self.__sensor_2 = s2
        self.__sensor_3 = s3
        self.__sensor_4 = s4
        self.__sensor_5 = s5
        self.__sensor_6 = s6
        self.__sensor_7 = s7
        self.__sensor_8 = s8
        self.__sensor_9 = s9

        self.__io = io

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
        self.__io.open_valve_3()
        time.sleep(0.5)
        self.__io.open_valve_2()
        time.sleep(0.5)
        self.__io.open_valve_1()
        time.sleep(0.5)

    def close_valve(self):
        self.__io.close_valve_1()
        time.sleep(0.5)
        self.__io.close_valve_2()
        time.sleep(0.5)
        self.__io.close_valve_3()
        time.sleep(0.5)

    def force_follow(self):
        self.__motor_1.set_control_mode("speed_control", check=False)
        self.__motor_2.set_control_mode("speed_control", check=False)
        self.__motor_3.set_control_mode("speed_control", check=False)
        self.__motor_4.set_control_mode("speed_control", check=False)
        self.__motor_5.set_control_mode("speed_control", check=False)
        self.__motor_6.set_control_mode("speed_control", check=False)
        self.__motor_7.set_control_mode("speed_control", check=False)
        self.__motor_8.set_control_mode("speed_control", check=False)
        self.__motor_9.set_control_mode("speed_control", check=False)

        self.__motor_1.set_speed(0, is_pdo=True)
        self.__motor_2.set_speed(0, is_pdo=True)
        self.__motor_3.set_speed(0, is_pdo=True)
        self.__motor_4.set_speed(0, is_pdo=True)
        self.__motor_5.set_speed(0, is_pdo=True)
        self.__motor_6.set_speed(0, is_pdo=True)
        self.__motor_7.set_speed(0, is_pdo=True)
        self.__motor_8.set_speed(0, is_pdo=True)
        self.__motor_9.set_speed(0, is_pdo=True)

        self.__motor_1.halt(is_pdo=True)
        self.__motor_2.halt(is_pdo=True)
        self.__motor_3.halt(is_pdo=True)
        self.__motor_4.halt(is_pdo=True)
        self.__motor_5.halt(is_pdo=True)
        self.__motor_6.halt(is_pdo=True)
        self.__motor_7.halt(is_pdo=True)
        self.__motor_8.halt(is_pdo=True)
        self.__motor_9.halt(is_pdo=True)

        self.__motor_1.enable_operation(is_pdo=True)
        self.__motor_2.enable_operation(is_pdo=True)
        self.__motor_3.enable_operation(is_pdo=True)
        self.__motor_4.enable_operation(is_pdo=True)
        self.__motor_5.enable_operation(is_pdo=True)
        self.__motor_6.enable_operation(is_pdo=True)
        self.__motor_7.enable_operation(is_pdo=True)
        self.__motor_8.enable_operation(is_pdo=True)
        self.__motor_9.enable_operation(is_pdo=True)

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

            self.__error_1[1] = self.__outside_reference_force - self.__sensor_1.force
            self.__error_2[1] = self.__outside_reference_force - self.__sensor_2.force
            self.__error_3[1] = self.__outside_reference_force - self.__sensor_3.force

            self.__error_4[1] = self.__midside_reference_force - self.__sensor_4.force
            self.__error_5[1] = self.__midside_reference_force - self.__sensor_5.force
            self.__error_6[1] = self.__midside_reference_force - self.__sensor_6.force

            self.__error_7[1] = self.__inside_reference_force - self.__sensor_7.force
            self.__error_8[1] = self.__inside_reference_force - self.__sensor_8.force
            self.__error_9[1] = self.__inside_reference_force - self.__sensor_9.force

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

            self.__motor_1.set_speed(int(speed_1), is_pdo=True, log=False)
            self.__motor_2.set_speed(int(speed_2), is_pdo=True, log=False)
            self.__motor_3.set_speed(int(speed_3), is_pdo=True, log=False)
            self.__motor_4.set_speed(int(speed_4), is_pdo=True, log=False)
            self.__motor_5.set_speed(int(speed_5), is_pdo=True, log=False)
            self.__motor_6.set_speed(int(speed_6), is_pdo=True, log=False)
            self.__motor_7.set_speed(int(speed_7), is_pdo=True, log=False)
            self.__motor_8.set_speed(int(speed_8), is_pdo=True, log=False)
            self.__motor_9.set_speed(int(speed_9), is_pdo=True, log=False)
        
        self.__motor_1.set_speed(0, is_pdo=True)
        self.__motor_2.set_speed(0, is_pdo=True)
        self.__motor_3.set_speed(0, is_pdo=True)
        self.__motor_4.set_speed(0, is_pdo=True)
        self.__motor_5.set_speed(0, is_pdo=True)
        self.__motor_6.set_speed(0, is_pdo=True)
        self.__motor_7.set_speed(0, is_pdo=True)
        self.__motor_8.set_speed(0, is_pdo=True)
        self.__motor_9.set_speed(0, is_pdo=True)

        self.__motor_1.disable_operation(is_pdo=True)
        self.__motor_2.disable_operation(is_pdo=True)
        self.__motor_3.disable_operation(is_pdo=True)
        self.__motor_4.disable_operation(is_pdo=True)
        self.__motor_5.disable_operation(is_pdo=True)
        self.__motor_6.disable_operation(is_pdo=True)
        self.__motor_7.disable_operation(is_pdo=True)
        self.__motor_8.disable_operation(is_pdo=True)
        self.__motor_9.disable_operation(is_pdo=True)

    def set_zero(self):
        self.__motor_1.zero_position = self.__motor_1.current_position
        self.__motor_2.zero_position = self.__motor_2.current_position
        self.__motor_3.zero_position = self.__motor_3.current_position
        self.__motor_4.zero_position = self.__motor_4.current_position
        self.__motor_5.zero_position = self.__motor_5.current_position
        self.__motor_6.zero_position = self.__motor_6.current_position
        self.__motor_7.zero_position = self.__motor_7.current_position
        self.__motor_8.zero_position = self.__motor_8.current_position
        self.__motor_9.zero_position = self.__motor_9.current_position
    
    def stop(self):
        self.__is_stop = True



''' 速度模式 '''
class JointControlSpeedModeThread(QThread):
    def __init__(self, motor, speed: int, /, *, is_forward: bool) -> None:
        super().__init__()

        self.__is_stop = False

        self.__motor = motor

        self.__is_forward = is_forward

        if self.__is_forward: self.__speed = speed
        else: self.__speed = - speed
    
    def run(self):
        self.__motor.set_control_mode("speed_control")
        
        self.__motor.set_speed(self.__speed, is_pdo=True)

        self.__motor.halt(is_pdo=True)
        
        while not self.__is_stop:
            if self.__motor.is_in_range():
                self.__motor.enable_operation(is_pdo=True)
            else:
                if self.__motor.current_position > self.__motor.max_position:
                    if self.__is_forward: self.__motor.halt(is_pdo=True)
                    else: self.__motor.enable_operation(is_pdo=True)
                else:
                    if not self.__is_forward: self.__motor.halt(is_pdo=True)
                    else: self.__motor.enable_operation(is_pdo=True)
    
    def stop(self):
        self.__is_stop = True
        self.__motor.set_speed(0, is_pdo=True)
        self.__motor.disable_operation(is_pdo=True)
        self.__motor.set_control_mode("position_control")





''' 电机10 归零 '''
class BallScrewGoZeroThread(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, speed=300, /, *, motor: Motor, io: IoModule, start_signal, finish_signal) -> None:
        super().__init__()

        self.__is_stop = False

        self.__motor = motor
        self.__io = io

        self.__speed = speed

        self.__start_signal.connect(start_signal)
        self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()
        
        self.__io.open_valve_4()
        time.sleep(1)
        
        if not self.__io.input_1:
            while not self.__motor.set_control_mode("speed_control"): time.sleep(0.1)
        
            self.__motor.set_speed(self.__speed, is_pdo=True)

            self.__motor.halt(is_pdo=True)

            self.__motor.enable_operation(is_pdo=True)
        else:
            self.__finish_signal.emit()
            return

        while not self.__is_stop:
            if self.__io.input_1:
                self.__motor.halt(is_pdo=True)

                self.__finish_signal.emit()

                break
    
    def stop(self):
        self.__is_stop = True

        self.__motor.set_speed(0, is_pdo=True)
        self.__motor.disable_operation(is_pdo=True)

''' 电机10 移动 '''
class BallScrewMoveThread(QThread):
    __start_signal = pyqtSignal()
    __finish_signal = pyqtSignal()
    
    def __init__(self, distance: int, /, *, velocity=100, is_relative=False, is_close=False, motor: Motor, io: IoModule, start_signal=None, finish_signal=None) -> None:
        super().__init__()

        self.__distance = distance
        self.__velocity = velocity
        self.__is_relative = is_relative
        self.__is_close = is_close

        self.__motor = motor
        self.__io = io

        if start_signal != None: self.__start_signal.connect(start_signal)
        if finish_signal != None: self.__finish_signal.connect(finish_signal)
    
    def run(self):
        self.__start_signal.emit()
        
        if self.__is_close: self.__io.close_valve_4()
        else: self.__io.open_valve_4()
        # time.sleep(0.5)

        self.__motor.set_control_mode("position_control", check=False)

        if self.__is_relative:
            position = - self.__distance * 5120
            self.__motor.set_position(position, velocity=self.__velocity, is_pdo=True)
        else:
            if self.__distance > 0:
                position = self.__motor.zero_position - self.__distance * 5120
            else: return

        self.__motor.set_position(position, velocity=self.__velocity, is_pdo=True)
        
        self.__motor.ready(is_pdo=True)

        self.__motor.action(is_immediate=True, is_relative=self.__is_relative, is_pdo=True)

        if self.__is_relative: time.sleep(self.__distance * 0.2)

        while not self.__is_relative:
            time.sleep(0.1)
            if abs(self.__motor.current_position - position) < 10000: break
            
        self.__finish_signal.emit()

''' 电机1-9 移动 '''
class RopeMoveThread(QThread):
    RATIO = 12536.512440

    def __init__(self, *args: tuple) -> None:
        super().__init__()

        self.__motor_list = []

        for object in args:
            self.__motor_list.append(object)
            
            motor = object[0]
            motor.set_control_mode("position_control")
    
    def run(self):
        for object in self.__motor_list:
            motor = object[0]
            is_rel = object[1]
            p = object[2]
            v = object[3]

            distance = int(p * RopeMoveThread.RATIO) if is_rel else motor.zero_position + int(p * RopeMoveThread.RATIO)

            motor.set_position(distance, velocity=v, is_pdo=True)

            motor.ready(is_pdo=True)

        for object in self.__motor_list:
            motor = object[0]
            is_rel = object[1]

            motor.action(is_immediate=False, is_relative=is_rel, is_pdo=True)

        for object in self.__motor_list:
            motor = object[0]

            while True:
                last_position = motor.current_position
                time.sleep(0.01)
                if last_position == motor.current_position: break









''' 控制界面 '''
class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)

        self.usbcan_0 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("0")
        self.usbcan_1 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("1")

        self.usbcan_0.set_timer("250K")
        self.usbcan_1.set_timer("1000K")

        self.__usbcan_0_is_start = False
        self.__usbcan_1_is_start = False

        CanOpenBusProcessor.link_device(self.usbcan_0)
        Sensor.link_device(self.usbcan_1)

        self.motor_1 = Motor(1, speed_range=[-200,200])
        self.motor_2 = Motor(2, speed_range=[-200,200])
        self.motor_3 = Motor(3, speed_range=[-200,200])
        self.motor_4 = Motor(4, speed_range=[-200,200])
        self.motor_5 = Motor(5, speed_range=[-200,200])
        self.motor_6 = Motor(6, speed_range=[-200,200])
        self.motor_7 = Motor(7, speed_range=[-200,200])
        self.motor_8 = Motor(8, speed_range=[-200,200])
        self.motor_9 = Motor(9, speed_range=[-200,200])
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

        self.io = IoModule(11, update_output_status_slot_function=self.show_valve_status)

        self.ballscrew_is_set_zero = False
        self.ballscrew_position = None # mm
        self.ballscrew_is_moving = False

        self.rope_move_mutex = QMutex()

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

        self.ui.test_4.clicked.connect(lambda: self.ballscrew_move(221, velocity=300))
        self.ui.test_5.clicked.connect(lambda: self.ballscrew_move(237, velocity=10))
        self.ui.test_6.clicked.connect(lambda: self.ballscrew_move(348))

        self.ui.test_7.clicked.connect(lambda: self.ballscrew_move(358))

        self.ui.test_8.clicked.connect(self.force_test)
        self.ui.test_9.clicked.connect(self.force_test_stop)

        self.ui.test_11.clicked.connect(lambda: self.rope_move(5))
        self.ui.test_12.clicked.connect(lambda: self.rope_move(-5))

        self.ui.test_13.clicked.connect(self.test_move)
        
        self.show() # 显示界面


    ''' Jumping '''
    def signal_connect_slot(self) -> None:
        ''' 菜单 '''
        self.ui.control_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.set_zero_panel.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))

        self.ui.statusBar.setSizeGripEnabled(False)
        self.ui.statusBar.showMessage("Welcome to Continnum Robot Control Panel", 10000)
        
        ''' 打开设备 '''
        self.ui.bt_open_device.setEnabled(True)
        self.ui.bt_open_device.setText("Open Device")
        self.ui.bt_open_device.clicked.connect(self.open_device)

        ''' 初始化机器人 '''
        self.ui.bt_init_robot.setEnabled(False)
        self.ui.bt_init_robot.setText("Initialize Robot")
        self.ui.bt_init_robot.clicked.connect(self.initialize_robot)

        ''' 界面 '''
        self.ui.control.setEnabled(False)
        self.ui.control_all.setEnabled(False)
        self.ui.status.setEnabled(False)
        self.ui.param.setEnabled(False)

        ''' 电机 状态控制 '''
        self.ui.shut_down_1.clicked.connect(lambda: self.motor_1.shut_down(is_pdo=True))
        self.ui.shut_down_2.clicked.connect(lambda: self.motor_2.shut_down(is_pdo=True))
        self.ui.shut_down_3.clicked.connect(lambda: self.motor_3.shut_down(is_pdo=True))
        self.ui.shut_down_4.clicked.connect(lambda: self.motor_4.shut_down(is_pdo=True))
        self.ui.shut_down_5.clicked.connect(lambda: self.motor_5.shut_down(is_pdo=True))
        self.ui.shut_down_6.clicked.connect(lambda: self.motor_6.shut_down(is_pdo=True))
        self.ui.shut_down_7.clicked.connect(lambda: self.motor_7.shut_down(is_pdo=True))
        self.ui.shut_down_8.clicked.connect(lambda: self.motor_8.shut_down(is_pdo=True))
        self.ui.shut_down_9.clicked.connect(lambda: self.motor_9.shut_down(is_pdo=True))
        self.ui.shut_down_10.clicked.connect(lambda: self.motor_10.shut_down(is_pdo=True))

        self.ui.switch_on_1.clicked.connect(lambda: self.motor_1.switch_on(is_pdo=True))
        self.ui.switch_on_2.clicked.connect(lambda: self.motor_2.switch_on(is_pdo=True))
        self.ui.switch_on_3.clicked.connect(lambda: self.motor_3.switch_on(is_pdo=True))
        self.ui.switch_on_4.clicked.connect(lambda: self.motor_4.switch_on(is_pdo=True))
        self.ui.switch_on_5.clicked.connect(lambda: self.motor_5.switch_on(is_pdo=True))
        self.ui.switch_on_6.clicked.connect(lambda: self.motor_6.switch_on(is_pdo=True))
        self.ui.switch_on_7.clicked.connect(lambda: self.motor_7.switch_on(is_pdo=True))
        self.ui.switch_on_8.clicked.connect(lambda: self.motor_8.switch_on(is_pdo=True))
        self.ui.switch_on_9.clicked.connect(lambda: self.motor_9.switch_on(is_pdo=True))
        self.ui.switch_on_10.clicked.connect(lambda: self.motor_10.switch_on(is_pdo=True))

        self.ui.enable_operation_1.clicked.connect(lambda: self.motor_1.enable_operation(is_pdo=True))
        self.ui.enable_operation_2.clicked.connect(lambda: self.motor_2.enable_operation(is_pdo=True))
        self.ui.enable_operation_3.clicked.connect(lambda: self.motor_3.enable_operation(is_pdo=True))
        self.ui.enable_operation_4.clicked.connect(lambda: self.motor_4.enable_operation(is_pdo=True))
        self.ui.enable_operation_5.clicked.connect(lambda: self.motor_5.enable_operation(is_pdo=True))
        self.ui.enable_operation_6.clicked.connect(lambda: self.motor_6.enable_operation(is_pdo=True))
        self.ui.enable_operation_7.clicked.connect(lambda: self.motor_7.enable_operation(is_pdo=True))
        self.ui.enable_operation_8.clicked.connect(lambda: self.motor_8.enable_operation(is_pdo=True))
        self.ui.enable_operation_9.clicked.connect(lambda: self.motor_9.enable_operation(is_pdo=True))
        self.ui.enable_operation_10.clicked.connect(lambda: self.motor_10.enable_operation(is_pdo=True))

        self.ui.disable_operation_1.clicked.connect(lambda: self.motor_1.disable_operation(is_pdo=True))
        self.ui.disable_operation_2.clicked.connect(lambda: self.motor_2.disable_operation(is_pdo=True))
        self.ui.disable_operation_3.clicked.connect(lambda: self.motor_3.disable_operation(is_pdo=True))
        self.ui.disable_operation_4.clicked.connect(lambda: self.motor_4.disable_operation(is_pdo=True))
        self.ui.disable_operation_5.clicked.connect(lambda: self.motor_5.disable_operation(is_pdo=True))
        self.ui.disable_operation_6.clicked.connect(lambda: self.motor_6.disable_operation(is_pdo=True))
        self.ui.disable_operation_7.clicked.connect(lambda: self.motor_7.disable_operation(is_pdo=True))
        self.ui.disable_operation_8.clicked.connect(lambda: self.motor_8.disable_operation(is_pdo=True))
        self.ui.disable_operation_9.clicked.connect(lambda: self.motor_9.disable_operation(is_pdo=True))
        self.ui.disable_operation_10.clicked.connect(lambda: self.motor_10.disable_operation(is_pdo=True))

        self.ui.disable_voltage_1.clicked.connect(lambda: self.motor_1.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_2.clicked.connect(lambda: self.motor_2.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_3.clicked.connect(lambda: self.motor_3.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_4.clicked.connect(lambda: self.motor_4.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_5.clicked.connect(lambda: self.motor_5.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_6.clicked.connect(lambda: self.motor_6.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_7.clicked.connect(lambda: self.motor_7.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_8.clicked.connect(lambda: self.motor_8.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_9.clicked.connect(lambda: self.motor_9.disable_voltage(is_pdo=True))
        self.ui.disable_voltage_10.clicked.connect(lambda: self.motor_10.disable_voltage(is_pdo=True))

        self.ui.quick_stop_1.clicked.connect(lambda: self.motor_1.quick_stop(is_pdo=True))
        self.ui.quick_stop_2.clicked.connect(lambda: self.motor_2.quick_stop(is_pdo=True))
        self.ui.quick_stop_3.clicked.connect(lambda: self.motor_3.quick_stop(is_pdo=True))
        self.ui.quick_stop_4.clicked.connect(lambda: self.motor_4.quick_stop(is_pdo=True))
        self.ui.quick_stop_5.clicked.connect(lambda: self.motor_5.quick_stop(is_pdo=True))
        self.ui.quick_stop_6.clicked.connect(lambda: self.motor_6.quick_stop(is_pdo=True))
        self.ui.quick_stop_7.clicked.connect(lambda: self.motor_7.quick_stop(is_pdo=True))
        self.ui.quick_stop_8.clicked.connect(lambda: self.motor_8.quick_stop(is_pdo=True))
        self.ui.quick_stop_9.clicked.connect(lambda: self.motor_9.quick_stop(is_pdo=True))
        self.ui.quick_stop_10.clicked.connect(lambda: self.motor_10.quick_stop(is_pdo=True))

        self.ui.fault_reset_1.clicked.connect(lambda: self.motor_1.quick_stop(is_pdo=True))
        self.ui.fault_reset_2.clicked.connect(lambda: self.motor_2.quick_stop(is_pdo=True))
        self.ui.fault_reset_3.clicked.connect(lambda: self.motor_3.quick_stop(is_pdo=True))
        self.ui.fault_reset_4.clicked.connect(lambda: self.motor_4.quick_stop(is_pdo=True))
        self.ui.fault_reset_5.clicked.connect(lambda: self.motor_5.quick_stop(is_pdo=True))
        self.ui.fault_reset_6.clicked.connect(lambda: self.motor_6.quick_stop(is_pdo=True))
        self.ui.fault_reset_7.clicked.connect(lambda: self.motor_7.quick_stop(is_pdo=True))
        self.ui.fault_reset_8.clicked.connect(lambda: self.motor_8.quick_stop(is_pdo=True))
        self.ui.fault_reset_9.clicked.connect(lambda: self.motor_9.quick_stop(is_pdo=True))
        self.ui.fault_reset_10.clicked.connect(lambda: self.motor_10.quick_stop(is_pdo=True))

        # for node_id in range(1, 11):
        #     getattr(self.ui, "shut_down_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "switch_on_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "enable_operation_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "disable_operation_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "disable_voltage_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "quick_stop_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))
        #     getattr(self.ui, "fault_reset_{}".format(node_id)).clicked.connect(lambda: getattr(self, "motor_{}".format(node_id)).shut_down(is_pdo=True))

        ''' 控制 '''
        self.ui.shut_down_11.clicked.connect(self.shut_down_all)
        self.ui.switch_on_11.clicked.connect(self.switch_on_all)
        self.ui.enable_operation_11.clicked.connect(self.enable_operation_all)
        self.ui.disable_operation_11.clicked.connect(self.disable_operation_all)
        self.ui.disable_voltage_11.clicked.connect(self.disable_voltage_all)
        self.ui.quick_stop_11.clicked.connect(self.quick_stop_all)
        self.ui.fault_reset_11.clicked.connect(self.fault_reset_all)

        ''' 电磁阀 '''
        self.ui.open_valve_1.clicked.connect(self.io.open_valve_1)
        self.ui.open_valve_2.clicked.connect(self.io.open_valve_2)
        self.ui.open_valve_3.clicked.connect(self.io.open_valve_3)
        self.ui.open_valve_4.clicked.connect(self.io.open_valve_4)

        self.ui.close_valve_1.clicked.connect(self.io.close_valve_1)
        self.ui.close_valve_2.clicked.connect(self.io.close_valve_2)
        self.ui.close_valve_3.clicked.connect(self.io.close_valve_3)
        self.ui.close_valve_4.clicked.connect(self.io.close_valve_4)

        ''' 速度 关节 '''
        for node_id in Motor.motor_dict:
            getattr(self.ui, f"speed_forward_{node_id}").pressed.connect(getattr(self, f"speed_forward_{node_id}"))
            getattr(self.ui, f"speed_forward_{node_id}").released.connect(getattr(self, f"speed_stop_{node_id}"))
            getattr(self.ui, f"speed_reverse_{node_id}").pressed.connect(getattr(self, f"speed_reverse_{node_id}"))
            getattr(self.ui, f"speed_reverse_{node_id}").released.connect(getattr(self, f"speed_stop_{node_id}"))

        ''' 滚珠 调0 归0 '''
        self.ui.set_ballscrew_zero.clicked.connect(self.ballscrew_set_zero)

        ''' 线 适应 调0 '''
        self.ui.start_adjust.clicked.connect(self.rope_force_adapt)
        self.ui.set_rope_zero.clicked.connect(self.rope_set_zero)

        # self.ui.set_zero.clicked.connect(self.ballscrew_set_zero)
        self.ui.go_zero.clicked.connect(self.ballscrew_go_zero)


    ''' 打开设备 '''
    def open_device(self) -> None:
        if UsbCan.open_device():
            
            if not self.__usbcan_0_is_start and self.usbcan_0.init_can() and self.usbcan_0.start_can():
                self.read_canopen_thread = CANopenUpdate(pdo_1_slot_function=self.show_pdo_1, pdo_2_slot_function=self.show_pdo_2, pdo_4_slot_function=self.show_pdo_4, status_signal=self.show_status)
                self.read_canopen_thread.start()

                self.__usbcan_0_is_start = True

            if not self.__usbcan_1_is_start and self.usbcan_1.init_can() and self.usbcan_1.start_can():
                self.read_sensor_thread = SensorResolve(update_signal=self.show_force)
                self.read_sensor_thread.start()

                self.__usbcan_1_is_start = True

            if self.__usbcan_0_is_start and self.__usbcan_1_is_start:
                self.ui.bt_open_device.setEnabled(False)

                self.ui.bt_init_robot.setEnabled(True)

                self.show_status("Open Device !!!")
        else: self.show_status("Open Device Failed")
    
    ''' 初始化机器人 '''
    def initialize_robot(self) -> None:
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
        
        self.init_robot_thread = RobotInit(times=1, running_signal=change, finish_signal=next)
        self.send_request_thread = SensorRequest()

        self.init_robot_thread.start()
        self.send_request_thread.start()

    ''' 丝杠 调零 '''
    def ballscrew_set_zero(self):
        def start():
            self.ballscrew_is_set_zero = False
            self.ui.ballscrew_set_zero.setEnabled(False)
            self.show_status("Ballscrew is being setting zero ...")
        
        def finish():
            self.ballscrew_is_set_zero = True
            self.ui.ballscrew_set_zero.setEnabled(True)
            self.show_status("Ballscrew is set zero !")
            self.ui.ballscrew.setText("<span style=\"color:#00ff00;\">Zero</span>")
        
        distance = float(self.ui.forward_distance.text()) if self.ui.forward_distance.text() != "" \
            else float(self.ui.forward_distance.placeholderText())
        velocity = int(self.ui.forward_velocity.text()) if self.ui.forward_velocity.text() != "" \
            else int(self.ui.forward_velocity.placeholderText())
        speed = int(self.ui.backward_speed.text()) if self.ui.backward_speed.text() != "" \
            else int(self.ui.backward_speed.placeholderText())
        
        self.ballscrew_set_zero_thread = BallScrewSetZero(distance, velocity, speed, 
                                                          motor=self.motor_10, io=self.io, 
                                                          start_signal=start, finish_signal=finish)

        self.ballscrew_set_zero_thread.start()
    
    ''' 线 调零 '''
    def rope_force_adapt(self):
        def start():
            self.rope_is_set_zero = False
            self.ui.start_adjust.setEnabled(False)
            self.ui.set_rope_zero.setEnabled(True)
            self.show_status("All ropes are being adapting force ...")

        def finish():
            self.rope_is_set_zero = True
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
        
        self.rope_force_adapt_thread = ContinuumAttitudeAdjust(self.motor_1, self.motor_2, self.motor_3, self.motor_4, self.motor_5, self.motor_6, self.motor_7, self.motor_8, self.motor_9, 
                                                               self.sensor_1, self.sensor_2, self.sensor_3, self.sensor_4, self.sensor_5, self.sensor_6, self.sensor_7, self.sensor_8, self.sensor_9, 
                                                               self.io, 
                                                               i_f=i_f, m_f=m_f, o_f=o_f, 
                                                               i_pid=i_pid, m_pid=m_pid, o_pid=o_pid, 
                                                               start_signal=start, finish_signal=finish)

        self.rope_force_adapt_thread.start()
    def rope_set_zero(self):
        self.rope_force_adapt_thread.stop()
        self.rope_force_adapt_thread.wait()



    ''' 显示 TPDO1 '''
    def show_pdo_1(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
            # 状态
            status = getattr(self, f"motor_{node_id}").servo_status
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
        
        # IO
        elif node_id in IoModule.io_dict.keys():
            if self.io.input_1 or self.io.input_1:
                # 电机10关键动作
                # 电机10关键动作
                pass
                
                


                # self.motor_10.set_servo_status("quick_stop") # 先停止电机10
                # self.motor_10.permission = False # 取消运动权限
                
                


                
                # 电机10关键动作 
                # 电机10关键动作
            
            warning = "<span style=\"color:#ff0000;\">{}</span>".format("WARNING")
            clear = "<span style=\"color:#00ff00;\">{}</span>".format("CLEAR")

            self.ui.switch_1.setText(warning if self.io.input_1 else clear)
            self.ui.switch_2.setText(warning if self.io.input_2 else clear)

    ''' 显示 TPDO2 '''
    def show_pdo_2(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
            if node_id == 10:
                if self.ballscrew_is_set_zero:
                    self.ballscrew_position = (self.motor_10.zero_position - self.motor_10.current_position) / 5120
                    color = "#00ff00" if self.ballscrew_position >=0 else "#ff0000"
                    self.ui.ballscrew.setText("<span style=\"color:{};\">{}</span>".format(color, round(self.ballscrew_position, 3)))
                else: self.ui.ballscrew.setText("<span style=\"color:#ff0000;\">No Zero</span>")
            else:
                if self.rope_is_set_zero:
                    value = (getattr(self, f"motor_{node_id}").current_position - getattr(self, f"motor_{node_id}").zero_position) / 12536.512440
                    setattr(self, f"rope_{node_id}_position", value)
                    color = "#00ff00" if value >=0 else "#ff0000"
                    getattr(self.ui, "rope_{}".format(node_id)).setText("<span style=\"color:{};\">{}</span>".format(color, round(value, 3)))
                else: getattr(self.ui, "rope_{}".format(node_id)).setText("<span style=\"color:#ff0000;\">No Zero</span>")
            
            # 位置
            position = getattr(self, f"motor_{node_id}").current_position

            max_position = getattr(self, f"motor_{node_id}").max_position
            min_position = getattr(self, f"motor_{node_id}").min_position
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
            speed = getattr(self, f"motor_{node_id}").current_speed

            max_speed = getattr(self, f"motor_{node_id}").max_speed
            min_speed = getattr(self, f"motor_{node_id}").min_speed
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
    
    ''' 显示 TPDO4 '''
    def show_pdo_4(self, node_id):
        # 电机
        if node_id in Motor.motor_dict.keys():
            control_mode = getattr(self, f"motor_{node_id}").control_mode
            if control_mode == "position_control": mode_str = "<span style=\"color:#00ff00;\">POSITION</span>"
            elif control_mode == "speed_control": mode_str = "<span style=\"color:#00ff00;\">SPEED</span>"
            else: pass
            getattr(self.ui, f"mode_{node_id}").setText(mode_str)
    
    ''' 显示 传感器 '''
    def show_force(self, node_id):
        if node_id in Sensor.sensor_dict.keys():
            force = getattr(self, f"sensor_{node_id}").force

            if force > 0: color = "#0000ff"
            else:
                if abs(force) <= 10: color = "#00ff00"
                else: color = "#ffff00"
            
            force_str = "<span style=\"color:{};\">{}</span>".format(color, round(abs(force), 2))

            getattr(self.ui, f"force_{node_id}").setText(force_str)

    ''' 显示 电磁阀 '''
    def show_valve_status(self):
        # 小爪 开启状态较为危险
        open = "<span style=\"color:#ffff00;\">{}</span>".format("OPEN")
        close = "<span style=\"color:#00ff00;\">{}</span>".format("CLOSE")

        if self.io.output_1:
            self.ui.valve_1.setText(open)

            self.ui.open_valve_1.setEnabled(False)
            self.ui.close_valve_1.setEnabled(True)
        else:
            self.ui.valve_1.setText(close)

            self.ui.open_valve_1.setEnabled(True)
            self.ui.close_valve_1.setEnabled(False)
        
        if self.io.output_2:
            self.ui.valve_2.setText(open)

            self.ui.open_valve_2.setEnabled(False)
            self.ui.close_valve_2.setEnabled(True)
        else:
            self.ui.valve_2.setText(close)

            self.ui.open_valve_2.setEnabled(True)
            self.ui.close_valve_2.setEnabled(False)

        if self.io.output_3:
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

        if self.io.output_4:
            self.ui.valve_4.setText(close)

            self.ui.open_valve_4.setEnabled(True)
            self.ui.close_valve_4.setEnabled(False)
        else:
            self.ui.valve_4.setText(open)

            self.ui.open_valve_4.setEnabled(False)
            self.ui.close_valve_4.setEnabled(True)
    
    ''' 显示 程序状态 '''
    def show_status(self, message):
        self.ui.statusBar.showMessage(message, 5000)



    ''' 集体控制 '''
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

    ''' 电机 速度 正 '''
    def speed_forward_factory(self, node_id):
        setattr(self, f"joint_{node_id}", JointControlSpeedModeThread(getattr(self, "motor_{}".format(node_id)), int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value()), is_forward=True))
        
        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)

        getattr(self, f"joint_{node_id}").start()
    for node_id in range(1,11):
        exec(f"def speed_forward_{node_id}(self): self.speed_forward_factory({node_id})")
    
    ''' 电机 速度 反 '''
    def speed_reverse_factory(self, node_id):
        setattr(self, f"joint_{node_id}", JointControlSpeedModeThread(getattr(self, "motor_{}".format(node_id)), int(getattr(self.ui, "speed_adjust_{}".format(node_id)).value()), is_forward=False))
        
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(False)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(False)

        getattr(self, f"joint_{node_id}").start()
    for node_id in range(1,11):
        exec(f"def speed_reverse_{node_id}(self): self.speed_reverse_factory({node_id})")

    ''' 电机 速度 停 '''
    def speed_stop_factory(self, node_id):
        getattr(self, f"joint_{node_id}").stop()
        getattr(self, f"joint_{node_id}").wait()

        getattr(self.ui, f"speed_reverse_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_forward_{node_id}").setEnabled(True)
        getattr(self.ui, f"speed_adjust_{node_id}").setEnabled(True)
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
        
        self.ballscrew_go_zero_thread = BallScrewGoZeroThread(motor=self.motor_10, io=self.io, start_signal=start, finish_signal=finish)

        self.ballscrew_go_zero_thread.start()
    
    ''' 丝杠 移动 '''
    def ballscrew_move(self, distance, /, *, velocity=100, is_close=False, is_relative=False) -> bool:
        if not self.ballscrew_is_set_zero:
            self.show_status("Ballscrew is not set zero.")
            return False

        while self.ballscrew_is_moving:
            self.show_status("Ballscrew is moving, you cannot move it now.")
            time.sleep(1)
        
        def start():
            self.ballscrew_is_moving = True
        
        def finish():
            self.ballscrew_is_moving = False
        
        self.ballscrew_move_thread = BallScrewMoveThread(distance, velocity=velocity, 
                                                        is_close=is_close, is_relative=is_relative, 
                                                        motor=self.motor_10, io=self.io, 
                                                        start_signal=start, finish_signal=finish)
        self.ballscrew_move_thread.start()
        return True


    ''' 线 移动 '''
    def rope_move(self, dis):
        self.rope_move_thread = RopeMoveThread((self.motor_1, True, dis, 100))
        self.rope_move_thread.start()

    def move_test(self):
        ...




    ''' 缩放 内段 线 '''
    def stretch_inside(self):
        self.stretch_inside_thread = StretchInsideThread(-50, 
            # self.motor_1, self.motor_2, self.motor_3, 
            # self.motor_4, self.motor_5, self.motor_6, 
            self.motor_7, self.motor_8, self.motor_9)
        self.stretch_inside_thread.start()
    def release_inside(self):
        self.stretch_inside_thread = StretchInsideThread(50, 
            # self.motor_1, self.motor_2, self.motor_3, 
            # self.motor_4, self.motor_5, self.motor_6, 
            self.motor_7, self.motor_8, self.motor_9)
        self.stretch_inside_thread.start()
    def stop_inside(self):
        self.stretch_inside_thread.stop()
        self.stretch_inside_thread.wait()

    ''' 缩放 中段 线 '''
    def stretch_midside(self):
        self.stretch_midside_thread = StretchMidsideThread(-50, 
            # self.motor_1, self.motor_2, self.motor_3, 
            self.motor_4, self.motor_5, self.motor_6)
        self.stretch_midside_thread.start()
    def release_midside(self):
        self.stretch_midside_thread = StretchMidsideThread(50, 
            # self.motor_1, self.motor_2, self.motor_3, 
            self.motor_4, self.motor_5, self.motor_6)
        self.stretch_midside_thread.start()
    def stop_midside(self):
        self.stretch_midside_thread.stop()
        self.stretch_midside_thread.wait()
    
    ''' 缩放 外段 线 '''
    def stretch_outside(self):
        self.stretch_outide_thread = StretchOutsideThread(-50, 
            self.motor_1, self.motor_2, self.motor_3)
        self.stretch_outide_thread.start()
    def release_outside(self):
        self.stretch_outide_thread = StretchOutsideThread(50, 
            self.motor_1, self.motor_2, self.motor_3)
        self.stretch_outide_thread.start()
    def stop_outside(self):
        self.stretch_outide_thread.stop()
        self.stretch_outide_thread.wait()
    
    
    
    
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

    def test_move(self):
        
        self.test_move_thread = MoveTest(self.motor_10, self.io, 
                                         self.motor_1, self.motor_2, self.motor_3, 
                                         self.motor_4, self.motor_5, self.motor_6, 
                                         self.motor_7, self.motor_8, self.motor_9)
        self.test_move_thread.start()

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

        while not self.__motor.set_control_mode("speed_control", check=False): time.sleep(0.1)
        
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

class MoveTest(QThread):
    def __init__(self, motor: Motor, io: IoModule, 
                 rope_1: Motor, rope_2: Motor, rope_3: Motor, 
                 rope_4: Motor, rope_5: Motor, rope_6: Motor, 
                 rope_7: Motor, rope_8: Motor, rope_9: Motor) -> None:
        
        super().__init__()

        self.__is_stop = False

        self.__io = io

        self.__rope_1 = rope_1
        self.__rope_2 = rope_2
        self.__rope_3 = rope_3
        self.__rope_4 = rope_4
        self.__rope_5 = rope_5
        self.__rope_6 = rope_6
        self.__rope_7 = rope_7
        self.__rope_8 = rope_8
        self.__rope_9 = rope_9

        self.__motor = motor

        self.__motor.set_control_mode("position_control", check=False)
        self.__rope_1.set_control_mode("position_control", check=False)
        self.__rope_2.set_control_mode("position_control", check=False)
        self.__rope_3.set_control_mode("position_control", check=False)
        self.__rope_4.set_control_mode("position_control", check=False)
        self.__rope_5.set_control_mode("position_control", check=False)
        self.__rope_6.set_control_mode("position_control", check=False)
        self.__rope_7.set_control_mode("position_control", check=False)
        self.__rope_8.set_control_mode("position_control", check=False)
        self.__rope_9.set_control_mode("position_control", check=False)

        self.__inside_start = 348
        self.__inside_end = 358
    
    def run(self):
        self.__io.open_valve_4()

        self.__io.open_valve_3()
        self.__io.open_valve_2()
        time.sleep(1)

        distance_1 = 348
        distance_2 = 358

        position_1 = self.__motor.zero_position - distance_1 * 5120
        position_2 = self.__motor.zero_position - distance_2 * 5120

        v = 100

        times = 10
        
        while not self.__is_stop and times != 0:

            self.__motor.set_position(position_1, velocity=v, is_pdo=True)

            self.__motor.ready(is_pdo=True)

            self.__motor.action(is_immediate=False, is_relative=False, is_pdo=True)
            
            time.sleep((10 * 5120) / (v * 440)+0.5)

            self.__io.close_valve_4()
            time.sleep(1)
            self.__io.open_valve_1()
            time.sleep(1)

            self.__motor.set_position(position_2, velocity=v, is_pdo=True)
            self.__rope_1.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_2.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_3.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_4.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_5.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_6.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_7.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_8.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)
            self.__rope_9.set_position(int(10 * 12536.512440), velocity=245, is_pdo=True)

            self.__motor.ready(is_pdo=True)
            self.__rope_1.ready(is_pdo=True)
            self.__rope_2.ready(is_pdo=True)
            self.__rope_3.ready(is_pdo=True)
            self.__rope_4.ready(is_pdo=True)
            self.__rope_5.ready(is_pdo=True)
            self.__rope_6.ready(is_pdo=True)
            self.__rope_7.ready(is_pdo=True)
            self.__rope_8.ready(is_pdo=True)
            self.__rope_9.ready(is_pdo=True)

            self.__motor.action(is_immediate=False, is_relative=False, is_pdo=True)
            self.__rope_1.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_2.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_3.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_4.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_5.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_6.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_7.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_8.action(is_immediate=False, is_relative=True, is_pdo=True)
            self.__rope_9.action(is_immediate=False, is_relative=True, is_pdo=True)
            
            time.sleep((10 * 5120) / (v * 440)+0.5)
            
            self.__io.close_valve_1()
            time.sleep(1)
            self.__io.open_valve_4()
            time.sleep(1)
            
            times -= 1

    def stop(self):
        self.__is_stop = True

    def ballscrew_move_abs(self, point: float, /, *, velocity: float):
        target_position = int(round(self.__motor.zero_position - point * 5120, 0))
        profile_velocity = int(round(5120 * velocity / 440, 0))

        self.__motor.set_position(target_position, velocity=profile_velocity, is_pdo=True)
        self.__motor.ready(is_pdo=True)
        self.__motor.action(is_immediate=True, is_relative=False, is_pdo=True)

        duration_time = abs(target_position - self.__motor.current_position) / (profile_velocity * 440) + 1
        time.sleep(duration_time)
    
    def ballscrew_move_rel(self, distance: float, /, *, velocity: float):
        target_position = int(round(distance * 5120, 0))
        profile_velocity = int(round(5120 * velocity / 440, 0))

        self.__motor.set_position(target_position, velocity=profile_velocity, is_pdo=True)
        self.__motor.ready(is_pdo=True)
        self.__motor.action(is_immediate=False, is_relative=True, is_pdo=True)

        duration_time = target_position / (profile_velocity * 440) + 1
        time.sleep(duration_time)

    def rope_move_rel(self, distance: float, /, *, velocity: float):
        ...