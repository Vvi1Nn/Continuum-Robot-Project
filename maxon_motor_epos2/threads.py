# -*- coding:utf-8 -*-


''' threads.py 在GUI中使用的线程 v1.0 '''


from PyQt5.QtCore import QThread, pyqtSignal

# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
import maxon_motor_epos2.protocol as protocol
from maxon_motor_epos2.processor import CanOpenBusProcessor
from maxon_motor_epos2.motor import Motor


''' CANopen 总线数据处理 '''
class CANopenUpdateThread(QThread):
    pdo_1_update_signal = pyqtSignal(int)
    pdo_2_update_signal = pyqtSignal(int)
    
    def __init__(self, pdo_1_slot_function, pdo_2_slot_function) -> None:
        super().__init__()
        self.__is_stop = False

        self.pdo_1_update_signal.connect(pdo_1_slot_function)
        self.pdo_2_update_signal.connect(pdo_2_slot_function)
    
    def run(self):
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
                                    else: pass
                        
                        # 其他的ID
                        else: pass

                        self.pdo_1_update_signal.emit(node_id)
                    
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

                        self.pdo_2_update_signal.emit(node_id)
                    
                    # SDO
                    elif msg[i].ID > 0x580 and msg[i].ID < 0x600:
                        node_id = msg[i].ID - 0x580
                        
                        command =  msg[i].Data[0]
                        if command == protocol.CMD_R["read_16"] or protocol.CMD_R["read_32"] or protocol.CMD_R["read_8"]: status = True
                        elif command == protocol.CMD_R["write"]: status = True
                        elif command == protocol.CMD_R["error"]: status = False
                        else: status = None
                        
                        index = self.__match_index(msg[i].Data[1], msg[i].Data[2], msg[i].Data[3])
                        for key in protocol.OD: # 遍历字典关键字
                            if index == protocol.OD[key]: # 在每一个关键字对应的列表中 核对数值
                                label = key
                                break
                            else: pass
                        else: label = ""
                        
                        value_list = [msg[i].Data[j] for j in range(4,8)]
                        
                        # print("OLD", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)

                        wait_time = 1
                        time_stamp = time.time()
                        while CanOpenBusProcessor.node_dict[node_id].sdo_feedback[0] and time.time() - time_stamp < wait_time: time.sleep(0.1)
                        
                        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value_list)

                        print("[SDO NEW] ", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)
                    
                    # NMT
                    elif msg[i].ID > 0x700 and msg[i].ID < 0x780:
                        node_id = msg[i].ID - 0x700

                        for key in protocol.NMT_STATUS:
                            if msg[0].Data[0] & 0b01111111 == protocol.NMT_STATUS[key]:
                                label = key
                                break
                            else: pass
                        else: label = ""

                        # print("old  ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
                        
                        print("[NMT NEW] ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                    
                    # 其他
                    else: pass
            else: pass
    
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


# ''' 电机初始化 '''
# class MotorInitThread(QThread):
#     running_signal = pyqtSignal(bool)
#     check_signal = pyqtSignal(int)
#     finish_signal = pyqtSignal()
    
#     def __init__(self) -> None:
#         super().__init__()
#         self.__check_count = 0
    
#     def run(self):
#         self.running_signal.emit(True)

#         for node_id in Motor.motor_dict:
#             print("=============================================================")
#             times = 3
#             while times != 0:
#                 if Motor.motor_dict[node_id].check_bus_status():
#                     if Motor.motor_dict[node_id].start_pdo(log=True, delay=1):
#                         if Motor.motor_dict[node_id].check_servo_status():
#                             success_1 = Motor.motor_dict[node_id].set_control_mode()
#                             success_2 = Motor.motor_dict[node_id].set_inhibit_time("2")
#                             success_3 = Motor.motor_dict[node_id].set_profile_acceleration()
#                             success_4 = Motor.motor_dict[node_id].set_profile_deceleration()
#                             success_5 = Motor.motor_dict[node_id].set_quick_stop_deceleration()
#                             success_6 = Motor.motor_dict[node_id].set_motion_profile_type()
#                             if success_1 and success_2 and success_3 and success_4 and success_5 and success_6:
#                                 self.check_signal.emit(node_id)
#                                 self.__check_count += 1
#                                 break
#                             else: pass
#                         else: pass
#                     else: pass
#                 else: pass

#                 times -= 1
#         if self.__check_count == len(Motor.motor_dict): self.finish_signal.emit()
#         else: self.running_signal.emit(False)
        

''' 参数生效 '''
class ParamLaunchThread(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self, motion_profile_type, profile_acceleration, profile_deceleration, quick_stop_deceleration, inhibit_time) -> None:
        super().__init__()

        self.__motion_profile_type = motion_profile_type
        self.__profile_acceleration = profile_acceleration
        self.__profile_deceleration = profile_deceleration
        self.__quick_stop_deceleration = quick_stop_deceleration
        self.__inhibit_time = inhibit_time
    
    def run(self):
        self.running_signal.emit(True)

        for node_id in Motor.motor_dict:
            print("=============================================================")
            times = 3
            while times != 0:
                success_1 = Motor.motor_dict[node_id].set_inhibit_time("2", self.__inhibit_time)
                success_2 = Motor.motor_dict[node_id].set_profile_acceleration(self.__profile_acceleration)
                success_3 = Motor.motor_dict[node_id].set_profile_deceleration(self.__profile_deceleration)
                success_4 = Motor.motor_dict[node_id].set_quick_stop_deceleration(self.__quick_stop_deceleration)
                success_5 = Motor.motor_dict[node_id].set_motion_profile_type(self.__motion_profile_type)
                if success_1 and success_2 and success_3 and success_4 and success_5: break
                else: times -= 1
        
        self.running_signal.emit(False)


''' 锁定模式 '''
class JointControlLockModeThread(QThread):
    def __init__(self, motor, speed: int, /, *, is_forward: bool) -> None:
        super().__init__()

        self.__is_stop = False

        self.__motor = motor

        self.__is_forward = is_forward

        if self.__is_forward: self.__speed = speed
        else: self.__speed = - speed
    
    def run(self):
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
