# -*- coding:utf-8 -*-


''' threads.py 在GUI中使用的线程 v1.0 更改canopen通道状态更新 修改关节控制超出范围无法操作的bug'''


from PyQt5.QtCore import QThread, pyqtSignal
import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor
from motor.function import Motor
from io_module.function import IoModule


class CANopenUpdateThread(QThread):
    update_signal = pyqtSignal(int)
    
    def __init__(self, slot_function) -> None:
        super().__init__()
        self.__is_stop = False
        self.update_signal.connect(slot_function)
    
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

    def run(self):
        while not self.__is_stop:
            node_id = 0
            ret = CanOpenBusProcessor.device.read_buffer(1, wait_time=0)
            if ret != None:
                [num, msg] = ret
                for i in range(num):
                    if msg[i].ID > 0x180 and msg[i].ID < 0x200:
                        node_id = msg[i].ID - 0x180
                        # 电机的ID
                        if node_id != 0xB:
                            status = self.__hex_list_to_int([msg[i].Data[0]]) # 状态字
                            for key in protocol.STATUS_WORD: # 遍历字典关键字
                                for r in protocol.STATUS_WORD[key]: # 在每一个关键字对应的列表中 核对数值
                                    if status == r:
                                        Motor.motor_dict[node_id].motor_status = key # 更新电机的伺服状态
                        # IO模块的ID
                        else:
                            data_low = bin(msg[i].Data[0])[2:] # 首先转换为bin 去除0b
                            data_low = '0' * (8 - len(data_low)) + data_low # 头部补齐
                            data_high = bin(msg[i].Data[1])[2:]
                            data_high = '0' * (8 - len(data_high)) + data_high
                            data = data_high + data_low # 拼接
                            for i, c in enumerate(data):
                                setattr(IoModule.io_dict[node_id], f"switch_{16-i}", False if c == "0" else True)
                    elif msg[i].ID > 0x280 and msg[i].ID < 0x300:
                        node_id = msg[i].ID - 0x280
                        position = self.__hex_list_to_int([msg[i].Data[j] for j in range(0,4)]) # 当前位置
                        speed = self.__hex_list_to_int([msg[i].Data[j] for j in range(4,8)]) # 当前速度
                        Motor.motor_dict[node_id].current_position = position
                        Motor.motor_dict[node_id].current_speed = speed
                    else: pass
            # 发送ID
            if node_id != 0: self.update_signal.emit(node_id)
    
    def stop(self):
        self.__is_stop = True


class JointControlThread(QThread):
    def __init__(self, motor, is_forward: bool, position: int, velocity: int) -> None:
        super().__init__()
        self.__is_stop = False
        self.__motor = motor
        self.__is_forward = is_forward
        self.__position = position
        self.__velocity = velocity
    
    def run(self):
        while not self.__is_stop:
            if self.__motor.is_in_range():
                self.__motor.set_servo_status("position_mode_ready")
                if self.__is_forward:
                    self.__motor.set_position_and_velocity(self.__position, self.__velocity)
                else:
                    self.__motor.set_position_and_velocity(-self.__position, self.__velocity)
                self.__motor.set_servo_status("position_mode_action")
            else:
                if self.__motor.current_position > self.__motor.max_position:
                    if not self.__is_forward:
                        self.__motor.set_servo_status("position_mode_ready")
                        self.__motor.set_position_and_velocity(-self.__position, self.__velocity)
                        self.__motor.set_servo_status("position_mode_action")
                else:
                    if self.__is_forward:
                        self.__motor.set_servo_status("position_mode_ready")
                        self.__motor.set_position_and_velocity(self.__position, self.__velocity)
                        self.__motor.set_servo_status("position_mode_action")
    
    def stop(self):
        self.__is_stop = True


class InitMotorThread(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.init_config() # 将所有参数生效给所有电机
        self.running_signal.emit(False)
    

class CheckMotorThread(QThread):
    running_signal = pyqtSignal(bool)
    check_signal = pyqtSignal(int)
    finish_signal = pyqtSignal()
    
    def __init__(self) -> None:
        super().__init__()
        self.__check_count = 0
    
    def run(self):
        self.running_signal.emit(True)
        for node_id in Motor.motor_dict:
            if not Motor.motor_dict[node_id].motor_is_checked:
                Motor.motor_dict[node_id].check_bus_status()
                Motor.motor_dict[node_id].check_motor_status()
            if Motor.motor_dict[node_id].motor_is_checked:
                self.check_signal.emit(node_id)
                self.__check_count += 1
        if self.__check_count == 9:
            self.finish_signal.emit()
            return
        self.running_signal.emit(False)



class StartPDO(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.start_feedback()
        self.running_signal.emit(False)


class StopPDO(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.stop_feedback()
        self.running_signal.emit(False)
