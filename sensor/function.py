# -*- coding:utf-8 -*-


''' function.py 力传感器数据读取 v1.1 新增检查 '''


import time
from PyQt5.QtCore import QThread, pyqtSignal, QMutex


class SensorUpdateThread(QThread):
    update_signal = pyqtSignal()
    
    def __init__(self, mutex: QMutex, slot_function) -> None:
        super().__init__()
        self.__is_stop = False
        self.__mutex = mutex
        self.update_signal.connect(slot_function)
    
    def run(self):
        Sensor.update_init()
        while not self.__is_stop:
            self.__mutex.lock()
            Sensor.update_force()
            self.__mutex.unlock()
            self.update_signal.emit()

    def stop(self):
        self.__is_stop = True


class Sensor():
    __device = None
    __sensor_dict = {}
    __msg = [0x49, 0xAA, 0x0D, 0x0A]
    
    def __init__(self, node_id) -> None:
        self.__node_id = node_id
        self.__sensor_dict[node_id] = self
        self.__is_ready = False
        self.__init = 0
        self.force = 0

    @classmethod
    def link_device(cls, device):
        cls.__device = device
        return cls
    
    def is_ready(self) -> bool:
        if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
            time.sleep(0.01)
            ret = self.__device.read_buffer(1)
            if ret != None:
                [num, msg] = ret
                if num != 0: return True
        return False
    
    @staticmethod
    def check_status() -> bool:
        for sensor in Sensor.__sensor_dict.values():
            if sensor.is_ready(): sensor.__is_ready = True
            else:
                print("\033[0;31m[Sensor {}] unchecked\033[0m".format(sensor.__node_id))
                return False
        else: return True


    def get_init(self, count=50) -> None:
        value = 0
        times = count
        while times != 0:
            if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
                time.sleep(0.01)
                ret = self.__device.read_buffer(1)
                if ret != None:
                    [num, msg] = ret
                    if num!= 0:
                        value += self.__hex_list_to_float([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]])
                        times -= 1
        self.__init = value / count

    @staticmethod
    def update_init() -> None:
        for sensor in Sensor.__sensor_dict.values():
            sensor.get_init()

    def get_force(self) -> None:
        if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
            time.sleep(0.01)
            ret = self.__device.read_buffer(1)
            if ret != None:
                [num, msg] = ret
                if num!= 0:
                    self.force = round(self.__hex_list_to_float([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]]) - self.__init, 2)
    
    @staticmethod
    def update_force() -> None:
        for sensor in Sensor.__sensor_dict.values():
            sensor.get_force()

    @staticmethod
    def __hex_list_to_float(data_list) -> float:
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
