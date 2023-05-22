# -*- coding:utf-8 -*-


''' function.py 力传感器数据读取 v1.0 '''


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
    
    def __init__(self, node_id):
        self.__node_id = node_id
        self.__sensor_dict[node_id] = self
        self.force = 0

    @classmethod
    def link_device(cls, device):
        cls.__device = device
        return cls
    
    def get_force(self):
        if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
            time.sleep(0.01)
            ret = self.__device.read_buffer(1)
            if ret != None:
                [num, msg] = ret
                if num!= 0:
                    self.force = self.__hex_list_to_int([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]])
    
    @staticmethod
    def update_force():
        for sensor in Sensor.__sensor_dict.values():
            sensor.get_force()

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
    