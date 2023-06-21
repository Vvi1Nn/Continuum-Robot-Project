# -*- coding:utf-8 -*-


''' function.py 力传感器数据读取 v1.3 优化is_ready状态 避免单次检测不成功 '''


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
        self.force = None

    @staticmethod
    def link_device(device) -> None:
        Sensor.__device = device
    
    ''' 发送数据 检查是否可以读取到回复 确保传感器处于准备状态 '''
    def is_ready(self, times=2) -> bool:
        while times != 0:
            if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
                time.sleep(0.01)
                ret = self.__device.read_buffer(1)
                if ret != None:
                    [num, msg] = ret
                    if num != 0: return True
                    else:
                        print("\033[0;31m[Sensor {}] In function [is_ready], no data in buffer\033[0m".format(self.__node_id))
                        times -= 1
                else:
                    print("\033[0;31m[Sensor {}] In function [is_ready], return is none\033[0m".format(self.__node_id))
                    times -= 1
            else:
                print("\033[0;31m[Sensor {}] In function [is_ready], usbcan send message failed\033[0m".format(self.__node_id))
                times -= 1
        return False
    
    @staticmethod
    def check_status() -> bool:
        print("=============================================================")
        check_num = 0
        for sensor in Sensor.__sensor_dict.values():
            if sensor.is_ready():
                sensor.__is_ready = True
                check_num += 1
                print("\033[0;32m[Sensor {}] READY\033[0m".format(sensor.__node_id))
            else: print("\033[0;31m[Sensor {}] NOT READY\033[0m".format(sensor.__node_id))
        if check_num == len(Sensor.__sensor_dict): return True
        else: return False

    ''' 读取一系列数据 校准传感器零位 '''
    def get_init(self, count=50) -> None:
        if self.__is_ready:
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
                        else: print("\033[0;31m[Sensor {}] In function [get_init], no data in buffer\033[0m".format(self.__node_id))
                    else: print("\033[0;31m[Sensor {}] In function [get_init], read buffer is none\033[0m".format(self.__node_id))
                else: print("\033[0;31m[Sensor {}] In function [get_init], usbcan send message failed\033[0m".format(self.__node_id))
            self.__init = value / count
            print("\033[0;32m[Sensor {}] INIT\033[0m".format(self.__node_id))
        else: print("\033[0;31m[Sensor {}] In function [get_init], sensor is not ready\033[0m".format(self.__node_id))

    @staticmethod
    def update_init() -> None:
        print("=============================================================")
        for sensor in Sensor.__sensor_dict.values():
            sensor.get_init()

    ''' 获取数据 '''
    def get_force(self) -> None:
        if self.__is_ready:
            if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
                time.sleep(0.01)
                ret = self.__device.read_buffer(1)
                if ret != None:
                    [num, msg] = ret
                    if num!= 0:
                        self.force = round(self.__hex_list_to_float([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]]) - self.__init, 2)
        else: print("\033[0;31m[Sensor {}] In function [get_init], sensor is not ready\033[0m".format(self.__node_id))

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
