# -*- coding:utf-8 -*-


''' sensor.py 力传感器 '''


import time
from PyQt5.QtCore import QThread, pyqtSignal, QMutex


''' 功能 '''
class Sensor():
    device = None

    sensor_dict = {}

    msg = [0x49, 0xAA, 0x0D, 0x0A]
    
    def __init__(self, node_id) -> None:
        self.__node_id = node_id

        self.sensor_dict[node_id] = self

        self.__is_ready = False

        self.original_data = None

        self.zero = 0
        self.force = None

    ''' 绑定 '''
    @staticmethod
    def link_device(device) -> None:
        Sensor.__device = device
    
    ''' 发送请求 '''
    def send_request(self) -> bool:
        return self.device.send(self.__node_id, [Sensor.msg], data_len="sensor")

    ''' 调零 '''
    def set_zero(self):
        ...
    
    
    # ''' 发送数据 检查是否可以读取到回复 确保传感器处于准备状态 '''
    # def is_ready(self, times=2) -> bool:
    #     while times != 0:
    #         if self.__device.send(self.__node_id, [self.__msg], data_len="sensor"):
    #             time.sleep(0.01)
    #             ret = self.__device.read_buffer(1)
    #             if ret != None:
    #                 [num, msg] = ret
    #                 if num != 0: return True
    #                 else:
    #                     print("\033[0;31m[Sensor {}] In function [is_ready], no data in buffer\033[0m".format(self.__node_id))
    #                     times -= 1
    #             else:
    #                 print("\033[0;31m[Sensor {}] In function [is_ready], return is none\033[0m".format(self.__node_id))
    #                 times -= 1
    #         else:
    #             print("\033[0;31m[Sensor {}] In function [is_ready], usbcan send message failed\033[0m".format(self.__node_id))
    #             times -= 1
    #     return False
    
    # @staticmethod
    # def check_status() -> bool:
    #     print("=============================================================")
    #     check_num = 0
    #     for sensor in Sensor.__sensor_dict.values():
    #         if sensor.is_ready():
    #             sensor.__is_ready = True
    #             check_num += 1
    #             print("\033[0;32m[Sensor {}] READY\033[0m".format(sensor.__node_id))
    #         else: print("\033[0;31m[Sensor {}] NOT READY\033[0m".format(sensor.__node_id))
    #     if check_num == len(Sensor.__sensor_dict): return True
    #     else: return False

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

    # @staticmethod
    # def update_init() -> None:
    #     print("=============================================================")
    #     for sensor in Sensor.__sensor_dict.values():
    #         sensor.get_init()

        
    # @staticmethod
    # def update_force() -> None:
    #     for sensor in Sensor.__sensor_dict.values():
    #         sensor.get_force()


''' 发送请求 '''
class SensorRequestThread(QThread):
    def __init__(self) -> None:
        super().__init__()

        self.__is_stop = False
    
    def run(self):
        clear_success = Sensor.device.clear_buffer()

        print("Sensor Request Sending")
        
        while not self.__is_stop:
            for sensor in Sensor.sensor_dict.values():
                send_success = sensor.send_request()

        print("Sensor Request Thread Stopped")

    def stop(self):
        self.__is_stop = True

        print("Stopping Sensor Request Thread")


''' 解析数据 '''
class SensorResolveThread(QThread):
    __update_signal = pyqtSignal()
    
    def __init__(self, /, *, update_screen_slot_function) -> None:
        super().__init__()

        self.__is_stop = False

        self.__update_signal.connect(update_screen_slot_function)
    
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
                        # Other
                        else: pass
                    else: pass
            else: pass

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
