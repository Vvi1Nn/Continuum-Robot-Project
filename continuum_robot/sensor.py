# -*- coding:utf-8 -*-


''' sensor.py 力传感器 '''


import time


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
    def linkDevice(device) -> None:
        Sensor.device = device
    
    ''' 发送请求 '''
    def send_request(self) -> bool:
        return self.device.sendMsg(self.__node_id, [Sensor.msg], data_len="sensor")

    ''' 调零 '''
    def set_zero(self, num=100):
        count = num
        value = 0
        while count != 0:
            value += self.original_data
            count -= 1
        self.zero = value / num
