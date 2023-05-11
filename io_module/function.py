# -*- coding:utf-8 -*-


import time
import sys, os

# 添加模块路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from usbcan.function import UsbCan
from canopen.processor import CanOpenBusProcessor


class IoModule(CanOpenBusProcessor):
    def __init__(self, node_id=0xB) -> None:
        super().__init__(node_id)
    
    def check_bus_status(self) -> None:
        if super().check_bus_status():
            print("\033[0;32m[IO module] bus ready\033[0m")
    
    def start_output(self) -> bool:
        self.set_bus_status("start_remote_node")
        if self.bus_status == "operational":
            print("\033[0;32m[IO module] start output\033[0m")
            return True
    
    def stop_output(self) -> bool:
        self.set_bus_status("stop_remote_node")
        if self.bus_status == "stopped":
            print("\033[0;32m[IO module] stop output\033[0m")
            return True
    
    def set_output(self, c1=0, c2=0, c3=0, c4=0, c5=0, c6=0, c7=0, c8=0):
        if self.bus_status == "operational":
            value_str = ""
            for i in range(8):
                value_str = str(locals()['c' + str(i+1)]) + value_str
            value_low = int(value_str, 2)
            value_high = 0
            self.rpdo("1", value_low, value_high)
            print("\033[0;32m[IO module] output status: {}\033[0m".format(value_str))
        else: print("\033[0;31m[IO module] cannot set output, bus_status wrong\033[0m")


if __name__ == "__main__":
    UsbCan.open_device()
    usbcan_0 = UsbCan("0")
    CanOpenBusProcessor.link_device(usbcan_0)
    
    usbcan_0.init_can()
    usbcan_0.start_can()
    
    io_module = IoModule(11)
    io_module.start_output()
    while True:
        io_module.set_output(1, 1, 1, 1, 1, 1, 1, 1)
        time.sleep(0.5)
        io_module.set_output(0, 0, 0, 0, 0, 0, 0, 0)
        time.sleep(0.5)
        io_module.set_output(0, 1, 0, 1, 0, 1, 0, 1)
        time.sleep(0.5)
        io_module.set_output(1, 0, 1, 0, 1, 0, 1, 0)
        time.sleep(0.5)