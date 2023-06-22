# -*- coding:utf-8 -*-


''' function.py IO模块 v2.0 '''


import sys, os

# 添加模块路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PyQt5.QtCore import pyqtSignal, QObject
from canopen.processor import CanOpenBusProcessor


class IoModule(CanOpenBusProcessor, QObject):
    update_signal = pyqtSignal()

    io_dict = {}
    
    def __init__(self, node_id, slot_function) -> None:
        CanOpenBusProcessor.__init__(self, node_id)
        QObject.__init__(self)

        self.io_dict[node_id] = self

        self.update_signal.connect(slot_function)

        self.__is_ready = False
        self.is_start = False

        self.channel_1 = False
        self.channel_2 = False
        self.channel_3 = False
        self.channel_4 = False
        self.channel_5 = False
        self.channel_6 = False
        self.channel_7 = False
        self.channel_8 = False

        self.switch_1 = None
        self.switch_2 = None
        self.switch_3 = None
        self.switch_4 = None
        self.switch_5 = None
        self.switch_6 = None
        self.switch_7 = None
        self.switch_8 = None
        self.switch_9 = None
        self.switch_10 = None
        self.switch_11 = None
        self.switch_12 = None
        self.switch_13 = None
        self.switch_14 = None
        self.switch_15 = None
        self.switch_16 = None
    
    def __check_bus_status(self) -> bool:
        if self.check_bus_status():
            print("\033[0;32m[IO {}] BUS READY\033[0m".format(self.node_id))
            return True
        else:
            if self.set_bus_status("enter_pre-operational_state"):
                print("\033[0;32m[IO {}] BUS READY\033[0m".format(self.node_id))
                return True
            else: 
                print("\033[0;31m[IO {}] In function [__check_bus_status], set bus status failed\033[0m".format(self.node_id))
                return False
    
    ''' 把TPDO1的模式更改成定时器模式 '''
    def __set_tpdo_mode(self) -> bool:
        if self.sdo_write_8("tpdo_1_transtype", 0xFF):
            self.__is_ready = True
            print("\033[0;32m[IO {}] MODE CHANGE\033[0m".format(self.node_id))
        else: print("\033[0;31m[IO {}] In function [__set_tpdo_mode], sdo write failed\033[0m".format(self.node_id))
        return self.__is_ready
    
    ''' 配置TPDO参数 启动PDO通讯 控制电磁阀的初始状态 '''
    @staticmethod
    def start_output() -> None:
        for io in IoModule.io_dict.values():
            print("=============================================================")
            if io.__check_bus_status():
                if io.__set_tpdo_mode():
                    if io.set_bus_status("start_remote_node"):
                        io.is_start = True
                        # io.set_channel_status(False, "1", "2", "3") # 3个小爪
                        # io.set_channel_status(True, "4") # 大爪
                        print("\033[0;32m[IO {}] START OUTPUT\033[0m".format(io.node_id))
                    else: print("\033[0;31m[IO {}] In function [start_output], set bus status failed\033[0m".format(io.node_id))
                else: print("\033[0;31m[IO {}] In function [start_output], set tpdo mode failed\033[0m".format(io.node_id))
            else: print("\033[0;31m[IO {}] In function [start_output], check bus status failed\033[0m".format(io.node_id))

    ''' 关闭PDO '''
    @staticmethod
    def stop_output() -> None:
        for io in IoModule.io_dict.values():
            if io.set_bus_status("enter_pre-operational_state"):
                io.is_start = False
                # io.set_channel_status(False, "1", "2", "3", "4", "5", "6", "7", "8")
                print("\033[0;32m[IO {}] STOP OUTPUT\033[0m".format(io.node_id))
            else: print("\033[0;31m[IO {}] In function [stop_output], set bus status failed\033[0m".format(io.node_id))
    
    ''' 设置输出 '''
    def set_channel_status(self, status: bool, *args: str) -> bool:
        if self.is_start:
            for channel in args:
                try:
                    setattr(self, "channel_" + channel, status)
                except Exception as e:
                    print(e)
                    return False
            value_str = ""
            for i in range(1, 9):
                value_str = "1" + value_str if getattr(self, f"channel_{i}") else "0" + value_str
            if self.rpdo("1", int(value_str, 2), format=8):
                self.update_signal.emit() # 向外发送信号 提示更新状态显示
                # print("\033[0;32m[IO {}] OUTPUT STATUS: {}\033[0m".format(self.node_id, value_str))
                return True
            else:
                print("\033[0;31m[IO {}] In function [set_channel_status], send rpdo message failed\033[0m".format(self.node_id))
                return False
        else:
            print("\033[0;31m[IO {}] In function [set_channel_status], pdo communication is not start\033[0m".format(self.node_id))
            return False


if __name__ == "__main__":
    # UsbCan.open_device()
    # usbcan_0 = UsbCan("0")
    # CanOpenBusProcessor.link_device(usbcan_0)
    
    # usbcan_0.init_can()
    # usbcan_0.start_can()
    
    # io_module = IoModule(11)
    # io_module.start_output()
    # while True:
    #     io_module.set_output(1, 1, 1, 1, 1, 1, 1, 1)
    #     time.sleep(0.5)
    #     io_module.set_output(0, 0, 0, 0, 0, 0, 0, 0)
    #     time.sleep(0.5)
    #     io_module.set_output(0, 1, 0, 1, 0, 1, 0, 1)
    #     time.sleep(0.5)
    #     io_module.set_output(1, 0, 1, 0, 1, 0, 1, 0)
    #     time.sleep(0.5)

    io = IoModule(11)
    io.set_channel_status(True, "1", "2", "4")

    io.set_channel_status(False, "1")