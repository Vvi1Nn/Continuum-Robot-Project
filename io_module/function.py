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

        self.__can_start = False
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
            print("\033[0;32m[IO {}] bus ready\033[0m".format(self.node_id))
            return True
        else:
            if self.set_bus_status("enter_pre-operational_state"): return True
        print("\033[0;31m[IO {}] bus not ready\033[0m".format(self.node_id))
        return False
    
    ''' 把TPDO1的模式更改成定时器模式 '''
    def __set_tpdo_mode(self) -> bool:
        if self.sdo_write_8("tpdo_1_transtype", 0xFF):
            self.__can_start = True
            print("\033[0;32m[IO {}] mode change\033[0m".format(self.node_id))
        return self.__can_start
    
    @staticmethod
    def start_output() -> None:
        for io in IoModule.io_dict.values():
            if io.__check_bus_status():
                if io.__set_tpdo_mode():
                    if io.set_bus_status("start_remote_node"):
                        io.is_start = True
                        io.set_channel_status(False, "1", "2", "3", "4", "5", "6", "7", "8")
                        print("\033[0;32m[IO {}] start output\033[0m".format(io.node_id))
            else: print("\033[0;31m[IO {}] start output failed\033[0m".format(io.node_id))
        
    @staticmethod
    def stop_output() -> None:
        for io in IoModule.io_dict.values():
            if io.set_bus_status("enter_pre-operational_state"):
                io.is_start = False
                io.set_channel_status(False, "1", "2", "3", "4", "5", "6", "7", "8")
                print("\033[0;32m[IO {}] stop output\033[0m".format(io.node_id))
            else: print("\033[0;31m[IO {}] stop output failed\033[0m".format(io.node_id))
    
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
                self.update_signal.emit()
                # print("\033[0;32m[IO {}] output status: {}\033[0m".format(self.node_id, value_str))
                return True
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