# -*- coding:utf-8 -*-


''' io.py IO模块 '''


from PyQt5.QtCore import pyqtSignal, QObject

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from continuum_robot.processor import CanOpenBusProcessor


class IoModule(CanOpenBusProcessor, QObject):
    update_signal = pyqtSignal()

    io_dict = {}
    
    def __init__(self, node_id, slot_function) -> None:
        CanOpenBusProcessor.__init__(self, node_id)
        QObject.__init__(self)

        self.io_dict[node_id] = self

        self.update_signal.connect(slot_function)

        self.__is_initialized = False

        self.output_1 = False
        self.output_2 = False
        self.output_3 = False
        self.output_4 = False
        self.output_5 = False
        self.output_6 = False
        self.output_7 = False
        self.output_8 = False

        self.input_1 = None
        self.input_2 = None
        self.input_3 = None
        self.input_4 = None
        self.input_5 = None
        self.input_6 = None
        self.input_7 = None
        self.input_8 = None
        self.input_9 = None
        self.input_10 = None
        self.input_11 = None
        self.input_12 = None
        self.input_13 = None
        self.input_14 = None
        self.input_15 = None
        self.input_16 = None
    
    ''' 检查总线 '''
    def check_bus_status(self, /, *, times=1, log=False) -> bool:
        if super().check_bus_status(times=times, log=log):
            print("\033[0;32m[IO {}] BUS OK\033[0m".format(self.node_id))
            return True
        else:
            print("\033[0;31m[IO {}] BUS UNCKECKED\033[0m".format(self.node_id))
            return False

    ''' 初始化 '''
    def initialize_device(self, /, *, times=3, check=True, log=False) -> bool:
        if self.bus_is_checked:
            while times != 0:
                if (self.set_tpdo_mode("asynchronous", channel="1", times=1, check=check, delay=0.5, log=log)
                    and
                    self.set_tpdo_inhibit_time(10, channel="1", times=1, check=check, delay=0.5, log=log)):
                    
                    self.__is_initialized = True
                    print("\033[0;32m[IO {}] DEVICE INIT\033[0m".format(self.node_id))
                    return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[IO {}] initializing device ...\033[0m".format(self.node_id))
                    else: pass
            else: 
                if log: print("\033[0;31m[IO {}] initialize device failed\033[0m".format(self.node_id))
                else: pass
                return False
        else:
            if log: print("\033[0;31m[IO {}] bus is unchecked, cannot initialize device\033[0m".format(self.node_id))
            else: pass
            return False
    
    ''' 启动 PDO '''
    def start_device(self, /, *, times=3, check=False, delay=0.5, log=False) -> bool:
        if self.__is_initialized:
            while times != 0:
                if self.start_pdo(times=1, check=check, delay=delay, log=log):
                    print("\033[0;32m[IO {}] DEVICE START\033[0m".format(self.node_id))
                    return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[IO {}] starting device ...\033[0m".format(self.node_id))
                    else: pass
            else:
                if log: print("\033[0;31m[IO {}] start device failed\033[0m".format(self.node_id))
                else: pass
                return False
        else:
            print("\033[0;31m[IO {}] device is uninitialized, cannot start device\033[0m".format(self.node_id))
            return False
    
    ''' 停止 PDO '''
    def stop_device(self, /, *, times=3, check=False, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.stop_pdo(times=1, check=check, delay=delay, log=log):
                print("\033[0;32m[IO {}] DEVICE STOP\033[0m".format(self.node_id))
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[IO {}] stopping device ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[IO {}] stop device failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 设置输出 '''
    def set_output(self, status: bool, *args: str, log=False, times=1) -> bool:
        if self.bus_status == "operational":
            for channel in args:
                try:
                    setattr(self, "output_" + channel, status)
                except Exception as e:
                    print(e)
                    return False
            
            value_str = ""
            for i in range(1, 9):
                value_str = "1" + value_str if getattr(self, f"output_{i}") else "0" + value_str
            
            while times != 0:
                if self.rpdo("1", int(value_str, 2), format=8):
                    self.update_signal.emit() # 向外发送信号 提示更新状态显示

                    if log: print("\033[0;32m[IO {}] OUTPUT STATUS: {}\033[0m".format(self.node_id, value_str))
                    else: pass
                    return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[IO {}] setting output ...\033[0m".format(self.node_id))
                    else: pass
            else:
                for channel in args: setattr(self, "output_" + channel, not status)
                
                if log: print("\033[0;31m[IO {}] set output failed\033[0m".format(self.node_id))
                else: pass
                return False
        else:
            if log: print("\033[0;31m[IO {}] bus status is not operational\033[0m".format(self.node_id))
            else: pass
            return False
    
    ''' 1 '''
    def open_valve_1(self) -> bool:
        return self.set_output(True, "1")
    
    def close_valve_1(self) -> bool:
        return self.set_output(False, "1")
    
    ''' 2 '''
    def open_valve_2(self) -> bool:
        return self.set_output(True, "2")
    
    def close_valve_2(self) -> bool:
        return self.set_output(False, "2")
    
    ''' 3 '''
    def open_valve_3(self) -> bool:
        return self.set_output(True, "3")
    
    def close_valve_3(self) -> bool:
        return self.set_output(False, "3")
    
    ''' 4 '''
    def open_valve_4(self) -> bool:
        return self.set_output(False, "4")
    
    def close_valve_4(self) -> bool:
        return self.set_output(True, "4")
    
    # ''' 配置TPDO参数 启动PDO通讯 控制电磁阀的初始状态 '''
    # @staticmethod
    # def start_output() -> None:
    #     for io in IoModule.io_dict.values():
    #         print("=============================================================")
    #         if io.__check_bus_status():
    #             if io.__set_tpdo_mode():
    #                 if io.set_bus_status("start_remote_node"):
    #                     io.is_start = True
    #                     # io.set_channel_status(False, "1", "2", "3") # 3个小爪
    #                     # io.set_channel_status(True, "4") # 大爪
    #                     print("\033[0;32m[IO {}] START OUTPUT\033[0m".format(io.node_id))
    #                 else: print("\033[0;31m[IO {}] In function [start_output], set bus status failed\033[0m".format(io.node_id))
    #             else: print("\033[0;31m[IO {}] In function [start_output], set tpdo mode failed\033[0m".format(io.node_id))
    #         else: print("\033[0;31m[IO {}] In function [start_output], check bus status failed\033[0m".format(io.node_id))
    
   