# -*- coding:utf-8 -*-


''' processor.py CAN总线处理模块 '''


import time

from PyQt5.QtCore import QThread, pyqtSignal

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入自定义模块
from continuum_robot.canopen import CanOpenMsgGenerator


''' CANopen 发送 '''
class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None # CANopen总线在CAN卡的通道

    node_dict = {} # 节点对象字典

    def __init__(self, node_id) -> None:
        super().__init__(node_id)

        self.bus_status = None # 总线状态

        self.bus_is_checked = False # 总线检查

        self.sdo_feedback = (False, None, "", None) # 是否有反馈 状态 标签 数据
        
        self.nmt_feedback = (False, "") # 是否有反馈 状态 标签

        CanOpenBusProcessor.node_dict[node_id] = self # 加入节点对象列表
    

    ''' 绑定设备 '''
    @classmethod
    def link_device(cls, device: object) -> None:
        cls.device = device


    ''' 获取总线状态 '''
    def get_bus_status(self, /, *, times=1, delay=0.5, log=False) -> str:
        [cob_id, data] = self.nmt_get_status() # 生成消息

        self.nmt_feedback = (False, "")

        while times != 0:
            if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="remote", data_len="default"):
                time_stamp = time.time()
                while time.time() - time_stamp < delay:
                    if self.nmt_feedback[0]:
                        self.bus_status = self.nmt_feedback[1]
                        if log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
                        return self.bus_status
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] no nmt feedback ...\033[0m".format(self.node_id))
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
        else: return "error"

    ''' 设置总线状态 '''
    def set_bus_status(self, label: str, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        [cob_id, data] = self.nmt_change_status(label) # 生成消息

        while times != 0:
            if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="remote"):
                if check:
                    if self.get_bus_status(delay=delay, log=log) != "error":
                        # 判断当前状态和设置的状态是否一致
                        if label == "start_remote_node" and self.bus_status == "operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            return True
                        elif label == "stop_remote_node" and self.bus_status == "stopped":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            return True
                        elif label == "enter_pre-operational_state" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            return True
                        elif label == "reset_node" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            return True
                        elif label == "reset_communication" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            return True
                        else:
                            if log: print("\033[0;31m[Node-ID {}] bus status not match\033[0m".format(self.node_id))
                            return False
                    else:
                        if log: print("\033[0;31m[Node-ID {}] get bus status failed, return is error\033[0m".format(self.node_id))
                        return False
                else:
                    if label == "start_remote_node": self.bus_status = "operational"
                    elif label == "stop_remote_node": self.bus_status = "stopped"
                    elif label == "enter_pre-operational_state": self.bus_status = "pre-operational"
                    elif label == "reset_node": self.bus_status = "pre-operational"
                    elif label == "reset_communication": self.bus_status = "pre-operational"
                    else: pass

                    if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                    return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
                else: pass
        else: return False
    
    ''' 检查总线的状态 '''
    def check_bus_status(self, /, *, times=1, log=False) -> bool:
        if not self.bus_is_checked:
            while times != 0:
                if self.bus_status == "pre-operational" or self.set_bus_status("enter_pre-operational_state", log=log):
                    self.bus_is_checked = True
                    if log: print("\033[0;32m[Node-ID {}] checked\033[0m".format(self.node_id))
                    return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] checking bus status ...\033[0m".format(self.node_id))
            else: 
                if log: print("\033[0;31m[Node-ID {}] unchecked\033[0m".format(self.node_id))
                return False
        else:
            if log: print("\033[0;32m[Node-ID {}] already checked\033[0m".format(self.node_id))
            return True

    ''' SDO 读 '''
    def sdo_read(self, label: str, /, *, times=1, delay=0.5, format=1, log=False):
        if self.bus_is_checked and self.bus_status != "stopped":
            if not self.sdo_feedback[0]:
                [cob_id, data] = super().sdo_read(label) # 生成消息

                while times != 0:
                    if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        if format == 1:
                                            value_list = [self.__hex_list_to_int(self.sdo_feedback[3])]
                                        elif format == 2:
                                            value_list = [self.__hex_list_to_int(self.sdo_feedback[3][0:2]), self.__hex_list_to_int(self.sdo_feedback[3][2:4])]
                                        elif format == 4:
                                            value_list = [self.__hex_list_to_int([self.sdo_feedback[3][0]]), self.__hex_list_to_int([self.sdo_feedback[3][1]]), self.__hex_list_to_int([self.sdo_feedback[3][2]]), self.__hex_list_to_int([self.sdo_feedback[3][3]])]
                                        else: value_list = None
                                        self.sdo_feedback = (False, None, "", None)
                                        return value_list
                                    else: return None
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Node-ID {}] reading {} ...\033[0m".format(self.node_id, label))
                        else: pass
                else:
                    if log: print("\033[0;31m[Node-ID {}] read {} failed\033[0m".format(self.node_id, label))
                    else: pass
                    return None
            else:
                if log: print("\033[0;31m[Node-ID {}] sdo_feedback is wrong\033[0m".format(self.node_id))
                else: pass
                return None
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return None

    ''' SDO 写 32bit '''
    def sdo_write_32(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":
            [cob_id, data] = super().sdo_write_32(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    ''' SDO 写 16bit '''
    def sdo_write_16(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":
            [cob_id, data] = super().sdo_write_16(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    '''' SDO 写 8bit '''
    def sdo_write_8(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":    
            [cob_id, data] = super().sdo_write_8(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False

    ''' RPD写操作 分别写低字和高字 '''
    def rpdo(self, channel: str, *args: int, format=None, times=1, log=False) -> bool:
        if self.bus_is_checked and self.bus_status == "operational":
            [cob_id, data] = super().rpdo(channel, *args, format=format) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"): return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] rpdo {} ...\033[0m".format(self.node_id, channel))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] rpdo() {} failed\033[0m".format(self.node_id, channel))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no pdo\033[0m".format(self.node_id))
            return False


    ''' 设置 TPDO 模式 '''
    def set_tpdo_mode(self, label="asynchronous", /, *, channel: str, times=1, check=True, delay=0.5, log=True) -> bool:
        while times != 0:
            if self.sdo_write_8("tpdo_{}_transmission_type".format(channel), CanOpenMsgGenerator.TPDO_MODE[label], check=check, delay=delay):
                if log: print("\033[0;32m[Node-ID {}] tpdo {} transmission type: {}\033[0m".format(self.node_id, channel, label))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] setting tpdo {} transmission type ...\033[0m".format(self.node_id, channel))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] set tpdo {} transmission type failed\033[0m".format(self.node_id, channel))
            else: pass
            return False
    
    ''' 设置 TPDO 禁止时间 '''
    def set_tpdo_inhibit_time(self, value=10, /, *, channel: str, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_8("tpdo_{}_inhibit_time".format(channel), value, check=check, delay=delay):
                if log: print("\033[0;32m[Node-ID {}] tpdo {} inhibit time: {}\033[0m".format(self.node_id, channel, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] setting tpdo {} inhibit time ...\033[0m".format(self.node_id, channel))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] set tpdo {} inhibit time failed\033[0m".format(self.node_id, channel))
            else: pass
            return False


    ''' 开启PDO '''
    def start_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("start_remote_node", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Node-ID {}] PDO START\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] starting pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] start pdo failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 关闭PDO '''
    def stop_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("enter_pre-operational_state", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Node-ID {}] PDO STOP\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] stopping pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] stop pdo failed\033[0m".format(self.node_id))
            else: pass
            return False


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
