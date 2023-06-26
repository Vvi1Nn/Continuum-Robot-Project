# -*- coding:utf-8 -*-


''' processor.py CANopen总线消息处理模块 v1.6 '''


import time

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入自定义模块
import maxon_motor_epos2.protocol as protocol
from maxon_motor_epos2.generator import CanOpenMsgGenerator


class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None # CANopen总线在CAN卡的通道

    __is_log = False # 是否打印日志

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

    ''' 是否打印日志 '''
    @classmethod
    def is_show_log(cls, is_log: bool) -> None:
        cls.__is_log = is_log

    ''' 匹配地址 '''
    @staticmethod
    def __match_index(index_low, index_high, subindex) -> list:
        # 低位 int转hex
        index_low_str = hex(index_low)[2:].upper()
        index_low_str = (2 - len(index_low_str)) * "0" + index_low_str
        # 高位 int转hex
        index_high_str = hex(index_high)[2:].upper()
        index_high_str = (2 - len(index_high_str)) * "0" + index_high_str
        # 合并 转int
        index = int(index_high_str + index_low_str, 16)
        # 返回列表的形式 以供比较
        return [index, subindex]
    
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

    ''' 获取总线状态 '''
    def get_bus_status(self, /, *, times=1, delay=0.5) -> str:
        if not self.nmt_feedback[0]:
            [cob_id, data] = self.nmt_get_status() # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="remote", data_len="default"):
                    time_stamp = time.time()
                    while time.time() - time_stamp < delay:
                        if self.nmt_feedback[0]:
                            self.bus_status = self.nmt_feedback[1]
                            self.nmt_feedback = (False, "")
                            if CanOpenBusProcessor.__is_log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
                            else: pass
                            return self.bus_status
                        else: pass
                    else:
                        times -= 1
                        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] waiting bus status feedback ...\033[0m".format(self.node_id))
                        else: pass
                else:
                    times -= 1
                    if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
            else: return "error"
        else:
            self.bus_status = self.nmt_feedback[1]
            self.nmt_feedback = (False, "")
            if CanOpenBusProcessor.__is_log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
            else: pass
            return self.bus_status

    ''' 设置总线状态 可设置重复次数 '''
    def set_bus_status(self, label, /, *, times=1) -> bool:
        [cob_id, data] = self.nmt_change_status(label) # 生成消息

        while times != 0:
            if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="remote"):
                if self.get_bus_status(delay=0.5) != "error":
                    # 判断当前状态和设置的状态是否一致
                    if label == "start_remote_node" and self.bus_status == "operational": return True
                    elif label == "stop_remote_node" and self.bus_status == "stopped": return True
                    elif label == "enter_pre-operational_state" and self.bus_status == "pre-operational": return True
                    elif label == "reset_node" and self.bus_status == "pre-operational": return True
                    elif label == "reset_communication" and self.bus_status == "pre-operational": return True
                    else: pass
                else: return False
            else:
                times -= 1
                if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
        else: return False
    
    ''' 检查总线的状态 在操作之前 '''
    def check_bus_status(self, /, *, times=1) -> bool:
        while times != 0:
            if self.set_bus_status("enter_pre-operational_state"):
                self.bus_is_checked = True
                if CanOpenBusProcessor.__is_log: print("\033[0;32m[Node-ID {}] checked\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] checking bus status ...\033[0m".format(self.node_id))
                else: pass
        else: 
            if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] unchecked\033[0m".format(self.node_id))
            else: pass
            return False

    ''' SDO 读 '''
    def sdo_read(self, label: str, /, *, times=1, delay=0.5, format=1):
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
                            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else:
                        times -= 1
                        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] reading {} ...\033[0m".format(self.node_id, label))
                        else: pass
                else:
                    if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] read {} failed\033[0m".format(self.node_id, label))
                    else: pass
                    return None
            else:
                if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] sdo_feedback is wrong\033[0m".format(self.node_id))
                else: pass
                return None
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return None

    ''' SDO 写 32bit '''
    def sdo_write_32(self, label: str, value: int, /, *, times=1, check=True, delay=0.5) -> bool:
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
                            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    ''' SDO 写 16bit '''
    def sdo_write_16(self, label: str, value: int, /, *, times=1, check=True, delay=0.5) -> bool:
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
                            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    '''' SDO 写 8bit '''
    def sdo_write_8(self, label: str, value: int, /, *, times=1, check=True, delay=0.5) -> bool:
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
                            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False

    ''' RPD写操作 分别写低字和高字 '''
    def rpdo(self, channel: str, *args: int, format=None, times=1) -> bool:
        if self.bus_is_checked and self.bus_status == "operational":
            [cob_id, data] = super().rpdo(channel, *args, format=format) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"): return True
                else:
                    times -= 1
                    if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] rpdo {} ...\033[0m".format(self.node_id, channel))
                    else: pass
            else:
                if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] rpdo() {} failed\033[0m".format(self.node_id, channel))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no pdo\033[0m".format(self.node_id))
            return False
