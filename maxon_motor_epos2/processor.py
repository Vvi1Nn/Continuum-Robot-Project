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
    def get_bus_status(self, /, *, times=1, delay=0.2) -> str:
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
                        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] getting bus status ...\033[0m".format(self.node_id))
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
    def check_bus_status(self) -> bool:
        self.get_bus_status() # 先获取总线的状态并更新
        # 判断状态是否是预操作状态
        if self.bus_status == "pre-operational":
            self.bus_is_checked = True
            if CanOpenBusProcessor.__is_log: print("\033[0;32m[Node-ID {}] checked\033[0m".format(self.node_id))
            return True
        # 不是预操作状态 人为设置一次总线状态 提示再次检查
        if self.set_bus_status("enter_pre-operational_state"):
            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] try check again\033[0m".format(self.node_id))
            return False
        # 人为设置也失败了
        if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] unchecked\033[0m".format(self.node_id))
        return False

    ''' 使用SDO进行寄存器读取数据 '''
    def sdo_read(self, label: str, /, *, repeat=0, format=1) -> list:
        [cob_id, data] = super().sdo_read(label) # 生成消息
        ret = self.__send_msg(cob_id, data)
        if ret != False: # 消息发送成功
            [num, msg] = ret # 接收应答数据
            if num != 0: # 有数据
                # 判断应答的数据里 CMD和地址是否对应正确
                if msg[0].ID == self.node_id + protocol.CAN_ID["SDO_T"] and (msg[0].Data[0] == protocol.CMD_R["read_16"] or msg[0].Data[0] == protocol.CMD_R["read_32"] or msg[0].Data[0] == protocol.CMD_R["read_8"]) and self.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                    if format == 1: # 4个hex转换成1个int
                        return [self.__hex_list_to_int([msg[0].Data[4], msg[0].Data[5], msg[0].Data[6], msg[0].Data[7]])]
                    elif format == 2: # 4个hex转换成2个int
                        return [self.__hex_list_to_int([msg[0].Data[4], msg[0].Data[5]]), self.__hex_list_to_int([msg[0].Data[6], msg[0].Data[7]])]
                    elif format == 4: # 4个hex转换成4个int
                        return [self.__hex_list_to_int([msg[0].Data[4]]), self.__hex_list_to_int([msg[0].Data[5]]), self.__hex_list_to_int([msg[0].Data[6]]), self.__hex_list_to_int([msg[0].Data[7]])]
        # 上述所有操作有失败
        if repeat == 0:
            if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] read {} failed\033[0m".format(self.node_id, label))
            return None
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] read {} ...\033[0m".format(self.node_id, label))
        return self.sdo_read(label, repeat=repeat-1)

    ''' 使用SDO进行寄存器写操作 可设置是否需要进行写操作应答的校对 '''
    def sdo_write_32(self, label: str, value: int, /, *, check=True, repeat=0) -> bool:
        [cob_id, data] = super().sdo_write_32(label, value) # 生成消息
        ret = self.__send_msg(cob_id, data, check=check) # 先接收返回值
        if ret != False: # 发送成功
            if check: # 需要校对
                [num, msg] = ret # 接收应答消息
                if num != 0: # 有消息
                    if msg[0].ID == self.node_id + protocol.CAN_ID["SDO_T"] and msg[0].Data[0] == protocol.CMD_R["write"] and self.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                        return True # 应答消息的CMD和地址正确
            else: return True # 无需校对
        # 上述所有操作有失败
        if repeat == 0:
            if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
            return False
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] write {} ...\033[0m".format(self.node_id, label))
        return self.sdo_write_32(label, value, repeat=repeat-1)
    
    ''' SDO 8bit '''
    def sdo_write_8(self, label: str, value: int, /, *, check=True, repeat=0) -> bool:
        [cob_id, data] = super().sdo_write_8(label, value) # 生成消息
        ret = self.__send_msg(cob_id, data, check=check) # 先接收返回值
        if ret != False: # 发送成功
            if check: # 需要校对
                [num, msg] = ret # 接收应答消息
                if num != 0: # 有消息
                    if msg[0].ID == self.node_id + protocol.CAN_ID["SDO_T"] and msg[0].Data[0] == protocol.CMD_R["write"] and self.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                        return True # 应答消息的CMD和地址正确
            else: return True # 无需校对
        # 上述所有操作有失败
        if repeat == 0:
            if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
            return False
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] write {} ...\033[0m".format(self.node_id, label))
        return self.sdo_write_8(label, value, repeat=repeat-1)

    ''' RPD写操作 分别写低字和高字 '''
    def rpdo(self, channel: str, *args: int, format=None, repeat=0) -> bool:
        [cob_id, data] = super().rpdo(channel, *args, format=format) # 生成消息
        # 直接发送 无需校验
        if self.__send_msg(cob_id, data, check=False): return True
        # 上述所有操作有失败
        if repeat == 0:
            if CanOpenBusProcessor.__is_log: print("\033[0;31m[Node-ID {}] rpdo() {} failed\033[0m".format(self.node_id, channel))
            return False
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] rpdo {} ...\033[0m".format(self.node_id, channel))
        return self.rpdo(channel, *args, format=format, repeat=repeat-1)
