# -*- coding:utf-8 -*-


import time

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import canopen.protocol as protocol
from canopen.generator import CanOpenMsgGenerator


class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None
    __is_log = True

    def __init__(self, node_id) -> None:
        super().__init__(node_id)
        self.bus_status = None
    
    @classmethod
    def link_device(cls, device: object) -> None:
        cls.device = device

    @classmethod
    def is_show_log(cls, is_log):
        cls.__is_log = is_log

    @staticmethod
    def __match_index(index_low, index_high, subindex) -> list:
        index_low_str = hex(index_low)[2:].upper()
        index_low_str = (2 - len(index_low_str)) * "0" + index_low_str
        
        index_high_str = hex(index_high)[2:].upper()
        index_high_str = (2 - len(index_high_str)) * "0" + index_high_str
        
        index_str = index_high_str + index_low_str
        
        index = int(index_str, 16)

        return [index, subindex]
    
    @staticmethod
    def __hex_list_to_int(data_list) -> int:
        data_str = ""
        
        for i in range(len(data_list)):
            data_bin = bin(data_list[i])[2:]
            if len(data_bin) < 8:
                data_bin = '0' * (8 - len(data_bin)) + data_bin
            data_str = data_bin + data_str

        if int(data_str[0]) == 0:
            return int(data_str, 2)
        else:
            return - ((int(data_str, 2) ^ 0xFFFFFFFF) + 1)

    def __send_msg(self, cob_id, data, /, *, remote_flag="data", data_len="default", check=True):
        CanOpenBusProcessor.device.clear_buffer()
        while not CanOpenBusProcessor.device.send(cob_id, [data], remote_flag, data_len):
            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] sending message ...\033[0m".format(self.node_id))
            time.sleep(0.05)
        time.sleep(0.05)
        if check:
            [num, msg] = CanOpenBusProcessor.device.read_buffer(1)
            return [num, msg]
    
    def get_bus_status(self, wait=0) -> str:
        [cob_id, data] = self.nmt_get_status() # 计算消息内容
        [num, msg] = self.__send_msg(cob_id, data, remote_flag="remote")
        if num != 0:
            if msg[0].ID == self.node_id + protocol.CAN_ID["NMT_S"]:
                for k in protocol.NMT_STATUS:
                    if msg[0].Data[0] & 0b01111111 == protocol.NMT_STATUS[k]:
                        self.bus_status = k
                        if CanOpenBusProcessor.__is_log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
                        return self.bus_status
        if wait == 0:
            print("\033[0;31m[Node-ID {}] get_bus_status() failed\033[0m".format(self.node_id))
            return "get_bus_status_error"
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] trying get_bus_status() again ...\033[0m".format(self.node_id))
        return self.get_bus_status(wait=wait-1)

    def set_bus_status(self, label, /, *, wait=0) -> bool:
        [cob_id, data] = self.nmt_change_status(label)
        self.__send_msg(cob_id, data, data_len="remote", check=False)
        self.get_bus_status()
        if label == "start_remote_node" and self.bus_status == "operational": return True
        elif label == "stop_remote_node" and self.bus_status == "stopped": return True
        elif label == "enter_pre-operational_state" and self.bus_status == "pre-operational": return True
        elif label == "reset_node" and self.bus_status == "pre-operational": return True
        elif label == "reset_communication" and self.bus_status == "pre-operational": return True
        if wait == 0:
            print("\033[0;31m[Node-ID {}] set_bus_status() failed\033[0m".format(self.node_id))
            return False
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] trying set_bus_status() again ...\033[0m".format(self.node_id))
        return self.set_bus_status(label, wait=wait-1)
    
    def sdo_read(self, label: str, /, *, wait=0, format=1) -> list:
        [cob_id, data] = super().sdo_read(label)
        [num, msg] = self.__send_msg(cob_id, data)
        if num != 0:
            if msg[0].ID == self.node_id + protocol.CAN_ID["SDO_T"] and (msg[0].Data[0] == protocol.CMD_R["read_16"] or msg[0].Data[0] == protocol.CMD_R["read_32"] or msg[0].Data[0] == protocol.CMD_R["read_8"]) and self.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                if format == 1:
                    return [self.__hex_list_to_int([msg[0].Data[4], msg[0].Data[5], msg[0].Data[6], msg[0].Data[7]])]
                elif format == 2:
                    return [self.__hex_list_to_int([msg[0].Data[4], msg[0].Data[5]]), self.__hex_list_to_int([msg[0].Data[6], msg[0].Data[7]])]
                elif format == 4:
                    return [self.__hex_list_to_int([msg[0].Data[4]]), self.__hex_list_to_int([msg[0].Data[5]]), self.__hex_list_to_int([msg[0].Data[6]]), self.__hex_list_to_int([msg[0].Data[7]])]
        if wait == 0:
            print("\033[0;31m[Node-ID {}] sdo_read() failed\033[0m".format(self.node_id))
            return None
        if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] trying sdo_read() again ...\033[0m".format(self.node_id))
        return self.sdo_read(label, wait=wait-1)

    def sdo_write_32(self, label: str, value: int, /, *, check=True, wait=0) -> bool:
        [cob_id, data] = super().sdo_write_32(label, value)
        [num, msg] = self.__send_msg(cob_id, data, check=check)
        if check:
            if num != 0:
                if msg[0].ID == self.node_id + protocol.CAN_ID["SDO_T"] and msg[0].Data[0] == protocol.CMD_R["write"] and self.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                    return True
            if wait == 0:
                print("\033[0;31m[Node-ID {}] sdo_write_32() failed\033[0m".format(self.node_id))
                return False
            if CanOpenBusProcessor.__is_log: print("\033[0;33m[Node-ID {}] trying sdo_write_32() again ...\033[0m".format(self.node_id))
            return self.sdo_write_32(label, value, wait=wait-1)
    
    def rpdo(self, channel: str, value_low: int, value_high: int) -> None:
        [cob_id, data] = super().rpdo(channel, value_low, value_high)
        self.__send_msg(cob_id, data, check=False)

    def check_bus_status(self) -> bool:
        self.get_bus_status()
        if self.bus_status == "pre-operational":
            print("\033[0;32m[Node-ID {}] checked\033[0m".format(self.node_id))
            return True
        self.set_bus_status("enter_pre-operational_state")
        print("\033[0;33m[Node-ID {}] set right status, try check again\033[0m".format(self.node_id))
    
    