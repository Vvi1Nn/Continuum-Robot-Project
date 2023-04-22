# -*- coding:utf-8 -*-


import time
import protocol
from generator import CanOpenMsgGenerator


class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None
    __is_log = True

    def __init__(self, is_print=False) -> None:
        super().__init__(is_print)
    
    @classmethod
    def link_device(cls, device: object) -> None:
        cls.device = device

    @staticmethod
    def __match_index(index_low, index_high, subindex) -> list:
        index_low_str = hex(index_low)[2:].upper()
        index_high_str = hex(index_high)[2:].upper()
        
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

    @classmethod
    def get_bus_status(cls, node_id: int, wait = 5):
        [cob_id, data] = CanOpenMsgGenerator.nmt_get_status(node_id) # 计算消息内容
        cls.device.clear_buffer() # 清空缓存，让下一条接收的消息置于最前面，以供读取
        while not cls.device.send(cob_id, [data], remote_flag = "remote"):
            if cls.__is_log: print("\033[0;33m[Node-ID {}] sending message ...\033[0m".format(node_id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = cls.device.read_buffer(1)
        if num != 0:
            if msg[0].ID == node_id + protocol.CAN_ID["NMT_S"]:
                for k in protocol.NMT_STATUS:
                    if msg[0].Data[0] & 0b01111111 == protocol.NMT_STATUS[k]:
                        return k
            else:
                if wait == 0:
                    print("\033[0;31m[Node-ID {}] get_bus_status() failed\033[0m".format(node_id))
                    return False
                if cls.__is_log: print("\033[0;33m[Node-ID {}] trying get_bus_status() again ...\033[0m".format(node_id))
                return cls.get_bus_status(wait=wait-1)
        else:
            if wait == 0:
                    print("\033[0;31m[Node-ID {}] get_bus_status() failed\033[0m".format(node_id))
                    return False
            if cls.__is_log: print("\033[0;33m[Node-ID {}] trying get_bus_status() again ...\033[0m".format(node_id))
            return cls.get_bus_status(wait=wait-1)
        
    @classmethod
    def sdo_read(cls, node_id: int, label: str):
        [cob_id, data] = super().sdo_read(node_id, label)
        cls.device.clear_buffer()
        while not cls.device.send(cob_id, [data]):
            if cls.__is_log: print("\033[0;33m[Motor {}] trying send motor status updating msg ...\033[0m".format(node_id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = cls.device.read_buffer(1)
        if num != 0:
            if msg[0].ID == node_id + protocol.CAN_ID["SDO_T"] and (msg[0].Data[0] == protocol.CMD_R["read_16"] or msg[0].Data[0] == protocol.CMD_R["read_32"] or msg[0].Data[0] == protocol.CMD_R["read_8"]) and cls.__match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == protocol.OD[label]:
                value = cls.__hex_list_to_int([msg[0].Data[4]])
                for key in protocol.STATUS_WORD:
                    for r in protocol.STATUS_WORD[key]:
                        if value == r:
                            self.motor_status = key
                            if log: print("\033[0;32m[Motor {}] motor status: {}\033[0m".format(self.id, self.motor_status))
                            return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] update motor status failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying update motor status again ...\033[0m".format(self.id))
                return self.__update_motor_status(wait = wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] update motor status failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying update motor status again ...\033[0m".format(self.id))
            return self.__update_motor_status(wait = wait - 1)