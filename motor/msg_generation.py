# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as protocol

class Motor:
    
    def __init__(self, node_id, usbcan) -> None:
        self.node_id = node_id
        self.usbcan = usbcan
    
    def __split_index(self, index) -> list:
        hex_str = hex(index)[2:].upper()
        high_str = hex_str[0:2]
        low_str = hex_str[2:]
        high_int = int(high_str, 16)
        low_int = int(low_str, 16)
        return [low_int, high_int]
    
    def __int2hex(self, value_int) -> list:
        value_str = hex(value_int)[2:].upper()
        if len(value_str) < 8:
            value_str = '0'*(8-len(value_str)) + value_str
        value_list = [0x0] * 4
        value_list[0] = int(value_str[6:8], 16)
        value_list[1] = int(value_str[4:6], 16)
        value_list[2] = int(value_str[2:4], 16)
        value_list[3] = int(value_str[0:2], 16)
        return value_list

    def sdo_read(self, flag, isSend = False) -> dict:
        if type(flag) != str:
            print("sdo_read(): flag 错误!!!")
            return None
        num = 0
        for key in protocol.OD.keys():
            if flag != key:
                num = num + 1
            else:
                break
            if num == len(protocol.OD.keys()):
                print("{}不存在!!!".format(flag))
                return None
        index = protocol.OD[flag][0]
        subindex = protocol.OD[flag][1]
        cob_id = protocol.CAN_ID["SDO_T"] + self.node_id
        data = [0x00] * 8
        data[0] = protocol.CMD_T["read"]
        data[1] = self.__split_index(index)[0]
        data[2] = self.__split_index(index)[1]
        data[3] = subindex
        if isSend:
            self.usbcan.SendMsgs(cob_id, data)
        return {"COB-ID": cob_id, "data": data}

    def sdo_write_32(self, flag, value, isSend = False) -> dict:
        if type(flag) != str:
            print("sdo_write_32(): flag 错误!!!")
            return None
        num = 0
        for key in protocol.OD.keys():
            if flag != key:
                num = num + 1
            else:
                break
            if num == len(protocol.OD.keys()):
                print("{}不存在!!!".format(flag))
                return None
        if type(value) != int:
            print("sdo_write_32(): value 错误!!!")
            return None
        if value < 0 or value > 0xFFFFFFFF:
            print("value超出范围!!!")
            return None
        index = protocol.OD[flag][0]
        subindex = protocol.OD[flag][1]
        cob_id = protocol.CAN_ID["SDO_T"] + self.node_id
        data = [0x00] * 8
        data[0] = protocol.CMD_T["write_32"]
        data[1] = self.__split_index(index)[0]
        data[2] = self.__split_index(index)[1]
        data[3] = subindex
        value = self.__int2hex(value)
        for i in range(4):
            data[4+i] = value[i]
        if isSend:
            self.usbcan.SendMsgs(cob_id, data)
        return {"COB-ID": cob_id, "data": data}

    def rpdo_1(self, value):
        if type(value) != int:
            print("sdo_write_32(): value 错误!!!")
            return None
        if value < 0 or value > 0xFFFFFFFF:
            print("value超出范围!!!")
            return None
        cob_id = protocol.CAN_ID["RPDO_1"] + self.node_id

        pass