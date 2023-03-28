# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as protocol

class Motor:
    
    def __init__(self, node_id) -> None:
        self.node_id = node_id
    
    def __split_index(self, index):
        hex_str = hex(index)[2:].upper()
        high_str = hex_str[0:2]
        low_str = hex_str[2:]
        high_int = int(high_str, 16)
        low_int = int(low_str, 16)
        return {"high": high_int, "low": low_int}
        pass

    def read(self, index, subindex) -> dict or bool:
        cob_id = protocol.CAN_ID["SDO_T"] + self.node_id
        data = [0x00] * 8
        data[0] = protocol.CMD_T["read"]
        data[1] = self.__split_index(index)["low"]
        data[2] = self.__split_index(index)["high"]
        data[3] = subindex
        return {"COB-ID": cob_id, "data": data}
