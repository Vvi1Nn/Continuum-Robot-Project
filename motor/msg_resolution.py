# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as pro

def resolve(cob_id, data) -> str:
    msg = ""
    if len(data) < 8:
        for i in range(8-len(data)):
            data.append(0x00)

    if cob_id > 0x180 and cob_id < 0x200:
        
        mode = "TPDO1"
        node_id = cob_id - 0x200

        value_low = __hex2int([data[0], data[1], data[2], data[3]])
        value_high = __hex2int([data[4], data[5], data[6], data[7]])

        msg = "[{}] 低 = {} 高 = {}".format(mode, value_low, value_high)

    elif cob_id > 0x280 and cob_id < 0x300:
        
        mode = "TPDO2"
        node_id = cob_id - 0x300

        value_low = __hex2int([data[0], data[1], data[2], data[3]])
        value_high = __hex2int([data[4], data[5], data[6], data[7]])

        msg = "[{}] 低 = {} 高 = {}".format(mode, value_low, value_high)
    
    elif cob_id > 0x380 and cob_id < 0x400:
        
        mode = "TPDO3"
        node_id = cob_id - 0x400

        value_low = __hex2int([data[0], data[1], data[2], data[3]])
        value_high = __hex2int([data[4], data[5], data[6], data[7]])

        msg = "[{}] 低 = {} 高 = {}".format(mode, value_low, value_high)
    
    elif cob_id > 0x480 and cob_id < 0x500:
        
        mode = "TPDO4"
        node_id = cob_id - 0x500

        value_low = __hex2int([data[0], data[1], data[2], data[3]])
        value_high = __hex2int([data[4], data[5], data[6], data[7]])

        msg = "[{}] 低 = {} 高 = {}".format(mode, value_low, value_high)
    
    elif cob_id > 0x580 and cob_id < 0x600:

        mode = "SDO"
        node_id = cob_id - 0x580
        
        for key in pro.CMD_R.keys():
            if data[0] == pro.CMD_R[key]:
                cmd = key
        
        address = match_index(data[1], data[2], data[3])
        for key in pro.OD.keys():
            if address == pro.OD[key]:
                od = key
        
        value = __hex2int([data[4], data[5], data[6], data[7]])
        
        if cmd[0:4] == "read":
            if od == "control_word" or od == "status_word":
                value_bin = bin(value)[2:]
                value_bin = '0' * (8 - len(value_bin)) + value_bin
                msg = "[{}] 读取 {} = {}".format(mode, od, value_bin)
            else:
                msg = "[{}] 读取 {} = {}".format(mode, od, value)
        elif cmd == "write":
            msg = "[{}] 写入 [{}] 成功!".format(mode, od)
        else:
            msg = "[{}] {}... [{}] 不存在".format(mode, cmd, od)
        
    return msg

def match_index(index_low, index_high, subindex) -> list:
    
    index_low_str = hex(index_low)[2:].upper()
    index_high_str = hex(index_high)[2:].upper()
    
    index_str = index_high_str + index_low_str
    
    index = int(index_str, 16)

    return [index, subindex]

def __hex2int(data_list) -> int:
    
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

if __name__ == "__main__":
    ret = resolve(0x581, [0x43,0x40,0x60,0x00,0x7f,0x00,0x00,0x00])
    print(ret)
