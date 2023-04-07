# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as pro

def sdo_read(node_id, flag, is_print = False) -> list:
    
    if type(flag) != str:
        print("\033[0;31m[MsgGen] sdo_read() flag error\033[0m")
        return False
    
    num = 0
    for key in pro.OD.keys():
        if flag != key:
            num = num + 1
        else:
            break
        if num == len(pro.OD.keys()):
            print("\033[0;31m[MsgGen] sdo_read() {} doesn't exist\033[0m".format(flag))
            return False
    
    index = pro.OD[flag][0]
    subindex = pro.OD[flag][1]
    
    cob_id = pro.CAN_ID["SDO_R"] + node_id
    
    data = [0x00] * 8
    data[0] = pro.CMD_T["read"]
    data[1] = __split_index(index)[0]
    data[2] = __split_index(index)[1]
    data[3] = subindex
    
    if is_print:
        hex_data = ["00"] * 8
        print("[SDOread] {}".format(flag))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")

    return [cob_id, data]

def sdo_write_32(node_id, flag, value, is_print = False) -> list:
    
    if type(flag) != str:
        print("[0;31m[MsgGen] sdo_write_32() flag error\033[0m")
        return False
    
    num = 0
    for key in pro.OD.keys():
        if flag != key:
            num = num + 1
        else:
            break
        if num == len(pro.OD.keys()):
            print("\033[0;31m[MsgGen] sdo_write_32() {} doesn't exist\033[0m".format(flag))
            return False
    
    if type(value) != int or value < -0x80000000 or value > 0x7FFFFFFF:
        print("[0;31m[MsgGen] sdo_write_32() value error\033[0m")
        return False
    
    index = pro.OD[flag][0]
    subindex = pro.OD[flag][1]
    
    cob_id = pro.CAN_ID["SDO_R"] + node_id
    
    data = [0x00] * 8
    data[0] = pro.CMD_T["write_32"]
    data[1] = __split_index(index)[0]
    data[2] = __split_index(index)[1]
    data[3] = subindex
    
    value = __int2hex(value)
    for i in range(4):
        data[4+i] = value[i]
    
    if is_print:
        hex_data = ["00"] * 8
        print("[SDOwrite32] {}".format(flag))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def rpdo(num, node_id, value_low, value_high, is_print = False) -> dict:
    
    if type(num) != int or num < 1 or num > 4:
        print("\033[0;31m[MsgGen]: rpdo() num error\033[0m")
        return False
    if type(node_id) != int or node_id < 1 or node_id > 0x7F:
        print("\033[0;31m[MsgGen]: rpdo() node_id error\033[0m")
        return False
    if type(value_low) != int or value_low < -0x80000000 or value_low > 0x7FFFFFFF:
        print("\033[0;31m[MsgGen]: rpdo() value_low error\033[0m")
        return False
    if type(value_high) != int or value_high < -0x80000000 or value_high > 0x7FFFFFFF:
        print("\033[0;31m[MsgGen]: rpdo() value_high error\033[0m")
        return False
    
    cob_id = pro.CAN_ID["RPDO_" + str(num)] + node_id

    data = [0x00] * 8
    value_low = __int2hex(value_low)
    for i in range(4):
        data[i] = value_low[i]
    value_high = __int2hex(value_high)
    for i in range(4):
        data[4+i] = value_high[i]

    if is_print:
        hex_data = ["00"] * 8
        print("[RPDO_{}]".format(num))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def tpdo_start(node_id, is_print = False) -> dict:
    if type(node_id) != int or node_id < 1 or node_id > 0x7F:
        print("\033[0;31m[MsgGen]: tpdo_start() node_id error\033[0m")
        return False
    
    cob_id = 0x0
    data = [0x01, node_id, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]

    if is_print:
        hex_data = ["00"] * 8
        print("[StartPDO]")
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def __split_index(index) -> list:
    
    hex_str = hex(index)[2:].upper()
    
    high_str = hex_str[0:2]
    low_str = hex_str[2:]
    
    high_int = int(high_str, 16)
    low_int = int(low_str, 16)
    
    return [low_int, high_int]

def __int2hex(value_int) -> list:
    
    if type(value_int) != int or value_int > 0x7FFFFFFF or value_int < -0x80000000:
        print("[__int2hex] value 错误!!!")
        return False
    
    if value_int >= 0:
        value_str = hex(value_int)[2:].upper()
        if len(value_str) < 8:
            value_str = '0'*(8-len(value_str)) + value_str
    else:
        value_str = hex(int(bin(- value_int ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
    
    value_list = [0x0] * 4
    value_list[0] = int(value_str[6:8], 16)
    value_list[1] = int(value_str[4:6], 16)
    value_list[2] = int(value_str[2:4], 16)
    value_list[3] = int(value_str[0:2], 16)
    
    return value_list

if __name__ == "__main__":
    sdo_read(1, "control_mode", True) # 查看控制模式
    # sdo_write_32(1, "control_mode", pro.CONTROL_MODE["position_control"], True) # 位置模式
    # sdo_write_32(1, "acceleration", 1000, True) # 加速度1000
    # sdo_write_32(1, "deceleration", 10000, True) # 减速度10000
    # sdo_write_32(1, "velocity", 100, True) # 速度100
    # sdo_write_32(1, "target_position", 10000, True) # 目标位置10000
    # sdo_write_32(1, "control_word", 0x6F, True) # 相对使能
    # sdo_write_32(1, "control_word", 0x7F, True) # 启动
    # start_pdo(3, True)
    rpdo(1, 5, 10000, -10000, True)
    sdo_write_32(1, "tpdo_2_timer", 100, True)