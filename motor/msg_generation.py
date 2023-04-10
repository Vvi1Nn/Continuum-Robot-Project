# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as pro
import motor.module as module


def sdo_read(node_id, flag, is_print = False) -> list:
    
    index = pro.OD[flag][0]
    subindex = pro.OD[flag][1]
    
    cob_id = pro.CAN_ID["SDO_R"] + node_id
    
    data = [0x00] * 8
    data[0] = pro.CMD_T["read"]
    data[1] = module.split_index(index)[0]
    data[2] = module.split_index(index)[1]
    data[3] = subindex
    
    if is_print:
        hex_data = ["00"] * 8
        print("[sdo_read] {}".format(flag))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")

    return [cob_id, data]

def sdo_write_32(node_id, flag, value, is_print = False) -> list:
    
    index = pro.OD[flag][0]
    subindex = pro.OD[flag][1]
    
    cob_id = pro.CAN_ID["SDO_R"] + node_id
    
    data = [0x00] * 8
    data[0] = pro.CMD_T["write_32"]
    data[1] = module.split_index(index)[0]
    data[2] = module.split_index(index)[1]
    data[3] = subindex
    
    value = module.int2hex(value)
    for i in range(4):
        data[4+i] = value[i]
    
    if is_print:
        hex_data = ["00"] * 8
        print("[sdo_write_32] {}".format(flag))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def rpdo(num, node_id, value_low, value_high, is_print = False) -> dict:
    
    cob_id = pro.CAN_ID["RPDO_" + str(num)] + node_id

    data = [0x00] * 8
    value_low = module.int2hex(value_low)
    for i in range(4):
        data[i] = value_low[i]
    value_high = module.int2hex(value_high)
    for i in range(4):
        data[4+i] = value_high[i]

    if is_print:
        hex_data = ["00"] * 8
        print("[rpdo_{}]".format(num))
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def nmt_change_status(node_id, flag, is_print = False) -> list:
    
    cob_id = pro.CAN_ID["NMT_C"]
    data = [pro.CMD_NMT[flag], node_id, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]

    if is_print:
        hex_data = ["00"] * 8
        print("[nmt_control]")
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]

def nmt_get_status(node_id, is_print = False) -> list:
    
    cob_id = pro.CAN_ID["NMT_S"] + node_id
    data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

    if is_print:
        hex_data = ["00"] * 8
        print("[nmt_status]")
        print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
        print("Data-List: ", end = "")
        for i in range(len(data)):
            hex_data[i] = hex(data[i])[2:].upper()
            print("{} ".format(hex_data[i]), end = "")
        print("")
    
    return [cob_id, data]


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
    nmt_change_status(2, "start_remote_node", True)
    nmt_get_status(2, True)
