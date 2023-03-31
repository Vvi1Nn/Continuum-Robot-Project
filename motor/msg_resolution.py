# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as pro

def resolve(cob_id, data) -> str:
    msg = ""

    if cob_id > 0x200 and cob_id < 0x280:
        msg = msg + "[RPDO1]"
        node_id = cob_id - 0x200


    elif cob_id > 0x300 and cob_id < 0x380:
        msg = msg + "[RPDO2]"
        node_id = cob_id - 0x300
    
    elif cob_id > 0x400 and cob_id < 0x480:
        msg = msg + "[RPDO3]"
        node_id = cob_id - 0x400
    
    elif cob_id > 0x500 and cob_id < 0x580:
        msg = msg + "[RPDO4]"
        node_id = cob_id - 0x500
    
    elif cob_id > 0x580 and cob_id < 0x600:
        msg = msg + "[SDO_R] "
        node_id = cob_id - 0x580
        
        if data[0] == pro.CMD_R["read_32"] or data[0] == pro.CMD_R["read_16"] or data[0] == pro.CMD_R["read_8"]:
            msg = msg + "电机{} ".format(node_id)
            address = __match_index(data[1], data[2], data[3])
            if address == pro.OD["control_word"]:
                msg = msg + "控制字: "
            elif address == pro.OD["status_word"]:
                msg = msg + "状态字: "
            elif address == pro.OD["show_mode"]:
                msg = msg + "控制模式: "
            elif address == pro.OD["position_feedback"]:
                msg = msg + "当前位置: "
            elif address == pro.OD["speed_feedback"]:
                msg = msg + "当前速度: "
            elif address == pro.OD["target_position"]:
                msg = msg + "目标位置: "
            elif address == pro.OD["velocity"]:
                msg = msg + "运行速度: "
            elif address == pro.OD["acceleration"]:
                msg = msg + "加速度: "
            elif address == pro.OD["deceleration"]:
                msg = msg + "减速度: "
            elif address == pro.OD["target_speed"]:
                msg = msg + "目标速度: "
        
        elif data[0] == pro.CMD_R["write"]:
            address = __match_index(data[1], data[2], data[3])
            if address == pro.OD["control_word"]:
                msg = msg + "成功写入电机{}的控制字".format(node_id)
            elif address == pro.OD["control_mode"]:
                msg = msg + "成功写入电机{}的控制模式".format(node_id)
            elif address == pro.OD["target_position"]:
                msg = msg + "成功写入电机{}的目标位置".format(node_id)
            elif address == pro.OD["velocity"]:
                msg = msg + "成功写入电机{}的运行速度".format(node_id)
            elif address == pro.OD["acceleration"]:
                msg = msg + "成功写入电机{}的加速度".format(node_id)
            elif address == pro.OD["deceleration"]:
                msg = msg + "成功写入电机{}的减速度".format(node_id)
            elif address == pro.OD["target_speed"]:
                msg = msg + "成功写入电机{}的目标速度".format(node_id)
        
        elif data[0] == pro.CMD_R["read_error"] or data[0] == pro.CMD_R["write_error"]:
            msg = msg + "错误: 无法对电机{}的地址 [{}{}--{}] 进行操作!!!".format(node_id, hex(data[2])[2:].upper(), hex(data[1])[2:].upper(), hex(data[3])[2:].upper())

    return msg

def __match_index(index_low, index_high, subindex) -> list:
    
    index_low_str = hex(index_low)[2:].upper()
    index_high_str = hex(index_high)[2:].upper()
    
    index_str = index_high_str + index_low_str
    
    index = int(index_str, 16)

    return [index, subindex]

def __hex2int(data_list):
    
    pass

if __name__ == "__main__":
    ret = resolve(0x581, [0X80,0X61,0X60,0X00,0X01,0X00,0X00,0X00])
    print(ret)