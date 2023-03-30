# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so")

import motor.protocol as pro
import motor.msg_generation as gen
import usbcan.struct as struct
import usbcan.param as param

class Motor:
    
    def __init__(self,
                #  device,
                 node_id,
                 control_mode = "position_control",
                 acceleration = 1000,
                 deceleration = 10000,) -> None:
        
        self.id = node_id
        self.mode = control_mode
        self.acc = acceleration
        self.dec = deceleration
        # self.device = device

        self.__set_mode(self.mode)
    
    def __set_mode(self, flag):
        
        if type(flag) != str:
            print("[SetMode] flag类型错误!!!")
            return None
    
        num = 0
        for key in pro.CONTROL_MODE.keys():
            if flag != key:
                num = num + 1
            else:
                break
            if num == len(pro.CONTROL_MODE.keys()):
                print("[SetMode] {}不存在!!!".format(flag))
                return None

        ret = gen.sdo_write_32(self.id, "control_mode", pro.CONTROL_MODE[flag])
        cob_id = ret["id"]
        data = ret["data"]

        msg = (struct.ZCAN_CAN_OBJ * 1)()

        msg[0].ID           =   cob_id
        msg[0].TimeStamp    =   param.TIME_STAMP["off"]
        msg[0].TimeFlag     =   param.TIME_FLAG["off"]
        msg[0].SendType     =   param.SEND_TYPE["normal"]
        msg[0].RemoteFlag   =   param.REMOTE_FLAG["data"]
        msg[0].ExternFlag   =   param.EXTERN_FLAG["standard"]
        msg[0].DataLen      =   param.DATA_LEN["default"]

        for i in range(msg[0].DataLen):
            msg[0].Data[i] = data[i]
        
        send_num = USBCAN_Lib.VCI_Transmit(4, 0, 0, byref(msg), 1)
        if send_num == 1:
            print("[ModeSetting] {} 成功!!!".format(flag))
        else:
            print("[ModeSetting] 失败...")