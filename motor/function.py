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
                 deceleration = 10000,
                 velocity     = 100,
                 ) -> None:
        
        self.id = node_id
        self.mode = control_mode
        self.acc = acceleration
        self.dec = deceleration
        self.vel = velocity
        # self.device = device

        self.__set_mode()
        self.__set_acc()
        self.__set_dec()
        self.__set_vel()
    
    def __set_mode(self) -> None:
        
        if type(self.mode) != str:
            print("[SetMode] flag类型错误!!!")
            return None
    
        num = 0
        for key in pro.CONTROL_MODE.keys():
            if self.mode != key:
                num = num + 1
            else:
                break
            if num == len(pro.CONTROL_MODE.keys()):
                print("[SetMode] {}不存在!!!".format(self.mode))
                return None

        ret = gen.sdo_write_32(self.id, "control_mode", pro.CONTROL_MODE[self.mode])
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
            print("[SetMode] {} 成功!!!".format(self.mode))
        else:
            print("[SetMode] 失败...")

    def __set_acc(self) -> None:
        
        if type(self.acc) != int:
            print("[SetAcc] acceleration类型错误!!!")
            return None
    
        if self.acc < 0 or self.acc > 1000:
            print("[SetAcc] acceleration超出范围!!!")
            return None

        ret = gen.sdo_write_32(self.id, "acceleration", self.acc)
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
            print("[SetAcc] {} 成功!!!".format(self.acc))
        else:
            print("[SetAcc] 失败...")

    def __set_dec(self) -> None:
        
        if type(self.dec) != int:
            print("[SetDec] deceleration类型错误!!!")
            return None
    
        if self.dec < 0 or self.dec > 10000:
            print("[SetDec] deceleration超出范围!!!")
            return None

        ret = gen.sdo_write_32(self.id, "deceleration", self.dec)
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
            print("[SetDec] {} 成功!!!".format(self.dec))
        else:
            print("[SetDec] 失败...")

    def __set_vel(self) -> None:
        
        if type(self.vel) != int:
            print("[SetVel] velocity类型错误!!!")
            return None
    
        if self.vel < 0 or self.vel > 200:
            print("[SetVel] velocity超出范围!!!")
            return None

        ret = gen.sdo_write_32(self.id, "velocity", self.vel)
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
            print("[SetVel] {} 成功!!!".format(self.vel))
        else:
            print("[SetVel] 失败...")