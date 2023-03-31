import threading
import time
import platform
import sys
import os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

import usbcan.function as usbcan_fun
import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param
import motor.msg_generation as motor_gen
import motor.protocol as motor_proto 
import motor.function as motor_fun

def input_thread():
    input()

if __name__=="__main__":
    
    usbcan_1 = usbcan_fun.UsbCan()

    motor_2 = motor_fun.Motor(2)
    motor_1 = motor_fun.Motor(1)

    msg = (usbcan_struct.ZCAN_CAN_OBJ*8)()
    for i in range(8):
        msg[i].ID           =   0x601
        msg[i].TimeStamp    =   usbcan_param.TIME_STAMP["off"]
        msg[i].TimeFlag     =   usbcan_param.TIME_FLAG["off"]
        msg[i].SendType     =   usbcan_param.SEND_TYPE["normal"]
        msg[i].RemoteFlag   =   usbcan_param.REMOTE_FLAG["data"]
        msg[i].ExternFlag   =   usbcan_param.EXTERN_FLAG["standard"]
        msg[i].DataLen      =   usbcan_param.DATA_LEN["default"]

    data = motor_gen.sdo_write_32(1, "target_position", 20000)["data"] # 目标位置10000
    for j in range(msg[5].DataLen):
        msg[5].Data[j] = data[j]
    
    data = motor_gen.sdo_write_32(1, "control_word", 0x6F)["data"] # 相对使能
    for j in range(msg[6].DataLen):
        msg[6].Data[j] = data[j]
    
    data = motor_gen.sdo_write_32(1, "control_word", 0x7F)["data"] # 启动
    for j in range(msg[7].DataLen):
        msg[7].Data[j] = data[j]
    
    LEN = 8
    msgs_2 = (usbcan_struct.ZCAN_CAN_OBJ*LEN)()
    for i in range(LEN):
        msgs_2[i].ID           =   0x602
        msgs_2[i].TimeStamp    =   usbcan_param.TIME_STAMP["off"]
        msgs_2[i].TimeFlag     =   usbcan_param.TIME_FLAG["off"]
        msgs_2[i].SendType     =   usbcan_param.SEND_TYPE["normal"]
        msgs_2[i].RemoteFlag   =   usbcan_param.REMOTE_FLAG["data"]
        msgs_2[i].ExternFlag   =   usbcan_param.EXTERN_FLAG["standard"]
        msgs_2[i].DataLen      =   usbcan_param.DATA_LEN["default"]

    data = motor_gen.sdo_write_32(1, "target_position", 20000)["data"] # 目标位置10000
    for j in range(msgs_2[5].DataLen):
        msgs_2[5].Data[j] = data[j]
    
    data = motor_gen.sdo_write_32(1, "control_word", 0x6F)["data"] # 相对使能
    for j in range(msgs_2[6].DataLen):
        msgs_2[6].Data[j] = data[j]
    
    data = motor_gen.sdo_write_32(1, "control_word", 0x7F)["data"] # 启动
    for j in range(msgs_2[7].DataLen):
        msgs_2[7].Data[j] = data[j]

    sendret = USBCAN_Lib.VCI_Transmit(4, 0, 0, byref(msgs_2), LEN)
    if LEN == sendret:
        print("transmit success, sendcount is: %d " % sendret)
    else:
        print("transmit fail, sendcounet is: %d " % sendret)


    sendret = USBCAN_Lib.VCI_Transmit(4, 0, 0, byref(msg), LEN)
    if LEN == sendret:
        print("transmit success, sendcount is: %d " % sendret)
    else:
        print("transmit fail, sendcounet is: %d " % sendret)

    thread = threading.Thread(target = input_thread)
    thread.start()

    while True:
        time.sleep(0.1)
        ret = USBCAN_Lib.VCI_GetReceiveNum(4, 0, 0)
        if ret:
            rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ*ret)()
            ret1 = USBCAN_Lib.VCI_Receive(4, 0, 0, byref(rcv_msgs), ret, 100) 
            for i in range(ret1):
                print("GetNum:%d, OrderNUM:%d, Timestamp:%d, id:%s, dlc:%d, data:%s"%(ret,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
                                                                                       rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))
        if thread.is_alive() == False:
            break