# -*- coding:utf-8 -*-

from ctypes import *
import threading
import time

import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param
import motor.msg_generation as motor_gen
import motor.protocol as motor_pro
import motor.function as motor_fun

lib = cdll.LoadLibrary("./libusbcan.so")

def input_thread():
    input()

# class ZCAN_CAN_BOARD_INFO(Structure):
#     _fields_ = [("hw_Version",c_ushort),
#                 ("fw_Version",c_ushort),
#                 ("dr_Version",c_ushort),
#                 ("in_Version",c_ushort),
#                 ("irq_Num",c_ushort),
#                 ("can_Num",c_ubyte),
#                 ("str_Serial_Num",c_ubyte*20),
#                 ("str_hw_Type",c_ubyte*40),
#                 ("Reserved",c_ubyte*4)]

#     def __str__(self):
#         return "Hardware Version:%s\nFirmware Version:%s\nDriver Version:%s\nInterface:%s\nInterrupt Number:%s\nCAN_number:%d"%(\
#                 self.hw_Version,  self.fw_Version,  self.dr_Version,  self.in_Version,  self.irq_Num,  self.can_Num)

#     def serial(self):
#         serial=''
#         for c in self.str_Serial_Num:
#             if c>0:
#                 serial +=chr(c)
#             else:
#                 break
#         return serial   
        
#     def hw_Type(self):
#         hw_Type=''
#         for c in self.str_hw_Type:
#             if c>0:
#                 hw_Type +=chr(c)
#             else:
#                 break
#         return hw_Type   

# class ZCAN_CAN_INIT_CONFIG(Structure):
#     _fields_ = [("AccCode",c_int),
#                 ("AccMask",c_int),
#                 ("Reserved",c_int),
#                 ("Filter",c_ubyte),
#                 ("Timing0",c_ubyte),
#                 ("Timing1",c_ubyte),
#                 ("Mode",c_ubyte)]

# class ZCAN_CAN_OBJ(Structure):
#     _fields_ = [("ID",c_uint32),
#                 ("TimeStamp",c_uint32),
#                 ("TimeFlag",c_uint8),
#                 ("SendType",c_byte),
#                 ("RemoteFlag",c_byte),
#                 ("ExternFlag",c_byte),
#                 ("DataLen",c_byte),
#                 ("Data",c_ubyte*8),
#                 ("Reserved",c_ubyte*3)]

def GetDeviceInf(DeviceType,DeviceIndex):
    try:
        info = usbcan_struct.ZCAN_CAN_BOARD_INFO()
        ret  = lib.VCI_ReadBoardInfo(DeviceType,DeviceIndex,byref(info))
        return info if ret==1 else None
    except:
        print("Exception on readboardinfo")
        raise
                
def can_start(DEVCIE_TYPE,DEVICE_INDEX,CHANNEL):
    init_config  = usbcan_struct.ZCAN_CAN_INIT_CONFIG()
    init_config.AccCode    = 0
    init_config.AccMask    = 0xFFFFFFFF
    init_config.Reserved   = 0
    init_config.Filter     = 1
    init_config.Timing0    = 0x01
    init_config.Timing1    = 0x1c
    init_config.Mode       = 0
    ret=lib.VCI_InitCAN(DEVCIE_TYPE,DEVICE_INDEX,CHANNEL,byref(init_config))
    if ret ==0:
        print("InitCAN fail!")
    else:
        print("InitCAN success!")
        
    ret=lib.VCI_StartCAN(DEVCIE_TYPE,DEVICE_INDEX,CHANNEL)
    if ret ==0:
        print("StartCAN fail!")
    else:
        print("StartCAN success!")
    return ret

if __name__=="__main__":

    ret = lib.VCI_OpenDevice(4, 0, 0)
    if ret == 0:
        print("Opendevice fail!")
    else:
        print("Opendevice success!")
    
    canstart = can_start(4, 0, 0)
    
    # LEN = 8
    # msgs = (usbcan_struct.ZCAN_CAN_OBJ*LEN)()
    # for i in range(LEN):
    #     msgs[i].ID           =   0x601
    #     msgs[i].TimeStamp    =   0
    #     msgs[i].TimeFlag     =   0
    #     msgs[i].SendType     =   usbcan_param.SEND_TYPE["normal"]
    #     msgs[i].RemoteFlag   =   0
    #     msgs[i].ExternFlag   =   0
    #     msgs[i].DataLen      =   8
    # # 查看控制模式
    # msgs[0].Data[0] = 0x40
    # msgs[0].Data[1] = 0x61
    # msgs[0].Data[2] = 0x60
    # msgs[0].Data[3] = 0x00
    # msgs[0].Data[4] = 0x00
    # msgs[0].Data[5] = 0x00
    # msgs[0].Data[6] = 0x00
    # msgs[0].Data[7] = 0x00
    # # 位置模式
    # msgs[1].Data[0] = 0x2F
    # msgs[1].Data[1] = 0x60
    # msgs[1].Data[2] = 0x60
    # msgs[1].Data[3] = 0x00
    # msgs[1].Data[4] = 0x01
    # msgs[1].Data[5] = 0x00
    # msgs[1].Data[6] = 0x00
    # msgs[1].Data[7] = 0x00
    # # 加速度1000
    # msgs[2].Data[0] = 0x23
    # msgs[2].Data[1] = 0x83
    # msgs[2].Data[2] = 0x60
    # msgs[2].Data[3] = 0x00
    # msgs[2].Data[4] = 0xE8
    # msgs[2].Data[5] = 0x03
    # msgs[2].Data[6] = 0x00
    # msgs[2].Data[7] = 0x00
    # # 减速度10000
    # msgs[3].Data[0] = 0x23
    # msgs[3].Data[1] = 0x84
    # msgs[3].Data[2] = 0x60
    # msgs[3].Data[3] = 0x00
    # msgs[3].Data[4] = 0x10
    # msgs[3].Data[5] = 0x27
    # msgs[3].Data[6] = 0x00
    # msgs[3].Data[7] = 0x00
    # # 速度100
    # msgs[4].Data[0] = 0x23
    # msgs[4].Data[1] = 0x81
    # msgs[4].Data[2] = 0x60
    # msgs[4].Data[3] = 0x00
    # msgs[4].Data[4] = 0x64
    # msgs[4].Data[5] = 0x00
    # msgs[4].Data[6] = 0x00
    # msgs[4].Data[7] = 0x00
    # # 目标位置10000
    # msgs[5].Data[0] = 0x23
    # msgs[5].Data[1] = 0x7A
    # msgs[5].Data[2] = 0x60
    # msgs[5].Data[3] = 0x00
    # msgs[5].Data[4] = 0x10
    # msgs[5].Data[5] = 0x27
    # msgs[5].Data[6] = 0x00
    # msgs[5].Data[7] = 0x00
    # # 相对使能
    # msgs[6].Data[0] = 0x2B
    # msgs[6].Data[1] = 0x40
    # msgs[6].Data[2] = 0x60
    # msgs[6].Data[3] = 0x00
    # msgs[6].Data[4] = 0x6F
    # msgs[6].Data[5] = 0x00
    # msgs[6].Data[6] = 0x00
    # msgs[6].Data[7] = 0x00
    # # 相对运行
    # msgs[7].Data[0] = 0x2B
    # msgs[7].Data[1] = 0x40
    # msgs[7].Data[2] = 0x60
    # msgs[7].Data[3] = 0x00
    # msgs[7].Data[4] = 0x7F
    # msgs[7].Data[5] = 0x00
    # msgs[7].Data[6] = 0x00
    # msgs[7].Data[7] = 0x00

    # sendret = lib.VCI_Transmit(4, 0, 0, byref(msgs), LEN)
    # if LEN == sendret:
    #     print("transmit success, sendcount is: %d " % sendret)
    # else:
    #     print("transmit fail, sendcounet is: %d " % sendret)
    
    motor_2 = motor_fun.Motor(2)
    
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
    # 位置模式
    # msgs_2[1].Data[0] = 0x2F
    # msgs_2[1].Data[1] = 0x60
    # msgs_2[1].Data[2] = 0x60
    # msgs_2[1].Data[3] = 0x00
    # msgs_2[1].Data[4] = 0x01
    # msgs_2[1].Data[5] = 0x00
    # msgs_2[1].Data[6] = 0x00
    # msgs_2[1].Data[7] = 0x00
    # data = motor_gen.sdo_write_32(1, "control_mode", motor_pro.CONTROL_MODE["position_control"])["data"] # 位置模式
    # for j in range(msgs_2[1].DataLen):
    #     msgs_2[1].Data[j] = data[j]
    # 加速度1000
    # msgs_2[2].Data[0] = 0x23
    # msgs_2[2].Data[1] = 0x83
    # msgs_2[2].Data[2] = 0x60
    # msgs_2[2].Data[3] = 0x00
    # msgs_2[2].Data[4] = 0xE8
    # msgs_2[2].Data[5] = 0x03
    # msgs_2[2].Data[6] = 0x00
    # msgs_2[2].Data[7] = 0x00
    # data = motor_gen.sdo_write_32(1, "acceleration", 1000)["data"] # 加速度1000
    # for j in range(msgs_2[2].DataLen):
    #     msgs_2[2].Data[j] = data[j]
    # 减速度10000
    # msgs_2[3].Data[0] = 0x23
    # msgs_2[3].Data[1] = 0x84
    # msgs_2[3].Data[2] = 0x60
    # msgs_2[3].Data[3] = 0x00
    # msgs_2[3].Data[4] = 0x10
    # msgs_2[3].Data[5] = 0x27
    # msgs_2[3].Data[6] = 0x00
    # msgs_2[3].Data[7] = 0x00
    # data = motor_gen.sdo_write_32(1, "deceleration", 10000)["data"] # 减速度10000
    # for j in range(msgs_2[3].DataLen):
    #     msgs_2[3].Data[j] = data[j]
    # 速度100
    # msgs_2[4].Data[0] = 0x23
    # msgs_2[4].Data[1] = 0x81
    # msgs_2[4].Data[2] = 0x60
    # msgs_2[4].Data[3] = 0x00
    # msgs_2[4].Data[4] = 0x64
    # msgs_2[4].Data[5] = 0x00
    # msgs_2[4].Data[6] = 0x00
    # msgs_2[4].Data[7] = 0x00
    # data = motor_gen.sdo_write_32(1, "velocity", 100)["data"] # 速度100
    # for j in range(msgs_2[4].DataLen):
    #     msgs_2[4].Data[j] = data[j]
    # 目标位置10000
    # msgs_2[5].Data[0] = 0x23
    # msgs_2[5].Data[1] = 0x7A
    # msgs_2[5].Data[2] = 0x60
    # msgs_2[5].Data[3] = 0x00
    # msgs_2[5].Data[4] = 0x10
    # msgs_2[5].Data[5] = 0x27
    # msgs_2[5].Data[6] = 0x00
    # msgs_2[5].Data[7] = 0x00
    data = motor_gen.sdo_write_32(1, "target_position", 20000)["data"] # 目标位置10000
    for j in range(msgs_2[5].DataLen):
        msgs_2[5].Data[j] = data[j]
    # 相对使能
    # msgs_2[6].Data[0] = 0x2B
    # msgs_2[6].Data[1] = 0x40
    # msgs_2[6].Data[2] = 0x60
    # msgs_2[6].Data[3] = 0x00
    # msgs_2[6].Data[4] = 0x6F
    # msgs_2[6].Data[5] = 0x00
    # msgs_2[6].Data[6] = 0x00
    # msgs_2[6].Data[7] = 0x00
    data = motor_gen.sdo_write_32(1, "control_word", 0x6F)["data"] # 相对使能
    for j in range(msgs_2[6].DataLen):
        msgs_2[6].Data[j] = data[j]
    # 相对运行
    # msgs_2[7].Data[0] = 0x2B
    # msgs_2[7].Data[1] = 0x40
    # msgs_2[7].Data[2] = 0x60
    # msgs_2[7].Data[3] = 0x00
    # msgs_2[7].Data[4] = 0x7F
    # msgs_2[7].Data[5] = 0x00
    # msgs_2[7].Data[6] = 0x00
    # msgs_2[7].Data[7] = 0x00
    data = motor_gen.sdo_write_32(1, "control_word", 0x7F)["data"] # 启动
    for j in range(msgs_2[7].DataLen):
        msgs_2[7].Data[j] = data[j]

    sendret = lib.VCI_Transmit(4, 0, 0, byref(msgs_2), LEN)
    if LEN == sendret:
        print("transmit success, sendcount is: %d " % sendret)
    else:
        print("transmit fail, sendcounet is: %d " % sendret)
    
    

    thread = threading.Thread(target = input_thread)
    thread.start()

    while True:
        time.sleep(0.1)
        ret = lib.VCI_GetReceiveNum(4, 0, 0)
        if ret:
            rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ*ret)()
            ret1 = lib.VCI_Receive(4, 0, 0, byref(rcv_msgs), ret, 100) 
            for i in range(ret1):
                print("GetNum:%d, OrderNUM:%d, Timestamp:%d, id:%s, dlc:%d, data:%s"%(ret,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
                                                                                       rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))
        if thread.is_alive() == False:
            break

    ret = lib.VCI_ResetCAN(4, 0, 0)
    if ret == 0:
        print("ResetCAN fail!")
    else:
        print("ResetCAN success!")

    ret=lib.VCI_CloseDevice(4, 0)
    if ret == 0:
        print("Closedevice Failed!")
    else:
        print("Closedevice success!")
    
    del lib
    print('done')

