# -*- coding:utf-8 -*-

from ctypes import *
import threading
import time
lib = cdll.LoadLibrary("./libusbcan.so")

ZCAN_DEVICE_TYPE  = c_uint32
ZCAN_DEVICE_INDEX = c_uint32
ZCAN_Reserved     = c_uint32
ZCAN_CHANNEL      = c_uint32
LEN               = c_uint32

USBCAN2       =   ZCAN_DEVICE_TYPE(4)
DEVICE_INDEX  =   ZCAN_DEVICE_INDEX(0)
Reserved      =   ZCAN_Reserved(0)
CHANNEL       =   ZCAN_CHANNEL(0)

def input_thread():
    input()

class ZCAN_CAN_BOARD_INFO(Structure):
    _fields_ = [("hw_Version",c_ushort),
                ("fw_Version",c_ushort),
                ("dr_Version",c_ushort),
                ("in_Version",c_ushort),
                ("irq_Num",c_ushort),
                ("can_Num",c_ubyte),
                ("str_Serial_Num",c_ubyte*20),
                ("str_hw_Type",c_ubyte*40),
                ("Reserved",c_ubyte*4)]

    def __str__(self):
        return "Hardware Version:%s\nFirmware Version:%s\nDriver Version:%s\nInterface:%s\nInterrupt Number:%s\nCAN_number:%d"%(\
                self.hw_Version,  self.fw_Version,  self.dr_Version,  self.in_Version,  self.irq_Num,  self.can_Num)

    def serial(self):
        serial=''
        for c in self.str_Serial_Num:
            if c>0:
                serial +=chr(c)
            else:
                break
        return serial   
        
    def hw_Type(self):
        hw_Type=''
        for c in self.str_hw_Type:
            if c>0:
                hw_Type +=chr(c)
            else:
                break
        return hw_Type   

class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",c_int),
                ("AccMask",c_int),
                ("Reserved",c_int),
                ("Filter",c_ubyte),
                ("Timing0",c_ubyte),
                ("Timing1",c_ubyte),
                ("Mode",c_ubyte)]

class ZCAN_CAN_OBJ(Structure):
    _fields_ = [("ID",c_uint32),
                ("TimeStamp",c_uint32),
                ("TimeFlag",c_uint8),
                ("SendType",c_byte),
                ("RemoteFlag",c_byte),
                ("ExternFlag",c_byte),
                ("DataLen",c_byte),
                ("Data",c_ubyte*8),
                ("Reserved",c_ubyte*3)]

def GetDeviceInf(DeviceType,DeviceIndex):
    try:
        info = ZCAN_CAN_BOARD_INFO()
        ret  = lib.VCI_ReadBoardInfo(DeviceType,DeviceIndex,byref(info))
        return info if ret==1 else None
    except:
        print("Exception on readboardinfo")
        raise
                
def can_start(DEVCIE_TYPE,DEVICE_INDEX,CHANNEL):
    init_config  = ZCAN_CAN_INIT_CONFIG()
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
    
    LEN = 1
    msgs = (ZCAN_CAN_OBJ*LEN)()
    for i in range(LEN):
        msgs[i].ID           =   0x601
        msgs[i].TimeStamp    =   0
        msgs[i].TimeFlag     =   0
        msgs[i].SendType     =   2
        msgs[i].RemoteFlag   =   0
        msgs[i].ExternFlag   =   0
        msgs[i].DataLen      =   8
        # 查看控制模式
        msgs[i].Data[0] = 0x40
        msgs[i].Data[1] = 0x61
        msgs[i].Data[2] = 0x60
        msgs[i].Data[3] = 0x00
        msgs[i].Data[4] = 0x00
        msgs[i].Data[5] = 0x00
        msgs[i].Data[6] = 0x00
        msgs[i].Data[7] = 0x00

    sendret = lib.VCI_Transmit(4, 0, 0, byref(msgs), LEN)
    if LEN == sendret:
        print("transmit success, sendcount is: %d " % sendret)
    else:
        print("transmit fail, sendcounet is: %d " % sendret)

    thread = threading.Thread(target = input_thread)
    thread.start()

    while True:
        time.sleep(0.1)
        ret  =  lib.VCI_GetReceiveNum(4, 0, 0)
        if ret:
            rcv_msgs = (ZCAN_CAN_OBJ*ret)()
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
