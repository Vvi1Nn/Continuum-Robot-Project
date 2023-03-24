# -*- coding:utf-8 -*-
import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so")

from USBCAN.Structs import *
from USBCAN.Parameters import *

class DeviceUSBCAN:
    def __init__(self,
                 DeviceType  = device_param.DEVICE_TYPE["USBCAN2"], 
                 DeviceIndex = device_param.DEVICE_INDEX["0"],
                 Channel     = device_param.CHANNEL["0"],
                 Reserved    = device_param.RESERVED,
                 Timer0      = init_param.TIMER["250K"][0], 
                 Timer1      = init_param.TIMER["250K"][1],
                 AccCode     = init_param.ACC_CODE["default"],
                 AccMask     = init_param.ACC_MASK["default"],
                 Filter      = init_param.FILTER["single"],
                 Mode        = init_param.MODE["listen_only"],
                 ) -> None:
        
        self.DeviceType = DeviceType
        self.DeviceIndex = DeviceIndex
        self.Channel = Channel
        # self.Reserved = Reserved
        # self.Timer0 = Timer0
        # self.Timer1 = Timer1
        # self.AccCode = AccCode
        # self.AccMask = AccMask
        # self.Filter = Filter
        # self.Mode = Mode

        self.DeviceInfo = ZCAN_CAN_BOARD_INFO()

        self.InitConfig = ZCAN_CAN_INIT_CONFIG()
        self.InitConfig.AccCode  = AccCode
        self.InitConfig.AccMask  = AccMask
        self.InitConfig.Reserved = Reserved
        self.InitConfig.Filter   = Filter
        self.InitConfig.Timer0   = Timer0
        self.InitConfig.Timer1   = Timer1
        self.InitConfig.Mode     = Mode

    def __str__(self) -> str:
        return "------USBCAN当前配置------\n型号:{} 设备号:{} 通道:{}".format(self.DeviceType, self.DeviceIndex, self.Channel)

    def Open(self) -> bool:
        open_device_success = USBCAN_Lib.VCI_OpenDevice(self.DeviceType, self.DeviceIndex, self.Reserved)
        if open_device_success == 1:
            print("设备已成功打开！")
        else:
            print("无法打开设备...")
        return True if open_device_success == 1 else False
    
    def Reset(self) -> bool:
        reset_success = USBCAN_Lib.VCI_ResetCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if reset_success == 1:
            print("重置设备成功!!!")
        else:
            print("重置设备失败...")
        return True if reset_success == 1 else False
    
    def Close(self) -> bool:
        close_success = USBCAN_Lib.VCI_CloseDevice(self.DeviceType, self.DeviceIndex, self.Channel)
        if close_success == 1:
            print("关闭设备成功!!!")
        else:
            print("关闭设备失败...")
        return True if close_success == 1 else False
    
    def GetInfo(self) -> ZCAN_CAN_BOARD_INFO:
        try:
            read_info_success = USBCAN_Lib.VCI_ReadBoardInfo(self.DeviceType, self.DeviceIndex, byref(self.DeviceInfo))
            return self.DeviceInfo if read_info_success == 1 else None
        except:
            print("【异常】GetInfo() ...")
            raise
    
    def StartCAN(self) -> bool:
        init_success = USBCAN_Lib.VCI_InitCAN(self.DeviceType, self.DeviceIndex, self.Channel, byref(self.InitConfig))
        if init_success == 1:
            print("CAN初始化成功!!!")
        else:
            print("CAN初始化失败...")
        start_success = USBCAN_Lib.VCI_StartCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if start_success == 1:
            print("CAN启动成功!!!")
        else:
            print("CAN启动失败...")
        return True if start_success == 1 else False

    def SendMsgs(self, id, data, length = 1,
                 data_len    = send_param.DATA_LEN["default"],
                 time_stamp  = send_param.TIME_STAMP["off"],
                 time_flag   = send_param.TIME_FLAG["off"],
                 send_type   = send_param.SEND_TYPE["normal"], 
                 remote_flag = send_param.REMOTE_FLAG["data"],
                 extern_flag = send_param.EXTERN_FLAG["standard"],
                 ) -> bool:
        msgs = (ZCAN_CAN_OBJ * length)()
        for i in range(length):
            msgs[i].ID         = id
            msgs[i].TimeStamp  = time_stamp
            msgs[i].TimeFlag   = time_flag
            msgs[i].SendType   = send_type
            msgs[i].RemoteFlag = remote_flag
            msgs[i].ExternFlag = extern_flag
            msgs[i].DataLen    = data_len
            if type(data) == list:
                if len(data) != data_len:
                    print("数据长度错误!!!")
                    return False
                else:
                    pass
            elif type(data) == str: # 例如: "40 60 60 00 00 00 00 00"
                for j in range(msgs[i].DataLen):
                    msgs[i].Data[j] = int(data[3*i:3*(i+1)], 16)
        
        send_num = USBCAN_Lib.VCI_Transmit(self.DeviceType, self.DeviceIndex, self.Channel, byref(msgs), length)
        if length == send_num:
            print("传输成功!!! 数量:{}".format(send_num))
            return True
        else:
            print("传输失败... 数量:{}".format(send_num))
            return False

# def OpenDevice(DeviceType, DeviceIndex):
#     open_device_success = USBCAN_Lib.VCI_OpenDevice(DeviceType, DeviceIndex, Reserved)
#     if open_device_success == 1:
#         print("设备已成功打开！")
#     else:
#         print("无法打开设备...")
#     return True if open_device_success == 1 else False

# def ResetDevice(DeviceType, DeviceIndex, Channel):
#     reset_success = USBCAN_Lib.VCI_ResetCAN(DeviceType, DeviceIndex, Channel)
#     if reset_success == 1:
#         print("重置设备成功!!!")
#     else:
#         print("重置设备失败...")
#     return True if reset_success == 1 else False

# def CloseDevice(DeviceType, DeviceIndex, Channel):
#     close_success = USBCAN_Lib.VCI_CloseDevice(DeviceType, DeviceIndex, Channel)
#     if close_success == 1:
#         print("关闭设备成功!!!")
#     else:
#         print("关闭设备失败...")
#     return True if close_success == 1 else False

# def GetDeviceInfo(DeviceType, DeviceIndex):
#     try:
#         device_info = ZCAN_CAN_BOARD_INFO()
#         success = USBCAN_Lib.VCI_ReadBoardInfo(DeviceType, DeviceIndex, byref(device_info))
#         return device_info if success == 1 else "读取信息失败..."
#     except:
#         print("【异常】GetDeviceInfo() ...")
#         raise

# def StartCAN(DeviceType, DeviceIndex, Channel,
#              acc_code = 0,
#              acc_mask = 0xFFFFFFFF,
#              reserved = 0 ,
#              filter   = 1,
#              timing0  = 0x00,
#              timing1  = 0x1c,
#              mode     = 0,
#              ):
    
#     init_config = ZCAN_CAN_INIT_CONFIG()
    
#     init_config.AccCode  = acc_code
#     init_config.AccMask  = acc_mask
#     init_config.Reserved = reserved
#     init_config.Filter   = filter
#     init_config.Timing0  = timing0
#     init_config.Timing1  = timing1
#     init_config.Mode     = mode

#     init_success = USBCAN_Lib.VCI_InitCAN(DeviceType, DeviceIndex, 0, byref(init_config))
#     if init_success == 1:
#         print("CAN初始化成功!!!")
#     else:
#         print("CAN初始化失败...")
        
#     start_success = USBCAN_Lib.VCI_StartCAN(DeviceType, DeviceIndex, Channel)
#     if start_success == 1:
#         print("CAN启动成功!!!")
#     else:
#         print("CAN启动失败...")
#     return True if start_success == 1 else False

# def SendMsgs(DeviceType, DeviceIndex, Channel, id, data, 
#              length      = 1,
#              time_stamp  = 0, 
#              time_flag   = 0, 
#              send_type   = 0, 
#              remote_flag = 0, 
#              extern_flag = 0, 
#              data_len    = 8,
#              ) -> bool:
    
#     msgs = (ZCAN_CAN_OBJ * length)()
#     for i in range(length):
#         msgs[i].ID         = id
#         msgs[i].TimeStamp  = time_stamp
#         msgs[i].TimeFlag   = time_flag
#         msgs[i].SendType   = send_type
#         msgs[i].RemoteFlag = remote_flag
#         msgs[i].ExternFlag = extern_flag
#         msgs[i].DataLen    = data_len
#         if type(data) == list:
#             if len(data) != data_len:
#                 print("数据长度错误!!!")
#                 return False
#             else:
#                 pass
#         elif type(data) == str: # 例如: "40 60 60 00 00 00 00 00"
#             for j in range(msgs[i].DataLen):
#                 msgs[i].Data[j] = int(data[3*i:3*(i+1)], 16)
    
#     send_num = USBCAN_Lib.VCI_Transmit(DeviceType, DeviceIndex, Channel, byref(msgs), length)
#     if length == send_num:
#         print("传输成功!!! 数量:{}".format(send_num))
#         return True
#     else:
#         print("传输失败... 数量:{}".format(send_num))
#         return False

device_1 = DeviceUSBCAN()
print(device_1)