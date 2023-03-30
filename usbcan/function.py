# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so")

import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param

class UsbCan:
    
    def __init__(self,
                 DeviceType  = usbcan_param.DEVICE_TYPE["USBCAN2"], 
                 DeviceIndex = usbcan_param.DEVICE_INDEX["0"],
                 Channel     = usbcan_param.CHANNEL["0"],
                 Reserved    = usbcan_param.RESERVED,
                 Timer0      = usbcan_param.TIMER["250K"][0], 
                 Timer1      = usbcan_param.TIMER["250K"][1],
                 AccCode     = usbcan_param.ACC_CODE["default"],
                 AccMask     = usbcan_param.ACC_MASK["default"],
                 Filter      = usbcan_param.FILTER["single"],
                 Mode        = usbcan_param.MODE["normal"],
                 ) -> None:
        
        self.DeviceType = DeviceType
        self.DeviceIndex = DeviceIndex
        self.Channel = Channel

        self.DeviceInfo = usbcan_struct.ZCAN_CAN_BOARD_INFO()

        self.InitConfig = usbcan_struct.ZCAN_CAN_INIT_CONFIG()
        self.InitConfig.AccCode  = AccCode
        self.InitConfig.AccMask  = AccMask
        self.InitConfig.Reserved = Reserved
        self.InitConfig.Filter   = Filter
        self.InitConfig.Timing0  = Timer0
        self.InitConfig.Timing1  = Timer1
        self.InitConfig.Mode     = Mode

        print(self)
        open_success = self.__Open()
        device_info = self.__GetInfo()
        print("[USBCAN] 设备信息\n{}".format(device_info))
        start_success = self.__StartCAN()

    def __str__(self) -> str:
        print("a")
        return "[USBCAN] 型号:{} 设备号:{} 通道:{}".format(self.DeviceType, self.DeviceIndex, self.Channel)

    def __Open(self) -> bool:
        open_device_success = USBCAN_Lib.VCI_OpenDevice(self.DeviceType, self.DeviceIndex, self.InitConfig.Reserved)
        if open_device_success == 1:
            print("[USBCAN] 已成功打开!!!")
        else:
            print("[USBCAN] 无法打开...")
        return True if open_device_success == 1 else False
    
    def __GetInfo(self) -> usbcan_struct.ZCAN_CAN_BOARD_INFO:
        try:
            read_info_success = USBCAN_Lib.VCI_ReadBoardInfo(self.DeviceType, self.DeviceIndex, byref(self.DeviceInfo))
            return self.DeviceInfo if read_info_success == 1 else None
        except:
            print("[USBCAN] GetInfo()异常...")
            raise

    def __StartCAN(self) -> bool:
        init_success = USBCAN_Lib.VCI_InitCAN(self.DeviceType, self.DeviceIndex, self.Channel, byref(self.InitConfig))
        if init_success == 1:
            print("[USBCAN] CAN初始化成功!!!")
        else:
            print("[USBCAN] CAN初始化失败...")
        start_success = USBCAN_Lib.VCI_StartCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if start_success == 1:
            print("[USBCAN] CAN启动成功!!!")
        else:
            print("[USBCAN] CAN启动失败...")
        return True if start_success == 1 else False
    
    def Reset(self) -> bool:
        reset_success = USBCAN_Lib.VCI_ResetCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if reset_success == 1:
            print("[USBCAN] 重置成功!!!")
        else:
            print("[USBCAN] 重置失败...")
        return True if reset_success == 1 else False
    
    def Close(self) -> bool:
        close_success = USBCAN_Lib.VCI_CloseDevice(self.DeviceType, self.DeviceIndex, self.Channel)
        if close_success == 1:
            print("[USBCAN] 关闭成功!!!")
        else:
            print("[USBCAN] 关闭失败...")
        return True if close_success == 1 else False
    
    def SendMsgs(self, id, data,
                 length      = 1,
                 data_len    = usbcan_param.DATA_LEN["default"],
                 time_stamp  = usbcan_param.TIME_STAMP["off"],
                 time_flag   = usbcan_param.TIME_FLAG["off"],
                 send_type   = usbcan_param.SEND_TYPE["single"],
                 remote_flag = usbcan_param.REMOTE_FLAG["data"],
                 extern_flag = usbcan_param.EXTERN_FLAG["standard"],
                 ) -> bool:
        msgs = (usbcan_struct.ZCAN_CAN_OBJ * length)()
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
                    print("[USBCAN] 数据长度错误!!!")
                    return False
                else:
                    for j in range(msgs[i].DataLen):
                        msgs[i].Data[j] = data[j]
        
        send_num = USBCAN_Lib.VCI_Transmit(self.DeviceType, self.DeviceIndex, self.Channel, byref(msgs), length)
        if length == send_num:
            print("[USBCAN] 传输成功!!! 数量:{}".format(send_num))
            return True
        else:
            print("[USBCAN] 传输失败... 数量:{}".format(send_num))
            return False
    
    def rcv_msg(self):
        rcv_num = USBCAN_Lib.VCI_GetReceiveNum(self.DeviceType, self.DeviceIndex, self.Channel)
        pass