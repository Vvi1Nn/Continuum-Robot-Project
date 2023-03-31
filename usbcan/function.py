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
        open_success = self.__open()
        device_info = self.__get_info()
        print("[USBCAN] 设备信息\n{}".format(device_info))
        start_success = self.__start_can()

    def __str__(self) -> str:
        
        return "[USBCAN] 型号:{} 设备号:{} 通道:{}".format(self.DeviceType, self.DeviceIndex, self.Channel)

    def __open(self) -> bool:
        
        open_device_success = USBCAN_Lib.VCI_OpenDevice(self.DeviceType, self.DeviceIndex, self.InitConfig.Reserved)
        if open_device_success == 1:
            print("[USBCANopen] 成功")
        else:
            print("[USBCANopen] 失败...")
        
        return True if open_device_success == 1 else False
    
    def __get_info(self) -> usbcan_struct.ZCAN_CAN_BOARD_INFO:
        
        try:
            read_info_success = USBCAN_Lib.VCI_ReadBoardInfo(self.DeviceType, self.DeviceIndex, byref(self.DeviceInfo))
            return self.DeviceInfo if read_info_success == 1 else None
        except:
            print("[USBCANinfo] 异常...")
            raise

    def __start_can(self) -> bool:
        
        init_success = USBCAN_Lib.VCI_InitCAN(self.DeviceType, self.DeviceIndex, self.Channel, byref(self.InitConfig))
        if init_success == 1:
            print("[USBCANinit] 成功")
        else:
            print("[USBCANinit] 失败...")
        
        start_success = USBCAN_Lib.VCI_StartCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if start_success == 1:
            print("[USBCANstart] 成功")
        else:
            print("[USBCANstart] 失败...")
        
        return True if start_success == 1 else False
    
    def reset(self) -> bool:
        
        reset_success = USBCAN_Lib.VCI_ResetCAN(self.DeviceType, self.DeviceIndex, self.Channel)
        if reset_success == 1:
            print("[USBCANreset] 成功")
        else:
            print("[USBCANreset] 失败...")
        
        return True if reset_success == 1 else False
    
    def close(self) -> bool:
        
        close_success = USBCAN_Lib.VCI_CloseDevice(self.DeviceType, self.DeviceIndex, self.Channel)
        if close_success == 1:
            print("[USBCANclose] 成功")
        else:
            print("[USBCANclose] 失败...")
        
        return True if close_success == 1 else False
    
    def send_single(self, id, data,
                    log         = False,
                    length      = 1,
                    time_stamp  = usbcan_param.TIME_STAMP["off"],
                    time_flag   = usbcan_param.TIME_FLAG["off"],
                    send_type   = usbcan_param.SEND_TYPE["normal"],
                    remote_flag = usbcan_param.REMOTE_FLAG["data"],
                    extern_flag = usbcan_param.EXTERN_FLAG["standard"],
                    data_len    = usbcan_param.DATA_LEN["default"],
                    ) -> bool:
        
        if type(data) != list:
            print("[USBCANsend] data类型错误!!!")
            return False
        if len(data) != data_len:
            print("[USBCANsend] data长度错误!!!")
            return False 
        
        msgs = (usbcan_struct.ZCAN_CAN_OBJ * length)()
        for i in range(length):
            msgs[i].ID         = id
            msgs[i].TimeStamp  = time_stamp
            msgs[i].TimeFlag   = time_flag
            msgs[i].SendType   = send_type
            msgs[i].RemoteFlag = remote_flag
            msgs[i].ExternFlag = extern_flag
            msgs[i].DataLen    = data_len
            for j in range(msgs[i].DataLen):
                msgs[i].Data[j] = data[j]
        
        send_num = USBCAN_Lib.VCI_Transmit(self.DeviceType, self.DeviceIndex, self.Channel, byref(msgs), length)
        if length == send_num:
            if log:
                print("[USBCANsend] 传输成功!!! 数量:{}".format(send_num))
            return True
        else:
            if log:
                print("[USBCANsend] 传输失败... 期望:{} 实际:{}".format(length, send_num))
            return False
    
    def send(self, id, data,
             log         = False,
             time_stamp  = usbcan_param.TIME_STAMP["off"],
             time_flag   = usbcan_param.TIME_FLAG["off"],
             send_type   = usbcan_param.SEND_TYPE["normal"],
             remote_flag = usbcan_param.REMOTE_FLAG["data"],
             extern_flag = usbcan_param.EXTERN_FLAG["standard"],
             data_len    = usbcan_param.DATA_LEN["default"],
             ) -> bool:
        
        if type(data) != list:
            print("[USBCANsend] data类型错误!!!")
            return False
        
        length = len(data)
        for i in range(length): 
            if len(data[i]) != data_len:
                print("[USBCANsend] data长度错误!!!")
                return False
        
        msgs = (usbcan_struct.ZCAN_CAN_OBJ * length)()
        for i in range(length):
            msgs[i].ID         = id
            msgs[i].TimeStamp  = time_stamp
            msgs[i].TimeFlag   = time_flag
            msgs[i].SendType   = send_type
            msgs[i].RemoteFlag = remote_flag
            msgs[i].ExternFlag = extern_flag
            msgs[i].DataLen    = data_len
            for j in range(msgs[i].DataLen):
                msgs[i].Data[j] = data[i][j]
        
        send_num = USBCAN_Lib.VCI_Transmit(self.DeviceType, self.DeviceIndex, self.Channel, byref(msgs), length)
        if length == send_num:
            if log:
                print("[USBCANsend] 传输成功!!! 数量:{}".format(send_num))
            return True
        else:
            if log:
                print("[USBCANsend] 传输失败... 数量:{}".format(send_num))
            return False

    def receive_all(self, wait_time = 100) -> None:
        
        cache_num = USBCAN_Lib.VCI_GetReceiveNum(self.DeviceType, self.DeviceIndex, self.Channel)
        print("[Receive] 缓冲区数量:{}".format(cache_num))
        
        if cache_num > 0:
            rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ * cache_num)()
            rcv_num = USBCAN_Lib.VCI_Receive(self.DeviceType, self.DeviceIndex, self.Channel, byref(rcv_msgs), cache_num, wait_time)
            for i in range(rcv_num):
                print("[Receive No.{}]\nCOB-ID: {}\n数据: {}".format(i, hex(rcv_msgs[i].ID), ''.join(hex(rcv_msgs[i].Data[j]) + ' ' for j in range(rcv_msgs[i].DataLen))))

    def receive_single(self, wait_time = 100) -> None:
        
        cache_num = USBCAN_Lib.VCI_GetReceiveNum(self.DeviceType, self.DeviceIndex, self.Channel)
        print("[Receive] 缓冲区数量:{}".format(cache_num))
        
        if cache_num > 0:
            rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ * 1)()
            rcv_num = USBCAN_Lib.VCI_Receive(self.DeviceType, self.DeviceIndex, self.Channel, byref(rcv_msgs), 1, wait_time)
            print("[Receive]\nCOB-ID: {}\n数据: {}".format(hex(rcv_msgs[0].ID), ''.join(hex(rcv_msgs[0].Data[j]) + ' ' for j in range(rcv_msgs[0].DataLen))))