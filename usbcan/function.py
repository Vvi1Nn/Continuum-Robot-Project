# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary(BASE_DIR + "/libusbcan.so")

import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param


class UsbCan:
    
    __device_type  = usbcan_param.DEVICE_TYPE["USBCAN2"]
    __device_index = usbcan_param.DEVICE_INDEX["0"]
    __reserved     = usbcan_param.RESERVED

    __init_config = usbcan_struct.INIT_CONFIG()
    __init_config.AccCode  = usbcan_param.ACC_CODE["default"]
    __init_config.AccMask  = usbcan_param.ACC_MASK["default"]
    __init_config.Reserved = usbcan_param.RESERVED
    __init_config.Filter   = usbcan_param.FILTER["single"]
    __init_config.Timing0  = usbcan_param.TIMER["250K"][0]
    __init_config.Timing1  = usbcan_param.TIMER["250K"][1]
    __init_config.Mode     = usbcan_param.MODE["normal"]

    __time_stamp  = usbcan_param.TIME_STAMP["off"]
    __time_flag   = usbcan_param.TIME_FLAG["off"]
    __extern_flag = usbcan_param.EXTERN_FLAG["standard"]
    __data_len    = usbcan_param.DATA_LEN["default"]

    __is_open = False

    def __init__(self, channel = usbcan_param.CHANNEL["0"]) -> None:
        
        self.__channel = channel
        
        self.__is_start = False
        
        if UsbCan.__is_open:
            print(self)
            init_success = self.__init()
            if init_success:
                clear_success = USBCAN_Lib.VCI_ClearBuffer(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
                start_success = self.__start()
                if start_success:
                    self.__is_start = True
                else:
                    print("\033[0;31m[UsbCan] try start again\033[0m")
            else:
                print("\033[0;31m[UsbCan] try init again\033[0m")
        else:
            print("\033[0;31m[UsbCan] please open device first\033[0m")

    def __str__(self) -> str:
        return "[UsbCan] open channel {}".format(self.__channel)

    @classmethod
    def open(cls,
             device_type = usbcan_param.DEVICE_TYPE["USBCAN2"],
             device_index = usbcan_param.DEVICE_INDEX["0"]
             ) -> bool:
        cls.__device_type = device_type
        cls.__device_index = device_index
        if not cls.__is_open:
            open_device_success = USBCAN_Lib.VCI_OpenDevice(UsbCan.__device_type, UsbCan.__device_index, UsbCan.__reserved)
            if open_device_success == 1:
                print("\033[0;32m[UsbCan] type:{} index:{}\033[0m".format(UsbCan.__device_type, UsbCan.__device_index))
                cls.__is_open = True
                return True
            else:
                print("\033[0;31m[UsbCan] open failed\033[0m")
                return False
        else:
            print("\033[0;31m[UsbCan] cannot open device twice\033[0m")

    @classmethod
    def close(cls) -> bool:
        if cls.__is_open:
            close_success = USBCAN_Lib.VCI_CloseDevice(UsbCan.__device_type, UsbCan.__device_index)
            if close_success == 1:
                print("\033[0;32m[UsbCan] close\033[0m")
                cls.__is_open = False
                return True
            else:
                print("\033[0;31m[UsbCan] close failed\033[0m")
                return False
        else:
            print("[UsbCan] device is already closed")
            return True

    def __init(self) -> bool:
        init_success = USBCAN_Lib.VCI_InitCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(UsbCan.__init_config))
        if init_success == 1:
            print("\033[0;32m[Channel {}] initialize\033[0m".format(self.__channel))
            return True
        else:
            print("\033[0;31m[Channel {}] initialize failed\033[0m".format(self.__channel))
            return False

    def __start(self) -> bool:
        start_success = USBCAN_Lib.VCI_StartCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
        if start_success == 1:
            print("\033[0;32m[Channel {}] start\033[0m".format(self.__channel))
            return True
        else:
            print("\033[0;31m[Channel {}] start failed\033[0m".format(self.__channel))
            return False
    
    def reset(self) -> bool:
        if self.__is_start:
            reset_success = USBCAN_Lib.VCI_ResetCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
            if reset_success == 1:
                print("\033[0;32m[Channel {}] reset\033[0m".format(self.__channel))
                return True
            else:
                print("\033[0;31m[Channel {}] reset failed\033[0m".format(self.__channel))
                return False
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return False
    
    def send(self, id, data, log = False,
             send_type   = usbcan_param.SEND_TYPE["normal"],
             remote_flag = usbcan_param.REMOTE_FLAG["data"],
             ) -> bool:
        
        if self.__is_start:
            
            if type(data) != list:
                print("\033[0;31m[Channel {}] data type error\033[0m")
                return False
            
            length = len(data)
            for i in range(length): 
                if len(data[i]) != UsbCan.__data_len:
                    print("\033[0;31m[Channel {}] data length error\033[0m")
                    return False
            
            msgs = (usbcan_struct.CAN_OBJ * length)()
            for i in range(length):
                msgs[i].ID         = id
                msgs[i].TimeStamp  = UsbCan.__time_stamp
                msgs[i].TimeFlag   = UsbCan.__time_flag
                msgs[i].SendType   = send_type
                msgs[i].RemoteFlag = remote_flag
                msgs[i].ExternFlag = UsbCan.__extern_flag
                msgs[i].DataLen    = UsbCan.__data_len
                for j in range(msgs[i].DataLen):
                    msgs[i].Data[j] = data[i][j]
            
            send_num = USBCAN_Lib.VCI_Transmit(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(msgs), length)
            if length == send_num:
                if log:
                    print("\033[0;32m[Channel {}] send done! num: {}\033[0m".format(self.__channel, send_num))
                return True
            else:
                if log:
                    print("\033[0;31m[Channel {}] send failed, num: {}\033[0m".format(self.__channel, send_num))
                return False
        
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return False

    def get_buffer_num(self) -> int:   
        
        if self.__is_start:
            cache_num = USBCAN_Lib.VCI_GetReceiveNum(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
            print("[Channel {}] buffer num: {}".format(self.__channel, cache_num))
            return cache_num
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return None

    def read_buffer(self, read_num, wait_time = 100) -> list:
        
        if self.__is_start:
            cache_num = self.get_buffer_num()
            read_num = cache_num if cache_num < read_num else read_num
            
            rcv_msgs = (usbcan_struct.CAN_OBJ * read_num)()
            rcv_num = USBCAN_Lib.VCI_Receive(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(rcv_msgs), read_num, wait_time)
            return [rcv_num, rcv_msgs]
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return None
        
    def clear_buffer(self) -> bool:
        
        if self.__is_start:
            clear_success = USBCAN_Lib.VCI_ClearBuffer(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
            if clear_success == 1:
                print("\033[0;32m[Channel {}] clear buffer\033[0m".format(self.__channel))
                return True
            else:
                print("\033[0;31m[Channel {}] clear buffer failed\033[0m".format(self.__channel))
                return False
        
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return None
