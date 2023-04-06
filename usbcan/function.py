# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary(BASE_DIR + "/libusbcan.so")

import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param
import motor.msg_resolution as motor_msgres


class UsbCan:
    
    def __init__(self,
                 device_type  = usbcan_param.DEVICE_TYPE["USBCAN2"], 
                 device_index = usbcan_param.DEVICE_INDEX["0"],
                 channel      = usbcan_param.CHANNEL["0"],
                 reserved     = usbcan_param.RESERVED,
                 timer_0      = usbcan_param.TIMER["250K"][0], 
                 timer_1      = usbcan_param.TIMER["250K"][1],
                 acc_code     = usbcan_param.ACC_CODE["default"],
                 acc_mask     = usbcan_param.ACC_MASK["default"],
                 filter       = usbcan_param.FILTER["single"],
                 mode         = usbcan_param.MODE["normal"],
                 ) -> None:
        
        self.__device_type = device_type
        self.__device_index = device_index
        self.__channel = channel

        self.__init_config = usbcan_struct.ZCAN_CAN_INIT_CONFIG()
        self.__init_config.AccCode  = acc_code
        self.__init_config.AccMask  = acc_mask
        self.__init_config.Reserved = reserved
        self.__init_config.Filter   = filter
        self.__init_config.Timing0  = timer_0
        self.__init_config.Timing1  = timer_1
        self.__init_config.Mode     = mode
        
        print(self)
        open_success = self.__open()
        init_success = self.__init()
        start_success = self.__start()

    def __str__(self) -> str:

        return "[UsbCan] channel {}".format(self.__channel)

    def __open(self) -> bool:
        
        open_device_success = USBCAN_Lib.VCI_OpenDevice(self.__device_type, self.__device_index, usbcan_param.RESERVED)
        if open_device_success == 1:
            print("\033[0;32m[Channel {}] open\033[0m".format(self.__device_index))
        else:
            print("\033[0;31m[Channel {}] open failed\033[0m".format(self.__device_index))
        
        return True if open_device_success == 1 else False

    def __init(self) -> bool:
        
        init_success = USBCAN_Lib.VCI_InitCAN(self.__device_type, self.__device_index, self.__channel, byref(self.__init_config))
        if init_success == 1:
            print("\033[0;32m[Channel {}] initialize\033[0m".format(self.__device_index))
        else:
            print("\033[0;31m[Channel {}] initialize failed\033[0m".format(self.__device_index))
        
        return True if init_success == 1 else False

    def __start(self) -> bool:
        
        start_success = USBCAN_Lib.VCI_StartCAN(self.__device_type, self.__device_index, self.__channel)
        if start_success == 1:
            print("\033[0;32m[Channel {}] start\033[0m".format(self.__device_index))
        else:
            print("\033[0;31m[Channel {}] start failed\033[0m".format(self.__device_index))
        
        return True if start_success == 1 else False
    
    def reset(self) -> bool:
        
        reset_success = USBCAN_Lib.VCI_ResetCAN(self.__device_type, self.__device_index, self.__channel)
        if reset_success == 1:
            print("\033[0;32m[Channel {}] reset\033[0m".format(self.__device_index))
        else:
            print("\033[0;31m[Channel {}] reset failed\033[0m".format(self.__device_index))
        
        return True if reset_success == 1 else False
    
    def close(self) -> bool:
        
        close_success = USBCAN_Lib.VCI_CloseDevice(self.__device_type, self.__device_index, self.__channel)
        if close_success == 1:
            print("\033[0;32m[Channel {}] close\033[0m".format(self.__device_index))
        else:
            print("\033[0;31m[Channel {}] close failed\033[0m".format(self.__device_index))
        
        return True if close_success == 1 else False
    
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
            print("\033[0;31m[Channel {}] data type error\033[0m")
            return False
        
        length = len(data)
        for i in range(length): 
            if len(data[i]) != data_len:
                print("\033[0;31m[Channel {}] data length error\033[0m")
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
        
        send_num = USBCAN_Lib.VCI_Transmit(self.__device_type, self.__device_index, self.__channel, byref(msgs), length)
        if length == send_num:
            if log:
                print("[Channel {}] send done! num: {}".format(send_num))
            return True
        else:
            if log:
                print("[Channel {}] send failed, num: {}".format(send_num))
            return False

    def get_cache_num(self) -> int:   
        
        cache_num = USBCAN_Lib.VCI_GetReceiveNum(self.__device_type, self.__device_index, self.__channel)
        print("[Channel {}] cache num: {}".format(cache_num))
        
        return cache_num

    def read_cache(self, read_num, wait_time = 100) -> usbcan_struct.ZCAN_CAN_OBJ:
        
        cache_num = self.get_cache_num()
        read_num = cache_num if cache_num < read_num else read_num
        
        rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ * read_num)()
        rcv_num = USBCAN_Lib.VCI_Receive(self.__device_type, self.__device_index, self.__channel, byref(rcv_msgs), read_num, wait_time)
        return rcv_msgs
        
    def clear_cache(self) -> bool:
        pass
