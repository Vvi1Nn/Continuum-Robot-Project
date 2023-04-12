# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary(BASE_DIR + "/libusbcan.so")

import time

import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param


class UsbCan:
    
    __device_type  = usbcan_param.DEVICE_TYPE["USBCAN2"]
    __device_index = usbcan_param.DEVICE_INDEX["0"]
    __reserved     = usbcan_param.RESERVED

    is_open = False
    work_channel_count = 0

    def __init__(self,
                 channel = "0",
                 timer = "250K",
                 ) -> None:
        
        self.__channel = usbcan_param.CHANNEL[channel]

        self.__init_config = usbcan_struct.INIT_CONFIG()
        self.__init_config.AccCode  = usbcan_param.ACC_CODE["default"]
        self.__init_config.AccMask  = usbcan_param.ACC_MASK["default"]
        self.__init_config.Reserved = usbcan_param.RESERVED
        self.__init_config.Filter   = usbcan_param.FILTER["single"]
        self.__init_config.Timing0  = usbcan_param.TIMER[timer][0]
        self.__init_config.Timing1  = usbcan_param.TIMER[timer][1]
        self.__init_config.Mode     = usbcan_param.MODE["normal"]

        self.__time_stamp  = usbcan_param.TIME_STAMP["off"]
        self.__time_flag   = usbcan_param.TIME_FLAG["off"]
        self.__extern_flag = usbcan_param.EXTERN_FLAG["standard"]
        self.__data_len    = usbcan_param.DATA_LEN["default"]
        
        self.is_init = False
        self.is_start = False
        self.reset_count = 0

        self.__init_can()
        self.__start_can()

        if not (self.is_init and self.is_start):
            print("\033[0;33m[UsbCan] waiting for __init__ again ...\033[0m")
            time.sleep(3)
            return self.__init__(channel, timer)

        UsbCan.work_channel_count = UsbCan.work_channel_count + 1
        
    def __str__(self) -> str:
        return "channel {}".format(self.__channel)

    @classmethod
    def open_device(cls,
             device_type = "USBCAN2",
             device_index = "0",
             try_num = 10,
             ) -> None:
        cls.__device_type = usbcan_param.DEVICE_TYPE[device_type]
        cls.__device_index = usbcan_param.DEVICE_INDEX[device_index]
        if not cls.is_open:
            if USBCAN_Lib.VCI_OpenDevice(UsbCan.__device_type, UsbCan.__device_index, UsbCan.__reserved):
                cls.is_open = True
                print("\033[0;32m[UsbCan] type:{}  index:{}\n\033[0m".format(UsbCan.__device_type, UsbCan.__device_index))
            else:
                if try_num != 0:
                    print("\033[0;33m[UsbCan] trying open device again ...\033[0m")
                    time.sleep(1)
                    return UsbCan.open_device(cls.__device_type, cls.__device_index, try_num - 1)
                else:
                    print("\033[0;31m[UsbCan] cannot open device\n\033[0m")
        else:
            print("\033[0;32m[UsbCan] device is already opened\n\033[0m")

    @classmethod
    def close_device(cls, try_num = 10) -> None:
        if cls.is_open:
            if USBCAN_Lib.VCI_CloseDevice(UsbCan.__device_type, UsbCan.__device_index):
                print("\033[0;32m[UsbCan] device closed\n\033[0m")
                cls.is_open = False
            else:
                if try_num != 0:
                    print("\033[0;33m[UsbCan] trying close device again ...\033[0m")
                    time.sleep(1)
                    return UsbCan.close_device(try_num - 1)
                else:
                    print("\033[0;31m[UsbCan] cannot close device\n\033[0m")
        else:
            print("\033[0;32m[UsbCan] device is already closed\n\033[0m")

    def __init_can(self, try_num = 10) -> None:
        if UsbCan.is_open:
            if USBCAN_Lib.VCI_InitCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(self.__init_config)):
                self.is_init = True
                print("\033[0;32m[Channel {}] initialize\033[0m".format(self.__channel))
            else:
                if try_num != 0:
                    print("\033[0;33m[Channel {}] trying init can again ...\033[0m".format(self.__channel))
                    time.sleep(1)
                    return self.__init_can(try_num - 1)
                else:
                    print("\033[0;31m[Channel {}] cannot init can\n\033[0m".format(self.__channel))
        else:
            print("\033[0;31m[UsbCan] please open device first\n\033[0m")

    def __start_can(self, try_num = 10) -> None:
        if UsbCan.is_open and self.is_init:
            self.clear_buffer()
            if USBCAN_Lib.VCI_StartCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                self.is_start = True
                print("\033[0;32m[Channel {}] start\n\033[0m".format(self.__channel))
            else:
                if try_num != 0:
                    print("\033[0;33m[Channel {}] trying start can again ...\033[0m".format(self.__channel))
                    time.sleep(1)
                    return self.__start_can(try_num - 1)
                else:
                    print("\033[0;31m[Channel {}] cannot start can\n\033[0m".format(self.__channel))
        else:
            print("\033[0;31m[UsbCan] please open device and init can first\033[0m")
    
    def reset_can(self, try_num = 10) -> None:
        if UsbCan.is_open and self.is_init:
            self.clear_buffer()
            if USBCAN_Lib.VCI_ResetCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                self.reset_count = self.reset_count + 1
                self.__start_can()
                print("\033[0;32m[Channel {}] reset\n\033[0m".format(self.__channel))
            else:
                if try_num != 0:
                    print("\033[0;33m[Channel {}] trying reset can again ...\033[0m".format(self.__channel))
                    time.sleep(1)
                    return self.reset_can(try_num - 1)
                else:
                    print("\033[0;31m[Channel {}] reset can failed\n\033[0m".format(self.__channel))
        else:
            print("\033[0;31m[UsbCan] please open device and init can first\033[0m")
    
    def send(self, id, data,
             log         = False,
             send_type   = usbcan_param.SEND_TYPE["normal"],
             remote_flag = usbcan_param.REMOTE_FLAG["data"],
             ) -> bool:
        if self.is_start:
            length = len(data)
            msgs = (usbcan_struct.CAN_OBJ * length)()
            for i in range(length):
                msgs[i].ID         = id
                msgs[i].TimeStamp  = self.__time_stamp
                msgs[i].TimeFlag   = self.__time_flag
                msgs[i].SendType   = send_type
                msgs[i].RemoteFlag = remote_flag
                msgs[i].ExternFlag = self.__extern_flag
                msgs[i].DataLen    = self.__data_len
                for j in range(msgs[i].DataLen):
                    msgs[i].Data[j] = data[i][j]
            send_num = USBCAN_Lib.VCI_Transmit(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(msgs), length)
            if length == send_num:
                if log: print("\033[0;32m[Channel {}] send done! num: {}\033[0m".format(self.__channel, send_num))
                return True
            else:
                if log: print("\033[0;31m[Channel {}] send failed, num: {}\033[0m".format(self.__channel, send_num))
                return False
        else:
            print("\033[0;31m[UsbCan] please init channel first\033[0m")
            return False

    def get_buffer_num(self, log = False) -> int:   
        if self.is_start:
            buffer_num = USBCAN_Lib.VCI_GetReceiveNum(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
            if log: print("[Channel {}] buffer num: {}".format(self.__channel, buffer_num))
            return buffer_num
        else:
            print("\033[0;31m[UsbCan] please start channel first\033[0m")
            return None

    def read_buffer(self, read_num,
                    wait_time = -1,
                    ) -> list: 
        if self.is_start:
            cache_num = self.get_buffer_num()
            read_num = cache_num if cache_num < read_num else read_num
            rcv_msgs = (usbcan_struct.CAN_OBJ * read_num)()
            rcv_num = USBCAN_Lib.VCI_Receive(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(rcv_msgs), read_num, wait_time)
            return [rcv_num, rcv_msgs]
        else:
            print("\033[0;31m[UsbCan] please start channel first\033[0m")
            return None
        
    def clear_buffer(self, try_num = 10, log = False) -> bool:
        if UsbCan.is_open:
            if USBCAN_Lib.VCI_ClearBuffer(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                if log: print("\033[0;32m[Channel {}] clear buffer\033[0m".format(self.__channel))
                return True
            else:
                if try_num != 0:
                    print("\033[0;33m[Channel {}] trying clear buffer again ...\033[0m".format(self.__channel))
                    time.sleep(1)
                    return self.clear_buffer(try_num - 1, True)
                else:
                    print("\033[0;31m[Channel {}] clear buffer failed\n\033[0m".format(self.__channel))
        else:
            print("\033[0;31m[UsbCan] please open device first\033[0m")

    @staticmethod
    def print_msgs(num, msgs):
        for i in range(num):
            print("No:%d  ID: %s  Data: %s" % (i+1, hex(msgs[i].ID), ''.join(hex(msgs[i].Data[j])+ ' 'for j in range(msgs[i].DataLen))))
