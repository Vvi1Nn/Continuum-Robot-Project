# -*- coding:utf-8 -*-


''' function.py USBCAN卡功能函数 v2.2.2 '''


# 添加路径
import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

# 导入动态库
from ctypes import *
USBCAN_Lib = cdll.LoadLibrary(BASE_DIR + "/libusbcan.so")

# 导入自定义模块
import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param


class UsbCan:
    # 设备参数 默认
    __device_type  = usbcan_param.DEVICE_TYPE["USBCAN2"]
    __device_index = usbcan_param.DEVICE_INDEX["0"]
    __reserved     = usbcan_param.RESERVED

    __is_open = False # 标志符 设备是否开启

    __is_log = False # 标志符 是否打印日志

    __channel_list = [] # 对象列表

    def __init__(self, channel: str) -> None:
        UsbCan.__channel_list.append(self) # 记录对象
        
        self.__channel = usbcan_param.CHANNEL[channel] # 通道

        # 初始化结构体
        self.__init_config = usbcan_struct.INIT_CONFIG()
        self.__init_config.AccCode  = usbcan_param.ACC_CODE["default"]
        self.__init_config.AccMask  = usbcan_param.ACC_MASK["default"]
        self.__init_config.Reserved = usbcan_param.RESERVED
        self.__init_config.Filter   = usbcan_param.FILTER["single"]
        self.__init_config.Timing0  = usbcan_param.TIMER["500K"][0]
        self.__init_config.Timing1  = usbcan_param.TIMER["500K"][1]
        self.__init_config.Mode     = usbcan_param.MODE["normal"]

        # 消息结构体 一部分 剩余参数在函数中给定
        self.__time_stamp  = usbcan_param.TIME_STAMP["off"]
        self.__time_flag   = usbcan_param.TIME_FLAG["off"]
        self.__send_type   = usbcan_param.SEND_TYPE["single"] # 不自动重发
        self.__extern_flag = usbcan_param.EXTERN_FLAG["standard"]
        
        self.__is_init = False # 标志符 通道是否初始化
        self.__is_start = False # 标志符 通道是否启动

    ''' 初始化前 设置日志是否打印 '''
    @classmethod
    def is_show_log(cls, is_log: bool):
        cls.__is_log = is_log
        return cls # 返回类 可供直接初始化

    ''' 打开设备 指定设备的型号和索引 可设置重复次数 '''
    @classmethod
    def open_device(cls, device_type="USBCAN2", device_index="0", /, *, repeat=0) -> bool:
        # 将设备型号和索引更新至类属性
        cls.__device_type = usbcan_param.DEVICE_TYPE[device_type]
        cls.__device_index = usbcan_param.DEVICE_INDEX[device_index]
        # 设备未开启
        if not cls.__is_open:
            # 调用API
            if USBCAN_Lib.VCI_OpenDevice(UsbCan.__device_type, UsbCan.__device_index, UsbCan.__reserved):
                cls.__is_open = True # 成功 标志符开启
                if UsbCan.__is_log: print("\033[0;32m[UsbCan] opened! type:{} index:{}\033[0m".format(UsbCan.__device_type, UsbCan.__device_index))
                return True
            # 执行API失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[UsbCan] open failed\033[0m")
                return False # 不重复
            if UsbCan.__is_log: print("\033[0;33m[UsbCan] open ...\033[0m")
            return cls.open_device(cls.__device_type, cls.__device_index, repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备已开启
        else:
            if UsbCan.__is_log: print("\033[0;32m[UsbCan] already opened\033[0m")
            return False

    ''' 关闭设备 可设置重复次数 '''
    @classmethod
    def close_device(cls, repeat=0) -> bool:
        # 设备已开启
        if cls.__is_open:
            # 调用API
            if USBCAN_Lib.VCI_CloseDevice(UsbCan.__device_type, UsbCan.__device_index):
                cls.__is_open = False # 成功 标识符关闭
                for channel in cls.__channel_list:
                    channel.__is_init = False # 通道需重新初始化
                    channel.__is_start = False # 通道需重新打开
                if UsbCan.__is_log: print("\033[0;32m[UsbCan] closed\033[0m")
                return True
            # 执行API失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[UsbCan] close failed\n\033[0m")
                return False
            if UsbCan.__is_log: print("\033[0;33m[UsbCan] close ...\033[0m")
            return cls.close_device(repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备未开启
        else:
            if UsbCan.__is_log: print("\033[0;32m[UsbCan] already closed\n\033[0m")
            return False

    ''' 设置设备型号 不常用 功能补齐 '''
    @classmethod
    def set_device_type(cls, type: str, index: str):
        cls.__device_type  = usbcan_param.DEVICE_TYPE[type]
        cls.__device_index = usbcan_param.DEVICE_INDEX[index]

    ''' 设置波特率 '''
    def set_timer(self, timer: str) -> None:
        self.__init_config.Timing0 = usbcan_param.TIMER[timer][0]
        self.__init_config.Timing1 = usbcan_param.TIMER[timer][1]

    ''' 初始化通道 可设置重复次数 '''
    def init_can(self, repeat=0) -> bool:
        # 设备已开启 且设备未初始化
        if UsbCan.__is_open and not self.__is_init:
            # 调用API
            if USBCAN_Lib.VCI_InitCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(self.__init_config)):
                self.__is_init = True # 成功 初始化成功
                if UsbCan.__is_log: print("\033[0;32m[Channel {}] init\033[0m".format(self.__channel))
                return True
            # 执行API失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[Channel {}] init failed\033[0m".format(self.__channel))
                return False # 不重复
            if UsbCan.__is_log: print("\033[0;33m[Channel {}] init ...\033[0m".format(self.__channel))
            return self.init_can(repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备未开启
        else:
            if UsbCan.__is_log: print("\033[0;31m[UsbCan] open device first\n\033[0m")
            return False
    
    ''' 启动通道 可设置重复次数 '''
    def start_can(self, repeat=0) -> bool:
        # 通道已初始化
        if self.__is_init:
            self.clear_buffer() # 清空缓存区的数据
            # 调用API
            if USBCAN_Lib.VCI_StartCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                self.__is_start = True # 成功 启动成功
                if UsbCan.__is_log: print("\033[0;32m[Channel {}] start\033[0m".format(self.__channel))
                return True
            # 执行API失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[Channel {}] start failed\033[0m".format(self.__channel))
                return False # 不重复
            if UsbCan.__is_log: print("\033[0;33m[Channel {}] start ...\033[0m".format(self.__channel))
            return self.init_can(repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备未开启
        else:
            if UsbCan.__is_log: print("\033[0;31m[UsbCan] please open device then init can first\033[0m")
            return False

    ''' 重置通道 可设置重复次数 '''
    def reset_can(self, repeat=0) -> bool:
        # 通道已开启
        if self.__is_start:
            # 调用API
            if USBCAN_Lib.VCI_ResetCAN(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                self.__is_start = False # 成功 此时通道处于初始化但未启动的状态 通道开启置为False
                # 重新开启通道
                if self.start_can():
                    if UsbCan.__is_log: print("\033[0;32m[Channel {}] reset\033[0m".format(self.__channel))
                    return True
            # 执行API失败 或者重新开启通道失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[Channel {}] reset failed\033[0m".format(self.__channel))
                return False # 不重复
            if UsbCan.__is_log: print("\033[0;33m[Channel {}] reset ...\033[0m".format(self.__channel))
            return self.reset_can(repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备未开启
        else:
            if UsbCan.__is_log: print("\033[0;31m[UsbCan] please open device then init can first\033[0m")
            return False
    
    ''' 发送消息 设置帧类型和数据长度 '''
    def send(self, node_id: int, data: list, remote_flag="data", data_len="default") -> bool:
        # 通道已开启
        if self.__is_start:
            length = len(data) # 统计消息个数 data是列表的列表 
            msgs = (usbcan_struct.CAN_OBJ * length)() # 消息结构体
            # 设置每一条消息的结构体参数
            for i in range(length):
                msgs[i].ID         = node_id
                msgs[i].TimeStamp  = self.__time_stamp
                msgs[i].TimeFlag   = self.__time_flag
                msgs[i].SendType   = self.__send_type
                msgs[i].RemoteFlag = usbcan_param.REMOTE_FLAG[remote_flag]
                msgs[i].ExternFlag = self.__extern_flag
                msgs[i].DataLen    = usbcan_param.DATA_LEN[data_len]
                # 赋值消息中的数据
                for j in range(msgs[i].DataLen):
                    msgs[i].Data[j] = data[i][j]
            # 调用API
            send_num = USBCAN_Lib.VCI_Transmit(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(msgs), length)
            # 预期发送数量和实际发送数量一致 成功
            if length == send_num:
                if UsbCan.__is_log: print("\033[0;32m[Channel {}] send done! num: {}\033[0m".format(self.__channel, send_num))
                return True
            # 失败
            else:
                if UsbCan.__is_log: print("\033[0;31m[Channel {}] send failed, num: {}\033[0m".format(self.__channel, send_num))
                return False
        # 通道未打开
        else:
            if UsbCan.__is_log: print("\033[0;31m[UsbCan] start channel first\033[0m")
            return False

    ''' 直接打印读取的数据的原始值 可供核对 '''
    @staticmethod
    def print_msgs(num, msgs):
        for i in range(num):
            print("No:%d  ID: %s  Data: %s" % (i+1, hex(msgs[i].ID), ''.join(hex(msgs[i].Data[j])+ ' 'for j in range(msgs[i].DataLen))))

    ''' 获取当前通道缓冲区数据数量 '''
    def get_buffer_num(self) -> int:   
        # 通道已开启
        if self.__is_start:
            # 调用API 返回数量
            buffer_num = USBCAN_Lib.VCI_GetReceiveNum(UsbCan.__device_type, UsbCan.__device_index, self.__channel)
            if UsbCan.__is_log: print("[Channel {}] buffer num: {}".format(self.__channel, buffer_num))
            return buffer_num
        else:
            print("\033[0;31m[UsbCan] please start channel first\033[0m")
            return None

    ''' 读取缓冲区指定数量的数据 可设置未读取到数据时的等待时间 -1为永久等待 '''
    def read_buffer(self, read_num, wait_time=-1): 
        # 通道已开启
        if self.__is_start:
            # 获取当前通道缓冲区的数据数量
            cache_num = self.get_buffer_num()
            # 读取数量不可超过当前数据数量
            read_num = cache_num if cache_num < read_num else read_num
            # 接受数据的消息体
            rcv_msgs = (usbcan_struct.CAN_OBJ * read_num)()
            # 执行API 返回读取到的数量和消息
            rcv_num = USBCAN_Lib.VCI_Receive(UsbCan.__device_type, UsbCan.__device_index, self.__channel, byref(rcv_msgs), read_num, wait_time)
            return [rcv_num, rcv_msgs]
        # 通道未开启
        else:
            print("\033[0;31m[UsbCan] please start channel first\033[0m")
            return None
    
    ''' 清空当前通道缓冲区的数据 可设置重复次数 一定程度提高鲁棒性 '''
    def clear_buffer(self, repeat=0) -> bool:
        # 设备打开
        if UsbCan.__is_open:
            # 执行API成功
            if USBCAN_Lib.VCI_ClearBuffer(UsbCan.__device_type, UsbCan.__device_index, self.__channel):
                if UsbCan.__is_log: print("\033[0;32m[Channel {}] clear buffer\033[0m".format(self.__channel))
                return True
            # 执行API失败 判断是否重复
            if repeat == 0:
                if UsbCan.__is_log: print("\033[0;31m[Channel {}] clear buffer failed\033[0m".format(self.__channel))
                return False # 不重复
            if UsbCan.__is_log: print("\033[0;33m[Channel {}] clear buffer ...\033[0m".format(self.__channel))
            return self.clear_buffer(repeat=repeat-1) # 重复 再次尝试API 剩余次数减1
        # 设备未打开
        else:
            if UsbCan.__is_log: print("\033[0;31m[UsbCan] please open device first\033[0m")
            return False
