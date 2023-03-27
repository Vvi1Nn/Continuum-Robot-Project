# -*- coding:utf-8 -*-

from ctypes import *

class ZCAN_CAN_BOARD_INFO(Structure):
    _fields_ = [("hw_Version",     c_ushort),     # 硬件版本号，16进制
                ("fw_Version",     c_ushort),     # 固件版本号，16进制
                ("dr_Version",     c_ushort),     # 驱动程序版本号，16进制
                ("in_Version",     c_ushort),     # 接口库版本号，16进制
                ("irq_Num",        c_ushort),     # 板卡所使用的中断号
                ("can_Num",        c_ubyte),      # 表示有几路通道
                ("str_Serial_Num", c_ubyte * 20), # 板卡的序列号
                ("str_hw_Type",    c_ubyte * 40), # 硬件类型
                ("Reserved",       c_ubyte * 4),  # 仅作保留，不设置
                ]
    
    def __str__(self):
        return "Hardware Version: {}\nFirmware Version: {}\nDriver Version: {}\nInterface: {}\nInterrupt Number: {}\nCAN_number: {}".format \
            (self.hw_Version, self.fw_Version, self.dr_Version, self.in_Version, self.irq_Num, self.can_Num)
        
    def serial(self):
        serial = ''
        for c in self.str_Serial_Num:
            if c > 0:
                serial += chr(c)
            else:
                break
        return serial
        
    def hw_Type(self):
        hw_Type = ''
        for c in self.str_hw_Type:
            if c > 0:
                hw_Type += chr(c)
            else:
                break
        return hw_Type

class ZCAN_CAN_INIT_CONFIG(Structure):
    _fields_ = [("AccCode",  c_int),   # 验收码
                ("AccMask",  c_int),   # 屏蔽码
                ("Reserved", c_int),   # 保留
                ("Filter",   c_ubyte), # 滤波方式，1==单滤波，0==双滤波
                ("Timing0",  c_ubyte), # 波特率定时器0
                ("Timing1",  c_ubyte), # 波特率定时器1
                ("Mode",     c_ubyte), # 模式，0==正常模式，1==只听模式，只接收，不影响总线
                ]

class ZCAN_CAN_OBJ(Structure):
    _fields_ = [("ID",         c_uint32),    # 帧ID，32位变量，数据格式为靠右对齐
                ("TimeStamp",  c_uint32),    # 设备接收到某一帧的时间标识
                ("TimeFlag",   c_uint8),     # 是否使用时间标识，1==TimeStamp有效
                ("SendType",   c_byte),      # 发送帧类型，0==正常发送，1==单次发送，2==自发自收，3==单次自发自收
                ("RemoteFlag", c_byte),      # 是否是远程帧，0==数据帧，1==远程帧
                ("ExternFlag", c_byte),      # 是否是扩展帧，0==标准帧，1==扩展帧
                ("DataLen",    c_byte),      # 数据长度DLC
                ("Data",       c_ubyte * 8), # CAN帧的数据
                ("Reserved",   c_ubyte * 3), # 系统保留
                ]
