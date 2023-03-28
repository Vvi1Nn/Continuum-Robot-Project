# -*- coding:utf-8 -*-

from ctypes import *

USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库


ZCAN_DEVICE_TYPE  = c_uint32
ZCAN_DEVICE_INDEX = c_uint32
ZCAN_Reserved     = c_uint32
ZCAN_CHANNEL      = c_uint32


DEVICE_TYPE = {"USBCAN2": ZCAN_DEVICE_TYPE(4),
               }

DEVICE_INDEX = {"0": ZCAN_DEVICE_INDEX(0),
                }

CHANNEL = {"0": ZCAN_CHANNEL(0),
           "1": ZCAN_CHANNEL(1),
           }

RESERVED = 0
