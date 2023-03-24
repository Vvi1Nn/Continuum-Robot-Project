# -*- coding:utf-8 -*-

from ctypes import *

USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

ZCAN_DEVICE_TYPE  = c_uint32
ZCAN_DEVICE_INDEX = c_uint32
ZCAN_Reserved     = c_uint32
ZCAN_CHANNEL      = c_uint32

LEN               = c_uint32

# USBCAN2      = ZCAN_DEVICE_TYPE(4)  # 设备类型号
# DEVICE_INDEX = ZCAN_DEVICE_INDEX(0) # 设备索引号
# CHANNEL      = ZCAN_CHANNEL(0)      # CAN通道号
# RESERVED = ZCAN_Reserved(0)     # 保留参数

class DeviceParam:
       def __init__(self) -> None:
              self.DEVICE_TYPE = {"USBCAN2": ZCAN_DEVICE_TYPE(4),
                                  }
              self.DEVICE_INDEX = {"0": ZCAN_DEVICE_INDEX(0),
                                   }
              self.CHANNEL = {"0": ZCAN_CHANNEL(0),
                              "1": ZCAN_CHANNEL(1),
                              }
              self.RESERVED = 0

class InitParam:
       def __init__(self) -> None:
              self.ACC_CODE = {"default": 0,
                               }
              self.ACC_MASK = {"default": 0xFFFFFFFF,
                               }
              self.FILTER = {"single": 1,
                             "dual":   0,
                             }
              self.TIMER = {"5K":    [0xBF, 0xFF],
                            "10K":   [0x31, 0x1C],
                            "20K":   [0x18, 0x1C],
                            "40K":   [0x87, 0xFF],
                            "50K":   [0x09, 0x1C],
                            "80K":   [0x83, 0xFF],
                            "100K":  [0x04, 0x1C],
                            "125K":  [0x03, 0x1C],
                            "200K":  [0x81, 0xFA],
                            "250K":  [0x01, 0x1C],
                            "400K":  [0x80, 0xFA],
                            "500K":  [0x00, 0x1C],
                            "666K":  [0x80, 0xB6],
                            "800K":  [0x00, 0x16],
                            "1000K": [0x00, 0x14],
                            }
              self.MODE ={"normal": 0,
                          "listen_only": 1,
                          }

class SendParam:
       def __init__(self) -> None:
              self.TIME_STAMP = {"off": 0,
                                 "on":  1,
                                 }
              self.TIME_FLAG = {"off": 0,
                                "on":  1,
                                }
              self.SEND_TYPE = {"normal":      0,
                                "single":      1,
                                "test":        2,
                                "single_test": 3,
                                }
              self.REMOTE_FLAG = {"data":   0,
                                  "remote": 1,
                                  }
              self.EXTERN_FLAG = {"standard": 0,
                                  "extern":   1,
                                  }
              self.DATA_LEN = {"default": 8,
                               }

device_param = DeviceParam()
init_param = InitParam()
send_param = SendParam()