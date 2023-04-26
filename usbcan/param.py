# -*- coding:utf-8 -*-


'''param.py 存放USBCAN卡设备和通道的初始化参数 以字典的形式表示'''


'''设备参数'''
# 型号
DEVICE_TYPE = {"USBCAN2": 4,
               }

# 索引
DEVICE_INDEX = {"0": 0,
                }

# 通道
CHANNEL = {"0": 0,
           "1": 1,
           }

# 保留
RESERVED = 0


'''初始化通道的配置参数'''
# 验收码 配置滤波范围
ACC_CODE = {"default": 0x00000000,
            }

# 屏蔽码
ACC_MASK = {"default": 0xFFFFFFFF,
            }

# 滤波方法
FILTER = {"single": 1, # 单滤波
          "dual"  : 0, # 双滤波
          }

# 波特率定时器
TIMER = {"5K"   : [0xBF, 0xFF],
         "10K"  : [0x31, 0x1C],
         "20K"  : [0x18, 0x1C],
         "40K"  : [0x87, 0xFF],
         "50K"  : [0x09, 0x1C],
         "80K"  : [0x83, 0xFF],
         "100K" : [0x04, 0x1C],
         "125K" : [0x03, 0x1C],
         "200K" : [0x81, 0xFA],
         "250K" : [0x01, 0x1C],
         "400K" : [0x80, 0xFA],
         "500K" : [0x00, 0x1C],
         "666K" : [0x80, 0xB6],
         "800K" : [0x00, 0x16],
         "1000K": [0x00, 0x14],
       }

# 运行模式
MODE ={"normal"     : 0, # 常规模式 正常节点
       "listen_only": 1, # 只听模式 只接收 不影响总线
       }


'''发送消息的配置参数'''
# 时间标识
TIME_STAMP = {"off": 0,
              "on" : 1,
              }

# 是否使用时间标识
TIME_FLAG = {"off": 0,
             "on" : 1,
             }

# 发送方式
SEND_TYPE = {"normal"     : 0, # 正常发送 发送失败自动重发 1.5-3s
             "single"     : 1, # 单次发送 只发送一次 不自动重发
             "test"       : 2, # 自发自收 用于测试CAN卡是否损坏
             "single_test": 3, # 单次自发自收 只发送一次
             }

# 帧类型
REMOTE_FLAG = {"data"  : 0, # 数据帧
               "remote": 1, # 远程帧
               }

# 数据类型
EXTERN_FLAG = {"standard": 0, # 标准帧
               "extern"  : 1, # 扩展帧
               }

# 数据长度
DATA_LEN = {"default": 8, # 默认长度
            "remote" : 2, # 用于控制远程节点的长度
            }
