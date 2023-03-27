# -*- coding:utf-8 -*-

class SDOParam:
    def __init__(self) -> None:
        self.ID_Send = 0x600
        self.ID_Receive = 0x580
        self.CMD_Send = {"read": 0x40,
                       "write_32": 0x23,
                       "write_16": 0x2B,
                       "write_8": 0x2F,
                       }
        self.CMD_Receive = {"read_32": 0x43,
                       "read_16": 0x4B,
                       "read_8": 0x4F,
                       "read_error": 0x80,
                       "write": 0x60,
                       "write_error": 0x80,
                       }
class PDOParam:
    def __init__(self) -> None:
        pass

# # 偏移地址
# SDO_TX = 0x600
# SDO_RX = 0x580
# # 读
# SDO_CMD_READ = 0x40
# SDO_CALLBACK_READ_32 = 0x43
# SDO_CALLBACK_READ_16 = 0x4B
# SDO_CALLBACK_READ_8 = 0x4F
# SDO_CALLBACK_READ_ERROR = 0x80
# # 写
# SDO_CMD_WRITE_32 = 0x23
# SDO_CMD_WRITE_16 = 0x2B
# SDO_CMD_WRITE_8 = 0x2F
# SDO_CALLBACK_WRITE = 0x60
# SDO_CALLBACK_WRITE_ERROR = 0x80

class OD:
    def __init__(self) -> None:
        self.Index = {"control_word": 0x6040,
                      "status_word": 0x6041,
                      "control_mode": 0x6060,
                      }
        self.SubIndex = {"control_word": 0x00,
                         "status_word": 0x00,
                         "control_mode": 0x00,
                         }

# # 控制字
# CONTROL_WORD_INDEX = 0x6040
# CONTROL_WORD_SUBINDEX = 0x00
# # 状态字
# STATUS_WORD_INDEX = 0x6041
# STATUS_WORD_SUBINDEX = 0x00
# # 控制模式
# CONTROL_MODE_INDEX = 0x6060
# CONTROL_MODE_SUBINDEX = 0x00


SDO_param = SDOParam()
PDO_param = PDOParam()
od = OD()