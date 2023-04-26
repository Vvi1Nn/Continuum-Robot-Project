# -*- coding:utf-8 -*-


''' protocol.py CANopen总线协议 '''


''' CANopen协议通讯层对象字典 '''
# COB-ID的基址
CAN_ID = {"NMT_C" : 0x000,
          "TPDO_1": 0x180,
          "RPDO_1": 0x200,
          "TPDO_2": 0x280,
          "RPDO_2": 0x300,
          "TPDO_3": 0x380,
          "RPDO_3": 0x400,
          "TPDO_4": 0x480,
          "RPDO_4": 0x500,
          "SDO_T" : 0x580,
          "SDO_R" : 0x600,
          "NMT_S" : 0x700,
          }

# NMT操作的CMD
CMD_NMT = {"start_remote_node"          : 0x01,
           "stop_remote_node"           : 0x02,
           "enter_pre-operational_state": 0x80,
           "reset_node"                 : 0x81,
           "reset_communication"        : 0x82,
           }

# NMT当前状态
NMT_STATUS = {"boot_up"        : 0,
              "stopped"        : 4,
              "operational"    : 5,
              "pre-operational": 127,
              }

# SDO发送的CMD
CMD_T = {"read"    : 0x40,
         "write_32": 0x23,
         "write_16": 0x2B,
         "write_8" : 0x2F,
         }

# SDO接收的CMD
CMD_R = {"read_32" : 0x43,
         "read_16" : 0x4B,
         "read_8"  : 0x4F,
         "write"   : 0x60,
         "error"   : 0x80,
         }

''' CANopen协议应用层对象字典 '''
# 电机的控制字 按位
CONTROL_WORD_BIT = {"switch_on"        : {"bit": 0, False: 0, True: 1},
                    "enable_voltage"   : {"bit": 1, False: 0, True: 1},
                    "quick_stop"       : {"bit": 2, False: 1, True: 0},
                    "enable_operation" : {"bit": 3, False: 0, True: 1},
                    "action"           : {"bit": 4, False: 0, True: 1},
                    "is_immediate"     : {"bit": 5, False: 0, True: 1},
                    "is_relative"      : {"bit": 6, False: 0, True: 1},
                    "fault_reset"      : {"bit": 7, False: 0, True: 1},
                   }

# 电机的控制字
CONTROL_WORD = {"reset"             : 0b10000000, # 0x80 优先级 1
                "power_off"         : 0b00000000, # 0x00 优先级 2
                "quick_stop"        : 0b00000010, # 0x02 优先级 3
                "servo_close"       : 0b00000110, # 0x06 优先级 4
                "servo_ready/stop"  : 0b01100111, # 0x67 优先级 5 立即模式 相对运行
                "servo_enable/start": 0b01111111, # 0x7F 优先级 5 立即模式 相对运行
               }

# 电机的状态字 按位
STATUS_WORD_BIT = {"ready_to_switch_on"     : {"bit": 0, 0: False, 1: True},
                   "switched_on"            : {"bit": 1, 0: False, 1: True},
                   "operation_enabled"      : {"bit": 2, 0: False, 1: True},
                   "fault"                  : {"bit": 3, 0: False, 1: True},
                   "voltage_enabled"        : {"bit": 4, 0: False, 1: True},
                   "quick_stop"             : {"bit": 5, 1: False, 0: True},
                   "switch_on_disabled"     : {"bit": 6, 0: False, 1: True},
                   "warning"                : {"bit": 7, 0: False, 1: True},
                   "manufacturer"           : {"bit": 8, 0: None, 1: None},
                   "remote"                 : {"bit": 9, 0: False, 1: True},
                   "target_reached"         : {"bit": 10, 0: False, 1: True},
                   "internal_limit_active"  : {"bit": 11, 0: False, 1: True},
                   "set_point_acknowledge"  : {"bit": 12, 0: False, 1: True},
                   "following_error"        : {"bit": 13, 0: False, 1: True},
                   "manufacturer_specific_1": {"bit": 14, 0: None, 1: None},
                   "manufacturer_specific_2": {"bit": 15, 0: None, 1: None},
                   }

# 电机的状态字
STATUS_WORD = {"not_ready_to_switch_on": [0x00, 0x10, 0x20, 0x30, 0x80, 0x90, 0xA0, 0xB0], # 初始化
               "switch_on_disabled"    : [0x30, 0x40, 0x50, 0x60, 0xC0, 0xD0, 0xE0, 0xF0], # 伺服无故障
               "ready_to_switch_on"    : [0x21, 0x31, 0xA1, 0xB1],                         # 伺服准备好
               "switched_on"           : [0x23, 0x33, 0xA3, 0xB3],                         # 伺服等待使能
               "operation_enabled"     : [0x27, 0x37, 0xA7, 0xB7],                         # 伺服运行
               "quick_stop_active"     : [0x07, 0x17, 0x87, 0x97],                         # 快速停机
               "fault_reaction_active" : [0x0F, 0x1F, 0x2F, 0x3F, 0x8F, 0x9F, 0xAF, 0xBF], # 故障停机
               "fault"                 : [0x08, 0x18, 0x28, 0x38, 0x88, 0x98, 0xA8, 0xB8], # 故障
               }

# 控制模式
CONTROL_MODE = {"position_control": 1,
                "speed_control"   : 3,
                "home_control"    : 6,
                }

# 对象字典
OD = {"tpdo_1_transtype" : [0x1800, 0x02], # FE:数值变化触发 FF:定时触发
      "tpdo_1_inhibit"   : [0x1800, 0x03], # 禁止时间
      "tpdo_1_timer"     : [0x1800, 0x05], # 定时间隔 单位:ms
      "tpdo_2_transtype" : [0x1801, 0x02],
      "tpdo_2_inhibit"   : [0x1801, 0x03],
      "tpdo_2_timer"     : [0x1801, 0x05],
      "control_word"     : [0x6040, 0x00],
      "status_word"      : [0x6041, 0x00], # RO
      "control_mode"     : [0x6060, 0x00],
      "show_mode"        : [0x6061, 0x00], # RO
      "position_feedback": [0x6064, 0x00], # RO
      "speed_feedback"   : [0x606C, 0x00], # RO
      "target_position"  : [0x607A, 0x00],
      "velocity"         : [0x6081, 0x00],
      "acceleration"     : [0x6083, 0x00],
      "deceleration"     : [0x6084, 0x00],
      "target_speed"     : [0x60FF, 0x00],
      }
