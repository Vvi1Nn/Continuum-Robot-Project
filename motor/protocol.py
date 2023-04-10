# -*- coding:utf-8 -*-

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

CMD_NMT = {"start_remote_node"          : 0x01,
           "stop_remote_node"           : 0x02,
           "enter_pre-operational_state": 0x80,
           "reset_node"                 : 0x81,
           "reset_communication"        : 0x82,
           }

NMT_STATUS = {"boot_up"        : 0,
              "stopped"        : 4,
              "operational"    : 5,
              "pre-operational": 127,
              }

CMD_T = {"read"   : 0x40,
        "write_32": 0x23,
        "write_16": 0x2B,
        "write_8" : 0x2F,
        }

CMD_R = {"read_32": 0x43,
        "read_16" : 0x4B,
        "read_8"  : 0x4F,
        "write"   : 0x60,
        "error"   : 0x80,
        }

CONTROL_WORD = {}

STATUS_WORD = {}

CONTROL_MODE = {"position_control": 1,
                "speed_control"   : 3,
                "home_control"    : 6,
                }

OD = {"save_all"         : [0x1010, 0x01],
      "tpdo_1_transtype" : [0x1800, 0x02], # FE:数值变化触发 FF:定时触发
      "tpdo_1_timer"     : [0x1800, 0x05], # 定时间隔 单位:ms
      "tpdo_2_transtype" : [0x1801, 0x02],
      "tpdo_2_timer"     : [0x1801, 0x05],
      "tpdo_3_transtype" : [0x1801, 0x02],
      "tpdo_3_timer"     : [0x1801, 0x05],
      "tpdo_4_transtype" : [0x1801, 0x02],
      "tpdo_4_timer"     : [0x1801, 0x05],
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
