# -*- coding:utf-8 -*-

CAN_ID = {"TPDO_1": 0x180,
          "RPDO_1": 0x200,
          "TPDO_2": 0x280,
          "RPDO_2": 0x300,
          "TPDO_3": 0x380,
          "RPDO_3": 0x400,
          "TPDO_4": 0x480,
          "RPDO_4": 0x500,
          "SDO_T": 0x580,
          "SDO_R": 0x600,
          }

CMD_T = {"read": 0x40,
        "write_32": 0x23,
        "write_16": 0x2B,
        "write_8": 0x2F,
        }

CMD_R = {"read_32": 0x43,
        "read_16": 0x4B,
        "read_8": 0x4F,
        "read_error": 0x80,
        "write": 0x60,
        "write_error": 0x80,
        }

OD = {"control_word": [0x6040, 0x00],
      "status_word": [0x6041, 0x00],
      "control_mode": [0x6060, 0x00],
      "position_feedback": [0x6064, 0x00],
      "speed_feedback": [0x606C, 0x00],
      "target_position": [0x607A, 0x00],
      "velocity": [0x6081, 0x00],
      "acceleration": [0x6083, 0x00],
      "deceleration": [0x6084, 0x00],
      "target_speed": [0x60FF, 0x00],
      }
