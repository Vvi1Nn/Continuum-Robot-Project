# -*- coding:utf-8 -*-

CAN_ID = {"PDO_1_T": 0x180,
          "PDO_1_R": 0x200,
          "PDO_2_T": 0x280,
          "PDO_2_R": 0x300,
          "PDO_3_T": 0x380,
          "PDO_3_R": 0x400,
          "PDO_4_T": 0x480,
          "PDO_4_R": 0x500,
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
      }
