# -*- coding:utf-8 -*-

ACC_CODE = {"default": 0,
            }

ACC_MASK = {"default": 0xFFFFFFFF,
            }

FILTER = {"single": 1,
          "dual":   0,
          }

TIMER = {"5K":    [0xBF, 0xFF],
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

MODE ={"normal": 0,
       "listen_only": 1,
       }
