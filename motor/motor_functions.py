# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from motor.motor_parameters import *
from modules.data_process import *
from usbcan.usbcan_functions import *

class Motor:
    
    def __init__(self, NodeID) -> None:
        self.NodeID = NodeID
    
    def Read(self, Index, SubIndex) -> dict or bool:
        SDO_param = SDOParam()
        motor_processor = MotorProcessor()
        COB_ID = SDO_param.ID_Send + self.NodeID
        data = [0x00] * 8
        data[0] = SDO_param.CMD_Send["read"]
        data[1] = motor_processor.SplitIndex(Index)["low"]
        data[2] = motor_processor.SplitIndex(Index)["high"]
        data[3] = SubIndex
        return {"COB-ID": COB_ID, "data": data}
