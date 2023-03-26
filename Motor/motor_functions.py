# -*- coding:utf-8 -*-

import sys, os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

from Motor.motor_parameters import *
from Modules.data_process import *
from USBCAN.usbcan_functions import *

class Motor:
    def __init__(self, NodeID) -> None:
        self.NodeID = NodeID
    def Read(self, Index, SubIndex, isSend = False) -> dict or bool:
        SDO_param = SDOParam()
        motor_processor = MotorProcessor()
        COB_ID = SDO_param.ID_Send + self.NodeID
        data = [0x00] * 8
        data[0] = SDO_param.CMD_Send["read"]
        data[1] = motor_processor.SplitIndex(Index)["low"]
        data[2] = motor_processor.SplitIndex(Index)["high"]
        data[3] = SubIndex
        if isSend:
            send_success = device_1.SendMsgs(COB_ID, data)
            return send_success
        else:
            return {"COB-ID": COB_ID, "data": data}

if __name__ == "__main__":
    generator = Motor(NodeID = 1)
    ret = generator.Read(od.Index["control_word"], od.SubIndex["control_word"])
    print(ret["data"])