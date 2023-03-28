import threading
import time
import platform
import sys, os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from usbcan.usbcan_functions import *

import motor.function as motor_func
import motor.protocol as motor_proto 

if __name__=="__main__":
    
    usbcan_1 = UsbCan()
    motor_1 = motor_func.Motor(1)
    motor_2 = motor_func.Motor(2)

    usbcan_1.Open()

    usbcan_1.StartCAN()

    msg = motor_1.read(motor_proto.OD["control_word"][0], motor_proto.OD["control_word"][1])
    print(msg["COB-ID"])
    print(msg["data"])
    usbcan_1.SendMsgs(msg["COB-ID"], msg["data"])

    
    usbcan_1.Close()
