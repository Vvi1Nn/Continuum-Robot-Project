import threading
import time
import platform
import sys
import os

# from ctypes import *
# USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from usbcan.function import *
from motor.msg_generation import *
# import motor.protocol as motor_proto 

if __name__=="__main__":
    
    usbcan_1 = UsbCan()
    usbcan_1.Open()
    usbcan_1.StartCAN()

    motor_1 = Motor(1, usbcan_1)
    motor_2 = Motor(2, usbcan_1)
    motor_3 = Motor(3, usbcan_1)
    motor_4 = Motor(4, usbcan_1)
    motor_5 = Motor(5, usbcan_1)
    motor_6 = Motor(6, usbcan_1)
    motor_7 = Motor(7, usbcan_1)
    motor_8 = Motor(8, usbcan_1)
    motor_9 = Motor(9, usbcan_1)

    msg = motor_1.sdo_read("control_word", True)
    print(msg["COB-ID"])
    print(msg["data"])

    # msg = motor_1.sdo_write_32("control_word", 10000, False)
    # print(msg["COB-ID"])
    # print(msg["data"])

    # msg = motor_1.sdo_write_32("control_word")
    # print(msg["COB-ID"])
    # print(msg["data"])

    # usbcan_1.SendMsgs(msg["COB-ID"], msg["data"])

    
    # usbcan_1.Close()
