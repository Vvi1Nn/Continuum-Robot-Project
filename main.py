import threading
import time
import platform
import sys, os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from usbcan.usbcan_functions import *
from motor.motor_functions import *

if __name__=="__main__":
    
    usbcan_1 = UsbCan()
    motor_1 = Motor(1)
    motor_2 = Motor(2)

    usbcan_1.Open()

    usbcan_1.StartCAN()

    msg = motor_1.Read(od.Index["control_word"], od.SubIndex["control_word"])
    print(msg["COB-ID"])
    print(msg["data"])
    usbcan_1.SendMsgs(msg["COB-ID"], msg["data"])

    
    usbcan_1.Close()
