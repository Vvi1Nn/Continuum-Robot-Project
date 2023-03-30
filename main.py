import threading
import time
import platform
import sys
import os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from usbcan.function import *
from motor.msg_generation import *
# import motor.protocol as motor_proto 

   
if __name__=="__main__":
    usbcan_1 = UsbCan()