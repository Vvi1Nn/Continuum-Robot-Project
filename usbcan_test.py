import threading
import time
import platform
import sys, os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from USBCAN.Parameters_USBCAN import *
from USBCAN.Structs_USBCAN import *
from USBCAN.Functions_USBCAN import *

if __name__=="__main__":
    msgs = (ZCAN_CAN_OBJ * 1)()
    msgs[0].ID = 5
    msgs[0].DataLen = 1
    msgs[0].Data[0] = 0x64
    print(msgs[0].ID)
    print(type(msgs[0].ID))
    print(msgs[0].Data[0])
    print(type(msgs[0].Data[0]))

    a = [0x00]*8
    print(type(a))
    if type(a) == list:
        print("a")
    else: print("b")
    print(len(a))

    val = 6459615
    print(hex(val)[2:].upper())

    