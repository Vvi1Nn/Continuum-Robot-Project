import threading
import time
import platform
import sys
import os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

import usbcan.function as usbcan_fun
import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param
import motor.msg_generation as motor_gen
import motor.protocol as motor_proto 
import motor.function as motor_fun

if __name__=="__main__":
    
    usbcan_1 = usbcan_fun.UsbCan()

    motor_1 = motor_fun.Motor(usbcan_1, 1, "position_control")
    motor_2 = motor_fun.Motor(usbcan_1, 2, "speed_control")

    while True:
        num = input("请输入电机序号: ")
        if num == '1':
            p = input("请输入目标位置: ")
            motor_1.feedback()
            motor_1.set_position(int(p))
            motor_1.execute()
        elif num == '2':
            p = input("请输入速度: ")
            if int(p) > 25:
                p = 25
            motor_2.feedback()
            motor_2.set_speed(p)
            motor_2.execute()
        else:
            motor_1.shut_down()
            motor_2.shut_down()
            print("结束!!!")
            break
    
    usbcan_1.receive()