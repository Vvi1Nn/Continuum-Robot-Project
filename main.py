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
    
    usbcan_1 = usbcan_fun.UsbCan(channel=0)

    motor_1 = motor_fun.Motor(usbcan_1, 1, "position_control")
    motor_2 = motor_fun.Motor(usbcan_1, 2, "position_control")
    
    Action = True
    while Action:
        num = input("请输入电机序号: ")
        if num == '1':
            p = input("请输入目标位置: ")
            motor_1.feedback()
            motor_1.set_position(int(p))
            motor_1.execute(is_relative = False)
            usbcan_1.receive()
        elif num == '2':
            p = input("请输入目标位置: ")
            motor_2.feedback()
            motor_2.set_position(int(p))
            motor_2.execute()
            usbcan_1.receive()
        else:
            motor_1.shut_down()
            motor_2.shut_down()
            print("结束!!!")
            break
    
    ret = motor_gen.sdo_read(1, "tpdo2_timer", True)
    usbcan_1.send(ret["id"], [ret["data"]])
    usbcan_1.receive()
    # while True:
    #     time.sleep(0.1)
    #     ret = USBCAN_Lib.VCI_GetReceiveNum(4, 0, 0)
    #     if ret:
    #         rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ*ret)()
    #         ret1 = USBCAN_Lib.VCI_Receive(4, 0, 0, byref(rcv_msgs), ret, 100) 
    #         for i in range(ret1):
    #             print("GetNum:%d, OrderNUM:%d, Timestamp:%d, id:%s, dlc:%d, data:%s"%(ret,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
    #                                                                                    rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))