import threading
import time
import platform
import sys
import os

from ctypes import *
USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

from usbcan.function import *
import usbcan.struct as usbcan_struct
import usbcan.param as usbcan_param
import motor.msg_generation as motor_gen
import motor.protocol as motor_proto 
from motor.function import *

if __name__=="__main__":
    
    UsbCan.open()
    usbcan_0 = UsbCan(usbcan_param.CHANNEL["0"])
    usbcan_1 = UsbCan(usbcan_param.CHANNEL["1"])

    Motor.config(usbcan_0)
    motor_1 = Motor(1)
    motor_2 = Motor(2)
    motor_3 = Motor(3)
    # motor_4 = Motor(4)
    # motor_5 = Motor(5)
    # motor_6 = Motor(6)
    # motor_7 = Motor(7)
    # motor_8 = Motor(8)
    # motor_9 = Motor(9)
    # motor_10 = Motor(10)

    while True:
        pass

    Action = True
    while Action:
        num = input("请输入电机序号: ")
        if num == '1':
            p = input("请输入目标位置: ")
            motor_1.feedback()
            motor_1.set_position(int(p))
            motor_1.execute(is_relative = False)
        elif num == '2':
            p = input("请输入目标位置: ")
            motor_2.feedback()
            motor_2.set_position(int(p))
            motor_2.execute()
        else:
            motor_1.shut_down()
            motor_2.shut_down()
            print("结束!!!")
            break
    
    # ret = motor_gen.sdo_read(1, "tpdo_2_timer", True)
    # usbcan_0.send(ret["id"], [ret["data"]], True)
    print(usbcan_0.get_buffer_num())

    [num, msgs] = usbcan_0.read_buffer(100)
    for i in range(num):
        print("GetNum:%d, OrderNUM:%d, Timestamp:%d, id:%s, dlc:%d, data:%s"%(num,i,(msgs[i].TimeStamp),hex(msgs[i].ID),\
                                                                            msgs[i].DataLen,''.join(hex(msgs[i].Data[j])+ ' 'for j in range(msgs[i].DataLen))))
    
    usbcan_0.clear_buffer()
    print(usbcan_0.get_buffer_num())

    UsbCan.close()
    
    
    # while True:
    #     time.sleep(0.1)
    #     ret = USBCAN_Lib.VCI_GetReceiveNum(4, 0, 0)
    #     if ret:
    #         rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ*ret)()
    #         ret1 = USBCAN_Lib.VCI_Receive(4, 0, 0, byref(rcv_msgs), ret, 100) 
    #         for i in range(ret1):
    #             print("GetNum:%d, OrderNUM:%d, Timestamp:%d, id:%s, dlc:%d, data:%s"%(ret,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
    #                                                                                    rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))