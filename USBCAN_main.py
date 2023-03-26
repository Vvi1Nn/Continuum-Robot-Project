# -*- coding:utf-8 -*-

from ctypes import *
import threading
import time
import platform
from USBCAN.Parameters_USBCAN import *
from USBCAN.Structs_USBCAN import *
from USBCAN.Functions_USBCAN import *

USBCAN_Lib = cdll.LoadLibrary("./libusbcan.so") # 调用动态链接库

def input_thread():
   input()

if __name__=="__main__":
    
    print("-------------------")
    print(0x20)
    print("-------------------")

    # 打开设备
    open_success = OpenDevice(USBCAN2, DEVICE_INDEX)
    
    # 获取信息
    device_info = GetDeviceInfo(USBCAN2, DEVICE_INDEX)
    print("设备信息:\n{}".format(device_info))
    
    # 初始化 + 启动
    start_success = StartCAN(USBCAN2, DEVICE_INDEX, CHANNEL)

    # 发送
    LEN = 10
    msgs = (ZCAN_CAN_OBJ * LEN)() # 结构体数组
    for i in range(LEN):
        msgs[i].ID         = 0x100
        msgs[i].TimeStamp  = 0
        msgs[i].TimeFlag   = 0
        msgs[i].SendType   = 2 
        msgs[i].RemoteFlag = 0
        msgs[i].ExternFlag = 0
        msgs[i].DataLen    = 8
        for j in range(msgs[i].DataLen):
            msgs[i].Data[j] = j
    
    send_num = USBCAN_Lib.VCI_Transmit(USBCAN2, DEVICE_INDEX, CHANNEL, byref(msgs), LEN)
    if LEN == send_num:
        print("传输成功!!! 数量:{}".format(send_num))
    else:
        print("传输失败... 数量:{}".format(send_num))

    thread = threading.Thread(target = input_thread)
    thread.start()

    while True:
        time.sleep(0.1)
        receive_num = USBCAN_Lib.VCI_GetReceiveNum(USBCAN2, DEVICE_INDEX, CHANNEL)
        if receive_num:
            rcv_msgs = (ZCAN_CAN_OBJ * receive_num)()
            receive_success = USBCAN_Lib.VCI_Receive(USBCAN2, DEVICE_INDEX, CHANNEL, byref(rcv_msgs), receive_num, 100) 
            for i in range(receive_success):
                    print("GetNum:%d, OrderNUM :%d,Timestamp:%d, id:%s , dlc:%d ,data:%s"%(receive_num,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
                        rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))
        
        if thread.is_alive() == False:
            break
    
    # 重置设备
    reset_success = ResetDevice(USBCAN2, DEVICE_INDEX)

    # 关闭设备
    close_success = CloseDevice(USBCAN2, DEVICE_INDEX)
    
    del USBCAN_Lib
    print("结束!!!")
