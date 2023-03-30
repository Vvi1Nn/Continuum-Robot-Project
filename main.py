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
   print(usbcan_1)
   usbcan_1.Open()
   usbcan_1.StartCAN()
   
   # motor_1 = Motor(1, usbcan_1)
   # motor_2 = Motor(2, usbcan_1)
   # motor_3 = Motor(3, usbcan_1)
   # motor_4 = Motor(4, usbcan_1)
   # motor_5 = Motor(5, usbcan_1)
   # motor_6 = Motor(6, usbcan_1)
   # motor_7 = Motor(7, usbcan_1)
   # motor_8 = Motor(8, usbcan_1)
   # motor_9 = Motor(9, usbcan_1)

   # msg = motor_1.sdo_read("control_mode", True)
   # print("COB-ID: {}".format(hex(msg["COB-ID"])))
   # print("DATA: {}".format(msg["data"]))
   usbcan_1.SendMsgs(0x601,[0x40,0x61,0x60,0x00,0x00,0x00,0x00,0x00])
   thread = threading.Thread(target=input())
   thread.start()

   while True:
      time.sleep(0.1)
      ret = USBCAN_Lib.VCI_GetReceiveNum(4, 0, 0)
      if ret:
         rcv_msgs = (usbcan_struct.ZCAN_CAN_OBJ * ret)()
         ret1 = USBCAN_Lib.VCI_Receive(4,0,0,byref(rcv_msgs),ret,-1) 
         for i in range(ret1):
                  print("GetNum:%d, OrderNUM :%d,Timestamp:%d, id:%s , dlc:%d ,data:%s"%(ret,i,(rcv_msgs[i].TimeStamp),hex(rcv_msgs[i].ID),\
                     rcv_msgs[i].DataLen,''.join(hex(rcv_msgs[i].Data[j])+ ' 'for j in range(rcv_msgs[i].DataLen))))
      if thread.is_alive() == False:
         break
   
   # msg = motor_1.sdo_write_32("control_word", 10000, False)
   # print(msg["COB-ID"])
   # print(msg["data"])

   # msg = motor_1.sdo_write_32("control_word")
   # print(msg["COB-ID"])
   # print(msg["data"])

   # usbcan_1.SendMsgs(msg["COB-ID"], msg["data"])

   
   # usbcan_1.Close()
