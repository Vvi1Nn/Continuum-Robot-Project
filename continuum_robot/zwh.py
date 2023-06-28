# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from continuum_robot.usbcan import UsbCan
from continuum_robot.processor import CanOpenBusProcessor
from continuum_robot.io import IoModule

from PyQt5.QtWidgets import QApplication
from PyQt5.QtCore import QThread, pyqtSignal

class CANopenUpdateThread(QThread):
    # pdo_1_update_signal = pyqtSignal(int)
    # pdo_2_update_signal = pyqtSignal(int)
    
    def __init__(self) -> None:
        super().__init__()
        self.__is_stop = False

        # self.pdo_1_update_signal.connect(pdo_1_slot_function)
        # self.pdo_2_update_signal.connect(pdo_2_slot_function)
    
    def run(self):
        while not self.__is_stop:
            ret = CanOpenBusProcessor.device.read_buffer(1, wait_time=0)
            
            if ret != None:
                [num, msg] = ret
                
                for i in range(num):
                    
                    
                    # SDO
                    if msg[i].ID > 0x580 and msg[i].ID < 0x600:
                        node_id = msg[i].ID - 0x580
                        
                        command =  msg[i].Data[0]
                        if command == CanOpenBusProcessor.CMD_R["read_16"] or CanOpenBusProcessor.CMD_R["read_32"] or CanOpenBusProcessor.CMD_R["read_8"]: status = True
                        elif command == CanOpenBusProcessor.CMD_R["write"]: status = True
                        elif command == CanOpenBusProcessor.CMD_R["error"]: status = False
                        else: status = None
                        
                        index = self.__match_index(msg[i].Data[1], msg[i].Data[2], msg[i].Data[3])
                        for key in CanOpenBusProcessor.OD: # 遍历字典关键字
                            if index == CanOpenBusProcessor.OD[key]: # 在每一个关键字对应的列表中 核对数值
                                label = key
                                break
                            else: pass
                        else: label = ""
                        
                        value_list = [msg[i].Data[j] for j in range(4,8)]
                        
                        # print("OLD", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)

                        wait_time = 1
                        time_stamp = time.time()
                        while CanOpenBusProcessor.node_dict[node_id].sdo_feedback[0] and time.time() - time_stamp < wait_time: time.sleep(0.1)
                        
                        CanOpenBusProcessor.node_dict[node_id].sdo_feedback = (True, status, label, value_list)

                        print("[SDO NEW] ", CanOpenBusProcessor.node_dict[node_id].sdo_feedback)
                    
                    # NMT
                    elif msg[i].ID > 0x700 and msg[i].ID < 0x780:
                        node_id = msg[i].ID - 0x700

                        for key in CanOpenBusProcessor.NMT_STATUS:
                            if msg[0].Data[0] & 0b01111111 == CanOpenBusProcessor.NMT_STATUS[key]:
                                label = key
                                break
                            else: pass
                        else: label = ""

                        # print("old  ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                        CanOpenBusProcessor.node_dict[node_id].nmt_feedback = (True, label)
                        
                        print("[NMT NEW] ", CanOpenBusProcessor.node_dict[node_id].nmt_feedback)
                    
                    # 其他
                    else: pass
            else: pass
    
    def stop(self):
        self.__is_stop = True

    ''' hex列表转换为int '''
    @staticmethod
    def __hex_list_to_int(data_list) -> int:
        data_str = ""
        for i in range(len(data_list)):
            data_bin = bin(data_list[i])[2:] # 首先转换为bin 去除0b
            data_bin = '0' * (8 - len(data_bin)) + data_bin # 头部补齐
            data_str = data_bin + data_str # 拼接
        # 首位是0 正数
        if int(data_str[0]) == 0: return int(data_str, 2)
        # 首位是1 负数
        else: return - ((int(data_str, 2) ^ 0xFFFFFFFF) + 1)
    
    ''' 匹配地址 '''
    @staticmethod
    def __match_index(index_low, index_high, subindex) -> list:
        # 低位 int转hex
        index_low_str = hex(index_low)[2:].upper()
        index_low_str = (2 - len(index_low_str)) * "0" + index_low_str
        # 高位 int转hex
        index_high_str = hex(index_high)[2:].upper()
        index_high_str = (2 - len(index_high_str)) * "0" + index_high_str
        # 合并 转int
        index = int(index_high_str + index_low_str, 16)
        # 返回列表的形式 以供比较
        return [index, subindex]



def abc():
    print("111")
# CAN卡实例化
usbcan_0 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("0") # 通道0
usbcan_1 = UsbCan.set_device_type(type="USBCAN2", index="0").is_show_log(False)("1") # 通道1

CanOpenBusProcessor.link_device(usbcan_0) # 将CANopen总线绑定至CAN卡的通道0

io = IoModule(11, abc)


app = QApplication(sys.argv)

if UsbCan.open_device():
    usbcan_0.set_timer("250K")
    usbcan_1.set_timer("1000K")


    
    if usbcan_0.init_can() and usbcan_0.start_can():
        CanOpenBusProcessor.link_device(usbcan_0)
        read_canopen_thread = CANopenUpdateThread()
        read_canopen_thread.start()
        print("222")
    time.sleep(0.5)

    io.check_bus_status()
    time.sleep(0.5)

    io.initialize_device(log=True)
    time.sleep(0.5)

    io.start_device(log=True)
    time.sleep(0.5)

    io.close_valve_4()

    # if usbcan_1.init_can() and usbcan_1.start_can(): CanOpenBusProcessor.link_device(usbcan_0)
    # else: pass

else: print("1111")