# -*- coding:utf-8 -*-


''' processor.py CAN总线处理模块 '''


import time

from PyQt5.QtCore import QThread, pyqtSignal

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入自定义模块
from continuum_robot.canopen import CanOpenMsgGenerator
from continuum_robot.motor import Motor
from continuum_robot.io import IoModule


''' CANopen 发送 '''
class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None # CANopen总线在CAN卡的通道

    node_dict = {} # 节点对象字典

    def __init__(self, node_id) -> None:
        super().__init__(node_id)

        self.bus_status = None # 总线状态

        self.bus_is_checked = False # 总线检查

        self.sdo_feedback = (False, None, "", None) # 是否有反馈 状态 标签 数据
        
        self.nmt_feedback = (False, "") # 是否有反馈 状态 标签

        CanOpenBusProcessor.node_dict[node_id] = self # 加入节点对象列表
    

    ''' 绑定设备 '''
    @classmethod
    def link_device(cls, device: object) -> None:
        cls.device = device


    ''' 获取总线状态 '''
    def get_bus_status(self, /, *, times=1, delay=0.5, log=False) -> str:
        if not self.nmt_feedback[0]:
            [cob_id, data] = self.nmt_get_status() # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="remote", data_len="default"):
                    time_stamp = time.time()
                    while time.time() - time_stamp < delay:
                        if self.nmt_feedback[0]:
                            self.bus_status = self.nmt_feedback[1]
                            self.nmt_feedback = (False, "")
                            if log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
                            else: pass
                            return self.bus_status
                        else: pass
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Node-ID {}] no nmt feedback ...\033[0m".format(self.node_id))
                        else: pass
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
            else: return "error"
        else:
            self.bus_status = self.nmt_feedback[1]
            self.nmt_feedback = (False, "")
            if log: print("\033[0;32m[Node-ID {}] bus status: {}\033[0m".format(self.node_id, self.bus_status))
            else: pass
            return self.bus_status

    ''' 设置总线状态 '''
    def set_bus_status(self, label: str, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        [cob_id, data] = self.nmt_change_status(label) # 生成消息

        while times != 0:
            if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="remote"):
                if check:
                    if self.get_bus_status(delay=delay, log=log) != "error":
                        # 判断当前状态和设置的状态是否一致
                        if label == "start_remote_node" and self.bus_status == "operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            else: pass
                            return True
                        elif label == "stop_remote_node" and self.bus_status == "stopped":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            else: pass
                            return True
                        elif label == "enter_pre-operational_state" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            else: pass
                            return True
                        elif label == "reset_node" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            else: pass
                            return True
                        elif label == "reset_communication" and self.bus_status == "pre-operational":
                            if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                            else: pass
                            return True
                        else:
                            if log: print("\033[0;31m[Node-ID {}] bus status not match\033[0m".format(self.node_id))
                            else: pass
                            return False
                    else:
                        if log: print("\033[0;31m[Node-ID {}] get bus status failed, return is error\033[0m".format(self.node_id))
                        else: pass
                        return False
                else:
                    if label == "start_remote_node": self.bus_status = "operational"
                    elif label == "stop_remote_node": self.bus_status = "stopped"
                    elif label == "enter_pre-operational_state": self.bus_status = "pre-operational"
                    elif label == "reset_node": self.bus_status = "pre-operational"
                    elif label == "reset_communication": self.bus_status = "pre-operational"
                    else: pass

                    if log: print("\033[0;32m[Node-ID {}] set bus: {}\033[0m".format(self.node_id, label))
                    else: pass
                    return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id))
                else: pass
        else: return False
    
    ''' 检查总线的状态 '''
    def check_bus_status(self, /, *, times=1, log=False) -> bool:
        if not self.bus_is_checked:
            while times != 0:
                if self.set_bus_status("enter_pre-operational_state"):
                    self.bus_is_checked = True
                    if log: print("\033[0;32m[Node-ID {}] checked\033[0m".format(self.node_id))
                    else: pass
                    return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] checking bus status ...\033[0m".format(self.node_id))
                    else: pass
            else: 
                if log: print("\033[0;31m[Node-ID {}] unchecked\033[0m".format(self.node_id))
                else: pass
                return False
        else:
            if log: print("\033[0;32m[Node-ID {}] already checked\033[0m".format(self.node_id))
            else: pass
            return True

    ''' SDO 读 '''
    def sdo_read(self, label: str, /, *, times=1, delay=0.5, format=1, log=False):
        if self.bus_is_checked and self.bus_status != "stopped":
            if not self.sdo_feedback[0]:
                [cob_id, data] = super().sdo_read(label) # 生成消息

                while times != 0:
                    if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        if format == 1:
                                            value_list = [self.__hex_list_to_int(self.sdo_feedback[3])]
                                        elif format == 2:
                                            value_list = [self.__hex_list_to_int(self.sdo_feedback[3][0:2]), self.__hex_list_to_int(self.sdo_feedback[3][2:4])]
                                        elif format == 4:
                                            value_list = [self.__hex_list_to_int([self.sdo_feedback[3][0]]), self.__hex_list_to_int([self.sdo_feedback[3][1]]), self.__hex_list_to_int([self.sdo_feedback[3][2]]), self.__hex_list_to_int([self.sdo_feedback[3][3]])]
                                        else: value_list = None
                                        self.sdo_feedback = (False, None, "", None)
                                        return value_list
                                    else: return None
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Node-ID {}] reading {} ...\033[0m".format(self.node_id, label))
                        else: pass
                else:
                    if log: print("\033[0;31m[Node-ID {}] read {} failed\033[0m".format(self.node_id, label))
                    else: pass
                    return None
            else:
                if log: print("\033[0;31m[Node-ID {}] sdo_feedback is wrong\033[0m".format(self.node_id))
                else: pass
                return None
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return None

    ''' SDO 写 32bit '''
    def sdo_write_32(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":
            [cob_id, data] = super().sdo_write_32(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    ''' SDO 写 16bit '''
    def sdo_write_16(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":
            [cob_id, data] = super().sdo_write_16(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False
    
    '''' SDO 写 8bit '''
    def sdo_write_8(self, label: str, value: int, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        if self.bus_is_checked and self.bus_status != "stopped":    
            [cob_id, data] = super().sdo_write_8(label, value) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"):
                    if check:
                        time_stamp = time.time()
                        while time.time() - time_stamp < delay:
                            if self.sdo_feedback[0]:
                                if self.sdo_feedback[2] == label:
                                    if self.sdo_feedback[1]:
                                        self.sdo_feedback = (False, None, "", None)
                                        return True
                                    else: return False
                                else: pass
                            else: pass
                        else:
                            if log: print("\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id))
                            else: pass
                            times -= 1
                    else: return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] writing {} ...\033[0m".format(self.node_id, label))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] write {} failed\033[0m".format(self.node_id, label))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id))
            return False

    ''' RPD写操作 分别写低字和高字 '''
    def rpdo(self, channel: str, *args: int, format=None, times=1, log=False) -> bool:
        if self.bus_is_checked and self.bus_status == "operational":
            [cob_id, data] = super().rpdo(channel, *args, format=format) # 生成消息

            while times != 0:
                if CanOpenBusProcessor.device.send(cob_id, [data], remote_flag="data", data_len="default"): return True
                else:
                    times -= 1
                    if log: print("\033[0;33m[Node-ID {}] rpdo {} ...\033[0m".format(self.node_id, channel))
                    else: pass
            else:
                if log: print("\033[0;31m[Node-ID {}] rpdo() {} failed\033[0m".format(self.node_id, channel))
                else: pass
                return False
        else:
            print("\033[0;31m[Node-ID {}] bus status wrong, no pdo\033[0m".format(self.node_id))
            return False


    ''' 设置 TPDO 模式 '''
    def set_tpdo_mode(self, label="asynchronous", /, *, channel: str, times=1, check=True, delay=0.5, log=True) -> bool:
        while times != 0:
            if self.sdo_write_8("tpdo_{}_transmission_type".format(channel), CanOpenMsgGenerator.TPDO_MODE[label], check=check, delay=delay):
                if log: print("\033[0;32m[Node-ID {}] tpdo {} transmission type: {}\033[0m".format(self.node_id, channel, label))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] setting tpdo {} transmission type ...\033[0m".format(self.node_id, channel))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] set tpdo {} transmission type failed\033[0m".format(self.node_id, channel))
            else: pass
            return False
    
    ''' 设置 TPDO 禁止时间 '''
    def set_tpdo_inhibit_time(self, value=10, /, *, channel: str, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_8("tpdo_{}_inhibit_time".format(channel), value, check=check, delay=delay):
                if log: print("\033[0;32m[Node-ID {}] tpdo {} inhibit time: {}\033[0m".format(self.node_id, channel, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] setting tpdo {} inhibit time ...\033[0m".format(self.node_id, channel))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] set tpdo {} inhibit time failed\033[0m".format(self.node_id, channel))
            else: pass
            return False


    ''' 开启PDO '''
    def start_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("start_remote_node", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Node-ID {}] PDO START\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] starting pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] start pdo failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 关闭PDO '''
    def stop_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("enter_pre-operational_state", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Node-ID {}] PDO STOP\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Node-ID {}] stopping pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Node-ID {}] stop pdo failed\033[0m".format(self.node_id))
            else: pass
            return False


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


''' CANopen 接收 数据处理 '''
class CANopenUpdateThread(QThread):
    __pdo_1_update_signal = pyqtSignal(int)
    __pdo_2_update_signal = pyqtSignal(int)
    
    def __init__(self, /, *, pdo_1_slot_function, pdo_2_slot_function) -> None:
        super().__init__()
        self.__is_stop = False

        self.__pdo_1_update_signal.connect(pdo_1_slot_function)
        self.__pdo_2_update_signal.connect(pdo_2_slot_function)
    
    def run(self):
        while not self.__is_stop:
            ret = CanOpenBusProcessor.device.read_buffer(1, wait_time=0)
            
            if ret != None:
                [num, msg] = ret
                
                for i in range(num):
                    # TPDO1
                    if msg[i].ID > 0x180 and msg[i].ID < 0x200:
                        node_id = msg[i].ID - 0x180
                        
                        # 电机的ID
                        if node_id in Motor.motor_dict.keys():
                            status = self.__hex_list_to_int([msg[i].Data[0]]) # 状态字
                            
                            for key in Motor.STATUS_WORD: # 遍历字典关键字
                                for r in Motor.STATUS_WORD[key]: # 在每一个关键字对应的列表中 核对数值
                                    if status == r:
                                        Motor.motor_dict[node_id].servo_status = key # 更新电机的伺服状态
                                        break
                                    else: pass
                        # IO模块的ID
                        elif node_id in IoModule.io_dict.keys():
                            data_low = bin(msg[i].Data[0])[2:] # 首先转换为bin 去除0b
                            data_low = '0' * (8 - len(data_low)) + data_low # 头部补齐

                            data_high = bin(msg[i].Data[1])[2:]
                            data_high = '0' * (8 - len(data_high)) + data_high

                            data = data_high + data_low # 拼接

                            for i, c in enumerate(data):
                                setattr(IoModule.io_dict[node_id], f"switch_{16-i}", False if c == "0" else True)
                        # 其他的ID
                        else: pass

                        self.__pdo_1_update_signal.emit(node_id)
                    
                    # TPDO2
                    elif msg[i].ID > 0x280 and msg[i].ID < 0x300:
                        node_id = msg[i].ID - 0x280
                        
                        # 电机的ID
                        if node_id in Motor.motor_dict.keys():
                            position = self.__hex_list_to_int([msg[i].Data[j] for j in range(0,4)]) # 当前位置
                            speed = self.__hex_list_to_int([msg[i].Data[j] for j in range(4,8)]) # 当前速度
                            Motor.motor_dict[node_id].current_position = position
                            Motor.motor_dict[node_id].current_speed = speed
                        
                        # 其他的ID
                        else: pass

                        self.__pdo_2_update_signal.emit(node_id)
                    
                    # SDO
                    elif msg[i].ID > 0x580 and msg[i].ID < 0x600:
                        node_id = msg[i].ID - 0x580
                        
                        command =  msg[i].Data[0]
                        if command == CanOpenMsgGenerator.CMD_R["read_16"] or CanOpenMsgGenerator.CMD_R["read_32"] or CanOpenMsgGenerator.CMD_R["read_8"]: status = True
                        elif command == CanOpenMsgGenerator.CMD_R["write"]: status = True
                        elif command == CanOpenMsgGenerator.CMD_R["error"]: status = False
                        else: status = None
                        
                        index = self.__match_index(msg[i].Data[1], msg[i].Data[2], msg[i].Data[3])
                        for key in CanOpenMsgGenerator.OD: # 遍历字典关键字
                            if index == CanOpenMsgGenerator.OD[key]: # 在每一个关键字对应的列表中 核对数值
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

                        for key in CanOpenMsgGenerator.NMT_STATUS:
                            if msg[0].Data[0] & 0b01111111 == CanOpenMsgGenerator.NMT_STATUS[key]:
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
