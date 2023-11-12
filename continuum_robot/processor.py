# -*- coding:utf-8 -*-


''' processor.py CAN总线处理模块 '''


import time

from canopen import CanOpenMsgGenerator


class CanOpenBusProcessor(CanOpenMsgGenerator):
    device = None # CANopen总线在CAN卡的通道

    node_dict = {} # 节点对象字典

    def __init__(self, node_id) -> None:
        super().__init__(node_id)

        self.bus_status = None # 总线状态

        self.bus_is_checked = False # 总线检查

        self.sdo_feedback = (False, None, "", None) # 是否有反馈 状态 标签 数据
        
        self.nmt_feedback = (False, "") # 是否有反馈 状态 标签

        self.node_dict[node_id] = self # 加入节点对象列表
    
    ''' 绑定设备 '''
    @classmethod
    def linkDevice(cls, device: object) -> None:
        cls.device = device

    ''' 获取总线状态 '''
    def getBusStatus(self, /, *, times=1, delay=0.5, log=False) -> str:
        print_dict = {
            "success":      "\033[0;32m[Node-ID {}] current bus status: {}\033[0m".format(self.node_id, self.bus_status),
            "no_feedback":  "\033[0;33m[Node-ID {}] no nmt feedback ...\033[0m".format(self.node_id),
            "fail":         "\033[0;31m[Node-ID {}] get bus fail\033[0m".format(self.node_id),
            "send_fail":    "\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return "error"
        
        self.nmt_feedback = (False, "")

        [cob_id, data] = self.nmt_get_status() # 生成消息
        if self.device.sendMsg(cob_id, [data], remote_flag="remote", data_len="default"):
            start_time = time.time()

            while time.time() - start_time < delay:
                if self.nmt_feedback[0]:
                    self.bus_status, status = self.nmt_feedback[1], "success"
                    break
            else: status = "no_feedback"
        else: status = "send_fail"

        if log: print(print_dict[status])
        return self.bus_status if status == "success" else self.getBusStatus(times=times-1, delay=delay, log=log)

    ''' 设置总线状态 '''
    def setBusStatus(self, label: str, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        print_dict = {
            "success":      "\033[0;32m[Node-ID {}] SET BUS: {}\033[0m".format(self.node_id, label),
            "fail":         "\033[0;31m[Node-ID {}] set bus fail\033[0m".format(self.node_id),
            "get_bus_fail": "\033[0;31m[Node-ID {}] get bus status failed, return is error\033[0m".format(self.node_id),
            "send_fail":    "\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return False
    
        bus_status = {
            "start_remote_node":           "operational",
            "stop_remote_node":            "stopped",
            "enter_pre-operational_state": "pre-operational",
            "reset_node":                  "pre-operational",
            "reset_communication":         "pre-operational",
        }

        [cob_id, data] = self.nmt_change_status(label) # 生成消息
        if self.device.sendMsg(cob_id, [data], remote_flag="data", data_len="remote"):
            if not check: self.bus_status, status = bus_status[label], "success"
            else:
                ret = self.getBusStatus(delay=delay, log=log)
                if ret == bus_status[label]: status = "success"
                else: status = "fail" if ret != "error" else "get_bus_fail"
        else: status = "send_fail"
        
        if log: print(print_dict[status])
        return True if status == "success" else self.setBusStatus(label, times=times-1, check=check, delay=delay, log=log)
    
    ''' 检查总线的状态 '''
    def checkBusStatus(self, /, *, times=1, log=False) -> bool:
        print_dict = {
            "success": "\033[0;32m[Node-ID {}] Bus is checked\033[0m".format(self.node_id),
            "again":   "\033[0;33m[Node-ID {}] Checking bus status ...\033[0m".format(self.node_id),
            "fail":    "\033[0;31m[Node-ID {}] Bus is unchecked\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return False
        
        if not self.bus_is_checked:
            if self.bus_status == "pre-operational" or self.setBusStatus("enter_pre-operational_state"):
                self.bus_is_checked, status = True, "success"
            else: status = "again"
        else: status = "success"

        if log: print(print_dict[status])
        return True if self.bus_is_checked else self.checkBusStatus(times=times-1, log=log)

    ''' SDO 读 '''
    def readSDO(self, label: str, /, *, times=1, delay=0.5, format=1, log=False):
        print_dict = {
            "bus_status_wrong":   "\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id),
            "sdo_feedback_wrong": "\033[0;31m[Node-ID {}] sdo_feedback is wrong\033[0m".format(self.node_id),
            "fail":               "\033[0;31m[Node-ID {}] read {} failed\033[0m".format(self.node_id, label),
            "send_fail":          "\033[0;33m[Node-ID {}] reading {} ...\033[0m".format(self.node_id, label),
            "no_feedback":        "\033[0;33m[Node-ID {}] waiting sdo feedback ...\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return None

        if self.bus_is_checked and self.bus_status != "stopped":
            if not self.sdo_feedback[0]:
                [cob_id, data] = super().sdo_read(label)
                if self.device.sendMsg(cob_id, [data], remote_flag="data", data_len="default"):
                    start_time = time.time()
                    while time.time() - start_time < delay:
                        if self.sdo_feedback[0] and self.sdo_feedback[2] == label:
                            if self.sdo_feedback[1]:
                                if format == 1: value_list = [self.__hex_list_to_int(self.sdo_feedback[3])]
                                elif format == 2: value_list = [self.__hex_list_to_int(self.sdo_feedback[3][i:i+2]) for i in range(0,4,2)]
                                elif format == 4: value_list = [self.__hex_list_to_int([self.sdo_feedback[3][i]]) for i in range(4)]
                                else: value_list = None

                                self.sdo_feedback = (False, None, "", None) # 清除
                                return value_list
                            else:
                                status = "fail"
                                break
                    else: status = "no_feedback"
                else: status = "send_fail"
            else: status = "sdo_feedback_wrong"
        else: status = "bus_status_wrong"

        if log: print(print_dict[status])
        return self.readSDO(times=times-1, delay=delay, format=format, log=log)

    ''' SDO 写 '''
    def writeSDO(self, label: str, value: int, bit: str, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        print_dict = {
            "bus_status_wrong": "\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id),
            "success":          "\033[0;32m[Node-ID {}] write SDO {}: {} = {}\033[0m".format(self.node_id, bit, label, value),
            "again":            "\033[0;33m[Node-ID {}] sdo_feedback status wrong, try again\033[0m".format(self.node_id),
            "no_feedback":      "\033[0;33m[Node-ID {}] no sdo feedback ...\033[0m".format(self.node_id),
            "send_fail":        "\033[0;33m[Node-ID {}] usbcan sending message ...\033[0m".format(self.node_id),
            "fail":             "\033[0;31m[Node-ID {}] write SDO {} fail\033[0m".format(self.node_id, label),
        }

        msg_gen = {
                "32": super().sdo_write_32,
                "16": super().sdo_write_16,
                "8":  super().sdo_write_8,
            }

        if times == 0 or bit not in msg_gen.keys():
            if log: print(print_dict["fail"])
            return False
        
        if self.bus_is_checked and self.bus_status != "stopped":
            [cob_id, data] = msg_gen[bit](label, value) # 生成消息

            if self.device.sendMsg(cob_id, [data], remote_flag="data", data_len="default"):
                if not check: status = "success"
                else:
                    start_time = time.time()
                    while time.time() - start_time < delay:
                        if self.sdo_feedback[0] and self.sdo_feedback[2] == label:
                            if self.sdo_feedback[1]: self.sdo_feedback, status = (False, None, "", None), "success"
                            else: status = "again"
                            break
                    else: status = "no_feedback"
            else: status = "send_fail"
        else: status = "bus_status_wrong"

        if log: print(print_dict[status])
        return True if status == "success" else self.writeSDO(label, value, bit, times=times-1, check=check, delay=delay, log=log)

    ''' RPD写操作 分别写低字和高字 '''
    def sendRPDO(self, channel: str, *args: int, format=None, times=1, log=False) -> bool:
        print_dict = {
            "send_fail":        "\033[0;33m[Node-ID {}] rpdo {} ...\033[0m".format(self.node_id, channel),
            "fail":             "\033[0;31m[Node-ID {}] rpdo() {} failed\033[0m".format(self.node_id, channel),
            "again":            "\033[0;31m[Node-ID {}] try rpdo() {} again\033[0m".format(self.node_id, channel),
            "bus_status_wrong": "\033[0;31m[Node-ID {}] bus status wrong, no sdo\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return False
        
        if self.bus_is_checked and self.bus_status == "operational":
            [cob_id, data] = super().rpdo(channel, *args, format=format) # 生成消息
            if self.device.sendMsg(cob_id, [data], remote_flag="data", data_len="default"): status = "success"
            else: status = "again"
        else: status = "bus_status_wrong"

        if log: print(print_dict[status])
        return True if status == "success" else self.sendRPDO(channel, *args, format=format, times=times-1, log=log)

    ''' 设置 TPDO 模式 '''
    def set_tpdo_mode(self, label="asynchronous", /, *, channel: str, times=1, check=True, delay=0.5, log=True) -> bool:
        print_dict = {
            "success": "\033[0;32m[Node-ID {}] tpdo {} transmission type: {}\033[0m".format(self.node_id, channel, label),
            "fail":    "\033[0;31m[Node-ID {}] set tpdo {} transmission type failed\033[0m".format(self.node_id, channel),
            "again":   "\033[0;33m[Node-ID {}] setting tpdo {} transmission type ...\033[0m".format(self.node_id, channel),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return False
        
        if self.writeSDO("tpdo_{}_transmission_type".format(channel), self.TPDO_MODE[label], "8", check=check, delay=delay): status = "success"
        else: status = "again"

        if log: print(print_dict[status])
        return True if status == "success" else self.set_tpdo_mode(label="asynchronous", channel=channel, times=times-1, check=check, delay=delay, log=log)
    
    ''' 设置 TPDO 禁止时间 '''
    def set_tpdo_inhibit_time(self, value=10, /, *, channel: str, times=1, log=True, check=True, delay=0.5) -> bool:
        print_dict = {
            "success": "\033[0;32m[Node-ID {}] tpdo {} inhibit time: {}\033[0m".format(self.node_id, channel, value),
            "fail":    "\033[0;31m[Node-ID {}] set tpdo {} inhibit time failed\033[0m".format(self.node_id, channel),
            "again":   "\033[0;33m[Node-ID {}] setting tpdo {} inhibit time ...\033[0m".format(self.node_id, channel),
        }

        if times == 0:
            if log: print(print_dict["fail"])
            return False
        
        if self.writeSDO("tpdo_{}_inhibit_time".format(channel), value, "8", check=check, delay=delay): status = "success"
        else: status = "again"

        if log: print(print_dict[status])
        return True if status == "success" else self.set_tpdo_inhibit_time(value, channel=channel, times=times-1, log=log, check=check, delay=delay)

    ''' 开启PDO '''
    def startPDO(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        print_dict = {
            "success": "\033[0;32m[Node-ID {}] PDO START\033[0m".format(self.node_id),
            "again": "\033[0;33m[Node-ID {}] starting pdo ...\033[0m".format(self.node_id),
            "fail": "\033[0;31m[Node-ID {}] start pdo failed\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print_dict["fail"]()
            return False
        
        if self.setBusStatus("start_remote_node", check=check, delay=delay, log=log): status = "success"
        else: status = "again"
        
        if log: print(print_dict[status])
        return True if status == "success" else self.start_pdo(times=times-1, check=check, delay=delay, log=log)

    ''' 关闭PDO '''
    def stopPDO(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        print_dict = {
            "success": "\033[0;32m[Node-ID {}] PDO STOP\033[0m".format(self.node_id),
            "again": "\033[0;33m[Node-ID {}] stopping pdo ...\033[0m".format(self.node_id),
            "fail": "\033[0;31m[Node-ID {}] stop pdo failed\033[0m".format(self.node_id),
        }

        if times == 0:
            if log: print_dict["fail"]()
            return False
        
        if self.setBusStatus("enter_pre-operational_state", check=check, delay=delay, log=log): status = "success"
        else: status = "again"
        
        if log: print(print_dict[status])
        return True if status == "success" else self.stop_pdo(times=times-1, check=check, delay=delay, log=log)

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
