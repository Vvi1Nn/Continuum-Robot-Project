# -*- coding:utf-8 -*-


# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# 导入自定义模块
import canopen.protocol as protocol


class CanOpenMsgGenerator():
    __is_print = False # 标志符 是否打印结果
    
    def __init__(self, node_id) -> None:
        self.node_id = node_id # 节点ID
    
    ''' 设置是否输出结果 '''
    @classmethod
    def is_print_msg(cls, is_print: bool):
        cls.__is_print = is_print
        return cls
    
    ''' 将OD的地址拆分成两个hex '''
    @staticmethod
    def __split_index(index: int) -> list:
        hex_str = hex(index)[2:].upper() # int转换str 去除0x 大写字母
        high_int = int(hex_str[0:2], 16) # 高位
        low_int = int(hex_str[2:], 16) # 低位
        return [low_int, high_int]
    
    ''' 将int值转换成拆分好的hex列表 '''
    @staticmethod
    def __int_to_hex_list(value: int) -> list:
        # 正数 int转换str 去除0x 大写字母 前面补0
        if value >= 0: value_str = '0'*(8-len(value_str)) + hex(value)[2:].upper()
        # 负数 取反加1 
        else: value_str = hex(int(bin(- value ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
        # 写成列表
        value_list = [0x0] * 4
        value_list[0] = int(value_str[6:8], 16)
        value_list[1] = int(value_str[4:6], 16)
        value_list[2] = int(value_str[2:4], 16)
        value_list[3] = int(value_str[0:2], 16)
        return value_list
    
    ''' SDO读取指定标签 '''
    def sdo_read(self, label: str) -> list:
        index = protocol.OD[label][0] # 获取地址
        subindex = protocol.OD[label][1] # 获取索引
        cob_id = protocol.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = protocol.CMD_T["read"] # CMD
        data[1] = self.__split_index(index)[0] # 地址低位
        data[2] = self.__split_index(index)[1] # 地址高位
        data[3] = subindex # 索引
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_read] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

    ''' SDO写指定标签 32bit数据 '''
    def sdo_write_32(self, label: str, value: int) -> list:
        index = protocol.OD[label][0] # 获取地址
        subindex = protocol.OD[label][1] # 获取索引
        cob_id = protocol.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = protocol.CMD_T["write_32"] # CMD
        data[1] = self.__split_index(index)[0] # 地址低位
        data[2] = self.__split_index(index)[1] # 地址高位
        data[3] = subindex # 索引
        value = self.__int_to_hex_list(value) # int转hex列表
        for i in range(4):
            data[4+i] = value[i] # 赋值
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_write_32] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

    ''' 写RPDO 指定通道 低字 高字 '''
    def rpdo(self, num: str, value_low: int, value_high: int) -> dict:
        cob_id = protocol.CAN_ID["RPDO_" + num] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        value_low = self.__int_to_hex_list(value_low) # 低字 int转hex列表
        for i in range(4):
            data[i] = value_low[i] # 赋值
        value_high = self.__int_to_hex_list(value_high) # 高字 int转hex列表
        for i in range(4):
            data[4+i] = value_high[i] # 赋值
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[rpdo_{}]".format(num))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

    ''' 写NMT 指定状态 '''
    def nmt_change_status(self, label: str) -> list:
        cob_id = protocol.CAN_ID["NMT_C"] # 计算COB-ID
        data = [protocol.CMD_NMT[label], self.node_id] # 数据
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[nmt_control]")
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

    ''' 读NMT '''
    def nmt_get_status(self) -> list:
        cob_id = protocol.CAN_ID["NMT_S"] + self.node_id # 计算COB-ID
        data = [0x00] * 8 # 空数据
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[nmt_status]")
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

''' 测试用 '''
if __name__ == "__main__":
    # sdo_read(1, "control_mode", True) # 查看控制模式
    # sdo_write_32(1, "control_mode", pro.CONTROL_MODE["position_control"], True) # 位置模式
    # sdo_write_32(1, "acceleration", 1000, True) # 加速度1000
    # sdo_write_32(1, "deceleration", 10000, True) # 减速度10000
    # sdo_write_32(1, "velocity", 100, True) # 速度100
    # sdo_write_32(1, "target_position", 10000, True) # 目标位置10000
    # sdo_write_32(1, "control_word", 0x6F, True) # 相对使能
    # sdo_write_32(1, "control_word", 0x7F, True) # 启动
    # start_pdo(3, True)
    # rpdo(1, 5, 10000, -10000, True)
    # sdo_write_32(1, "tpdo_2_timer", 100, True)
    # nmt_change_status(2, "start_remote_node", True)
    # nmt_change_status(2, "enter_pre-operational_state", True)
    # nmt_get_status(2, True)
    CanOpenMsgGenerator.is_print_msg(True)
    generator_1 = CanOpenMsgGenerator(0xB)
    generator_1.nmt_change_status("start_remote_node")
    
    generator_2 = CanOpenMsgGenerator(0x2)
    generator_2.sdo_write_32("tpdo_2_timer", 100)
    generator_2.sdo_write_32("tpdo_2_inhibit", 500)