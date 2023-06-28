# -*- coding:utf-8 -*-


''' canopen.py CANopen协议 消息生成 '''


class CanOpenMsgGenerator():
    ''' CANopen协议通讯层对象字典 '''
    # COB-ID的基址
    CAN_ID = {"NMT_C" : 0x000,
              "TPDO_1": 0x180,
              "RPDO_1": 0x200,
              "TPDO_2": 0x280,
              "RPDO_2": 0x300,
              "TPDO_3": 0x380,
              "RPDO_3": 0x400,
              "TPDO_4": 0x480,
              "RPDO_4": 0x500,
              "SDO_T" : 0x580,
              "SDO_R" : 0x600,
              "NMT_S" : 0x700,
    }

    # NMT操作的CMD
    CMD_NMT = {"start_remote_node"          : 0x01,
               "stop_remote_node"           : 0x02,
               "enter_pre-operational_state": 0x80,
               "reset_node"                 : 0x81,
               "reset_communication"        : 0x82,
    }

    # NMT当前状态
    NMT_STATUS = {"boot_up"        : 0,
                  "stopped"        : 4,
                  "operational"    : 5,
                  "pre-operational": 127,
    }

    # SDO发送的CMD
    CMD_T = {"read"    : 0x40,
             "write_32": 0x23,
             "write_16": 0x2B,
             "write_8" : 0x2F,
    }

    # SDO接收的CMD
    CMD_R = {"read_32" : 0x43,
             "read_16" : 0x4B,
             "read_8"  : 0x4F,
             "write"   : 0x60,
             "error"   : 0x80,
    }

    # TPDO 模式
    TPDO_MODE = {"synchronous": 0x01,
                 "remote_transmission_request": 0xFD,
                 "asynchronous": 0xFF,
    }

    # 对象字典
    OD = {"tpdo_1_transmission_type" : [0x1800, 0x02], # FE:数值变化触发 FF:定时触发
          "tpdo_1_inhibit_time"   : [0x1800, 0x03], # 禁止时间
          "tpdo_1_timer"     : [0x1800, 0x05], # 定时间隔 单位:ms
          "tpdo_2_transmission_type" : [0x1801, 0x02],
          "tpdo_2_inhibit_time"   : [0x1801, 0x03],
          "tpdo_2_timer"     : [0x1801, 0x05],
          "tpdo_3_transmission_type" : [0x1802, 0x02],
          "tpdo_3_inhibit_time"   : [0x1802, 0x03],
          "tpdo_3_timer"     : [0x1802, 0x05],
          "tpdo_4_transmission_type" : [0x1803, 0x02],
          "tpdo_4_inhibit_time"   : [0x1803, 0x03],
          "tpdo_4_timer"     : [0x1803, 0x05],

          "control_word"     : [0x6040, 0x00],
          "status_word"      : [0x6041, 0x00], # RO
          "control_mode"     : [0x6060, 0x00],
          "show_mode"        : [0x6061, 0x00], # RO
          "position_feedback": [0x6064, 0x00], # RO
          "speed_feedback"   : [0x606C, 0x00], # RO
          "target_position"  : [0x607A, 0x00],
          "velocity"         : [0x6081, 0x00],
          "acceleration"     : [0x6083, 0x00],
          "deceleration"     : [0x6084, 0x00],
          "quick_stop_deceleration": [0x6085, 0x00],
          "motion_profile_type"     : [0x6086, 0x00],
          "target_speed"     : [0x60FF, 0x00],
    }

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
    
    
    ''' SDO 读 标签 '''
    def sdo_read(self, label: str) -> list:
        index = CanOpenMsgGenerator.OD[label][0] # 获取地址
        subindex = CanOpenMsgGenerator.OD[label][1] # 获取索引
        cob_id = CanOpenMsgGenerator.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = CanOpenMsgGenerator.CMD_T["read"] # CMD
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

    ''' SDO写 标签 32bit '''
    def sdo_write_32(self, label: str, value: int) -> list:
        index = CanOpenMsgGenerator.OD[label][0] # 获取地址
        subindex = CanOpenMsgGenerator.OD[label][1] # 获取索引
        cob_id = CanOpenMsgGenerator.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = CanOpenMsgGenerator.CMD_T["write_32"] # CMD
        data[1] = self.__split_index(index)[0] # 地址低位
        data[2] = self.__split_index(index)[1] # 地址高位
        data[3] = subindex # 索引
        # 正数 int转换str 去除0x 大写字母 前面补0
        if value >= 0: value_str = '0'*(8-len(hex(value)[2:].upper())) + hex(value)[2:].upper()
        # 负数 取反加1 
        else: value_str = hex(int(bin(- value ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
        for i in range(4):
            data[7-i] = int(value_str[2*i:2*(i+1)], 16)
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

    ''' SDO写 标签 16bit '''
    def sdo_write_16(self, label: str, value: int) -> list:
        index = CanOpenMsgGenerator.OD[label][0] # 获取地址
        subindex = CanOpenMsgGenerator.OD[label][1] # 获取索引
        cob_id = CanOpenMsgGenerator.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = CanOpenMsgGenerator.CMD_T["write_16"] # CMD
        data[1] = self.__split_index(index)[0] # 地址低位
        data[2] = self.__split_index(index)[1] # 地址高位
        data[3] = subindex # 索引
        # 正数 int转换str 去除0x 大写字母 前面补0
        if value >= 0: value_str = '0'*(4-len(hex(value)[2:].upper())) + hex(value)[2:].upper()
        # 负数 取反加1 
        else: value_str = hex(int(bin(- value ^ 0xFFFF), 2) + 1)[2:].upper()
        for i in range(2):
            data[5-i] = int(value_str[2*i:2*(i+1)], 16)
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_write_16] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]
    
    ''' SDO写 标签 8bit '''
    def sdo_write_8(self, label: str, value: int) -> list:
        index = CanOpenMsgGenerator.OD[label][0] # 获取地址
        subindex = CanOpenMsgGenerator.OD[label][1] # 获取索引
        cob_id = CanOpenMsgGenerator.CAN_ID["SDO_R"] + self.node_id # 计算COB-ID
        data = [0x00] * 8
        data[0] = CanOpenMsgGenerator.CMD_T["write_8"] # CMD
        data[1] = self.__split_index(index)[0] # 地址低位
        data[2] = self.__split_index(index)[1] # 地址高位
        data[3] = subindex # 索引
        data[4] = value
        # 打印结果
        if CanOpenMsgGenerator.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_write_8] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        return [cob_id, data]

    ''' RPDO 通道 传入数值可为 1 2 4 自动补齐 '''
    def rpdo(self, num: str, *args: int, format=None) -> dict:
        cob_id = CanOpenMsgGenerator.CAN_ID["RPDO_" + num] + self.node_id # 计算COB-ID
        string = ""
        if format != None: args = args + (0,)*(format-len(args))
        for value in args:
            # 正数 int转换str 去除0x 大写字母 前面补0
            if value >= 0: string = '0'*(int(16/len(args))-len(hex(value)[2:].upper())) + hex(value)[2:].upper() + string
            # 负数 取反加1
            else: string = hex(int(bin(- value ^ int(int(16/len(args))*'F', 16)), 2) + 1)[2:].upper() + string
        data = [0x0] * 8
        for i in range(8):
            data[7-i] = int(string[2*i:2*(i+1)], 16)
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

    ''' 写NMT '''
    def nmt_change_status(self, label: str) -> list:
        cob_id = CanOpenMsgGenerator.CAN_ID["NMT_C"] # 计算COB-ID
        data = [CanOpenMsgGenerator.CMD_NMT[label], self.node_id] # 数据
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
        cob_id = CanOpenMsgGenerator.CAN_ID["NMT_S"] + self.node_id # 计算COB-ID
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
    # generator_1 = CanOpenMsgGenerator(11)
    # generator_1.nmt_change_status("start_remote_node")
    
    generator_2 = CanOpenMsgGenerator(2)
    # generator_2.sdo_write_32("tpdo_2_timer", 100)
    generator_2.sdo_write_32("tpdo_2_inhibit", 50)
    # generator_2.rpdo("1", 0b01111111, format=8)

    # generator_3 = CanOpenMsgGenerator(5)
    # generator_3.rpdo("1", 0b01111111, format=4)

    generator_4 = CanOpenMsgGenerator(11)
    generator_4.rpdo("1", 0b00001111, format=8)
    generator_4.sdo_write_8("tpdo_1_transtype", 255)

    g_5 = CanOpenMsgGenerator(10)
    g_5.rpdo("1", 0xFFFF, format=4)

    g_6 = CanOpenMsgGenerator(10)
    g_6.sdo_write_16("tpdo_2_inhibit", 0xFFFF)
