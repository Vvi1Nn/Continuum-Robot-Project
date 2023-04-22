# -*- coding:utf-8 -*-


import protocol


class CanOpenMsgGenerator():
    def __init__(self, is_print=False) -> None:
        self.__is_print = is_print
    
    @staticmethod
    def __split_index(index) -> list:
        hex_str = hex(index)[2:].upper()
        
        high_str = hex_str[0:2]
        low_str = hex_str[2:]
        
        high_int = int(high_str, 16)
        low_int = int(low_str, 16)
        
        return [low_int, high_int]
    
    @staticmethod
    def __int_to_hex_list(value_int) -> list:
        if value_int >= 0:
            value_str = hex(value_int)[2:].upper()
            if len(value_str) < 8:
                value_str = '0' * (8 - len(value_str)) + value_str
        else:
            value_str = hex(int(bin(- value_int ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
        
        value_list = [0x0] * 4
        value_list[0] = int(value_str[6:8], 16)
        value_list[1] = int(value_str[4:6], 16)
        value_list[2] = int(value_str[2:4], 16)
        value_list[3] = int(value_str[0:2], 16)
        
        return value_list
    
    def sdo_read(self, node_id: int, label: str) -> list:
        
        index = protocol.OD[label][0]
        subindex = protocol.OD[label][1]
        
        cob_id = protocol.CAN_ID["SDO_R"] + node_id
        
        data = [0x00] * 8
        data[0] = protocol.CMD_T["read"]
        data[1] = self.__split_index(index)[0]
        data[2] = self.__split_index(index)[1]
        data[3] = subindex
        
        if self.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_read] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")

        return [cob_id, data]

    def sdo_write_32(self, node_id: int, label: str, value: int) -> list:
        
        index = protocol.OD[label][0]
        subindex = protocol.OD[label][1]
        
        cob_id = protocol.CAN_ID["SDO_R"] + node_id
        
        data = [0x00] * 8
        data[0] = protocol.CMD_T["write_32"]
        data[1] = self.__split_index(index)[0]
        data[2] = self.__split_index(index)[1]
        data[3] = subindex
        
        value = self.__int_to_hex_list(value)
        for i in range(4):
            data[4+i] = value[i]
        
        if self.__is_print:
            hex_data = ["00"] * 8
            print("[sdo_write_32] {}".format(label))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        
        return [cob_id, data]

    def rpdo(self, num: str, node_id: int, value_low: int, value_high: int) -> dict:
        
        cob_id = protocol.CAN_ID["RPDO_" + num] + node_id

        data = [0x00] * 8
        value_low = self.__int_to_hex_list(value_low)
        for i in range(4):
            data[i] = value_low[i]
        value_high = self.__int_to_hex_list(value_high)
        for i in range(4):
            data[4+i] = value_high[i]

        if self.__is_print:
            hex_data = ["00"] * 8
            print("[rpdo_{}]".format(num))
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        
        return [cob_id, data]

    def nmt_change_status(self, node_id: int, label: str) -> list:
        
        cob_id = protocol.CAN_ID["NMT_C"]
        data = [protocol.CMD_NMT[label], node_id, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]

        if self.__is_print:
            hex_data = ["00"] * 8
            print("[nmt_control]")
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        
        return [cob_id, data]

    def nmt_get_status(self, node_id: int) -> list:
        
        cob_id = protocol.CAN_ID["NMT_S"] + node_id
        data = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

        if self.__is_print:
            hex_data = ["00"] * 8
            print("[nmt_status]")
            print("COB-ID: {}".format(hex(cob_id)[2:].upper()))
            print("Data-List: ", end = "")
            for i in range(len(data)):
                hex_data[i] = hex(data[i])[2:].upper()
                print("{} ".format(hex_data[i]), end = "")
            print("\n")
        
        return [cob_id, data]


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
    generator = CanOpenMsgGenerator(True)
    generator.sdo_write_32(1, "control_mode", 100)
