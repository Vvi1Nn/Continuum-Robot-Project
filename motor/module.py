# -*- coding:utf-8 -*-

def split_index(index) -> list:
    
    hex_str = hex(index)[2:].upper()
    
    high_str = hex_str[0:2]
    low_str = hex_str[2:]
    
    high_int = int(high_str, 16)
    low_int = int(low_str, 16)
    
    return [low_int, high_int]

def int2hex(value_int) -> list:
    
    if value_int >= 0:
        value_str = hex(value_int)[2:].upper()
        if len(value_str) < 8:
            value_str = '0'*(8-len(value_str)) + value_str
    else:
        value_str = hex(int(bin(- value_int ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
    
    value_list = [0x0] * 4
    value_list[0] = int(value_str[6:8], 16)
    value_list[1] = int(value_str[4:6], 16)
    value_list[2] = int(value_str[2:4], 16)
    value_list[3] = int(value_str[0:2], 16)
    
    return value_list

def match_index(index_low, index_high, subindex) -> list:
    
    index_low_str = hex(index_low)[2:].upper()
    index_high_str = hex(index_high)[2:].upper()
    
    index_str = index_high_str + index_low_str
    
    index = int(index_str, 16)

    return [index, subindex]

def hex2int(data_list) -> int:
    
    data_str = ""
    
    for i in range(len(data_list)):
        data_bin = bin(data_list[i])[2:]
        if len(data_bin) < 8:
            data_bin = '0' * (8 - len(data_bin)) + data_bin
        data_str = data_bin + data_str

    if int(data_str[0]) == 0:
        return int(data_str, 2)
    else:
        return - ((int(data_str, 2) ^ 0xFFFFFFFF) + 1)
