# -*- coding:utf-8 -*-

print("测试")

def __int2hex(value_int):
    
    if type(value_int) != int or value_int > 0x7FFFFFFF or value_int < -0x80000000:
        print("[int2hex] value 错误!!!")
        return False
    if value_int >= 0:
        value_str = hex(value_int & 0xFFFFFFFF)[2:].upper()
        if len(value_str) < 8:
            value_str = '0'*(8-len(value_str)) + value_str
    else:
        value_str = hex(int(bin(- value_int ^ 0xFFFFFFFF), 2) + 1)[2:].upper()
    # value_str = hex(value_int)[2:].upper()
    # if len(value_str) < 8:
    #     value_str = '0'*(8-len(value_str)) + value_str
    value_list = [0x0] * 4
    value_list[0] = value_str[6:8]
    value_list[1] = value_str[4:6]
    value_list[2] = value_str[2:4]
    value_list[3] = value_str[0:2]
    return value_str

print(__int2hex(10000))