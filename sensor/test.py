import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from usbcan.function import UsbCan


def hex_list_to_int(data_list) -> int:
    data_str = ""
    for i in range(len(data_list)):
        data_bin = bin(data_list[i])[2:] # 首先转换为bin 去除0b
        data_bin = '0' * (8 - len(data_bin)) + data_bin # 头部补齐
        data_str = data_bin + data_str # 拼接
    # 首位是0 正数
    if int(data_str[0]) == 0: return int(data_str, 2)
    # 首位是1 负数
    else: return - ((int(data_str, 2) ^ 0xFFFFFFFF) + 1)


if __name__ == "__main__":
    usbcan_1 = UsbCan.is_show_log(False)("1") # 通道1
    UsbCan.open_device()
    usbcan_1.set_timer("1000K")
    usbcan_1.init_can()
    usbcan_1.start_can()

    count = 0
    init = 0

    while count < 100:
        usbcan_1.send(0x02, [[0x49, 0xAA, 0x0D, 0x0A]], data_len="sensor")
        time.sleep(0.01)
        [num, msg] = usbcan_1.read_buffer(1)
        if num != 0:
            init += hex_list_to_int([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]])
            count += 1
    
    init = int(init / 100)
    
    while True:
        usbcan_1.send(0x02, [[0x49, 0xAA, 0x0D, 0x0A]], data_len="sensor")
        time.sleep(0.01)
        [num, msg] = usbcan_1.read_buffer(1)
        if num!= 0:
            value = hex_list_to_int([msg[0].Data[2], msg[0].Data[3], msg[0].Data[4], msg[0].Data[5]])
            if value < 0:
                print("pull {}".format(value-init))
            else: print("push {}".format(value-init))