# -*- coding:utf-8 -*-


from processor import CanOpenBusProcessor

class Motor(CanOpenBusProcessor):
    def __init__(self, node_id) -> None:
        self.__node_id = node_id

    



if __name__ == "__main__":
    CanOpenBusProcessor.link_device(device=1234)
    print(Motor.device)
    print(Motor.test())
    motor_1 = Motor(1)

    