# -*- coding:utf-8 -*-

import protocol
from message import CanOpenMsgGenerator

class CanOpenMsgProcessor(CanOpenMsgGenerator):
    __device = None
    def __init__(self, is_print=False) -> None:
        super().__init__(is_print)
    
    def get_bus_status(self, node_id: int, label: str):
        super().sdo_read(node_id, label)