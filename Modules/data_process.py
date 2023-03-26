# -*- coding:utf-8 -*-

class MotorProcessor:
    def __init__(self) -> None:
        pass
    def SplitIndex(self, Index):
        hex_str = hex(Index)[2:].upper()
        high_str = hex_str[0:2]
        low_str = hex_str[2:]
        high_int = int(high_str, 16)
        low_int = int(low_str, 16)
        return {"high": high_int, "low": low_int}

class SensorProcessor:
    def __init__(self) -> None:
        pass
