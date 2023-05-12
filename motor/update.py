# -*- coding:utf-8 -*-


''' update.py 更新电机状态 v1.1 '''


from PyQt5.QtCore import QThread, pyqtSignal
from motor.function import Motor


class MotorUpdateThread(QThread):
    update_signal = pyqtSignal(int)
    
    def __init__(self) -> None:
        super().__init__()
        self.__is_stop = False
    
    def run(self):
        while not self.__is_stop:
            node_id = Motor.update_motor_status()
            if node_id != 0: self.update_signal.emit(node_id)
    
    def stop(self):
        self.__is_stop = True
    