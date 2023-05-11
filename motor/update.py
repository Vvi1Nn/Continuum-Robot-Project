# -*- coding:utf-8 -*-


''' update.py 更新电机状态 v1.0 '''


from PyQt5.QtCore import QThread, pyqtSignal, QMutex
from motor.function import Motor


class MotorUpdateThread(QThread):
    button_signal_0 = pyqtSignal(int)
    
    def __init__(self, mutex: QMutex) -> None:
        super().__init__()
        self.__is_stop = False
        self.__lock = mutex
    
    def run(self):
        while not self.__is_stop:
            self.__lock.lock()
            Motor.resolve_tpdo_msg()
            self.__lock.unlock()
    
    def stop(self):
        self.__is_stop = True
    