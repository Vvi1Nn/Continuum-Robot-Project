# -*- coding:utf-8 -*-


''' threads.py 在GUI中使用的线程 v1.0 '''


from PyQt5.QtCore import QThread, pyqtSignal
from motor.function import Motor


class MotorUpdateThread(QThread):
    update_signal = pyqtSignal(int)
    
    def __init__(self, slot_function) -> None:
        super().__init__()
        self.__is_stop = False
        self.update_signal.connect(slot_function)
    
    def run(self):
        while not self.__is_stop:
            node_id = Motor.update_motor_status()
            if node_id != 0: self.update_signal.emit(node_id)
    
    def stop(self):
        self.__is_stop = True


class JointControlThread(QThread):
    def __init__(self, motor, is_forward: bool, position: int, velocity: int) -> None:
        super().__init__()
        self.__is_stop = False
        self.__motor = motor
        self.__is_forward = is_forward
        self.__position = position
        self.__velocity = velocity
    
    def run(self):
        while not self.__is_stop:
            if self.__motor.is_in_range():
                self.__motor.set_servo_status("position_mode_ready")
                if self.__is_forward:
                    self.__motor.set_position_and_velocity(self.__position, self.__velocity)
                else:
                    self.__motor.set_position_and_velocity(-self.__position, self.__velocity)
                self.__motor.set_servo_status("position_mode_action")
            else: pass
    
    def stop(self):
        self.__is_stop = True


class InitMotorThread(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.init_config() # 将所有参数生效给所有电机
        self.running_signal.emit(False)
    

class CheckMotorThread(QThread):
    running_signal = pyqtSignal(bool)
    check_signal = pyqtSignal(int)
    finish_signal = pyqtSignal()
    
    def __init__(self) -> None:
        super().__init__()
        self.__check_count = 0
    
    def run(self):
        self.running_signal.emit(True)
        for node_id in Motor.motor_dict:
            if not Motor.motor_dict[node_id].motor_is_checked:
                Motor.motor_dict[node_id].check_bus_status()
                Motor.motor_dict[node_id].check_motor_status()
            if Motor.motor_dict[node_id].motor_is_checked:
                self.check_signal.emit(node_id)
                self.__check_count += 1
        if self.__check_count == 9:
            self.finish_signal.emit()
            return
        self.running_signal.emit(False)



class StartPDO(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.start_feedback()
        self.running_signal.emit(False)


class StopPDO(QThread):
    running_signal = pyqtSignal(bool)
    
    def __init__(self) -> None:
        super().__init__()
    
    def run(self):
        self.running_signal.emit(True)
        Motor.stop_feedback()
        self.running_signal.emit(False)
