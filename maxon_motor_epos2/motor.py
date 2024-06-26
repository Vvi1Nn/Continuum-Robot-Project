# -*- coding:utf-8 -*-


''' motor.py for maxon motor epos2 v1.0 '''


import sys, os, time

from PyQt5.QtCore import QThread, pyqtSignal

# 添加模块路径
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from maxon_motor_epos2.processor import CanOpenBusProcessor


class Motor(CanOpenBusProcessor):
    CONTROL_WORD = {"disable_voltage"  : 0x0000,
                    "quick_stop"       : 0x0002,
                    "shut_down"        : 0x0006,
                    "switch_on"        : 0x0007,
                    "disable_operation": 0x0007,
                    "enable_operation" : 0x000F,
                    "halt"             : 0x010F,
                    "fault_reset"      : 0x0080,
    }
    
    STATUS_WORD = {"not_ready_to_switch_on": [0x00, 0x10, 0x20, 0x30, 0x80, 0x90, 0xA0, 0xB0], # 初始化
                   "switch_on_disabled"    : [0x30, 0x40, 0x50, 0x60, 0xC0, 0xD0, 0xE0, 0xF0], # 伺服无故障
                   "ready_to_switch_on"    : [0x21, 0x31, 0xA1, 0xB1],                         # 伺服准备好
                   "switched_on"           : [0x23, 0x33, 0xA3, 0xB3],                         # 伺服等待使能
                   "operation_enable"      : [0x27, 0x37, 0xA7, 0xB7],                         # 伺服运行
                   "quick_stop_active"     : [0x07, 0x17, 0x87, 0x97],                         # 快速停机
                   "fault_reaction_active" : [0x0F, 0x1F, 0x2F, 0x3F, 0x8F, 0x9F, 0xAF, 0xBF], # 故障停机
                   "fault"                 : [0x08, 0x18, 0x28, 0x38, 0x88, 0x98, 0xA8, 0xB8], # 故障
    }
    
    CONTROL_MODE = {"position_control": 1,
                    "speed_control"   : 3,
                    "home_control"    : 6,
    }
    
    MOTION_TYPE = {"trapezoidal": 0,
                   "sinusoidal" : 1,
    }
    
    control_mode = "speed_control"
    motion_profile_type = "trapezoidal"
    profile_acceleration = 10000
    profile_deceleration = 50000
    quick_stop_deceleration = 100000
    tpdo_inhibit_time = 10

    motor_dict = {}
    
    def __init__(self, node_id, position_range=[-100000,100000], speed_range=[-12700,12700]) -> None:
        super().__init__(node_id)

        Motor.motor_dict[node_id] = self

        self.min_position = position_range[0] if position_range != None else None # 最大位置
        self.max_position = position_range[1] if position_range != None else None # 最小位置
        self.min_speed = speed_range[0] if speed_range != None else None # 最大速度
        self.max_speed = speed_range[1] if speed_range != None else None # 最小位置
        
        self.servo_status = "unknown" # 电机状态
        self.servo_is_checked = False # 状态检查

        self.permission = False
        
        self.target_speed = 0 # 速度模式的目标速度

        self.current_position = None # 当前位置
        self.current_speed = None # 当前速度
    
    
    ''' 检查总线状态 '''
    def check_bus_status(self) -> bool:
        if super().check_bus_status():
            print("\033[0;32m[Motor {}] BUS OK\033[0m".format(self.node_id))
            return True
        else:
            print("\033[0;31m[Motor {}] BUS UNCKECKED\033[0m".format(self.node_id))
            return False
    


    ''' 获取伺服状态 SDO '''
    def get_servo_status(self, /, *, times=1, log=False) -> str:
        while times != 0:
            ret = self.sdo_read("status_word", format=4) 

            if ret != None:
                value = ret[0] # 读取状态字数据中的第1个hex即可

                for key in Motor.STATUS_WORD: # 遍历字典关键字
                    for r in Motor.STATUS_WORD[key]: # 在每一个关键字对应的列表中 核对数值
                        if value == r:
                            self.servo_status = key # 更新电机的伺服状态
                            
                            if log: print("\033[0;32m[Motor {}] current servo status: {}\033[0m".format(self.node_id, self.motor_status))
                            else: pass
                            
                            return self.servo_status
            else:
                times -= 1

                if log: print("\033[0;33m[Motor {}] getting servo status ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] get servo status failed\033[0m".format(self.node_id))
            return "error"
    
    ''' 设置伺服状态 SDO PDO 权限 '''
    def set_servo_status(self, label: str, /, *, is_pdo=False, times=1, log=False, check=True, delay=0.5, admin=False) -> bool:
        if self.permission or admin:
            # PDO
            if is_pdo:
                while times != 0:
                    if self.rpdo("1", Motor.CONTROL_WORD[label], format=4):
                        if log: print("\033[0;32m[Motor {}] set servo status: {}\033[0m".format(self.node_id, label))
                        else: pass
                        return True
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Motor {}] setting servo status ...\033[0m".format(self.node_id))
                        else: pass
                else: 
                    if log: print("\033[0;31m[Motor {}] set servo status failed\033[0m".format(self.node_id))
                    else: pass
                    return False
            # SDO
            else:
                while times != 0:
                    if self.sdo_write_16("control_word", Motor.CONTROL_WORD[label], check=check, delay=delay):
                        if log: print("\033[0;32m[Motor {}] set servo: {}\033[0m".format(self.node_id, label))
                        else: pass
                        return True
                else:
                    if log: print("\033[0;31m[Motor {}] set servo status failed\033[0m".format(self.node_id))
                    else: pass
                    return False
        else:
            print("\033[0;31m[Motor {}] no permission\033[0m".format(self.node_id))
            return False

    ''' 检查伺服状态 '''
    def check_servo_status(self, /, *, times=1, log=False) -> bool:
        if self.bus_is_checked: # 总线检查成功
            if self.servo_status == "switch_on_disabled":
                self.servo_is_checked = True
                self.permission = True
                print("\033[0;32m[Motor {}] SERVO OK\033[0m".format(self.node_id))
                return True
            else:
                while times != 0:
                    self.disable_voltage(is_pdo=False, check=True, delay=0.5, admin=True)
                    
                    if self.get_servo_status() == "switch_on_disabled":
                        self.servo_is_checked = True
                        self.permission = True
                        print("\033[0;32m[Motor {}] SERVO OK\033[0m".format(self.node_id))
                        return True
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Motor {}] checking servo status ...\033[0m".format(self.node_id))
                        else: pass
                else:
                    if log: print("\033[0;31m[Motor {}] check servo failed ...\033[0m".format(self.node_id))
                    else: pass
                    return False
        else: print("\033[0;31m[Motor {}] check bus first\033[0m".format(self.node_id)) # 需检查总线
    


    ''' SHUT DOWN '''
    def shut_down(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("shut_down", is_pdo=is_pdo, check=check, delay=delay)
    
    ''' SWITCH ON '''
    def switch_on(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("switch_on", is_pdo=is_pdo, check=check, delay=delay)
    
    ''' DISABLE OPERATION '''
    def disable_operation(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("disable_operation", is_pdo=is_pdo, check=check, delay=delay)

    ''' ENABLE OPERATION '''
    def enable_operation(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("enable_operation", is_pdo=is_pdo, check=check, delay=delay)

    ''' DISABLE VOLTAGE '''
    def disable_voltage(self, /, *, is_pdo=False, check=False, delay=0.5, admin=False) -> bool:
        return self.set_servo_status("disable_voltage", is_pdo=is_pdo, check=check, delay=delay, admin=admin)

    ''' QUICK STOP '''
    def quick_stop(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("quick_stop", is_pdo=is_pdo, check=check, delay=delay)
    
    ''' FAULT RESET '''
    def fault_reset(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("fault_reset", is_pdo=is_pdo, check=check, delay=delay)
    
    ''' HALT '''
    def halt(self, /, *, is_pdo=False, check=False, delay=0.5) -> bool:
        return self.set_servo_status("halt", is_pdo=is_pdo, check=check, delay=delay)

  

    ''' 开启PDO '''
    def start_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("start_remote_node", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Motor {}] PDO START\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] starting pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] start pdo failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 关闭PDO '''
    def stop_pdo(self, /, *, times=1, check=True, delay=0.5, log=False) -> bool:
        while times != 0:
            if self.set_bus_status("enter_pre-operational_state", check=check, delay=delay, log=log):
                if log: print("\033[0;32m[Motor {}] PDO STOP\033[0m".format(self.node_id))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] stopping pdo ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] stop pdo failed\033[0m".format(self.node_id))
            else: pass
            return False



    ''' 控制模式 '''
    def set_control_mode(self, mode=control_mode, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_8("control_mode", Motor.CONTROL_MODE[mode], check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] control mode: {}\033[0m".format(self.node_id, mode))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting control mode ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.node_id))
            else: pass
            return False
    
    ''' TPDO 禁止时间 '''
    def set_inhibit_time(self, channel, value=tpdo_inhibit_time, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_8("tpdo_{}_inhibit".format(channel), value, check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] tpdo {} inhibit time: {}\033[0m".format(self.node_id, channel, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting tpdo {} inhibit time ...\033[0m".format(self.node_id, channel))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set tpdo {} inhibit time failed\033[0m".format(self.node_id, channel))
            else: pass
            return False
    
    ''' 加速度 '''
    def set_profile_acceleration(self, value=profile_acceleration, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_32("acceleration", value, check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] acceleration: {}\033[0m".format(self.node_id, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting acceleration ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 减速度 '''
    def set_profile_deceleration(self, value=profile_deceleration, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_32("deceleration", value, check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] deceleration: {}\033[0m".format(self.node_id, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting deceleration ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 急停 减速度 '''
    def set_quick_stop_deceleration(self, value=quick_stop_deceleration, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_32("quick_stop_deceleration", value, check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] quick stop deceleration: {}\033[0m".format(self.node_id, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting quick stop deceleration ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set quick stop deceleration failed\033[0m".format(self.node_id))
            else: pass
            return False

    ''' 曲线 '''
    def set_motion_profile_type(self, value=motion_profile_type, /, *, times=1, log=True, check=True, delay=0.5) -> bool:
        while times != 0:
            if self.sdo_write_32("motion_profile_type", Motor.MOTION_TYPE[value], check=check, delay=delay):
                if log: print("\033[0;32m[Motor {}] motion profile type: {}\033[0m".format(self.node_id, value))
                else: pass
                return True
            else:
                times -= 1
                if log: print("\033[0;33m[Motor {}] setting motion profile type ...\033[0m".format(self.node_id))
                else: pass
        else:
            if log: print("\033[0;31m[Motor {}] set motion profile type failed\033[0m".format(self.node_id))
            else: pass
            return False



    ''' 速度 '''
    def set_speed(self, speed: int, /, *, is_pdo=False, times=1, log=True, check=True, delay=0.5) -> bool:
        if self.permission:
            if speed > self.max_speed: speed = self.max_speed
            elif speed < self.min_speed: speed = self.min_speed
            else: pass
            self.target_speed = speed
            
            # PDO
            if is_pdo:
                while times != 0:
                    if self.rpdo("3", self.target_speed):
                        if log: print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.node_id, self.target_speed))
                        else: pass
                        return True
                    else:
                        times -= 1
                        if log: print("\033[0;33m[Motor {}] setting target speed ...\033[0m".format(self.node_id))
                        else: pass
                else: 
                    if log: print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.node_id))
                    else: pass
                    return False
            # SDO
            else:
                while times != 0:
                    if self.sdo_write_32("target_speed", self.target_speed, check=check, delay=delay):
                        if log: print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.node_id, self.target_speed))
                        else: pass
                        return True
                else:
                    if log: print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.node_id))
                    else: pass
                    return False
        else:
            print("\033[0;31m[Motor {}] no permission\033[0m".format(self.node_id))
            return False


    ''' 是否超出范围 '''
    def is_in_range(self) -> bool:
        if self.max_position != None and self.min_position != None:
            if self.current_position < self.max_position and self.current_position > self.min_position: return True
            else:
                print("\033[0;31m[Motor {}] position out of range\033[0m".format(self.node_id))
                return False
        else: return True


''' 电机初始化 '''
class MotorInitThread(QThread):
    running_signal = pyqtSignal(bool)
    check_signal = pyqtSignal(int)
    finish_signal = pyqtSignal()
    
    def __init__(self) -> None:
        super().__init__()
        self.__check_count = 0
    
    def run(self):
        self.running_signal.emit(True)

        for node_id in Motor.motor_dict:
            print("=============================================================")
            times = 3
            while times != 0:
                if Motor.motor_dict[node_id].check_bus_status():
                    if Motor.motor_dict[node_id].start_pdo(log=True, check=False):
                        if Motor.motor_dict[node_id].check_servo_status():
                            success_1 = Motor.motor_dict[node_id].set_control_mode()
                            success_2 = Motor.motor_dict[node_id].set_inhibit_time("2")
                            success_3 = Motor.motor_dict[node_id].set_profile_acceleration()
                            success_4 = Motor.motor_dict[node_id].set_profile_deceleration()
                            success_5 = Motor.motor_dict[node_id].set_quick_stop_deceleration()
                            success_6 = Motor.motor_dict[node_id].set_motion_profile_type()
                            if success_1 and success_2 and success_3 and success_4 and success_5 and success_6:
                                self.check_signal.emit(node_id)
                                self.__check_count += 1
                                break
                            else: pass
                        else: pass
                    else: pass
                else: pass

                times -= 1
        if self.__check_count == len(Motor.motor_dict): self.finish_signal.emit()
        else: self.running_signal.emit(False)