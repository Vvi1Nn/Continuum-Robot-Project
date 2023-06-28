# -*- coding:utf-8 -*-


''' function.py 步进电机功能函数 v2.8 移除更新函数 给状态控制函数添加延时 保证数据发送准确'''


# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor


class Motor(CanOpenBusProcessor):
    control_mode = protocol.CONTROL_MODE["position_control"]
    acceleration = 100 # 加速度
    deceleration = 100 # 减速度
    velocity     = 100 # 动作速度
    position     = 50 # 动作间隔
    inhibit_time = 10 # 禁止时间 微秒

    motor_dict = {}
    motor_count = 0
    check_count = 0

    debug = False
    
    def __init__(self, node_id, position_range=[-100000,100], speed_range=[-100,100]) -> None:
        super().__init__(node_id)

        Motor.motor_dict[node_id] = self
        Motor.motor_count += 1
        
        self.motor_status = "unknown" # 电机状态
        self.motor_is_checked = False # 状态检查
        
        self.target_position = Motor.position # 位置模式的目标位置
        self.target_velocity = Motor.velocity # 位置模式的运行速度
        self.target_speed = 0 # 速度模式的目标速度

        self.current_position = None # 当前位置
        self.current_speed = None # 当前速度
        
        self.min_position = position_range[0] # 最大位置
        self.max_position = position_range[1] # 最小位置
        self.min_speed = speed_range[0] # 最大速度
        self.max_speed = speed_range[1] # 最小位置

        self.permission = False

    
    
    ''' 检查电机的总线状态 '''
    def check_bus_status(self) -> bool:
        if super().check_bus_status():
            print("\033[0;32m[Motor {}] bus ready\033[0m".format(self.node_id))
            return True
        else:
            print("\033[0;31m[Motor {}] bus unchecked, try again\033[0m".format(self.node_id))
            return False
    
    ''' 获取电机的伺服状态 '''
    def get_motor_status(self, repeat=0, log=True) -> str:
        ret = self.sdo_read("status_word", format=4) # 读取状态字数据中的第1个hex即可
        if ret != None:
            value = ret[0]
            for key in protocol.STATUS_WORD: # 遍历字典关键字
                for r in protocol.STATUS_WORD[key]: # 在每一个关键字对应的列表中 核对数值
                    if value == r:
                        self.motor_status = key # 更新电机的伺服状态
                        if log: print("\033[0;32m[Motor {}] current servo status: {}\033[0m".format(self.node_id, self.motor_status))
                        return self.motor_status
        # 上述操作不成功
        if repeat == 0:
            if log: print("\033[0;31m[Motor {}] get servo status failed\033[0m".format(self.node_id))
            return "error" # 不重复
        if log: print("\033[0;33m[Motor {}] get servo status ...\033[0m".format(self.node_id))
        return self.get_motor_status(repeat=repeat-1) # 重复
    
    ''' 获取电机当前位置与速度 '''
    def get_position_and_velocity(self, repeat=0, log=True) -> None:
        ret = self.sdo_read("position_feedback", format=1)
        if ret != None:
            self.current_position = ret[0]
            if log: print("\033[0;32m[Motor {}] current position: {}\033[0m".format(self.node_id, self.current_position))
            else: pass
        else: pass
        ret = self.sdo_read("speed_feedback", format=1)
        if ret != None:
            self.current_speed = ret[0]
            if log: print("\033[0;32m[Motor {}] current speed: {}\033[0m".format(self.node_id, self.current_speed))
            else: pass
        else: pass
    
    ''' 检查电机 '''
    def check_motor_status(self) -> None:
        if self.bus_is_checked: # 总线检查成功
            self.get_motor_status(log=False) # 更新伺服状态
            if self.motor_status == "switched_on": # 检查伺服状态
                self.motor_is_checked = True
                self.permission = True
                print("\033[0;32m[Motor {}] servo ready\033[0m".format(self.node_id))
                # 顺便获取一下电机的位置和速度
                self.get_position_and_velocity()
            else:
                if self.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_ready/stop"]): # 伺服状态不对 人为设置一次
                    print("\033[0;33m[Motor {}] try check servo again\033[0m".format(self.node_id))
                else: print("\033[0;31m[Motor {}] servo unchecked\033[0m".format(self.node_id)) # 设置失败 检查失败
        else: print("\033[0;31m[Motor {}] check bus first\033[0m".format(self.node_id)) # 需检查总线
    
    
    
    ''' 设置所有电机的基本参数 存入类属性 '''
    @staticmethod
    def config(mode="position_control", acc=1000, dec=10000, vel=100, pos=50, time=500) -> None:
        print("=============================================================")
        Motor.control_mode = protocol.CONTROL_MODE[mode]
        print("\033[0;32m[Motor] control_mode: {}\033[0m".format(Motor.control_mode))
        Motor.acceleration = acc if (acc >= 0 and acc <= 50000) else 1000 # 加速度限幅
        print("\033[0;32m[Motor] acceleration: {}\033[0m".format(Motor.acceleration))
        Motor.deceleration = dec if (dec >= 0 and dec <= 100000) else 10000 # 减速度限幅
        print("\033[0;32m[Motor] deceleration: {}\033[0m".format(Motor.deceleration))
        Motor.velocity = vel if (vel >= 0 and vel <= 200) else 100 # 动作速度限幅
        print("\033[0;32m[Motor] run velocity: {}\033[0m".format(Motor.velocity))
        Motor.position = pos if (pos >= 0 and pos <= 100) else 50 # 动作间隔限幅
        print("\033[0;32m[Motor] position: {}\033[0m".format(Motor.position))
        Motor.inhibit_time = time if (time >= 0 and time <= 500) else 500 # TPDO禁止时间限幅
        print("\033[0;32m[Motor] inhibit time: {}\033[0m".format(Motor.inhibit_time))

    ''' 类属性存放的参数 生效至电机 '''
    def __set_mode(self) -> None:
        if self.sdo_write_8("control_mode", Motor.control_mode):
            print("\033[0;32m[Motor {}] control mode: {}\033[0m".format(self.node_id, Motor.control_mode))
        else: print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.node_id))
    def __set_acceleration(self) -> None:
        if self.sdo_write_32("acceleration", Motor.acceleration):
            print("\033[0;32m[Motor {}] acceleration: {}\033[0m".format(self.node_id, Motor.acceleration))
        else: print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.node_id))
    def __set_deceleration(self) -> None:
        if self.sdo_write_32("deceleration", Motor.acceleration):
            print("\033[0;32m[Motor {}] deceleration: {}\033[0m".format(self.node_id, Motor.deceleration))
        else: print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.node_id))
    def __set_velocity(self) -> None:
        if self.sdo_write_32("velocity", Motor.velocity):
            print("\033[0;32m[Motor {}] velocity: {}\033[0m".format(self.node_id, Motor.velocity))
        else: print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.node_id))
    def __set_position(self) -> None:
        if self.sdo_write_32("target_position", Motor.position):
            print("\033[0;32m[Motor {}] target position: {}\033[0m".format(self.node_id, Motor.position))
        else: print("\033[0;31m[Motor {}] set target position failed\033[0m".format(self.node_id))
    def __set_inhibit_time(self) -> None:
        if self.sdo_write_32("tpdo_2_inhibit", Motor.inhibit_time):
            print("\033[0;32m[Motor {}] inhibit time: {}\033[0m".format(self.node_id, Motor.inhibit_time))
        else: print("\033[0;31m[Motor {}] set inhibit time failed\033[0m".format(self.node_id))
    
    ''' 电机初始化 '''
    @staticmethod
    def init_config() -> None:
        for motor in Motor.motor_dict.values():
            print("=============================================================")
            motor.__set_mode()
            motor.__set_acceleration()
            motor.__set_deceleration()
            motor.__set_velocity()
            motor.__set_position()
            motor.__set_inhibit_time()

    
    
    ''' 用RPDO更改电机的控制字 传入控制字对应的标签 '''
    def set_servo_status(self, label: str) -> bool:
        if self.permission:
            if self.rpdo("1", protocol.CONTROL_WORD[label], format=8):
                if label == "reset": self.motor_status = "switched_on"
                elif label == "power_off": self.motor_status == "switch_on_disabled"
                elif label == "quick_stop": self.motor_status == "switch_on_disabled"
                elif label == "servo_close":self.motor_status == "ready_to_switch_on"
                elif label == "servo_ready/stop": self.motor_status == "switched_on"
                elif label == "servo_enable/start": self.motor_status == "operation_enabled"
                else: pass
                if Motor.debug: print("\033[0;32m[Motor {}] set servo status: {}\033[0m".format(self.node_id, label))
                else: pass
                return True
            if Motor.debug: print("\033[0;31m[Motor {}] set servo status failed\033[0m".format(self.node_id))
            else: pass
            return False
        else:
            print("\033[0;31m[Motor {}] no permission\033[0m".format(self.node_id))
            return False

    ''' 用RPDO设置位置模式的动作幅度和速度 '''
    def set_position_and_velocity(self, pos, vel) -> bool:
        if self.permission:
            self.target_position = pos
            if vel > self.max_speed: vel = self.max_speed
            if vel < self.min_speed: vel = self.min_speed
            self.target_velocity = vel
            if self.rpdo("2", self.target_position, self.target_velocity):
                if Motor.debug: print("\033[0;32m[Motor {}] target position: {} velocity: {}\033[0m".format(self.node_id, self.target_position, self.target_velocity))
                else: pass
                return True
            if Motor.debug: print("\033[0;31m[Motor {}] set target position and velocity failed\033[0m".format(self.node_id))
            else: pass
            return False
        else:
            print("\033[0;31m[Motor {}] no permission\033[0m".format(self.node_id))
            return False
    
    ''' 用RPDO设置速度模式的速度 '''
    def set_speed(self, speed) -> bool:
        if self.permission:
            if speed > self.max_speed: speed = self.max_speed
            if speed < self.min_speed: speed = self.min_speed
            self.target_speed = speed
            if self.rpdo("3", self.target_speed):
                print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.node_id, self.target_speed))
                return True
            print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.node_id))
            return False
        else:
            print("\033[0;31m[Motor {}] no permission\033[0m".format(self.node_id))
            return False

    ''' 判断位置是否超出范围 '''
    def is_in_range(self, ratio=1) -> bool:
        if self.current_position < self.max_position*ratio and self.current_position > self.min_position*ratio: return True
        else:
            print("\033[0;31m[Motor {}] position out of range\033[0m".format(self.node_id))
            return False
    

    
    ''' 启动PDO通讯 '''
    @staticmethod
    def start_feedback() -> None:
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_bus_status("start_remote_node"):
                if Motor.debug: print("\033[0;32m[Motor {}] start pdo\033[0m".format(motor.node_id))
                else: pass
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] start pdo failed\033[0m".format(motor.node_id))
                else: pass
        else: print("\033[0;32m[Motor] ALL PDO START\033[0m")
    
    ''' 关闭PDO通讯 '''
    @staticmethod
    def stop_feedback() -> None:
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_bus_status("enter_pre-operational_state"):
                if Motor.debug: print("\033[0;32m[Motor {}] stop pdo\033[0m".format(motor.node_id))
                else: pass
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] stop pdo failed\033[0m".format(motor.node_id))
                else: pass
        else: print("\033[0;32m[Motor] ALL PDO STOP\033[0m")

    ''' 解除抱闸 '''
    @staticmethod
    def release_brake():
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_servo_status("servo_close"):
                if Motor.debug: print("\033[0;32m[Motor {}] release brake\033[0m".format(motor.node_id))
                else: pass
                time.sleep(0.001)
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] release brake failed\033[0m".format(motor.node_id))
                else: pass
        else: print("\033[0;32m[Motor] ALL RELEASE\033[0m")

    ''' 伺服使能 '''
    @staticmethod
    def enable_servo():
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_servo_status("servo_ready/stop"):
                if Motor.debug: print("\033[0;32m[Motor {}] lock brake\033[0m".format(motor.node_id))
                else: pass
                time.sleep(0.001)
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] lock brake failed\033[0m".format(motor.node_id))
                else: pass
        else: print("\033[0;32m[Motor] ALL ENABLE\033[0m")

    ''' 急停 '''
    @staticmethod
    def quick_stop():
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_servo_status("quick_stop"):
                if Motor.debug: print("\033[0;32m[Motor {}] quick stop\033[0m".format(motor.node_id))
                else: pass
                time.sleep(0.001)
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] quick stop failed\033[0m".format(motor.node_id))
                else: pass
        else: print("\033[0;32m[Motor] ALL STOP\033[0m")
   
    ''' 错误 重置 '''
    @staticmethod
    def reset():
        print("=============================================================")
        for motor in Motor.motor_dict.values():
            if motor.set_servo_status("reset"):
                if Motor.debug: print("\033[0;32m[Motor {}] reset\033[0m".format(motor.node_id))
                else: pass
                time.sleep(0.001)
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] reset failed\033[0m".format(motor.node_id))
                else: pass

    ''' 当程序退出时 对电机进行的复位操作 '''
    @staticmethod
    def homing():
        count = 0
        for motor in Motor.motor_dict.values():
            if motor.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_close"]):
                if Motor.debug: print("\033[0;32m[Motor {}] servo close\033[0m".format(motor.node_id))
                else: pass
                count += 1
            else:
                if Motor.debug: print("\033[0;31m[Motor {}] close servo failed\033[0m".format(motor.node_id))
                else: pass
            time.sleep(0.001)
        if count == len(Motor.motor_dict): return True
        else: return False
        
    


    