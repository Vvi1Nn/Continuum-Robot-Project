# -*- coding:utf-8 -*-


''' motor.py 步进电机功能函数 v2.6 '''


import time

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from usbcan.function import UsbCan
import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor


class Motor(CanOpenBusProcessor):
    control_mode = protocol.CONTROL_MODE["position_control"]
    acceleration = 1000 # 加速度
    deceleration = 10000 # 减速度
    velocity     = 100 # 动作速度
    position     = 50 # 动作间隔
    inhibit_time = 500 # 禁止时间 微秒

    __motor_list = []
    
    def __init__(self, node_id) -> None:
        super().__init__(node_id)

        Motor.__motor_list.append(self)
        
        self.motor_status = "None" # 电机状态
        
        self.target_position = Motor.position # 位置模式的目标位置
        self.target_speed = 0 # 速度模式的目标速度

        self.current_position = 0 # 当前位置
        self.current_speed = 0 # 当前速度

    ''' 检查电机的总线状态 '''
    def check_bus_status(self) -> bool:
        if super().check_bus_status():
            print("\033[0;32m[Motor {}] bus ready\033[0m".format(self.node_id))
            return True
        print("\033[0;31m[Motor {}] bus unchecked, try again\033[0m".format(self.node_id))
        return False
    
    ''' 获取电机的伺服状态 '''
    def get_motor_status(self, repeat=0, log=True) -> str:
        value = self.sdo_read("status_word", format=4)[0] # 读取状态字数据中的第1个hex即可
        if value != None:
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
    
    ''' 检查伺服状态 '''
    def check_motor_status(self) -> bool:
        self.get_motor_status(log=False) # 更新伺服状态
        if self.motor_status == "switched_on": # 检查伺服状态
            print("\033[0;32m[Motor {}] servo ready\033[0m".format(self.node_id))
            return True
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_ready/stop"]): # 伺服状态不对 人为设置一次
            print("\033[0;33m[Motor {}] try check servo again\033[0m".format(self.node_id))
            return False
        print("\033[0;31m[Motor {}] servo unchecked\033[0m".format(self.node_id)) # 设置失败 检查失败
        return False
    
    ''' 设置所有电机的基本参数 存入类属性 '''
    @classmethod
    def config(cls, mode="position_control", acc=1000, dec=10000, vel=100, pos=50, time=500) -> None:
        print("=============================================================")
        cls.control_mode = protocol.CONTROL_MODE[mode]
        print("\033[0;32m[Motor] control_mode: {}\033[0m".format(cls.control_mode))
        cls.acceleration = acc if (acc >= 1000 and acc <= 10000) else 1000 # 加速度限幅
        print("\033[0;32m[Motor] acceleration: {}\033[0m".format(cls.acceleration))
        cls.deceleration = dec if (dec >= 5000 and dec <= 10000) else 10000 # 减速度限幅
        print("\033[0;32m[Motor] deceleration: {}\033[0m".format(cls.deceleration))
        cls.velocity = vel if (vel >= 50 and vel <= 100) else 100 # 动作速度限幅
        print("\033[0;32m[Motor] run velocity: {}\033[0m".format(cls.velocity))
        cls.position = pos if (pos >= 0 and pos <= 50) else 50 # 动作间隔限幅
        print("\033[0;32m[Motor] position: {}\033[0m".format(cls.position))
        cls.inhibit_time = time if (time >= 100 and time <= 500) else 500 # TPDO禁止时间限幅
        print("\033[0;32m[Motor] inhibit time: {}\033[0m".format(cls.inhibit_time))

    ''' 类属性存放的参数 生效至电机 '''
    def __set_mode(self) -> None:
        if self.sdo_write_32("control_mode", Motor.control_mode):
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
    
    ''' 用RPDO更改电机的控制字 传入控制字对应的标签 '''
    def __set_servo_status(self, label: str) -> bool:
        if self.rpdo("1", protocol.CONTROL_WORD[label], format=8):
            if label == "reset": self.motor_status = "switched_on"
            elif label == "power_off": self.motor_status == "switch_on_disabled"
            elif label == "quick_stop": self.motor_status == "switch_on_disabled"
            elif label == "servo_close": self.motor_status == "ready_to_switch_on"
            elif label == "servo_ready/stop": self.motor_status == "switched_on"
            elif label == "servo_enable/start": self.motor_status == "operation_enabled"
            print("\033[0;32m[Motor {}] set servo status: {}\033[0m".format(self.node_id, label))
            return True
        print("\033[0;31m[Motor {}] set servo status failed\033[0m".format(self.node_id))
        return False

    ''' 用RPDO设置位置模式的动作幅度和速度 '''
    def __set_position_and_velocity(self, pos, vel) -> bool:
        if self.rpdo("2", pos, vel):
            print("\033[0;32m[Motor {}] target position: {} velocity: {}\033[0m".format(self.node_id, pos, vel))
            return True
        print("\033[0;31m[Motor {}] set target position and velocity failed\033[0m".format(self.node_id))
        return False
    
    ''' 用RPDO设置速度模式的速度 '''
    def __set_speed(self, spe):
        if self.rpdo("3", spe):
            print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.node_id, spe))
            return True
        print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.node_id))
        return False

    ''' 电机初始化 '''
    @classmethod
    def init_config(cls) -> None:
        for motor in cls.__motor_list:
            print("=============================================================")
            motor.__set_mode()
            motor.__set_acceleration()
            motor.__set_deceleration()
            motor.__set_velocity()
            motor.__set_position()
            motor.__set_inhibit_time()

    ''' 启动PDO通讯 '''
    @classmethod
    def start_feedback(cls) -> None:
        print("=============================================================")
        for motor in cls.__motor_list:
            if motor.set_bus_status("start_remote_node"):
                if motor.bus_status == "operational":
                    print("\033[0;32m[Motor {}] start pdo\033[0m".format(motor.node_id))
            print("\033[0;31m[Motor {}] start pdo failed\033[0m".format(motor.node_id))
    
    ''' 关闭PDO通讯 '''
    @classmethod
    def stop_feedback(cls) -> bool:
        print("=============================================================")
        for motor in cls.__motor_list:
            if motor.set_bus_status("stop_remote_node"):
                if motor.bus_status == "stopped":
                    print("\033[0;32m[Motor {}] stop pdo\033[0m".format(motor.node_id))
                    return True
            print("\033[0;31m[Motor {}] stop pdo failed\033[0m".format(motor.node_id))
            return False

    ''' 解除抱闸 '''
    @classmethod
    def release_brake(cls):
        print("=============================================================")
        for motor in cls.__motor_list:
            if motor.__set_servo_status("servo_close"):
                print("\033[0;32m[Motor {}] release brake\033[0m".format(motor.node_id))
                continue
            print("\033[0;31m[Motor {}] release brake failed\033[0m".format(motor.node_id))

    ''' 锁住抱闸 '''
    @classmethod
    def lock_brake(cls):
        print("=============================================================")
        for motor in cls.__motor_list:
            if motor.__set_servo_status("servo_ready/stop"):
                print("\033[0;32m[Motor {}] lock brake\033[0m".format(motor.node_id))
                continue
            print("\033[0;31m[Motor {}] lock brake failed\033[0m".format(motor.node_id))

    ''' 急停 '''
    @classmethod
    def quick_stop(cls):
        print("=============================================================")
        for motor in cls.__motor_list:
            if motor.__set_servo_status("quick_stop"):
                print("\033[0;32m[Motor {}] quick stop\033[0m".format(motor.node_id))
                continue
            print("\033[0;31m[Motor {}] quick stop failed\033[0m".format(motor.node_id))

    '''  '''
    def reset(self):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["reset"]):
            self.get_bus_status()
            self.get_motor_status()
            if self.motor_status == "switched_on" and self.bus_status == "pre-operational":
                print("\033[0;32m[Motor {}] reset motor\033[0m".format(self.node_id))
                return True
        print("\033[0;31m[Motor {}] reset motor failed\033[0m".format(self.node_id))

    ''' 单电机运动测试 位置控制模式 相对运行 立即模式 可实现点击一次按钮动作一次 '''
    def action(self):
        print("=============================================================")
        self.__set_servo_status("position_mode_ready")
        self.__set_position_and_velocity(Motor.position, Motor.velocity)
        self.__set_servo_status("position_mode_action")


''' 测试用 '''
if __name__ == "__main__":
    UsbCan.open_device()
    usbcan_0 = UsbCan("0")
    CanOpenBusProcessor.link_device(usbcan_0)
    
    usbcan_0.init_can()
    usbcan_0.start_can()