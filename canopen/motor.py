# -*- coding:utf-8 -*-


import time

# 添加模块路径
import sys, os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from usbcan.function import UsbCan
import canopen.protocol as protocol
from canopen.processor import CanOpenBusProcessor


class Motor(CanOpenBusProcessor):
    control_mode = protocol.CONTROL_MODE["position_control"]
    acceleration = 1000
    deceleration = 10000
    velocity     = 100
    position     = 0
    speed        = 0
    
    def __init__(self, node_id) -> None:
        super().__init__(node_id)
        
        self.motor_status = None
        
        self.position = Motor.position
        self.speed = Motor.speed

    def check_bus_status(self) -> bool:
        if super().check_bus_status():
            print("\033[0;32m[Motor {}] bus ready\033[0m".format(self.node_id))
            return True
    
    def get_motor_status(self) -> str:
        value = self.sdo_read("status_word", format=4)[0]
        for key in protocol.STATUS_WORD:
            for r in protocol.STATUS_WORD[key]:
                if value == r:
                    self.motor_status = key
                    print("\033[0;32m[Motor {}] motor status: {}\033[0m".format(self.node_id, self.motor_status))
                    return self.motor_status
    
    def check_motor_status(self) -> bool:
        self.get_motor_status()
        if self.motor_status == "switched_on":
            print("\033[0;32m[Motor {}] ready\033[0m".format(self.node_id))
            return True
        self.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_ready/stop"])
    
    @classmethod
    def config(cls, mode="position_control", acc=1000, dec=10000, vel=100, pos=0, spe=0) -> None:
        cls.control_mode = protocol.CONTROL_MODE[mode]
        print("\033[0;32m[Motor] control_mode: {}\033[0m".format(cls.control_mode))
        cls.acceleration = acc if (acc >= 1000 and acc <= 10000) else 1000
        print("\033[0;32m[Motor] acceleration: {}\033[0m".format(cls.acceleration))
        cls.deceleration = dec if (dec >= 5000 and dec <= 10000) else 10000
        print("\033[0;32m[Motor] deceleration: {}\033[0m".format(cls.deceleration))
        cls.velocity = vel if (vel >= 50 and vel <= 100) else 100
        print("\033[0;32m[Motor] run velocity: {}\033[0m".format(cls.velocity))
        cls.position = pos if pos == 0 else 0
        print("\033[0;32m[Motor] target position: {}\033[0m".format(cls.position))
        cls.speed = spe if spe == 0 else 0
        print("\033[0;32m[Motor] target speed: {}\n\033[0m".format(cls.speed))

    def __set_mode(self) -> None:
        if self.sdo_write_32("control_mode", Motor.control_mode):
            print("\033[0;32m[Motor {}] control mode: {}\033[0m".format(self.node_id, Motor.control_mode))
        else: print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.node_id))
    def __set_acceleration(self):
        if self.sdo_write_32("acceleration", Motor.acceleration):
            print("\033[0;32m[Motor {}] acceleration: {}\033[0m".format(self.node_id, Motor.acceleration))
        else: print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.node_id))
    def __set_deceleration(self):
        if self.sdo_write_32("deceleration", Motor.acceleration):
            print("\033[0;32m[Motor {}] deceleration: {}\033[0m".format(self.node_id, Motor.deceleration))
        else: print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.node_id))
    def __set_velocity(self):
        if self.sdo_write_32("velocity", Motor.velocity):
            print("\033[0;32m[Motor {}] velocity: {}\033[0m".format(self.node_id, Motor.velocity))
        else: print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.node_id))
    def __set_position(self):
        if self.sdo_write_32("target_position", Motor.position):
            print("\033[0;32m[Motor {}] target position: {}\033[0m".format(self.node_id, Motor.position))
        else: print("\033[0;31m[Motor {}] set target position failed\033[0m".format(self.node_id))
    def __set_speed(self):
        if self.sdo_write_32("target_speed", Motor.speed):
            print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.node_id, Motor.speed))
        else: print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.node_id))
    def init_config(self):
        self.__set_mode()
        self.__set_acceleration()
        self.__set_deceleration()
        self.__set_velocity()
        self.__set_position()
        self.__set_speed()

    def set_position(self, value):
        if self.sdo_write_32("target_position", value):
            self.position = value
            print("\033[0;32m[Motor {}] target_position: {}\033[0m".format(self.node_id, value))
        else: print("\033[0;31m[Motor {}] set target_position failed\033[0m".format(self.node_id))
    
    def set_speed(self, value):
        if self.sdo_write_32("target_speed", value):
            self.speed = value
            print("\033[0;32m[Motor {}] target_speed: {}\033[0m".format(self.node_id, value))
        else: print("\033[0;31m[Motor {}] set target_speed failed\033[0m".format(self.node_id))

    # 待测试，很可能会出问题
    def __switch_motor_status(self, label: str):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD[label]):
            self.get_bus_status()
            self.get_motor_status()
            if label == "reset" and self.motor_status == "switched_on" and self.bus_status == "pre-operational":
                print("\033[0;31m[Motor {}] action: reset\033[0m".format(self.node_id))
                return True
            elif label == "power_off" and self.motor_status == "switch_on_disabled":
                print("\033[0;31m[Motor {}] action: power_off\033[0m".format(self.node_id))
                return True
            elif label == "quick_stop" and self.motor_status == "switch_on_disabled":
                print("\033[0;31m[Motor {}] action: quick_stop\033[0m".format(self.node_id))
                return True
            elif label == "servo_close" and self.motor_status == "ready_to_switch_on": return True
            elif label == "servo_ready/stop" and self.motor_status == "switched_on": return True
            elif label == "servo_enable/start" and self.motor_status == "operation_enabled": return True
            else:
                print("\033[0;31m[Motor {}] set motor status failed\033[0m".format(self.node_id))
                return False

    def __start_feedback(self, timer):
        self.set_bus_status("start_remote_node")
        if self.bus_status == "operational":
            if self.sdo_write_32("tpdo_2_timer", timer): # 设置定时器时间间隔
                print("\033[0;32m[Motor {}] start feedback, duration is {}ms\033[0m".format(self.node_id, timer))
                return True
        print("\033[0;31m[Motor {}] start feedback failed\033[0m".format(self.node_id))

    def __stop_feedback(self):
        self.set_bus_status("stop_remote_node")
        if self.bus_status == "stopped":
            if self.sdo_write_32("tpdo_2_timer", 0): # 定时器归0
                print("\033[0;32m[Motor {}] stop feedback\033[0m".format(self.node_id))
                return True
        else: print("\033[0;31m[Motor {}] stop feedback failed\033[0m".format(self.node_id))

    def release_brake(self):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_close"]):
            self.get_motor_status()
            if self.motor_status == "ready_to_switch_on":
                print("\033[0;32m[Motor {}] release brake\033[0m".format(self.node_id))
                return True
        print("\033[0;31m[Motor {}] release brake failed\033[0m".format(self.node_id))

    def lock_brake(self):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["servo_ready/stop"]):
            self.get_motor_status()
            if self.motor_status == "switched_on":
                print("\033[0;32m[Motor {}] lock brake\033[0m".format(self.node_id))
                return True
        print("\033[0;31m[Motor {}] lock brake failed\033[0m".format(self.node_id))

    def run(self):
        pass

    def quick_stop(self):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["quick_stop"]):
            self.get_motor_status()
            if self.motor_status == "switch_on_disabled":
                print("\033[0;32m[Motor {}] quick stop\033[0m".format(self.node_id))
                return True
        print("\033[0;31m[Motor {}] quick stop failed\033[0m".format(self.node_id))

    def reset(self):
        if self.sdo_write_32("control_word", protocol.CONTROL_WORD["reset"]):
            self.get_bus_status()
            self.get_motor_status()
            if self.motor_status == "switched_on" and self.bus_status == "pre-operational":
                print("\033[0;32m[Motor {}] reset motor\033[0m".format(self.node_id))
                return True
        print("\033[0;31m[Motor {}] reset motor failed\033[0m".format(self.node_id))

if __name__ == "__main__":
    UsbCan.open_device()
    usbcan_0 = UsbCan("0")
    CanOpenBusProcessor.link_device(usbcan_0)
    
    usbcan_0.init_can()
    usbcan_0.start_can()