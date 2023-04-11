# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import time

import motor.protocol as pro
import motor.msg_generation as gen
import motor.module as module
import usbcan.param as param

class Motor:
    
    __control_mode = pro.CONTROL_MODE["position_control"]
    __acceleration = 1000
    __deceleration = 10000
    __velocity     = 100

    def __init__(self, node_id) -> None:
        
        self.id = node_id
        
        self.bus_status = None
        self.motor_status = None
        
        self.position = None
        self.speed = None
        
        self.__motor_init()
    
    def __str__(self) -> str:
        return "[Motor {}] {} acc:{} dec:{} vel:{}".format(self.id, Motor.__control_mode, Motor.__acceleration, Motor.__deceleration, Motor.__velocity)
    
    @classmethod
    def config(cls, device,
               control_mode = pro.CONTROL_MODE["position_control"],
               acceleration = 1000,
               deceleration = 10000,
               velocity     = 100,
               ) -> None:
        
        cls.__device = device
        print("\033[0;32m[Motor] info: {}\033[0m".format(cls.__device))

        cls.__control_mode = control_mode
        print("\033[0;32m[Motor] control_mode: {}\033[0m".format(cls.__control_mode))
        
        if acceleration < 1000 or acceleration > 10000:
            acceleration = 1000
        cls.__acceleration = acceleration
        print("\033[0;32m[Motor] acceleration: {}\033[0m".format(cls.__acceleration))
        
        if deceleration < 5000 or deceleration > 10000:
            deceleration = 10000
        cls.__deceleration = deceleration
        print("\033[0;32m[Motor] deceleration: {}\033[0m".format(cls.__deceleration))
        
        if velocity < 50 or velocity > 100:
            velocity = 75
        cls.__velocity = velocity
        print("\033[0;32m[Motor] velocity: {}\033[0m".format(cls.__velocity))
        
        print("===============================================================================")

    def __motor_init(self) -> None:
        
        self.__update_bus_status(False)
        if self.bus_status == "pre-operational":
            pass
        else:
            print("\033[0;33m[Motor {}] bus status {} wrong, trying set {} ...\033[0m".format(self.id, self.bus_status, "pre-operational"))
            [cob_id, data] = gen.nmt_change_status(self.id, "enter_pre-operational_state")
            while not Motor.__device.send(cob_id, [data]):
                time.sleep(0.05)
            time.sleep(0.05)
            return self.__motor_init()
        
        self.__update_motor_status(False)
        if self.motor_status == "switched_on":
            self.__set_mode(20)
            self.__set_acc(20)
            self.__set_dec(20)
            self.__set_vel(20)
            print("===============================================================================")
        else:
            print("\033[0;33m[Motor {}] motor status {} wrong, trying set {} ...\033[0m".format(self.id, self.motor_status, "switched_on"))
            [cob_id, data] = gen.sdo_write_32(self.id, "control_word", pro.CONTROL_WORD["servo_ready/stop"])
            while True:
                send_success = Motor.__device.send(cob_id, [data])
                if send_success: break
                time.sleep(0.05)
            return self.__motor_init()
        
    def __set_mode(self, wait = 100) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "control_mode", Motor.__control_mode)
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data]):
            print("\033[0;33m[Motor {}] trying send control mode setting msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["control_mode"]:
                print("\033[0;32m[Motor {}] control mode: {}\033[0m".format(self.id, Motor.__control_mode))
                return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying set control mode again ...\033[0m".format(self.id))
                self.__set_mode(wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying set control mode again ...\033[0m".format(self.id))
            self.__set_mode(wait - 1)

    def __set_acc(self, wait = 100) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "acceleration", Motor.__acceleration)
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data]):
            print("\033[0;33m[Motor {}] trying send acceleration setting msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["acceleration"]:
                print("\033[0;32m[Motor {}] acceleration: {}\033[0m".format(self.id, Motor.__acceleration))
                return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying set acceleration again ...\033[0m".format(self.id))
                self.__set_acc(wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying set acceleration again ...\033[0m".format(self.id))
            self.__set_acc(wait - 1)

    def __set_dec(self, wait = 100) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "deceleration", Motor.__deceleration)
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data]):
            print("\033[0;33m[Motor {}] trying send deceleration setting msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["deceleration"]:
                print("\033[0;32m[Motor {}] deceleration: {}\033[0m".format(self.id, Motor.__deceleration))
                return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying set deceleration again ...\033[0m".format(self.id))
                self.__set_acc(wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying set deceleration again ...\033[0m".format(self.id))
            self.__set_acc(wait - 1)

    def __set_vel(self, wait = 100) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "velocity", Motor.__velocity)
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data]):
            print("\033[0;33m[Motor {}] trying send velocity setting msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["velocity"]:
                print("\033[0;32m[Motor {}] velocity: {}\033[0m".format(self.id, Motor.__velocity))
                return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying set velocity again ...\033[0m".format(self.id))
                self.__set_acc(wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying set velocity again ...\033[0m".format(self.id))
            self.__set_acc(wait - 1)
    
    def __update_bus_status(self, log = False, wait = 10) -> bool:
        
        [cob_id, data] = gen.nmt_get_status(self.id)
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data], remote_flag = param.REMOTE_FLAG["remote"]):
            print("\033[0;33m[Motor {}] trying send bus status updating msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["NMT_S"]:
                for k in pro.NMT_STATUS:
                    if msg[0].Data[0] & 0b01111111 == pro.NMT_STATUS[k]:
                        self.bus_status = k
                        if log:  print("\033[0;32m[Motor {}] bus status: {}\033[0m".format(self.id, self.bus_status))
                        return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] update bus status failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying update bus status again ...\033[0m".format(self.id))
                return self.__update_bus_status(wait = wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] update bus status failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying update bus status again ...\033[0m".format(self.id))
            return self.__update_bus_status(wait = wait - 1)

    def __update_motor_status(self, log = False, wait = 10) -> bool:
        
        [cob_id, data] = gen.sdo_read(self.id, "status_word")
        Motor.__device.clear_buffer()
        while not Motor.__device.send(cob_id, [data]):
            print("\033[0;33m[Motor {}] trying send motor status updating msg ...\033[0m".format(self.id))
            time.sleep(0.05)
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and (msg[0].Data[0] == pro.CMD_R["read_16"] or msg[0].Data[0] == pro.CMD_R["read_32"] or msg[0].Data[0] == pro.CMD_R["read_8"]) and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["status_word"]:
                value = module.hex2int([msg[0].Data[4]])
                for key in pro.STATUS_WORD:
                    for r in pro.STATUS_WORD[key]:
                        if value == r:
                            self.motor_status = key
                            if log: print("\033[0;32m[Motor {}] motor status: {}\033[0m".format(self.id, self.motor_status))
                            return True
            else:
                if wait == 0:
                    print("\033[0;31m[Motor {}] update motor status failed\033[0m".format(self.id))
                    return False
                print("\033[0;33m[Motor {}] trying update motor status again ...\033[0m".format(self.id))
                return self.__update_motor_status(wait = wait - 1)
        else:
            if wait == 0:
                    print("\033[0;31m[Motor {}] update motor status failed\033[0m".format(self.id))
                    return False
            print("\033[0;33m[Motor {}] trying update motor status again ...\033[0m".format(self.id))
            return self.__update_motor_status(wait = wait - 1)

    def update_status(self, log = True, wait = 10) -> bool:
        return self.__update_bus_status(log, wait) and self.__update_motor_status(log, wait)

    def set_position(self, position) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "target_position", position)
        Motor.__device.clear_buffer()
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["target_position"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False
        if send_success and reply_success:
            self.position = position
            print("\033[0;32m[Motor {}] target position: {}\033[0m".format(self.id, position))
            return True
        else:
            print("\033[0;31m[Motor {}] set target position failed\033[0m".format(self.id))
            return False
    
    def set_speed(self, speed) -> bool:
        
        [cob_id, data] = gen.sdo_write_32(self.id, "target_speed", speed)
        Motor.__device.clear_buffer()
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["target_speed"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False
        if send_success and reply_success:
            self.speed = speed
            print("\033[0;32m[Motor {}] target speed: {}\033[0m".format(self.id, speed))
            return True
        else:
            print("\033[0;31m[Motor {}] set target speed failed\033[0m".format(self.id))
            return False
  
    def start_feedback(self) -> bool:
        
        [cob_id, data] = gen.nmt_change_status(self.id, "start_remote_node")
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        self.__update_bus_status()
        if send_success and self.bus_status == "operational":
            print("\033[0;32m[Motor {}] start feedback\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] start feedback failed\033[0m".format(self.id))
            return False

    def set_timer(self, duration) -> bool:
        
        [cob_id, data] = gen.sdo_write_32(self.id, "tpdo_2_timer", duration) # 设置定时器时间间隔
        Motor.__device.clear_buffer()
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["tpdo_2_timer"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False
        if send_success and reply_success:
            print("\033[0;32m[Motor {}] timer step: {}\033[0m".format(self.id, duration))
            return True
        else:
            print("\033[0;31m[Motor {}] set timer step failed\033[0m".format(self.id))
            return False

    def stop_feedback(self) -> bool:
        
        [cob_id, data] = gen.nmt_change_status(self.id, "stop_remote_node")
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        self.__update_bus_status()
        if send_success and self.bus_status == "stopped":
            print("\033[0;32m[Motor {}] stop feedback\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] stop feedback failed\033[0m".format(self.id))
            return False

    def set_motor_status(self, flag):
        pass
    
    def execute(self, is_relative = True, is_immediate = True):
        
        if is_relative:
            
            [cob_id, enable_msg] = gen.sdo_write_32(self.id, "control_word", 0x6F) # 相对使能
            [cob_id, launch_msg] = gen.sdo_write_32(self.id, "control_word", 0x7F) # 启动
            data = [enable_msg, launch_msg]

            Motor.__device.clear_buffer()

            send_success = Motor.__device.send(cob_id, data)
            time.sleep(0.05)

            if send_success:
                print("[Execute {}] 成功".format(self.id))
                return True
            else:
                print("[Execute {}] 失败".format(self.id))
                return False
        else:
            
            enable = gen.sdo_write_32(self.id, "control_word", 0x2F) # 绝对使能
            launch = gen.sdo_write_32(self.id, "control_word", 0x3F) # 启动
            
            cob_id = enable["id"]
            
            data = []
            data.append(enable["data"])
            data.append(launch["data"])

            success = self.device.send(cob_id, data)
            if success:
                print("[Execute {}] 成功".format(self.id))
                return True
            else:
                print("[Execute {}] 失败".format(self.id))
                return False
    
    def off_break(self):
        
        [cob_id, data] = gen.sdo_write_32(self.id, "control_word", pro.CONTROL_WORD["servo_close"])

        send_success = Motor.__device.send(cob_id, data)
        self.__update_motor_status()
        if send_success and self.motor_status == "ready_to_switch_on":
            print("\033[0;32m[Motor {}] off break\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] off break failed\033[0m".format(self.id))
            return False
    
    def quick_stop(self):
        
        [cob_id, data] = gen.sdo_write_32(self.id, "control_word", pro.CONTROL_WORD["quick_stop"])

        send_success = Motor.__device.send(cob_id, data)
        self.__update_motor_status()
        if send_success and self.motor_status == "switch_on_disabled":
            print("\033[0;32m[Motor {}] quick stop\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] quick stop failed\033[0m".format(self.id))
            return False

    def reset(self):
        
        [cob_id, data] = gen.sdo_write_32(self.id, "control_word", pro.CONTROL_WORD["reset"])

        send_success = Motor.__device.send(cob_id, data)
        self.__update_motor_status()
        self.__update_bus_status()
        if send_success and self.motor_status == "switched_on" and self.bus_status == "pre-operational":
            print("\033[0;32m[Motor {}] reset\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] reset failed\033[0m".format(self.id))
            return False
