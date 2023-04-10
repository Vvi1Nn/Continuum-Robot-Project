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
        
        self.current_status = ""
        self.position = 0
        self.speed = 0
        
        self.__init()
    
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
        print("\033[0;32m\n[Motor] info: {}\033[0m".format(cls.__device))

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
        print("\033[0;32m[Motor] velocity: {}\n\033[0m".format(cls.__velocity))

    def __init(self) -> None:
        current_status = self.get_status()
        if current_status == "pre-operational":
            mode_success = self.__set_mode()
            acc_success = self.__set_acc()
            dec_success = self.__set_dec()
            vel_success = self.__set_vel()
        else:
            print("\033[0;31m[Motor {}] init failed\033[0m".format(self.id))
        
    def __set_mode(self) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "control_mode", Motor.__control_mode)
        
        Motor.__device.clear_buffer()
        
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)
        
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["control_mode"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False
        
        if send_success and reply_success:
            print("\033[0;32m[Motor {}] control mode: {}\033[0m".format(self.id, Motor.__control_mode))
            return True
        else:
            print("\033[0;31m[Motor {}] set control mode failed\033[0m".format(self.id))
            return False

    def __set_acc(self) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "acceleration", Motor.__acceleration)

        Motor.__device.clear_buffer()

        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["acceleration"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False

        if send_success and reply_success:
            print("\033[0;32m[Motor {}] acceleration: {}\033[0m".format(self.id, Motor.__acceleration))
            return True
        else:
            print("\033[0;31m[Motor {}] set acceleration failed\033[0m".format(self.id))
            return False

    def __set_dec(self) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "deceleration", Motor.__deceleration)

        Motor.__device.clear_buffer()

        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["deceleration"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False

        if send_success and reply_success:
            print("\033[0;32m[Motor {}] deceleration: {}\033[0m".format(self.id, Motor.__deceleration))
            return True
        else:
            print("\033[0;31m[Motor {}] set deceleration failed\033[0m".format(self.id))
            return False

    def __set_vel(self) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "velocity", Motor.__velocity)

        Motor.__device.clear_buffer()

        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.05)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and module.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["velocity"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False

        if send_success and reply_success:
            print("\033[0;32m[Motor {}] velocity: {}\033[0m".format(self.id, Motor.__velocity))
            return True
        else:
            print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.id))
            return False
    
    def get_status(self) -> str:
        
        [cob_id, data] = gen.nmt_get_status(self.id)

        Motor.__device.clear_buffer()

        send_success = Motor.__device.send(cob_id, [data], remote_flag = param.REMOTE_FLAG["remote"])
        time.sleep(0.05)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["NMT_S"]:
                for k in pro.NMT_STATUS:
                    if msg[0].Data[0] & 0b01111111 == pro.NMT_STATUS[k]:
                        self.current_status = k
                        reply_success = True
                        break
                    else: reply_success = False
            else: reply_success = False
        else: reply_success = False

        if send_success and reply_success:
            print("\033[0;32m[Motor {}] current status: {}\033[0m".format(self.id, self.current_status))
            return self.current_status
        else:
            print("\033[0;31m[Motor {}] cannot get motor status\033[0m".format(self.id))
            return None

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
    
    def start_feedback(self) -> bool:
        
        [cob_id, data] = gen.nmt_change_status(self.id, "start_remote_node")
        Motor.__device.clear_buffer()
        send_success = Motor.__device.send(cob_id, [data])

        if send_success:
            print("\033[0;32m[Motor {}] start feedback\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] start feedback failed\033[0m".format(self.id))
            return False
        
    def stop_feedback(self) -> bool:
        
        [cob_id, data] = gen.nmt_change_status(self.id, "stop_remote_node")
        send_success = Motor.__device.send(cob_id, [data])
        Motor.__device.clear_buffer()

        if send_success:
            print("\033[0;32m[Motor {}] stop feedback\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] stop feedback failed\033[0m".format(self.id))
            return False

    def execute(self, is_relative = True):
        
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

    def shut_down(self):
        
        [cob_id, data] = gen.sdo_write_32(self.id, "control_word", 0x07) # 停止

        send_success = Motor.__device.send(cob_id, data)
        time.sleep(0.05)
        
        if send_success:
            print("\033[0;32m[Motor {}] shut down\033[0m".format(self.id))
            return True
        else:
            print("\033[0;31m[Motor {}] shut down failed\033[0m".format(self.id))
            return False

    def get_position(self):
        pass