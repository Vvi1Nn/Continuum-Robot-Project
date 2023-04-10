# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import time

import motor.protocol as pro
import motor.msg_generation as gen
import motor.msg_resolution as reso

class Motor:
    
    __control_mode = pro.CONTROL_MODE["position_control"]
    __acceleration = 1000
    __deceleration = 10000
    __velocity     = 100

    def __init__(self, node_id) -> None:
        
        self.id = node_id
        self.__init()
    
    def __str__(self) -> str:
        return "[Motor {}] {} acc:{} dec:{} vel:{}".format(self.id, Motor.__control_mode, Motor.__acceleration, Motor.__deceleration, Motor.__velocity)
    
    @classmethod
    def config(cls, device,
               control_mode = pro.CONTROL_MODE["position_control"],
               acceleration = 1000,
               deceleration = 10000,
               velocity     = 100,
               ):
        
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

    def __init(self):
        mode_success = self.__set_mode()
        acc_success = self.__set_acc()
        dec_success = self.__set_dec()
        vel_success = self.__set_vel()

    def __set_mode(self) -> bool:

        [cob_id, data] = gen.sdo_write_32(self.id, "control_mode", Motor.__control_mode)
        
        Motor.__device.clear_buffer()
        
        send_success = Motor.__device.send(cob_id, [data])
        time.sleep(0.1)
        
        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and reso.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["control_mode"]:
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
        time.sleep(0.1)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and reso.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["acceleration"]:
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
        time.sleep(0.1)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and reso.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["deceleration"]:
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
        time.sleep(0.1)

        [num, msg] = Motor.__device.read_buffer(1)
        if num != 0:
            if msg[0].ID == self.id + pro.CAN_ID["SDO_T"] and msg[0].Data[0] == pro.CMD_R["write"] and reso.match_index(msg[0].Data[1], msg[0].Data[2], msg[0].Data[3]) == pro.OD["velocity"]:
                reply_success = True
            else: reply_success = False
        else: reply_success = False

        if send_success and reply_success:
            print("\033[0;32m[Motor {}] velocity: {}\033[0m".format(self.id, Motor.__velocity))
            return True
        else:
            print("\033[0;31m[Motor {}] set velocity failed\033[0m".format(self.id))
            return False
    
    def set_position(self, position) -> bool:
        
        if type(position) != int:
            print("[SetPosition {}] position类型错误!!!".format(self.id))
            return False

        ret = gen.sdo_write_32(self.id, "target_position", position)
        cob_id = ret["id"]
        data = [ret["data"]]

        success = self.device.send(cob_id, data)
        if success:
            print("[SetPosition {}] {}".format(self.id, position))
            return True
        else:
            print("[SetPosition {}] 失败".format(self.id))
            return False
    
    def set_speed(self, speed) -> bool:
        
        if type(speed) != int:
            print("[SetSpeed {}] speed类型错误!!!".format(self.id))
            return False
        
        if speed < 0 or speed > 25:
            print("[SetSpeed {}] speed超出范围!!!".format(self.id))
            return False
        
        ret = gen.sdo_write_32(self.id, "target_speed", speed)
        cob_id = ret["id"]
        data = ret["data"]

        success = self.device.send_single(cob_id, data)
        if success:
            print("[SetSpeed {}] {}".format(self.id, speed))
            return True
        else:
            print("[SetSpeed {}] 失败".format(self.id))
            return False

    def execute(self, is_relative = True):
        
        if is_relative:
            
            enable = gen.sdo_write_32(self.id, "control_word", 0x6F) # 相对使能
            launch = gen.sdo_write_32(self.id, "control_word", 0x7F) # 启动
            
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
    
    def feedback(self):
        ret = gen.start_pdo(self.id)
        
        cob_id = ret["id"]
        data = ret["data"]

        success = self.device.send_single(cob_id, data)
        if success:
            print("[Feedback {}] 成功".format(self.id))
            return True
        else:
            print("[Feedback {}] 失败...".format(self.id))
            return False
    
    def shut_down(self):
        ret = gen.sdo_write_32(self.id, "control_word", 0x07) # 停止
        
        cob_id = ret["id"]
        data = [ret["data"]]

        success = self.device.send(cob_id, data)
        if success:
            print("[ShutDown {}] 成功".format(self.id))
            return True
        else:
            print("[ShutDown {}] 失败".format(self.id))
            return False

    def get_position(self):
        pass