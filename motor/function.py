# -*- coding:utf-8 -*-

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.protocol as pro
import motor.msg_generation as gen
import motor.msg_resolution as reso
import usbcan.struct as struct
import usbcan.param as param

class Motor:
    
    def __init__(self, device, node_id,
                 control_mode = "position_control",
                 acceleration = 1000,
                 deceleration = 10000,
                 velocity     = 100,
                 ) -> None:
        
        self.device = device
        self.id = node_id
        self.mode = control_mode
        self.acc = acceleration
        self.dec = deceleration
        self.vel = velocity

        mode_success = self.__set_mode()
        acc_success = self.__set_acc()
        dec_success = self.__set_dec()
        vel_success = self.__set_vel()
    
    def __set_mode(self) -> bool:
        
        if type(self.mode) != str:
            print("[SetMode {}] flag类型错误!!!".format(self.id))
            return False
    
        num = 0
        for key in pro.CONTROL_MODE.keys():
            if self.mode != key:
                num = num + 1
            else:
                break
            if num == len(pro.CONTROL_MODE.keys()):
                print("[SetMode {}] {}不存在!!!".format(self.id, self.mode))
                return False

        ret = gen.sdo_write_32(self.id, "control_mode", pro.CONTROL_MODE[self.mode])
        cob_id = ret["id"]
        data = [ret["data"]]
        
        success = self.device.send(cob_id, data)
        if success:
            print("[SetMode {}] {}".format(self.id, self.mode))
            return True
        else:
            print("[SetMode {}] 失败".format(self.id))
            return False

    def __set_acc(self) -> bool:
        
        if type(self.acc) != int:
            print("[SetAcc {}] acceleration类型错误!!!".format(self.id))
            return False
    
        if self.acc < 0 or self.acc > 1000:
            print("[SetAcc {}] acceleration超出范围!!!".format(self.id))
            return False

        ret = gen.sdo_write_32(self.id, "acceleration", self.acc)
        cob_id = ret["id"]
        data = [ret["data"]]

        success = self.device.send(cob_id, data)
        if success:
            print("[SetAcc {}] {}".format(self.id, self.acc))
            return True
        else:
            print("[SetAcc {}] 失败".format(self.id))
            return False

    def __set_dec(self) -> bool:
        
        if type(self.dec) != int:
            print("[SetDec {}] deceleration类型错误!!!".format(self.id))
            return False
    
        if self.dec < 0 or self.dec > 10000:
            print("[SetDec {}] deceleration超出范围!!!".format(self.id))
            return False

        ret = gen.sdo_write_32(self.id, "deceleration", self.dec)
        cob_id = ret["id"]
        data = [ret["data"]]

        success = self.device.send(cob_id, data)
        if success:
            print("[SetDec {}] {}".format(self.id, self.dec))
            return True
        else:
            print("[SetDec {}] 失败".format(self.id))
            return False

    def __set_vel(self) -> bool:
        
        if type(self.vel) != int:
            print("[SetVel {}] velocity类型错误!!!".format(self.id))
            return False
    
        if self.vel < 0 or self.vel > 200:
            print("[SetVel {}] velocity超出范围!!!".format(self.id))
            return False

        ret = gen.sdo_write_32(self.id, "velocity", self.vel)
        cob_id = ret["id"]
        data = [ret["data"]]

        success = self.device.send(cob_id, data)
        if success:
            print("[SetVel {}] {}".format(self.id, self.vel))
        else:
            print("[SetVel {}] 失败".format(self.id))
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