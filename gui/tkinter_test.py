# -*- coding:utf-8 -*-

import tkinter as tk

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.function as motor_func
import usbcan.function as usbcan_func

def init():
    global usbcan_1
    global motor_1, motor_2
    usbcan_1 = usbcan_func.UsbCan(Channel = 0)
    motor_1 = motor_func.Motor(usbcan_1, 1, "position_control")
    motor_2 = motor_func.Motor(usbcan_1, 2, "position_control")
    motor_init.config(text="初始化成功")
    motor_init.config(state="disabled")
    motor_1_f.config(state="active")
    motor_1_r.config(state="active")
    motor_2_f.config(state="active")
    motor_2_r.config(state="active")

def action_1_foreward():
    motor_1.set_position(50)
    motor_1.execute()
def action_1_reversal():
    motor_1.set_position(-50)
    motor_1.execute()
def action_1_position():
    pass
def action_2_foreward():
    motor_2.set_position(50)
    motor_2.execute()
def action_2_reversal():
    motor_2.set_position(-50)
    motor_2.execute()

# 1级窗口 GUI
main_gui = tk.Tk()
main_gui.title("【连续体机器人】控制面板")
main_gui.geometry("1200x800") # 全屏
main_gui.config(bg = "#FFFFFF") # 背景：白色

# 2级框架 电机
motor_frame = tk.LabelFrame(main_gui, text="步进电机", font=("黑体", 30), labelanchor="n")
motor_frame.pack(fill="x", padx=5)

motor_init = tk.Button(motor_frame, text="初始化CAN卡和电机", font=("黑体", 20), command=init, activeforeground="blue", activebackground="yellow")
motor_init.pack(fill="x", padx=15)

# 3级框架 电机1
motor_1 = tk.LabelFrame(motor_frame, text="No.1", font=("黑体", 20))
motor_1.pack(fill="x", padx=5)

motor_1_f = tk.Button(motor_1, text="正转", font=("黑体", 20), state="disabled", command=action_1_foreward, activeforeground="blue", activebackground="yellow", repeatdelay=25, repeatinterval=1)
motor_1_f.grid(row=0, column=0)

motor_1_r = tk.Button(motor_1, text="反转", font=("黑体", 20), state="disabled", command=action_1_reversal, activeforeground="blue", activebackground="yellow", repeatdelay=25, repeatinterval=1)
motor_1_r.grid(row=0, column=1)

motor_1_p = tk.Label(motor_1, text="当前位置", font=("黑体", 20))
motor_1_p.grid(row=0, column=2)

# 3级框架 电机2
motor_2 = tk.LabelFrame(motor_frame, text="No.2", font=("黑体", 20))
motor_2.pack(fill="x", padx=5)

motor_2_f = tk.Button(motor_2, text="正转", font=("黑体", 20), state="disabled", command=action_2_foreward, activeforeground="blue", activebackground="yellow", repeatdelay=25, repeatinterval=1)
motor_2_f.grid(row=0, column=0)

motor_2_r = tk.Button(motor_2, text="反转", font=("黑体", 20), state="disabled", command=action_2_reversal, activeforeground="blue", activebackground="yellow", repeatdelay=25, repeatinterval=1)
motor_2_r.grid(row=0, column=1)

motor_2_p = tk.Label(motor_2, text="当前位置", font=("黑体", 20))
motor_2_p.grid(row=0, column=2)


main_gui.mainloop()
