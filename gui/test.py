# -*- coding:utf-8 -*-

import tkinter as tk

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.function as motor_func
import usbcan.function as usbcan_func



def action_1():
    motor_1.set_position(10)
    motor_1.execute()

def action_2():
    motor_2.set_position(10)
    motor_2.execute()


main_gui = tk.Tk()
main_gui.title("【连续体机器人】控制面板")
main_gui.geometry("2560x1600") # 全屏
main_gui.config(bg = "#FFFFFF") # 背景：白色

usbcan_1 = usbcan_func.UsbCan()
motor_1 = motor_func.Motor(usbcan_1, 1, "position_control")
motor_2 = motor_func.Motor(usbcan_1, 2, "position_control")

motor_button_1 = tk.Button(main_gui, text = "No.1", command = action_1, activeforeground = "blue", activebackground = "yellow", repeatdelay = 25, repeatinterval = 1)
motor_button_2 = tk.Button(main_gui, text = "No.2", command = action_2, activeforeground = "blue", activebackground = "yellow", repeatdelay = 25, repeatinterval = 1)

motor_button_1.grid(row = 0, column = 0, sticky = "nwse")
motor_button_2.grid(row = 1, column = 0, sticky = "nwse")

main_gui.mainloop()
