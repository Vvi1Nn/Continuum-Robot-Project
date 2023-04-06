# -*- coding:utf-8 -*-

import tkinter as tk

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.function as motor_func
import usbcan.function as usbcan_func

isAction = True

def callback():
    print("你点了一下按钮")

def addnum():
    num = int(motor_button_1.cget("text")) #获取组件的参数选项
    motor_button_1.config(text=str(num + 1))

def action_1():
    if isAction:
        usbcan_1 = usbcan_func.UsbCan()
        motor_1 = motor_func.Motor(usbcan_1, 1, "position_control")
        isAction = False
    motor_1.set_position(10)
    motor_1.execute()

main_gui = tk.Tk()
main_gui.title("【连续体机器人】控制面板")
main_gui.geometry("2560x1600") # 全屏
main_gui.config(bg = "#FFFFFF") # 背景：白色

motor_button_1 = tk.Button(main_gui, text = "1", command = addnum, activeforeground = "blue", activebackground = "yellow", repeatdelay = 50, repeatinterval = 50)
motor_button_2 = tk.Button(main_gui, text = "No.2", command = callback, activeforeground = "blue", activebackground = "yellow")

motor_button_1.grid(row = 0, column = 0, sticky = "nwse")
motor_button_2.grid(row = 1, column = 0, sticky = "nwse")

main_gui.mainloop()
