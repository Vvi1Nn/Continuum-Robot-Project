# -*- coding:utf-8 -*-

import tkinter as tk

# 添加模块路径
import sys
import os
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(BASE_DIR)

import motor.function as motor_fun

gui = tk.Tk()

gui.title("控制面板")
# gui.iconbitmap("my_icon.ico")
gui.geometry("1000x800+800+500")
gui.config(bg = "#FFFFFF")
gui.resizable(True, True)

gui.mainloop()