# -*- coding:utf-8 -*-

# 显示方式          意义
# 0                终端默认设置
# 1                高亮显示
# 4                使用下划线
# 5                闪烁
# 7                反白显示
# 8                不可见

print("[ " + "\033[0;32mzwh.py\033[0m" + " ]" + "\033[0;37m 测试代码\033[0m")
print("\033[0;30m黑\033[0m")
print("\033[0;31m红\033[0m")
print("\033[1;32m绿+高亮\033[0m")
print("\033[4;33m黃+下划线\033[0m")
print("\033[5;34m蓝+闪烁\033[0m")
print("\033[7;35m紫红+反白\033[0m")
print("\033[0;36m青蓝\033[0m")
print("\033[8;37m白+不可见\033[0m")

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np

# fig, ax = plt.subplots()
# x=np.arange(1,10,0.1)
# y=np.sin(x)
# line, = ax.plot(x,y)

# def update(frame):
#     new_data = np.sin(x+frame)
#     line.set_ydata(new_data)
#     return line,
# ani = FuncAnimation(fig, update, frames=np.linspace(0, 2*np.pi, 128), interval=50)
# plt.show()
# print("111")

# from itertools import count
# import random
# x_vals = []
# y_vals = []
# index = count()
# def animate(i):
#     x = next(index)
#     y = random.randint(0,10)
#     x_vals.append(x)
#     y_vals.append(y)
#     if len(x_vals)>50:
#         x_vals.pop(0)
#         y_vals.pop(0)
#     plt.cla()
#     plt.plot(x_vals, y_vals)
#     plt.tight_layout()
# ani = FuncAnimation(plt.gcf(), animate, interval=50)
# plt.tight_layout()
# plt.show()

section = "a"
print(section == "a" or section == "b")