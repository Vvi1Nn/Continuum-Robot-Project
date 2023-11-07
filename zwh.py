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

import numpy as np
reference_force = np.array([[-5, -5, -5,
                                     -3, -3, -3,
                                     -2, -2, -2]]).T
previous_error = np.array([[0.1, 0, 0, 0, 0, 0, 0, 0, 0]]).T
print(np.maximum(previous_error, reference_force))