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

class A():
    def __init__(self, name) -> None:
        self.name = name

a = {}
for i in range(5):
    a[i] = A(i)
    print(a[i].name)