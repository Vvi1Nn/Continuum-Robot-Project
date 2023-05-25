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

def joint_forward_factory(i, n):
    print(n)
def joint_reverse_factory(i, n):
    print(n)
def joint_stop_factory(i, n):
    print(n)
for i in range(1,15):
    exec(f"def joint_forward_{i}(n): joint_forward_factory({i}, n)")
    exec(f"def joint_reverse_{i}(n): joint_reverse_factory({i}, n)")
    exec(f"def joint_stop_{i}(n): joint_stop_factory({i}, n)")
joint_forward_1(3)

