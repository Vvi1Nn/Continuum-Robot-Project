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

import threading
import time



cache = [[["label", None] for i in range(0)] for i in range(10)]
print(cache)
cache[1].append(["TPDO_1", 3])
cache[1].append(["TPDO_1", 13])
cache[1].append(["TPDO_1", 23])
print(cache)
cache[1].pop(0)
print(cache)
cache[0].append(["TPDO_1", 23])
print(cache)

import pygame
import time

pygame.init()
gamedisplay = pygame.display.set_mode((800,600))
pygame.display.set_caption("dddd")
clock = pygame.time.Clock()
while True:
    print(pygame.joystick.get_count())
    for event in pygame.event.get():
        if event.type==pygame.QUIT:
            pygame.quit()
            exit()
    # print(pygame.mouse.get_pressed())
    keys = pygame.key.get_pressed()
    if keys[pygame.K_ESCAPE]:
        pygame.quit()
        exit()
    if keys[pygame.K_SPACE]:
        gamedisplay.fill((255,0,0))
    else:
        gamedisplay.fill((0,0,0))
    pygame.display.update()
    # clock.tick(60)