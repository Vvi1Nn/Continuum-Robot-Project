# -*- coding:utf-8 -*-


''' function.py 操纵杆功能函数 v1.0 '''


import time
import pygame
from PyQt5.QtCore import QThread, pyqtSignal


class JoystickThread(QThread):
    button_signal_0 = pyqtSignal(int)
    button_signal_1 = pyqtSignal(int)
    button_signal_2 = pyqtSignal(int)
    button_signal_3 = pyqtSignal(int)
    button_signal_4 = pyqtSignal(int)
    button_signal_5 = pyqtSignal(int)
    button_signal_6 = pyqtSignal(int)
    button_signal_7 = pyqtSignal(int)
    button_signal_8 = pyqtSignal(int)
    button_signal_9 = pyqtSignal(int)
    button_signal_10 = pyqtSignal(int)
    axis_signal_0 = pyqtSignal(float)
    axis_signal_1 = pyqtSignal(float)
    axis_signal_2 = pyqtSignal(float)
    axis_signal_3 = pyqtSignal(float)
    axis_signal_4 = pyqtSignal(float)
    axis_signal_5 = pyqtSignal(float)
    hat_signal_0 = pyqtSignal(tuple)
    
    def __init__(self) -> None:
        super().__init__()
        self.__is_joystick = False
        self.__is_stop = False
        
    def __setup(self):
        self.__is_stop = False

        pygame.init() # 初始化模块

        # 界面
        self.screen = pygame.display.set_mode([800, 500])
        pygame.display.set_caption("连续体机器人遥操作界面")
        self.screen.fill((255, 255, 255))

        self.font = pygame.font.SysFont(None, 40) # 字体

        self.clock = pygame.time.Clock() # 时钟

        # 等待joystick插入
        while not self.__is_joystick:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: pygame.quit()
            if pygame.joystick.get_count() == 1:
                self.__is_joystick = True
                self.joystick = pygame.joystick.Joystick(0)
            # 显示
            self.screen.fill((255,255,255))
            self.__print("No Joystick!", x=10, y=10)
            # 刷新
            pygame.display.flip()
            self.clock.tick(20)

    def __print(self, text, /, *, x, y):
        textBitmap = self.font.render(text, True, (0,0,0))
        self.screen.blit(textBitmap, [x, y])
    
    def run(self):
        self.__setup() # 初始化

        while not self.__is_stop:
            # 检测
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.__is_stop = True
                elif event.type == pygame.JOYBUTTONDOWN: print("Button pressed")
                elif event.type == pygame.JOYBUTTONUP: print("Button released")
                elif event.type == pygame.JOYHATMOTION: print("Joystick hat")
                elif event.type == pygame.JOYAXISMOTION: print("Joystick axis")
                else: pass
            
            # 显示
            self.screen.fill((255,255,255))
            x_current, y_current = 10, 10
            self.__print("Joystick name: {}".format(self.joystick.get_name()), x=x_current, y=y_current)
            for i in range(self.joystick.get_numbuttons()):
                getattr(self, f"button_signal_{i}").emit(self.joystick.get_button(i))
                y_current += 40
                self.__print("Button {}:  {}".format(i, True if self.joystick.get_button(i) else False), x=x_current, y=y_current)
            x_current, y_current = 310, 10
            for i in range(self.joystick.get_numaxes()):
                y_current += 40
                self.__print("Axis {}:  {:.6f}".format(i, self.joystick.get_axis(i)), x=x_current, y=y_current)
            x_current, y_current = 610, 10
            for i in range(self.joystick.get_numhats()):
                y_current += 40
                self.__print("Hat {}:  {}".format(i, str(self.joystick.get_hat(i))), x=x_current, y=y_current)
            
            # 刷新
            pygame.display.flip()
            self.clock.tick(20)
        pygame.quit()

    def stop(self):
        self.__is_stop = True