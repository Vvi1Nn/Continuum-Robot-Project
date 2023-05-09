# -*- coding:utf-8 -*-


''' function.py 操纵杆功能函数 v1.0 '''


import time
import pygame
from PyQt5.QtCore import QObject, QThread, pyqtSignal


class TestThread(QThread):
    signal = pyqtSignal(str)

    def __init__(self, t) -> None:
        super().__init__()
        self.t = t

    def run(self):
        for i in range(self.t):
            time.sleep(1)
            self.signal.emit(str(i))


class TextPrint:
    def __init__(self):
        self.reset()
        self.font = pygame.font.Font(None, 40)
 
    def print(self, screen, textString):
        textBitmap = self.font.render(textString, True, BLACK)
        screen.blit(textBitmap, [self.x, self.y])
        self.y += self.line_height
        
    def reset(self):
        self.x = 10
        self.y = 10
        self.line_height = 30
        
    def indent(self):
        self.x += 10
        
    def unindent(self):
        self.x -= 10
BLACK = (   0,   0,   0)
WHITE = ( 255, 255, 255)
class JoystickThread(QThread):
    joystick_signal = pyqtSignal(str)

    def __init__(self) -> None:
        super().__init__()
        
        pygame.init() # 初始化模块

        self.screen = pygame.display.set_mode([800, 800])
        self.clock = pygame.time.Clock()
        self.done = False
        pygame.display.set_caption("My Game")
        pygame.joystick.init()
        self.textPrint = TextPrint()

    def run(self):
        while self.done == False:
            for event in pygame.event.get():
                if event.type == pygame.QUIT: self.done = True
                if event.type == pygame.JOYBUTTONDOWN: print("Joystick button pressed.")
                if event.type == pygame.JOYBUTTONUP: print("Joystick button released.")
            self.screen.fill(WHITE)
            self.textPrint.reset()
        
            # Get count of joysticks
            joystick_count = pygame.joystick.get_count()
        
            self.textPrint.print(self.screen, "Number of joysticks: {}".format(joystick_count) )
            self.textPrint.indent()
            
            # For each joystick:
            for i in range(joystick_count):
                joystick = pygame.joystick.Joystick(i)
                joystick.init()
            
                self.textPrint.print(self.screen, "Joystick {}".format(i) )
                self.textPrint.indent()
            
                # Get the name from the OS for the controller/joystick
                name = joystick.get_name()
                self.textPrint.print(self.screen, "Joystick name: {}".format(name) )
                
                # Usually axis run in pairs, up/down for one, and left/right for
                # the other.
                axes = joystick.get_numaxes()
                self.textPrint.print(self.screen, "Number of axes: {}".format(axes) )
                self.textPrint.indent()
                
                for i in range( axes ):
                    axis = joystick.get_axis( i )
                    self.textPrint.print(self.screen, "Axis {} value: {:>6.3f}".format(i, axis*100000) )
                self.textPrint.unindent()
                    
                buttons = joystick.get_numbuttons()
                self.textPrint.print(self.screen, "Number of buttons: {}".format(buttons) )
                self.textPrint.indent()
        
                for i in range( buttons ):
                    button = joystick.get_button( i )
                    self.textPrint.print(self.screen, "Button {:>2} value: {}".format(i,button) )
                    print(self.screen, "Button {:>2} value: {}".format(i,button) )
                    
                        
                self.textPrint.unindent()
                    
                # Hat switch. All or nothing for direction, not like joysticks.
                # Value comes back in an array.
                hats = joystick.get_numhats()
                self.textPrint.print(self.screen, "Number of hats: {}".format(hats) )
                self.textPrint.indent()
        
                for i in range( hats ):
                    hat = joystick.get_hat( i )
                    self.textPrint.print(self.screen, "Hat {} value: {}".format(i, str(hat)) )
                self.textPrint.unindent()
                
                self.textPrint.unindent()
            
            # Go ahead and update the screen with what we've drawn.
            # pygame.display.flip()
        
            # Limit to 20 frames per second
            self.clock.tick(20)
            
        # Close the window and quit.
        # If you forget this line, the program will 'hang'
        # on exit if running from IDLE.
        pygame.quit ()