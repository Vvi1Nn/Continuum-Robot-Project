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




from PyQt5.QtCore import QThread, pyqtSignal, QMutex
from PyQt5.QtWidgets import QApplication
import time,sys
cache = []
abc = 10

class Thread1(QThread):
    def __init__(self, mutex: QMutex) -> None:
        super().__init__()
        self.__is_stop = False
        self.__lock = mutex
    
    def run(self):
        while not self.__is_stop:
            # self.__lock.lock()
            cache.append("abc")
            print("change")
            # self.__lock.unlock()
            time.sleep(1)
    
    def stop(self):
        self.__is_stop = True

class Thread2(QThread):
    def __init__(self, mutex: QMutex) -> None:
        super().__init__()
        self.__is_stop = False
        self.__lock = mutex
    
    def run(self):
        while not self.__is_stop:
            # self.__lock.lock()
            cache.pop(0)
            print(cache)
            # self.__lock.unlock()
            time.sleep(5)
    
    def stop(self):
        self.__is_stop = True

app = QApplication(sys.argv)
    
mutex = QMutex()
thread1 = Thread1(mutex)
thread2 = Thread2(mutex)
thread1.start()
thread2.start()
sys.exit(app.exec_())