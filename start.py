# -*- coding:utf-8 -*-


import sys, time

from PyQt5.QtWidgets import QApplication

from gui.function_v3 import LoginPanel
from motor.function import Motor


def main():
    app = QApplication(sys.argv)
    win = LoginPanel()
    if not app.exec_():
        print("=============================================================")
        print("\033[0;33mShuting down, waiting for processing ...\033[0m")
        if win.control_panel.motor_is_running:
            win.control_panel.quick_stop()
            time.sleep(1)
            print("\033[0;32mquick stop!\033[0m")
            win.control_panel.stop_pdo()
            time.sleep(1)
            print("\033[0;32mstop pdo!\033[0m")
        else:
            if Motor.homing(): print("\033[0;32mDone!\033[0m")
            else: print("\033[0;31mFailed...\033[0m")
        sys.exit()

if __name__ == "__main__":
    main()
    