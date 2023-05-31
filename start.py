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
        print("\033[0;33mShuting down, waiting for homing ...\033[0m")
        if Motor.homing(): print("\033[0;32mDone!\033[0m")
        else: print("\033[0;31mFailed...\033[0m")
        print("=============================================================")
        sys.exit()

if __name__ == "__main__":
    main()
    