# -*- coding:utf-8 -*-


import sys, time

from PyQt5.QtWidgets import QApplication

# 添加模块路径
import sys, os, time
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from maxon_motor_epos2.gui import ControlPanel
from maxon_motor_epos2.motor import Motor


def main():
    app = QApplication(sys.argv)
    win = ControlPanel()
    if not app.exec_():
        print("=============================================================")
        print("\033[0;33mShuting down, waiting for processing ...\033[0m")
        win.motor_1.disable_voltage(is_pdo=True)
        win.motor_2.disable_voltage(is_pdo=True)
        print("=============================================================")
        sys.exit()

if __name__ == "__main__":
    main()
    