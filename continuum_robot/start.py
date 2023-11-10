# -*- coding:utf-8 -*-


from PyQt5.QtWidgets import QApplication


# 添加模块路径
import sys, os, json
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from continuum_robot.gui import ControlPanel


def main():
    app = QApplication(sys.argv)
    win = ControlPanel()
    if not app.exec_():
        print("=============================================================")
        print("\033[0;33mShuting down, saving...\033[0m")
        with open('robot_parameter.json', 'w') as file: json.dump(win.robot.PARAMETER, file) # 保存参数
        sys.exit()

if __name__ == "__main__":
    main()
