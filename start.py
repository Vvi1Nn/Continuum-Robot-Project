# -*- coding:utf-8 -*-


import sys

from PyQt5.QtWidgets import QApplication

from gui.function_new import LoginPanel


def main():
    app = QApplication(sys.argv)
    win = LoginPanel()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
    