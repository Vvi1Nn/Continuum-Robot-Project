# -*- coding:utf-8 -*-


import sys

from PyQt5.QtWidgets import QApplication

from function import LoginPanel


def main():
    app = QApplication(sys.argv)
    win = LoginPanel()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
