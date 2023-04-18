# -*- coding:utf-8 -*-

from PyQt5.QtWidgets import QApplication

from function import *

win = None

def main():
    app = QApplication([])
    win = LoginPanel()
    app.exec_()

if __name__ == "__main__":
    main()
