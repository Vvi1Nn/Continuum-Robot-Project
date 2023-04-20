# -*- coding:utf-8 -*-

from PyQt5.QtWidgets import QApplication

from function import *


def main():
    app = QApplication(sys.argv)
    win = LoginPanel()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
