# -*- coding:utf-8 -*-


from PyQt5.QtWidgets import QApplication, QMainWindow

from ui_login import Ui_MainWindow as Ui_LoginPanel
from ui_control import Ui_MainWindow as Ui_ControlPanel


class LoginPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_LoginPanel()
        self.ui.setupUi(self)
        
        self.show()
        self.__fix_window()
    
    def __fix_window(self, width = 1300, height = 631):
        self.resize(width, height)
        self.setFixedSize(width, height)


class ControlPanel(QMainWindow):
    def __init__(self) -> None:
        super().__init__()
        self.ui = Ui_ControlPanel()
        self.ui.setupUi(self)
        
        self.show()


if __name__ == "__main__":
    app = QApplication([])
    login_panel = LoginPanel()
    app.exec_()