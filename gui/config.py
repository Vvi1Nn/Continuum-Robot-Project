# -*- coding:utf-8 -*-

# from PySide2.QtWidgets import QApplication, QMainWindow, QPushButton
# from PySide2.QtUiTools import QUiLoader
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget
from PyQt5 import uic

from layout import Ui_main_window

class ControlPanel(QWidget):
    
    def __init__(self) -> None:
        
        # self.main_window = uic.loadUi("control_panel.ui")
        super().__init__()
        self.main_window = Ui_main_window()
        self.main_window.setupUi(self)
        
        self.main_window.page_stack.setCurrentIndex(0)
        self.main_window.button_setting.setEnabled(False)
        self.main_window.button_control.setEnabled(False)
        self.main_window.button_log.setEnabled(False)
        self.main_window.login.clicked.connect(self.login_check)

        self.main_window.button_account.clicked.connect(self.jump_to_account_page)
        self.main_window.button_setting.clicked.connect(self.jump_to_setting_page)
        self.main_window.button_control.clicked.connect(self.jump_to_control_page)
        self.main_window.button_log.clicked.connect(self.jump_to_log_page)
    
    def login_check(self):
        pw = self.main_window.password.toPlainText()
        if pw == "admin":
            self.main_window.button_setting.setEnabled(True)
            self.main_window.button_control.setEnabled(True)
            self.main_window.button_log.setEnabled(True)
            self.main_window.password.clear()
        elif pw == "user":
            self.main_window.button_control.setEnabled(True)
            self.main_window.button_log.setEnabled(True)
            self.main_window.password.clear()
        else:
            self.main_window.password.clear()
    def jump_to_account_page(self):
        self.main_window.page_stack.setCurrentIndex(0)
    def jump_to_setting_page(self):
        self.main_window.page_stack.setCurrentIndex(1)
    def jump_to_control_page(self):
        self.main_window.page_stack.setCurrentIndex(2)
    def jump_to_log_page(self):
        self.main_window.page_stack.setCurrentIndex(3)
    

if __name__ == "__main__":
    app = QApplication([])
    control_panel = ControlPanel()
    control_panel.show()
    app.exec_()
