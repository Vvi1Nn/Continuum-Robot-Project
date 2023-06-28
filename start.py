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
        
        # if win.control_panel.usbcan_is_running:
        #     if not win.control_panel.motor_is_running:
        #         win.control_panel.start_pdo()
        #         time.sleep(2)
        #     else: pass

        #     win.control_panel.quick_stop()
        #     time.sleep(0.1)

        #     win.control_panel.homing_pdo()
        #     print("\033[0;32mMOTOR 10 HOMED\033[0m")

        #     win.control_panel.stop_pdo()
        #     time.sleep(1)
                
            
        #     if win.control_panel.io_is_running:
        #         win.control_panel.close_module()
        #         print("\033[0;32mIO STOP\033[0m")
        #     else: pass

        #     if win.control_panel.sensor_is_running:
        #         win.control_panel.stop_force()
        #         print("\033[0;32mSENSOR STOP\033[0m")
        #     else: pass
        # else: pass
        
        print("=============================================================")
        sys.exit()

if __name__ == "__main__":
    main()
    