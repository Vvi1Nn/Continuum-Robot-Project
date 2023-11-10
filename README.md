# 贵航连续体机器人项目工程代码
文件夹`continuum_robot`为肌腱驱动连续体，文件夹`maxon_motor_epos2`为旋转进给机构。
## continuum_robot
`libusbcan.so` 致远电子的USBCAN-II动态库。
`usbcan.py` 致远电子的USBCAN-II功能函数。
`canopen.py` CANopen协议下，发送消息的生成函数。
`processor.py` CANopen协议下，总线的处理函数。
`motor.py` 和利时步进电机CANopen协议的功能函数。
`sensor.py` 坤维科技拉压传感器功能函数。
`io.py` 广成科技CANopen协议的IO模块功能函数。
`joystick.py` 罗技Extreme 3D操作杆功能函数。
`control_panel.ui` Qt设计师UI设计文件。
`control_panel.py` Qt设计师UI的python代码。
`gui.py` 上位机函数。
`robot.py` 机器人功能。
`start.py` 入口。
`robot_paramter.json` 机器人参数
## maxon_motor_epos2
略
