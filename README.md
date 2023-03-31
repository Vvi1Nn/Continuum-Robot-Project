# Continuum Robot Project
龙骨式原位无损检测连续体机器人技术研究工程代码

功能介绍如下

## CAN卡
设备为致远电子的USBCAN-II，支持Linux驱动和二次开发，可实现CAN总线与主机之间的CANopen协议的通讯。

该部分的程序均存放在`usbcan`文件夹，包含以下模块：

### 固定参数字典
`param_device.py`: 设备参数，如设备型号、设备索引、通道号。

`param_init.py`: 初始化参数，包含有波特率、收发模式等通讯前的协议确定。

`param_transmit.py`: 发送参数，包含从设备向CAN总线发送消息的参数。

`param.py`: 调用该模块即可实现所有字典的访问！

### 特定参数结构体
`struct.py`: 集成了二次开发API所需要使用的结构体。

### 二次开发库
`function.py`: 将二次开发API集成进Class中。

## 步进电机
设备为和利时的闭环步进一体机，支持CANopen协议讯通。

该部分的程序均存放在`motor`文件夹，包含以下模块：

### 通讯协议
`protocol`: 控制电机所需要的所有协议参数，包括对象字典、SDO和PDO通讯的CAN-ID、收发命令的CMD。

### 消息生成
`msg_generation.py`: 电机控制信号的生成，即COB-ID和8个字节的消息数据。

### 消息解析
`msg_resolution.py`: 解析通过USBCAN接收的消息。

### 电机控制
`function.py`: 电机控制函数。