# Bluetooth_uart
bluetooth to uart/serial depend on nRF51822 

SDK:10.0.0

Softdevice:S110 v8.0.0

功能
1，修改官方的example版本，现在可以转发任意长度，任意数据的串口数据；而不必须以'\n结尾或20byte之内。
2，根据设备MAC地址给设备名称添加一定后缀，让其在多个设备同时工作时更易区分。
3，支持部分AT命令。修改波特率，获取MAC地址等。
