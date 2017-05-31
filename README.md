# Bluetooth_uart
bluetooth to uart/serial depend on nRF51822 

SDK:10.0.0

Softdevice:S110 v8.0.0

官方example的版本，其串口转发给蓝牙的方式是，当收到'\n'或者超出20byte时，才会转发。
此版本是，从uart收到的任意数据均会转发，且当数据超出20bytes时，也会拆分2次转发。

AT命令暂不支持。
