TIME3=>motor_PWN

TIME4=>SERVO

2017/11/18
由于STM32的速度问题，会造成串口没收完，串口解析程序就解析完的情况，加入了判断语句，没数据就不解析

2017/11/18 19:01
修复GPS解析，串口解析部分问题，将不会出现ubx协议无法解析的问题，原因，在STM32中结构体会进行4字节对齐

2017/11/18 20:42
添加了OPTIC选项，需要修改三处，分别是types.h中的enum box 、Muiltiwill中的const char boxnames[]和const uint8_t boxids[]。

2017/11/18 13:42
修正storeWP()两处，航点校验和

2017/11/19 14:29
修正MSP_SET_NAV_CONFIG、SP_NAV_CONFIG两处的首发保证不受结构体内存对齐的影响

2017/12/27 9:32
GPS定点问题预计在GPS.C中818行附近，已验证坐标系转换无问题，可能问题出现在指针处

2017

PB3 TRI

PB4 echo

2018-3-5 串口3 标示设置反了

2018-3-5 16:10 加入串口4、5

2018-3-19 修改了FLASH存储的定义,完成定点跟踪

2018-3-22 修正AK8963方向

2018-3-27 加入5611气压计

2018-3-28 修复GPS定点

2018-4-1 修复24C08的驱动，航点可存28个,修复micros()函数中断频繁问题，已改为1ms中断一次，并且计算出的精度是1us，之前的精度是10us

无EEPROM时航点可以存入FLASH中，最多254航点，实现方法，先存到6k的堆栈中，上传完航点再写flash ，避免对flash 多次操作

2018-4-2 修正MSP_NAV_STATUS的CRC校验错误问题

2018-4-2 完全修正航点问题，能写入254个航点，不报错

2018-6-10 17:01备份惯性导航移植

2018-6-26 21:42 修复底层IIC驱动BUG，该BUG导致整个PB总线随着IIC改变而改变

2018-9-22 23:27 修改abs为ABS，增加导航拉杆角度20到22，根据观测，右拐掉高度，猜测因为abs引起，计算后发现定高限幅了


20210302测试更改


Quick  start :

On ubuntu 20.04:
```
    sudo apt-get install gcc-arm-none-eabi make -y
    export CROSS_COMPILE=arm-none-eabi-
    make STM32
```

On MAC:
```
    brew install arm-none-eabi-gcc
    export CROSS_COMPILE=arm-none-eabi-
    make STM32

```
 On Windows:
 ```
    just double click "MDK-ARM/Project.uvprojx"
 ```