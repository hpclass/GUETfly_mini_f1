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

这个版本还有很多小毛病，请各位多多提出建议，谢谢

2017
PB3 TRI
PB4 echo

20210302测试更改