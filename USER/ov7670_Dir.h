#ifndef __OV7670_Dir_H_
#define __OV7670_Dir_H_

//#include "stm32f1xx.h"
#include "stm32f10x_it.h"


extern double XPosition,YPosition;//位置信息
extern double DirectionXSpeed,DirectionYSpeed;//位移信息
extern double HIGH_CURRENT_CSB;//高度信息
extern double DOWNSPEED_CURRENT_CSB;//对地速度信息
extern uint8_t ov7670_flag;
extern int16_t angle_ov7670[2];
//串口解析协议
void ov7670_get_one_byte(uint8_t c);
void c_flow(void);














#endif