#ifndef __OV7670_Dir_H_
#define __OV7670_Dir_H_

//#include "stm32f1xx.h"
#include "stm32f10x_it.h"


extern double XPosition,YPosition;//λ����Ϣ
extern double DirectionXSpeed,DirectionYSpeed;//λ����Ϣ
extern double HIGH_CURRENT_CSB;//�߶���Ϣ
extern double DOWNSPEED_CURRENT_CSB;//�Ե��ٶ���Ϣ
extern uint8_t ov7670_flag;
extern int16_t angle_ov7670[2];
//���ڽ���Э��
void ov7670_get_one_byte(uint8_t c);
void c_flow(void);














#endif