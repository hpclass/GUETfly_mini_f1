/****************************************Copyright (c)****************************************************
STM32F103C8T6新建工程模板
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/

#ifdef STM32F10X_MD
#include "stm32f10x.h"
#endif
// #include "Multiwii.h"
//#include <stdio.h>
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "oled.h"
#include "soft_iic.h"
#include "gy_86.h"
// #include "def.h"

float pitch, roll, yaw;    // 欧拉角
short aacx, aacy, aacz;    // 加速度传感器原始数据
short gyrox, gyroy, gyroz; // 陀螺仪原始数据
int Mag_x = 0;
int Mag_y = 0;
int Mag_z = 0;
uint8_t exchange_num[8];
extern int TEMP;
extern int Pressure;

extern uint8_t TIM2CH1_CAPTURE_STA;  // 输入捕获状态
extern uint16_t TIM2CH1_CAPTURE_VAL; // 输入捕获值
extern uint16_t ppm_rx[];
//////////////////////////////////////////////

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
