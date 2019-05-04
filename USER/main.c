/****************************************Copyright (c)****************************************************
STM32F103C8T6�½�����ģ��
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/

#include "stm32f10x.h"
//#include "Multiwii.h"
#include <stdio.h>
#include "usart.h"
#include "delay.h"
#include "timer.h"
#include "oled.h"
#include "soft_iic.h"
#include "GY_86.h"
//#include "def.h"


unsigned int SysTick_Handler_times;

unsigned int delays_u;
unsigned int runtime_us;
unsigned int runtime_ms;
unsigned long	micros_time=0;
unsigned long	millis_time=0;


float pitch,roll,yaw; 		//ŷ����
short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
short gyrox,gyroy,gyroz;	//������ԭʼ����
int Mag_x=0;
int Mag_y=0;
int Mag_z=0;
u8 exchange_num[8];
extern int TEMP;
extern int Pressure;

#define FCLK 	72000000		//ϵͳʱ��72mhz
#define OSFREQ  100000		//�ж�Ƶ��10us
extern u8  TIM2CH1_CAPTURE_STA;		//���벶��״̬		    				
extern u16	TIM2CH1_CAPTURE_VAL;	//���벶��ֵ	
extern u16 ppm_rx[];
//////////////////////////////////////////////




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/

