#ifndef __timer_H__
#define __timer_H__
#include "sys.h"
void SysTick_Handler_time_rise(void);
void time_init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM1_Cap_Init(u16 arr,u16 psc);
unsigned long millis(void) ;
unsigned long micros(void) ;
void TIM1_CC_IRQHandler(void);
#endif

