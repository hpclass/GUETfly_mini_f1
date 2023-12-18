#ifndef __timer_H__
#define __timer_H__
#include "sys.h"
void SysTick_Handler_time_rise(void);
void time_init(void);
void TIM3_Int_Init(u16 arr,u16 psc);
void TIM3_PWM_Init(u16 arr,u16 psc);
void TIM4_PWM_Init(u16 arr,u16 psc);
void TIM2_PWM_Init(u16 arr,u16 psc);
void TIM8_PWM_Init(u16 arr,u16 psc);
void TIM1_Cap_Init(u16 arr,u16 psc);
uint64_t millis(void) ;
uint64_t micros(void) ;
void TIM1_CC_IRQHandler(void);
#endif

