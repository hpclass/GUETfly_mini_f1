#ifndef _GY_86_H
#define _GY_86_H
//#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "math.h"
#include "usart.h"
#include "def.h"
#include "config.h"
//IIC×ÜÏßÒý½ÅÅäÖÃ
#if defined(GUET_FLY_V1)
#define SCL_H         GPIOB->BSRR |= GPIO_Pin_6
#define SCL_L         GPIOB->BRR  |= GPIO_Pin_6

#define SDA_H         GPIOB->BSRR |= GPIO_Pin_7
#define SDA_L         GPIOB->BRR  |= GPIO_Pin_7

#define SCL_read      GPIOB->IDR  & GPIO_Pin_6
#define SDA_read      GPIOB->IDR  & GPIO_Pin_7

//#define SCL_H         GPIOB->BSRR = GPIO_Pin_13
//#define SCL_L         GPIOB->BRR  = GPIO_Pin_13

//#define SDA_H         GPIOB->BSRR = GPIO_Pin_15
//#define SDA_L         GPIOB->BRR  = GPIO_Pin_15

//#define SCL_read      GPIOB->IDR  & GPIO_Pin_13
//#define SDA_read      GPIOB->IDR  & GPIO_Pin_15

#else

#define SCL_H         GPIOB->BSRR |= GPIO_Pin_15
#define SCL_L         GPIOB->BRR  |= GPIO_Pin_15

#define SDA_H         GPIOB->BSRR |= GPIO_Pin_14
#define SDA_L         GPIOB->BRR  |= GPIO_Pin_14

#define SCL_read      GPIOB->IDR  & GPIO_Pin_15
#define SDA_read      GPIOB->IDR  & GPIO_Pin_14
#endif
//****************************************



void I2C_GPIO_Config(void);
static void I2C_Mode_Config(void);
void I2C_SendByte(u8 dat);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendACK(u8 i);
bool I2C_WaitAck(void);
u8 I2C_RecvByte(void);
#endif
