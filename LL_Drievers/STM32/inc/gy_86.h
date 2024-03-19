#ifndef _GY_86_H
#define _GY_86_H
// #include "stm32f10x.h"
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#include "stm32f10x_i2c.h"
#include "usart.h"
#endif
#include "stdint.h"
#include "math.h"
#include "def.h"
#include "config.h"
// IIC总线引脚配置
#if defined(GUET_FLY_V1)
#if defined(EXTERN_IIC1) // 使用了外置IIC

#define EX_SCL_H GPIOB->BSRR |= GPIO_Pin_6
#define EX_SCL_L GPIOB->BRR |= GPIO_Pin_6

#define EX_SDA_H GPIOB->BSRR |= GPIO_Pin_7
#define EX_SDA_L GPIOB->BRR |= GPIO_Pin_7

// #define EX_SCL_read      GPIOA->IDR  & GPIO_Pin_0
// #define EX_SDA_read      GPIOA->IDR  & GPIO_Pin_1
#define EX_SDA_read PBin(7)
#endif

// #define SCL_H         GPIOA->BSRR |= GPIO_Pin_0
// #define SCL_L         GPIOA->BRR  |= GPIO_Pin_0

// #define SDA_H         GPIOA->BSRR |= GPIO_Pin_1
// #define SDA_L         GPIOA->BRR  |= GPIO_Pin_1

// #define SCL_read      GPIOA->IDR  & GPIO_Pin_0
// #define SDA_read      GPIOA->IDR  & GPIO_Pin_1

#define SCL_H GPIOB->BSRR |= GPIO_Pin_13
#define SCL_L GPIOB->BRR &= ~GPIO_Pin_13
#define SDA_H GPIOB->BSRR |= GPIO_Pin_15
#define SDA_L GPIOB->BRR &= ~GPIO_Pin_15
#define SCL_read GPIOB->IDR &GPIO_Pin_13
#define SDA_read GPIOB->IDR &GPIO_Pin_15
#elif defined(GUET_FLY_MINI_V1)

#define SCL_H GPIOB->BSRR |= GPIO_Pin_15
#define SCL_L GPIOB->BRR |= GPIO_Pin_15

#define SDA_H GPIOB->BSRR |= GPIO_Pin_14
#define SDA_L GPIOB->BRR |= GPIO_Pin_14

#define SCL_read GPIOB->IDR &GPIO_Pin_15
#define SDA_read GPIOB->IDR &GPIO_Pin_14

// #define SCL_H         GPIOB->BSRR |= GPIO_Pin_15
// #define SCL_L         GPIOB->BRR  &= ~GPIO_Pin_15

// #define SDA_H         GPIOB->BSRR |= GPIO_Pin_14
// #define SDA_L         GPIOB->BRR  &= ~GPIO_Pin_14

// #define SCL_read      GPIOB->IDR  & GPIO_Pin_15
// #define SDA_read      GPIOB->IDR  & GPIO_Pin_14
#else

#define SCL_H GPIOA->BSRR |= GPIO_Pin_5
#define SCL_L GPIOA->BRR |= GPIO_Pin_5

#define SDA_H GPIOA->BSRR |= GPIO_Pin_4
#define SDA_L GPIOA->BRR |= GPIO_Pin_4

#define SCL_read GPIOA->IDR &GPIO_Pin_5
#define SDA_read GPIOA->IDR &GPIO_Pin_4
#endif
//****************************************

void I2C_GPIO_Config(void);
void I2C_SendByte(uint8_t dat);
void I2C_Start(void);
void I2C_Stop(void);
void I2C_SendACK(uint8_t i);
bool I2C_WaitAck(void);
uint8_t I2C_RecvByte(void);
uint8_t Single_ReadI2C(uint8_t , uint8_t );
bool Single_WriteI2C(uint8_t , uint8_t , uint8_t );
#endif
