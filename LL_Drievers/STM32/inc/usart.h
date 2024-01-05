#ifndef __USART_H
#define __USART_H
#include <stm32f10x_lib.h>
#include "stdio.h"
#include "config.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// Mini STM32开发板
// 串口1初始化
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2010/5/27
// 版本：V1.3
// 版权所有，盗版必究。
// Copyright(C) 正点原子 2009-2019
// All rights reserved
//********************************************************************************
// V1.3修改说明
// 支持适应不同频率下的串口波特率设置.
// 加入了对printf的支持
// 增加了串口接收命令功能.
// 修正了printf第一个字符丢失的bug
//////////////////////////////////////////////////////////////////////////////////

extern u8 USART_RX_BUF[1000]; // 接收缓冲,最大63个字节.末字节为换行符
extern u32 USART_RX_STA;      // 接收状态标记

// 如果想串口中断接收，请不要注释以下宏定义
#define EN_USART1_RX // 使能串口1接收
void uart_init(u32 pclk2, u32 bound);
void USART2_Configuration(u32 bound);
void USART3_Configuration(u32 bound);
void USART4_Configuration(u32 bound);
void USART5_Configuration(u32 bound);
#endif
#if defined(SBUS__)
extern uint16_t Cap_CH[13];
extern uint8_t SBUS_FLAG_;
void Capture();
#endif
