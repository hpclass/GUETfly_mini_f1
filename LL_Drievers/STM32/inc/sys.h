#ifndef __SYS_H
#define __SYS_H
#ifdef STM32F10X_MD
#include "stm32f10x.h"
#endif
#include "stdint.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK STM32开发板
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2012/8/18
// 版本：V1.7
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// 0,不支持ucos
// 1,支持ucos
#define SYSTEM_SUPPORT_UCOS 0 // 定义系统文件夹是否支持UCOS

// 位带操作,实现51类似的GPIO控制功能
// 具体实现思想,参考<<CM3权威指南>>第五章(87页~92页).
// IO口操作宏定义
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO口地址映射
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) // 0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) // 0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) // 0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) // 0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) // 0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) // 0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) // 0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) // 0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) // 0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) // 0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) // 0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) // 0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) // 0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) // 0x40011E08

// IO口操作,只对单一的IO口!
// 确保n的值小于16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) // 输出
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  // 输入

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // 输出
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // 输入

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) // 输出
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  // 输入

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) // 输出
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  // 输入

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) // 输出
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  // 输入

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) // 输出
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  // 输入

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) // 输出
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  // 输入

// JTAG模式设置定义
#define JTAG_SWD_DISABLE 0X02
#define SWD_ENABLE 0X01
#define JTAG_SWD_ENABLE 0X00

void NVIC_Configuration(void);
/////////////////////////////////////////////////////////////////
// void BKP_Write(uint8_t reg,uint16_t dat);	//写入后备寄存器
void Stm32_Clock_Init(uint8_t PLL);                                                                      // 时钟初始化
void Sys_Soft_Reset(void);                                                                          // 系统软复位
void Sys_Standby(void);                                                                             // 待机模式
void MY_NVIC_SetVectorTable(uint32_t NVIC_VectTab, uint32_t Offset);                                          // 设置偏移地址
void MY_NVIC_PriorityGroupConfig(uint8_t NVIC_Group);                                                    // 设置NVIC分组
void MY_NVIC_Init(uint8_t NVIC_PreemptionPriority, uint8_t NVIC_SubPriority, uint8_t NVIC_Channel, uint8_t NVIC_Group); // 设置中断
void Ex_NVIC_Config(uint8_t GPIOx, uint8_t BITx, uint8_t TRIM);                                                    // 外部中断配置函数(只对GPIOA~G)
void JTAG_Set(uint8_t mode);

#endif