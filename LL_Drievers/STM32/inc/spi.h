#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// ALIENTEK战舰STM32开发板V3
// SPI驱动 代码
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 创建日期:2015/1/15
// 版本：V1.0
// 版权所有，盗版必究。
// Copyright(C) 广州市星翼电子科技有限公司 2009-2019
// All rights reserved
//////////////////////////////////////////////////////////////////////////////////

// SPI总线速度设置
#define SPI_SPEED_2 0
#define SPI_SPEED_4 1
#define SPI_SPEED_8 2
#define SPI_SPEED_16 3
#define SPI_SPEED_32 4
#define SPI_SPEED_64 5
#define SPI_SPEED_128 6
#define SPI_SPEED_256 7

void SPI2_Init(void);             // 初始化SPI2口
void SPI2_SetSpeed(uint8_t SpeedSet);  // 设置SPI2速度
uint8_t SPI2_ReadWriteByte(uint8_t TxData); // SPI2总线读写一个字节

#endif
