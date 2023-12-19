#include <stm32f10x_lib.h>
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////
// 本程序只供学习使用，未经作者许可，不得用于其它任何用途
// Mini STM32开发板
// 系统时钟初始化
// 正点原子@ALIENTEK
// 技术论坛:www.openedv.com
// 修改日期:2010/5/27
// 版本：V1.4
// 版权所有，盗版必究。
// Copyright(C) 正点原子 2009-2019
// All rights reserved
//********************************************************************************
// V1.4修改说明
// 把NVIC KO了,没有使用任何库文件!
// 加入了JTAG_Set函数
//////////////////////////////////////////////////////////////////////////////////

// 设置向量表偏移地址
// NVIC_VectTab:基址
// Offset:偏移量
// CHECK OK
// 091207
void MY_NVIC_SetVectorTable(u32 NVIC_VectTab, u32 Offset)
{
    // 检查参数合法性
    assert_param(IS_NVIC_VECTTAB(NVIC_VectTab));
    assert_param(IS_NVIC_OFFSET(Offset));
    SCB->VTOR = NVIC_VectTab | (Offset & (u32)0x1FFFFF80); // 设置NVIC的向量表偏移寄存器
    // 用于标识向量表是在CODE区还是在RAM区
}
// 设置NVIC分组
// NVIC_Group:NVIC分组 0~4 总共5组
// CHECK OK
// 091209
void MY_NVIC_PriorityGroupConfig(u8 NVIC_Group)
{
    u32 temp, temp1;
    temp1 = (~NVIC_Group) & 0x07; // 取后三位
    temp1 <<= 8;
    temp = SCB->AIRCR;  // 读取先前的设置
    temp &= 0X0000F8FF; // 清空先前分组
    temp |= 0X05FA0000; // 写入钥匙
    temp |= temp1;
    SCB->AIRCR = temp; // 设置分组
}
// 设置NVIC
// NVIC_PreemptionPriority:抢占优先级
// NVIC_SubPriority       :响应优先级
// NVIC_Channel           :中断编号
// NVIC_Group             :中断分组 0~4
// 注意优先级不能超过设定的组的范围!否则会有意想不到的错误
// 组划分:
// 组0:0位抢占优先级,4位响应优先级
// 组1:1位抢占优先级,3位响应优先级
// 组2:2位抢占优先级,2位响应优先级
// 组3:3位抢占优先级,1位响应优先级
// 组4:4位抢占优先级,0位响应优先级
// NVIC_SubPriority和NVIC_PreemptionPriority的原则是,数值越小,越优先
// CHECK OK
// 100329
void MY_NVIC_Init(u8 NVIC_PreemptionPriority, u8 NVIC_SubPriority, u8 NVIC_Channel, u8 NVIC_Group)
{
    u32 temp;
    u8 IPRADDR = NVIC_Channel / 4;           // 每组只能存4个,得到组地址
    u8 IPROFFSET = NVIC_Channel % 4;         // 在组内的偏移
    IPROFFSET = IPROFFSET * 8 + 4;           // 得到偏移的确切位置
    MY_NVIC_PriorityGroupConfig(NVIC_Group); // 设置分组
    temp = NVIC_PreemptionPriority << (4 - NVIC_Group);
    temp |= NVIC_SubPriority & (0x0f >> NVIC_Group);
    temp &= 0xf; // 取低四位

    if (NVIC_Channel < 32)
        NVIC->ISER[0] |= 1 << NVIC_Channel; // 使能中断位(要清除的话,相反操作就OK)
    else
        NVIC->ISER[1] |= 1 << (NVIC_Channel - 32);
    NVIC->IP[IPRADDR] |= temp << IPROFFSET; // 设置响应优先级和抢断优先级
}

// 外部中断配置函数
// 只针对GPIOA~G;不包括PVD,RTC和USB唤醒这三个
// 参数:GPIOx:0~6,代表GPIOA~G;BITx:需要使能的位;TRIM:触发模式,1,下升沿;2,上降沿;3，任意电平触发
// 该函数一次只能配置1个IO口,多个IO口,需多次调用
// 该函数会自动开启对应中断,以及屏蔽线
// 待测试...
void Ex_NVIC_Config(u8 GPIOx, u8 BITx, u8 TRIM)
{
    u8 EXTADDR;
    u8 EXTOFFSET;
    EXTADDR = BITx / 4; // 得到中断寄存器组的编号
    EXTOFFSET = (BITx % 4) * 4;

    RCC->APB2ENR |= 0x01; // 使能io复用时钟

    AFIO->EXTICR[EXTADDR] &= ~(0x000F << EXTOFFSET); // 清除原来设置！！！
    AFIO->EXTICR[EXTADDR] |= GPIOx << EXTOFFSET;     // EXTI.BITx映射到GPIOx.BITx

    // 自动设置
    EXTI->IMR |= 1 << BITx; //  开启line BITx上的中断
    // EXTI->EMR|=1<<BITx;//不屏蔽line BITx上的事件 (如果不屏蔽这句,在硬件上是可以的,但是在软件仿真的时候无法进入中断!)
    if (TRIM & 0x01)
        EXTI->FTSR |= 1 << BITx; // line BITx上事件下降沿触发
    if (TRIM & 0x02)
        EXTI->RTSR |= 1 << BITx; // line BITx上事件上升降沿触发
}

// 不能在这里执行所有外设复位!否则至少引起串口不工作.
// 把所有时钟寄存器复位
// CHECK OK
// 091209
void MYRCC_DeInit(void)
{
    RCC->APB1RSTR = 0x00000000; // 复位结束
    RCC->APB2RSTR = 0x00000000;

    RCC->AHBENR = 0x00000014;  // 睡眠模式闪存和SRAM时钟使能.其他关闭.
    RCC->APB2ENR = 0x00000000; // 外设时钟关闭.
    RCC->APB1ENR = 0x00000000;
    RCC->CR |= 0x00000001;   // 使能内部高速时钟HSION
    RCC->CFGR &= 0xF8FF0000; // 复位SW[1:0],HPRE[3:0],PPRE1[2:0],PPRE2[2:0],ADCPRE[1:0],MCO[2:0]
    RCC->CR &= 0xFEF6FFFF;   // 复位HSEON,CSSON,PLLON
    RCC->CR &= 0xFFFBFFFF;   // 复位HSEBYP
    RCC->CFGR &= 0xFF80FFFF; // 复位PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE
    RCC->CIR = 0x00000000;   // 关闭所有中断
    // 配置向量表
#ifdef VECT_TAB_RAM
    MY_NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else
    MY_NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif
}
// THUMB指令不支持汇编内联
// 采用如下方法实现执行汇编指令WFI
// CHECK OK
// 091209
__asm void WFI_SET(void)
{
    WFI;
}
// 进入待机模式
// check ok
// 091202
void Sys_Standby(void)
{
    SCB->SCR |= 1 << 2;      // 使能SLEEPDEEP位 (SYS->CTRL)
    RCC->APB1ENR |= 1 << 28; // 使能电源时钟
    PWR->CSR |= 1 << 8;      // 设置WKUP用于唤醒
    PWR->CR |= 1 << 2;       // 清除Wake-up 标志
    PWR->CR |= 1 << 1;       // PDDS置位
    WFI_SET();               // 执行WFI指令
}
// 后备寄存器写入操作
// reg:寄存器编号
// reg:要写入的数值
////check ok
////091202
// void BKP_Write(u8 reg,u16 dat)
//{
//   RCC->APB1ENR|=1<<28;     //使能电源时钟
//	RCC->APB1ENR|=1<<27;     //使能备份时钟
//	PWR->CR|=1<<8;           //取消备份区写保护
//	switch(reg)
//	{
//		case 1:
//			BKP->DR1=dat;
//			break;
//		case 2:
//			BKP->DR2=dat;
//			break;
//		case 3:
//			BKP->DR3=dat;
//			break;
//		case 4:
//			BKP->DR4=dat;
//			break;
//		case 5:
//			BKP->DR5=dat;
//			break;
//		case 6:
//			BKP->DR6=dat;
//			break;
//		case 7:
//			BKP->DR7=dat;
//			break;
//		case 8:
//			BKP->DR8=dat;
//			break;
//		case 9:
//			BKP->DR9=dat;
//			break;
//		case 10:
//			BKP->DR10=dat;
//			break;
//	}
// }
// 系统软复位
// CHECK OK
// 091209
void Sys_Soft_Reset(void)
{
    SCB->AIRCR = 0X05FA0000 | (u32)0x04;
}

// JTAG模式设置,用于设置JTAG的模式
// mode:jtag,swd模式设置;00,全使能;01,使能SWD;10,全关闭;
// CHECK OK
// 100818
void JTAG_Set(u8 mode)
{
    u32 temp;
    temp = mode;
    temp <<= 25;
    RCC->APB2ENR |= 1 << 0;   // 开启辅助时钟
    AFIO->MAPR &= 0XF8FFFFFF; // 清除MAPR的[26:24]
    AFIO->MAPR |= temp;       // 设置jtag模式
}
// 系统时钟初始化函数
// pll:选择的倍频数，从2开始，最大值为16
// CHECK OK
// 091209
void Stm32_Clock_Init(u8 PLL)
{
    unsigned char temp = 0;
    MYRCC_DeInit();        // 复位并配置向量表
    RCC->CR |= 0x00010000; // 外部高速时钟使能HSEON
    while (!(RCC->CR >> 17))
        ;                   // 等待外部时钟就绪
    RCC->CFGR = 0X00000400; // APB1=DIV2;APB2=DIV1;AHB=DIV1;
    PLL -= 2;               // 抵消2个单位
    RCC->CFGR |= PLL << 18; // 设置PLL值 2~16
    RCC->CFGR |= 1 << 16;   // PLLSRC ON
    FLASH->ACR |= 0x32;     // FLASH 2个延时周期

    RCC->CR |= 0x01000000; // PLLON
    while (!(RCC->CR >> 25))
        ;                    // 等待PLL锁定
    RCC->CFGR |= 0x00000002; // PLL作为系统时钟
    while (temp != 0x02)     // 等待PLL作为系统时钟设置成功
    {
        temp = RCC->CFGR >> 2;
        temp &= 0x03;
    }
}
