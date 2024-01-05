/************************************

桂林电子科技大学
时间：2016-07-25

文件说明：

滴答时钟
************************************/

#include "timer.h"
#include "def.h"
#include "config.h"

unsigned int SysTick_Handler_times;
unsigned int delays_u;
unsigned int runtime_us;
unsigned int runtime_ms;
unsigned long micros_time = 0;
unsigned long millis_time = 0;
extern unsigned int delays;
// extern unsigned int SysTick_Handler_times;
// extern unsigned int delays_u;
// extern unsigned long	micros_time;
static uint64_t _micros_time_ = 0; // 微秒
uint64_t micros(void)
{
    return _micros_time_ + (72000 - SysTick->VAL) / 72; // 毫秒数加上定时器的值
}
static uint64_t _millis_time_ = 0; // 毫秒
uint64_t millis(void)
{
    return _millis_time_;
}
void SysTick_Handler(void)
{
    // SysTick_Handler_time_rise();
    _millis_time_++;
    _micros_time_ += 1000; // 产生中断加1000；
    if (delays != 0)
        delays--;
}
void time_init()
{
    SysTick_Handler_times = 0;
}

/**********************************************
定时器3，输出管脚：
PA6、PA7、PB0、PB1
***********************************************/
void TIM3_PWM_Init(u16 arr, u16 psc)
{

    RCC->APB1ENR |= 1 << 1; // TIM3时钟使能
    RCC->APB2ENR |= 1 << 2; // 使能PORTA时钟
    RCC->APB2ENR |= 1 << 3; // 使能PORTB时钟

    GPIOA->CRL &= 0X00FFFFFF; // PA6 CH1 PA7 CH2
    GPIOA->CRL |= 0XBB000000; // 复用功能输出
    GPIOB->CRL &= 0XFFFFFF00; // PB0 CH3 PB1 CH4
    GPIOB->CRL |= 0X000000BB; // 复用功能输出

    TIM3->ARR = arr; // 设定计数器自动重装值
    TIM3->PSC = psc; // 预分频器

    TIM3->CCMR1 |= 6 << 4; // CH1 PWM1模式
    TIM3->CCMR1 |= 1 << 3; // CH1预装载使能
    TIM3->CCER |= 1 << 0;  // OC1 输出使能

    TIM3->CCMR1 |= 6 << 12; // CH2 PWM1模式
    TIM3->CCMR1 |= 1 << 11; // CH2预装载使能
    TIM3->CCER |= 1 << 4;   // OC2 输出使能

    TIM3->CCMR2 |= 6 << 4; // CH3 PWM1模式
    TIM3->CCMR2 |= 1 << 3; // CH3预装载使能
    TIM3->CCER |= 1 << 8;  // OC3 输出使能

    TIM3->CCMR2 |= 6 << 12; // CH4 PWM1模式
    TIM3->CCMR2 |= 1 << 11; // CH4预装载使能
    TIM3->CCER |= 1 << 12;  // OC4 输出使能

    //    TIM3->CCR1=1000;
    //    TIM3->CCR2=1000;

    //    TIM3->CCR3=1000;

    //    TIM3->CCR4=1000;

    TIM3->CR1 |= 1 << 7; // ARPE使能
    TIM3->CR1 |= 0x01;   // 使能定时器3
    //    TIM_SetCompare1(TIM3,1000);
    //    TIM_SetCompare2(TIM3,1000);
    //    TIM_SetCompare3(TIM3,1000);
    //    TIM_SetCompare4(TIM3,1000);
}
/**********************************************
定时器8，输出管脚：
PD12,13,14,15 （重映射）
或者
PB6,7,8,9
***********************************************/

void TIM4_PWM_Init(u16 arr, u16 psc)
{
#if defined(GUET_FLY_V1)
    RCC->APB1ENR |= 1 << 2;   // TIM4时钟使能
    RCC->APB2ENR |= 1 << 5;   // 使能PORTD时钟
    AFIO->MAPR &= 0XFFFEFF;   // TIM4重映射，bit12,清零
    AFIO->MAPR |= 0X001000;   // TIM4重映射，bit12，写入
    GPIOD->CRH &= 0X0000FFFF; // 清零相应位PD12,13,14,15
    GPIOD->CRH |= 0XBBBB0000; // 复用功能输出
#elif defined(GUET_FLY_MINI_V1)
    RCC->APB1ENR |= 1 << 2;   // TIM4时钟使能
    RCC->APB2ENR |= 1 << 3;   // 使能PORTB时钟
    GPIOB->CRH &= 0XFFFFFF00; // 清零相应位PB8,9
    GPIOB->CRH |= 0X000000BB; // 复用功能输出
    GPIOB->CRL &= 0X00FFFFFF; // 清零相应位PB6,7
    GPIOB->CRL |= 0XBB000000; // 复用功能输出
#else
    RCC->APB1ENR |= 1 << 2;   // TIM4时钟使能
    RCC->APB2ENR |= 1 << 3;   // 使能PORTB时钟
    GPIOB->CRH &= 0XFFFFFF00; // 清零相应位PB8,9
    GPIOB->CRH |= 0X000000BB; // 复用功能输出
    GPIOB->CRL &= 0X00FFFFFF; // 清零相应位PB6,7
    GPIOB->CRL |= 0XBB000000; // 复用功能输出
#endif
    TIM4->ARR = arr;        // 设定计数器自动重装值
    TIM4->PSC = psc;        // 预分频器
    TIM4->CCMR1 |= 6 << 4;  // CH1 PWM1模式
    TIM4->CCMR1 |= 1 << 3;  // CH1预装载使能
    TIM4->CCER |= 1 << 0;   // OC1 输出使能
    TIM4->CCMR1 |= 6 << 12; // CH2 PWM1模式
    TIM4->CCMR1 |= 1 << 11; // CH2预装载使能
    TIM4->CCER |= 1 << 4;   // OC2 输出使能
    TIM4->CCMR2 |= 6 << 4;  // CH3 PWM1模式
    TIM4->CCMR2 |= 1 << 3;  // CH3预装载使能
    TIM4->CCER |= 1 << 8;   // OC3 输出使能
    TIM4->CCMR2 |= 6 << 12; // CH4 PWM1模式
    TIM4->CCMR2 |= 1 << 11; // CH4预装载使能
    TIM4->CCER |= 1 << 12;  // OC4 输出使能
    TIM4->CR1 |= 1 << 7;    // ARPE使能
    TIM4->CR1 |= 0x01;      // 使能定时器
}
/**********************************************
定时器2，输出管脚：
PA0、PA1、PA2、PA3
***********************************************/

void TIM2_PWM_Init(u16 arr, u16 psc)
{
    RCC->APB1ENR |= 1 << 0;   // TIM2时钟使能
    RCC->APB2ENR |= 1 << 1;   // 使能PORTA时钟
    GPIOA->CRL &= 0XFFFF0000; // 清零相应位PD12,13,14,15
    GPIOA->CRL |= 0X0000BBBB; // 复用功能输出
    TIM2->ARR = arr;          // 设定计数器自动重装值
    TIM2->PSC = psc;          // 预分频器
    TIM2->CCMR1 |= 6 << 4;    // CH1 PWM1模式
    TIM2->CCMR1 |= 1 << 3;    // CH1预装载使能
    TIM2->CCER |= 1 << 0;     // OC1 输出使能
    TIM2->CCMR1 |= 6 << 12;   // CH2 PWM1模式
    TIM2->CCMR1 |= 1 << 11;   // CH2预装载使能
    TIM2->CCER |= 1 << 4;     // OC2 输出使能
    TIM2->CCMR2 |= 6 << 4;    // CH3 PWM1模式
    TIM2->CCMR2 |= 1 << 3;    // CH3预装载使能
    TIM2->CCER |= 1 << 8;     // OC3 输出使能
    TIM2->CCMR2 |= 6 << 12;   // CH4 PWM1模式
    TIM2->CCMR2 |= 1 << 11;   // CH4预装载使能
    TIM2->CCER |= 1 << 12;    // OC4 输出使能
    TIM2->CR1 |= 1 << 7;      // ARPE使能
    TIM2->CR1 |= 0x01;        // 使能定时器3
}
/**********************************************
定时器8，输出管脚：
PC6、PC7、PC8、PC9
***********************************************/
void TIM8_PWM_Init(u16 arr, u16 psc)
{
    RCC->APB2ENR |= 1 << 13;  // TIM8时钟使能
    RCC->APB2ENR |= 1 << 4;   // 使能PORTC时钟
    GPIOC->CRH &= 0XFFFFFF00; // 清零相应位PC8,9
    GPIOC->CRH |= 0X000000BB; // 复用功能输出
    GPIOC->CRL &= 0X00FFFFFF; // 清零相应位PC6,7
    GPIOC->CRL |= 0XBB000000; // 复用功能输出
    TIM8->ARR = arr;          // 设定计数器自动重装值
    TIM8->PSC = psc;          // 预分频器
    TIM8->CCMR1 |= 6 << 4;    // CH1 PWM1模式
    TIM8->CCMR1 |= 1 << 3;    // CH1预装载使能
    TIM8->CCER |= 1 << 0;     // OC1 输出使能
    TIM8->CCMR1 |= 6 << 12;   // CH2 PWM1模式
    TIM8->CCMR1 |= 1 << 11;   // CH2预装载使能
    TIM8->CCER |= 1 << 4;     // OC2 输出使能
    TIM8->CCMR2 |= 6 << 4;    // CH3 PWM1模式
    TIM8->CCMR2 |= 1 << 3;    // CH3预装载使能
    TIM8->CCER |= 1 << 8;     // OC3 输出使能
    TIM8->CCMR2 |= 6 << 12;   // CH4 PWM1模式
    TIM8->CCMR2 |= 1 << 11;   // CH4预装载使能
    TIM8->CCER |= 1 << 12;    // OC4 输出使能
    TIM8->CR1 |= 1 << 7;      // ARPE使能
    TIM8->CR1 |= 0x01;        // 使能定时器
    TIM8->BDTR |= 1 << 15;    // 开启OC和OCN输出
}
void TIM1_PWM_Init(u16 arr, u16 psc)
{
    RCC->APB1ENR |= 1 << 11;  // TIM1时钟使能
    RCC->APB2ENR |= 1 << 6;   // 使能PORTD时钟
    AFIO->MAPR &= 0XFFFF3F;   // TIM1完全重映射，bit12,清零
    AFIO->MAPR |= 0X0000C0;   // TIM1完全重映射，bit12，写入
    GPIOE->CRH &= 0XF00F0F0F; // 清零相应位
    GPIOE->CRH |= 0X0BB0B0B0; // 复用功能输出

    TIM1->ARR = arr; // 设定计数器自动重装值
    TIM1->PSC = psc; // 预分频器

    //  TIM1->CCMR1|=6<<4;   //CH1 PWM1模式
    //  TIM1->CCMR1|=1<<3;      //CH1预装载使能
    //  TIM1->CCER|=1<<0;    //OC1 输出使能

    TIM1->CCMR1 |= 6 << 12; // CH2 PWM1模式
    TIM1->CCMR1 |= 1 << 11; // CH2预装载使能
    TIM1->CCER |= 1 << 4;   // OC2 输出使能

    TIM1->CCMR2 |= 6 << 4; // CH3 PWM1模式
    TIM1->CCMR2 |= 1 << 3; // CH3预装载使能
    TIM1->CCER |= 1 << 8;  // OC3 输出使能

    TIM1->CCMR2 |= 6 << 12; // CH4 PWM1模式
    TIM1->CCMR2 |= 1 << 11; // CH4预装载使能
    TIM1->CCER |= 1 << 12;  // OC4 输出使能
    // OCN默认不激活

    //    TIM1->CCR1=1000;
    //    TIM1->CCR2=1000;

    //    TIM1->CCR3=1000;

    //    TIM1->CCR4=1000;

    TIM1->CR1 |= 1 << 7; // ARPE使能
    TIM1->CR1 |= 0x01;   // 使能定时器

    TIM1->BDTR |= 1 << 15; // 开启OC和OCN输出
    //	TIM_CtrlPWMOutputs(TIM8,ENABLE);
}

// 定时器1通道1输入捕获配置
#if !defined(SBUS__)

void TIM1_Cap_Init(u16 arr, u16 psc)
{
    TIM_ICInitTypeDef TIM1_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  // 使能TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 使能GPIOA时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;     // PA0 清除之前设置
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; // PA0 输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_8); // PA0 下拉
    TIM_DeInit(TIM1);                  // 将外设TIM1寄存器重设为默认值
    // 初始化定时器1
    TIM_TimeBaseStructure.TIM_Period = arr;                     // 设定计数器自动重装值
    TIM_TimeBaseStructure.TIM_Prescaler = psc;                  // 预分频器
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     // 设置时钟分割:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // TIM向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);             // 根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

    // 初始化TIM2输入捕获参数
    TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1;                // CC1S=01 	选择输入端 IC1映射到TI1上
    TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;     // 上升沿捕获
    TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; // 映射到TI1上
    TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;           // 配置输入分频,不分频
    TIM1_ICInitStructure.TIM_ICFilter = 0x00;                        // IC1F=0000 配置输入滤波器 不滤波
    TIM_ICInit(TIM1, &TIM1_ICInitStructure);

    // 中断分组初始化
    NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;        // TIM1中断
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; // 先占优先级2级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        // 从优先级0级
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           // IRQ通道被使能
    NVIC_Init(&NVIC_InitStructure);                           // 根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    TIM_ITConfig(TIM1, TIM_IT_Update | TIM_IT_CC1, ENABLE); // 允许更新中断 ,允许CC1IE捕获中断

    TIM_Cmd(TIM1, ENABLE); // 使能定时器1
}

u32 temp = 0;
u8 TIM1CH1_CAPTURE_STA = 0, ppm_rx_sta = 0, ppm_rx_num = 0; // 输入捕获状态
u16 TIM1CH1_CAPTURE_VAL;                                    // 输入捕获值
u16 ppm_rx[RC_CHANS + 2];                                   // ppm_rx[0]   1   接收到ppm数据
// extern volatile uint16_t rcValue[RC_CHANS];
// 定时器1中断服务程序

void TIM1_CC_IRQHandler(void)
{

    if ((TIM1CH1_CAPTURE_STA & 0X80) == 0) // 还未成功捕获
    {
        if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)

        {
            if (TIM1CH1_CAPTURE_STA & 0X40) // 已经捕获到高电平了
            {
                if ((TIM1CH1_CAPTURE_STA & 0X3F) == 0X3F) // 高电平太长了
                {
                    TIM1CH1_CAPTURE_STA |= 0X80; // 标记成功捕获了一次
                    TIM1CH1_CAPTURE_VAL = 0XFFFF;
                }
                else
                    TIM1CH1_CAPTURE_STA++;
            }
        }
        else if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) // 捕获1发生捕获事件
        {
            if (TIM1CH1_CAPTURE_STA & 0X40) // 捕获到一个下降沿
            {
                TIM1CH1_CAPTURE_STA |= 0X80; // 标记成功捕获到一次高电平脉宽
                TIM1CH1_CAPTURE_VAL = TIM_GetCapture1(TIM1);
                TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); // CC1P=0 设置为上升沿捕获
            }
            else // 还未开始,第一次捕获上升沿
            {
                TIM1CH1_CAPTURE_STA = 0; // 清空
                TIM1CH1_CAPTURE_VAL = 0;
                TIM_SetCounter(TIM1, 0);
                TIM1CH1_CAPTURE_STA |= 0X40;                         // 标记捕获到了上升沿
                TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling); // CC1P=1 设置为下降沿捕获
            }
        }
    }

    // 处理帧数据
    if (TIM1CH1_CAPTURE_STA & 0X80) // 成功捕获到了一次上升沿
    {
        if (ppm_rx_sta == 1)
        {
            ppm_rx[ppm_rx_num + 1] = TIM1CH1_CAPTURE_VAL;
            ppm_rx_num++;
        }
        // printf("TIM1CH1_CAPTURE_VAL:%d\r\n",TIM1CH1_CAPTURE_VAL);
        if (4 > TIM1CH1_CAPTURE_STA & 0X3F > 0 || TIM1CH1_CAPTURE_VAL > 3000)
            ppm_rx_sta++; // 低电平时间大于3000us为起始帧
        if (ppm_rx_sta == 2)
        {
            ppm_rx[ppm_rx_num] = 0; // printf("receive\r\n");//ppm_rx_sta   1 表示接收到同步帧/ 2接收到到下一起始帧 ppm数据接收完毕
            ppm_rx_sta = 0;
            ppm_rx[0] = 1;
            ppm_rx_num = 0;
        }
        if (ppm_rx_num > 12)
            ppm_rx_num = 0; // 防止溢出

        TIM1CH1_CAPTURE_STA = 0; // 开启下一次捕获
    }

    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1 | TIM_IT_Update); // 清除中断标志位
}
#endif
void TIM2_Cap_Init(u16 arr, u16 psc)
{
    TIM_ICInitTypeDef TIM2_ICInitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC->APB1ENR |= 1 << 0; // 使能定时器2，TIM2
    RCC->APB2ENR |= 1 << 2; // 使能PORTA时钟

    GPIOA->CRL &= 0XFFFFFF0F; // PA1清零
    GPIOA->CRL |= 0X00000080; // 输入
    GPIOA->ODR |= 0 << 1;     // PA1 下拉

    TIM2->ARR = arr; // 设定计数器自动重装值
    TIM2->PSC = psc; // 预分频

    TIM2->CCMR1 |= 1 << 8;  // CC2S=10         选择输入端 IC2映射到TI2上
    TIM2->CCMR1 |= 0 << 12; // IC2F=0001 配置输入滤波器 不滤波
    TIM2->CCMR1 |= 0 << 10; // IC2PS=00         配置输入分频,不分频

    TIM2->CCER |= 0 << 5; // CC2P=0        上升沿捕获
    TIM2->CCER |= 1 << 4; // CC2E=1         允许捕获计数器的值到捕获寄存器中

    TIM2->DIER |= 1 << 2; // 允许捕获中断CC2IE=1
    TIM2->DIER |= 1 << 0; // 允许更新中断UIE=1
    TIM2->CR1 |= 0x01;    // 使能定时器2
    // 初始化TIM2输入捕获参数
    //    TIM2_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端 IC1映射到TI1上
    //    TIM2_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
    //    TIM2_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //映射到TI1上
    //    TIM2_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //配置输入分频,不分频
    //    TIM2_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 配置输入滤波器 不滤波
    //    TIM_ICInit(TIM2, &TIM2_ICInitStructure);

    //    //中断分组初始化
    //    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;  //TIM1中断
    //    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //先占优先级2级
    //    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
    //    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
    //    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

    //    TIM_ITConfig(TIM2,TIM_IT_Update|TIM_IT_CC2,ENABLE);//允许更新中断 ,允许CC1IE捕获中断

    //    TIM_Cmd(TIM2,ENABLE ); 	//使能定时器1
}
