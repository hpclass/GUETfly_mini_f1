#include "gy_86.h"
#include "stm32f10x.h"
/*硬件IIC*/
extern int16_t i2c_errors_count;
#if defined(USE_STM32_I2C1)
void I2C_GPIO_Config(void)
{

    RCC->APB1ENR |= 1 << 21; // 启用I2C1
    // RCC->APB1ENR |=1<<22;//启用I2C1
    RCC->APB2ENR |= 1 << 3;   // 启用GPIOB
    GPIOB->CRL &= 0X00FFFFFF; // PB6 PB7
    GPIOB->CRL |= 0XFF000000; // 复用开漏输出

    RCC->APB1RSTR |= 1 << 21;    // 复位I2C1
    RCC->APB1RSTR &= ~(1 << 21); // 复位结束I2C1

    // RCC->APB1RSTR  |= 1<<22;         //复位I2C2

    // I2C1 -> CR1 |=  1<<15;               //复位寄存器

    // I2C模块时钟频率,2～36MHz之间
    I2C1->CR2 |= 36; // 000000：禁用 000001：禁用 000010：2MHz ... 100100：36MHz

    I2C1->CCR |= 0 << 15; // I2C主模式  0：标准模式的I2C    1：快速模式的I2C
    I2C1->CCR |= 0 << 14; // 快速模式时的占空比 0 Tlow/Thigh = 2    1   Tlow/Thigh = 16/9

    // 得到200kHz频率
    I2C1->CCR |= 90 << 0; // 时钟控制分频系数  = PCLK1 /2/f    f 为想得到的频率
    // I2C1 -> CCR |= 45<<0;
    // 主模式最大上升时间
    // I2C1 -> TRISE |= 37;             //最大允许SCL上升时间为1000ns，故TRISE[5:0]中必须写入(1us/(1/36)us = 36+1)。
    I2C1->TRISE |= 10;
    I2C1->CR1 |= 1 << 10; // 打开ACK应答,在接收到一个字节后返回一个应答
    I2C1->CR1 |= 1 << 6;  // 广播呼叫使能

    I2C1->OAR1 |= 0 << 15; // 寻址模式   1 响应10位地址  0  响应7位地址

    I2C1->OAR1 |= 1 << 14; // 必须始终由软件保持为 1

    I2C1->OAR1 |= 0x7f << 1; // 设置接口地址的 7~1位

    // I2C1 -> OAR1 |=  0 ;           //设置10位地址模式时地址第0位
    // I2C1 -> OAR1 |= 0<<8;                //设置10位地址模式时地址第9~8位

    // I2C1 -> CR2 |=  1<<10;               //缓冲器中断使能
    I2C1->CR2 |= 1 << 9; // 事件中断使能
    I2C1->CR2 |= 1 << 8; // 出错中断使能

    I2C1->CR1 |= 1 << 0; // 开启I2C1
}
void i2c_Start()
{
    uint16_t i = 50;
    I2C1->CR1 |= 1 << 8; // I2C1产生起始条件
    while (!(I2C1->SR1 & 1 << 0) && i)
        i--;
    if (i == 0)
    {
        // iicerror
    }
    else
    {
    }
}

void i2c_stop()
{
    I2C1->CR1 |= 1 << 9; // I2C1产生停止条件
}

void i2c_write(u8 data)
{
    uint16_t i = 50;
    I2C1->DR = data;
    while (!(I2C1->SR1 & 1 << 1) && i) // 地址发送成功
        i--;
    if (i == 0)
    {
        // iicerror
    }
    else
    {
        i = I2C1->SR2;
    }
}

u8 i2c_readAck()
{
    uint16_t i = 50;
    I2C1->CR1 |= 1 << 10; // 打开ACK应答,在接收到一个字节后返回一个应答

    while (!(I2C1->SR1 & 1 << 6) && i) // 接收到数据标志位
    {
        i--;
    }

    return I2C1->DR;
}

u8 i2c_readNak()
{
    uint16_t i = 50;
    I2C1->CR1 &= ~(1 << 10);           // 关闭ACK应答,在接收到一个字节后返回一个应答
    while (!(I2C1->SR1 & 1 << 6) && i) // 接收到数据标志位
    {
        i--;
    }
    return I2C1->DR;
}
void I2c_End() // 关闭I2C
{
    I2C1->CR1 &= ~(1 << 0);
}
bool Single_WriteI2C(u8 Slave_Address, u8 REG_Address, u8 REG_data)
{
}
void I2C1_EV_IRQHandler()
{ // 事件中断
}

void I2C1_ER_IRQHandler()
{ // 错误中断
}

#else
/*******************************************
 模拟IIC
*****************************************/
void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

#if defined(GUET_FLY_V1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_GPIOD, ENABLE);
#if defined(EXTERN_IIC1) // 使用了外置IIC
    //    RCC->APB2ENR|=1<<2;     //使能PORTB时钟
    //    GPIOB->CRL&=0X00FFFFFF; //清零相应位
    //    GPIOB->CRL|=0X88000000; //上拉输入，PB6，PB7
    //    GPIOB->CRH&=0X000FFFFF; //清零相应位
    //    GPIOB->CRH|=0X71700000; //下拉输入，PB14，开漏输出13,15
    //    GPIOB->ODR= 0X000000C0;
    //		    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    //    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    //    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif

    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    //  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    //  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    //  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    //  GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_0 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    PDout(10) = 1;
#elif defined(GUET_FLY_MINI_V1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}
///////////IIC初始化//////////////

////////////粗略延时函数//////////
void Delay_1us(u16 n) // 约1us,1100k
{
    uint8_t i = 10; // i=10延时1.5us//这里可以优化速度 ，经测试最低到5还能写入
    while (i--)
        ;
}
void EX_Delay_1us(u16 n) // 约1us,1100k
{
    uint8_t i = 10; // i=10延时1.5us//这里可以优化速度 ，经测试最低到5还能写入
    while (i--)
        ;
}
////////IIC启动函数//////////
void I2C_Start(void)
{
    SDA_H;
    SCL_H;
    Delay_1us(1);
    if (!SDA_read)
        return; // SDA线为低电平则总线忙,退出
    SDA_L;
    Delay_1us(1);
    if (SDA_read)
        return; // SDA线为高电平则总线出错,退出
    SDA_L;
    Delay_1us(1);
    SCL_L;
    Delay_1us(1);
}
//**************************************
// IIC停止信号
//**************************************
void I2C_Stop(void)
{
    SDA_L;
    SCL_L;
    Delay_1us(1);
    SCL_H;
    Delay_1us(1);
    SDA_H;
    Delay_1us(1); // 延时
}
//**************************************
// IIC发送应答信号
// 入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(u8 i)
{
    if (1 == i)
        SDA_H; // 写应答信号
    else
        SDA_L;
    SCL_H;        // 拉高时钟线
    Delay_1us(1); // 延时
    SCL_L;        // 拉低时钟线
    Delay_1us(1);
}
//**************************************
// IIC等待应答
// 返回值：ack (1:ACK 0:NAK)
//**************************************
bool I2C_WaitAck(void) // 返回为:=1有ACK,=0无ACK
{
    unsigned int i;
    SDA_H;
    Delay_1us(1);
    SCL_H;
    Delay_1us(1);
    while (SDA_read)
    {
        i++;
        if (i >= 5000)
            break;
    }
    if (SDA_read)
    {
        SCL_L;
        Delay_1us(1);
        return false;
    }

    SCL_L;
    Delay_1us(1);
    return true;
}

//**************************************
// 向IIC总线发送一个字节数据
//**************************************
void I2C_SendByte(u8 dat)
{
    unsigned int i;
    //	unsigned char ack=1;

    SCL_L;
    Delay_1us(1);
    for (i = 0; i < 8; i++) // 8位计数器
    {
        if (dat & 0x80)
        {
            SDA_H; // 送数据口
        }
        else
            SDA_L;
        SCL_H;        // 拉高时钟线
        Delay_1us(1); // 延时
        SCL_L;        // 拉低时钟线
        Delay_1us(1); // 延时
        dat <<= 1;    // 移出数据的最高位
    }
}

//**************************************
// 从IIC总线接收一个字节数据
//**************************************
u8 I2C_RecvByte()
{
    u8 i;
    u8 dat = 0;
    SDA_H;                  // 使能内部上拉,准备读取数据,
    for (i = 0; i < 8; i++) // 8位计数器
    {

        dat <<= 1;
        SCL_H;        // 拉高时钟线
        Delay_1us(1); // 延时
        if (SDA_read) // 读数据
        {
            dat |= 0x01;
        }
        SCL_L; // 拉低时钟线
        Delay_1us(1);
    }

    return dat;
}
//**************************************
// 向IIC设备写入一个字节数据
//**************************************
bool Single_WriteI2C(u8 Slave_Address, u8 REG_Address, u8 REG_data)
{
    I2C_Start(); // 起始信号

    I2C_SendByte(Slave_Address); // 发送设备地址+写信号
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_Address); // 内部寄存器地址，
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_data); // 内部寄存器数据，
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    I2C_Stop(); // 发送停止信号
    return true;
}
//**************************************
// 从IIC设备读取一个字节数据
//**************************************
u8 Single_ReadI2C(u8 Slave_Address, u8 REG_Address)
{
    u8 REG_data;
    I2C_Start(); // 起始信号

    I2C_SendByte(Slave_Address); // 发送设备地址+写信号
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_Address); // 发送存储单元地址，从0开始
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    I2C_Start(); // 起始信号

    I2C_SendByte(Slave_Address + 1); // 发送设备地址+读信号
    if (!I2C_WaitAck())
    {
        i2c_errors_count++;
        I2C_Stop();
        return false;
    }

    REG_data = I2C_RecvByte(); // 读出寄存器数据

    I2C_SendACK(1); // 发送停止传输信号

    I2C_Stop(); // 停止信号
    return REG_data;
}

#if defined(EXTERN_IIC1) // 使用了外置IIC

#endif
#endif
