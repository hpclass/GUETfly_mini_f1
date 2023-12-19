#include "stm32f10x.h"
#include "math.h"
#include "delay.h"
#include "soft_iic.h"

extern u8 exchange_num[11];



//void IIC_EE_Init(void)
//{
//
//  GPIO_InitTypeDef  GPIO_InitStructure;
//
//  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
//
//  // Configure IIC1 pins: SCL and SDA
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7|GPIO_Pin_14;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);
//
//}

void IIC_EE_Init(void)
{

    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure IIC1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

}
void TWI_delay()
{
    uint8_t i=10; //i=10延时1.5us//这里可以优化速度 ，经测试最低到5还能写入
    while(i--);

}
/**************************************************************************
延时
ms：延时的毫秒数
CYCLECOUNTER / 72000000
***************************************************************************/
/*
void DelayMs(uint16_t ms)
{
 uint16_t iq0;
        uint16_t iq1;
 for(iq0 = ms; iq0 > 0; iq0--)
 {
  for(iq1 = 11998; iq1 > 0; iq1--); // ( (6*iq1+9)*iq0+15 ) / 72000000

 }
}
*/
u8 TWI_Start()
{
    SDAH;
    SCLH;
    TWI_delay();
    if(!SDAread)return TWI_BUS_BUSY; //SDA线为低电平则总线忙,退出
    SDAL;
    TWI_delay();
    if(SDAread) return TWI_BUS_ERROR; //SDA线为高电平则总线出错,退出
    SCLL;
    TWI_delay();
    return TWI_READY;
}
void TWI_Stop(void)
{
    SDAL;
    SCLL;
    TWI_delay();
    SCLH;
    TWI_delay();
    SDAH;
    TWI_delay();
}

void TWI_Ack(void)
{
    SCLL;
    TWI_delay();
    SDAL;
    TWI_delay();
    SCLH;
    TWI_delay();
    SCLL;
    TWI_delay();
}
void TWI_NoAck(void)
{
    SCLL;
    TWI_delay();
    SDAH;
    TWI_delay();
    SCLH;
    TWI_delay();
    SCLL;
    TWI_delay();
}
uint8_t TWI_WaitAck(void)   //返回为:=1有ACK,=0无ACK
{
// SCLL;
// TWI_delay();
// SDAH;
// TWI_delay();
// SCLH;
// TWI_delay();
// if(SDAread)
// {
//   SCLL;
//   return 0;
// }
// SCLL;
// return 1;、






    u16 i=0;
    SDAH;
    TWI_delay();
    SCLH;
    TWI_delay();
    while(SDAread) {
        i++;
        if(i>=5000)break;
    }
    if(SDAread)
    {
        SCLL;
        TWI_delay();
        return 0;

    }
    SCLL;
    TWI_delay();
    return 1;


}
void TWI_SendByte(uint8_t SendByte) //数据从高位到低位//
{
    uint8_t i=8;
    while(i--)
    {
        SCLL;
        TWI_delay();
        if(SendByte&0x80)
            SDAH;
        else
            SDAL;
        SendByte<<=1;
        TWI_delay();
        SCLH;
        TWI_delay();
    }
    SCLL;
    SDAL;
    TWI_delay();
}


uint8_t TWI_ReceiveByte(void)  //数据从高位到低位//
{
    uint8_t i=8;
    uint8_t ReceiveByte=0;

    SDAH;
    while(i--)
    {
        ReceiveByte <<= 1;
        SCLL;
        TWI_delay();
        SCLH;
        TWI_delay();
        if(SDAread)
        {
            ReceiveByte |= 0x01;
        }
    }
    SCLL;
    return ReceiveByte;
}
//返回：3写入成功；0写器件地址出错，1总线忙，2出错
//写入1字节数据           SendByte：待写入数据    WriteAddress：待写入地址
uint8_t TWI_WriteByte(uint8_t WriteAddress,uint8_t SendByte)
{
    uint8_t i;
    uint16_t j;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte( ADDR_24CXX & 0xFE);//写器件地址  写入：地址最低位是0，读取：地址最低位是1

    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(WriteAddress);   //设置起始地址
    TWI_WaitAck();
    TWI_SendByte(SendByte);           //写数据
    TWI_WaitAck();
    TWI_Stop();
//注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
    //delay_ms(12); //写入延时 12ms  写周期大于10ms即可
    return 3;
}




//返回：0写器件地址出错，1总线忙，2出错,
//读出1字节数据
//ReadAddress：待读出地址
uint8_t TWI_ReadByte( uint8_t ReadAddress)
{
    uint8_t i,temp;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte((ADDR_24CXX & 0xFE));//写器件地址，先执行一次伪写操作
    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(ReadAddress);   //设置起始地址
    TWI_WaitAck();
    TWI_Start();
    TWI_SendByte((ADDR_24CXX & 0xFE)|0x01);    //读器件地址    写入：地址最低位是0，读取：地址最低位是1
    TWI_WaitAck();

    //*pDat = TWI_ReceiveByte();
    temp = TWI_ReceiveByte();

    TWI_NoAck();

    TWI_Stop();
    return temp;//返回的如果是0，1，2则与错误代码相同了，再考虑一下
}


u8 Single_WriteI2C2(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
    uint8_t i,temp;
    i = TWI_Start();
    if(i)
        return 0;              //起始信号

    TWI_SendByte(Slave_Address);   //发送设备地址+写信号
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_Address);    //内部寄存器地址，
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_data);       //内部寄存器数据，
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_Stop();                   //发送停止信号
    return 1;
}
//**************************************
//从IIC设备读取一个字节数据
//**************************************
u8 Single_ReadI2C2(u8 Slave_Address,u8 REG_Address)
{
    u8 REG_data;
    TWI_Start();                   //起始信号

    TWI_SendByte(Slave_Address);    //发送设备地址+写信号
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_Address);     //发送存储单元地址，从0开始
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_Start();                   //起始信号

    TWI_SendByte(Slave_Address+1);  //发送设备地址+读信号
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    REG_data=TWI_ReceiveByte();       //读出寄存器数据

    TWI_NoAck();                //发送停止传输信号

    TWI_Stop();                    //停止信号
    return REG_data;
}

/***************************************************************************
向24c256中写多个字节
psrc_data：指向要写入数据数组的指针
adr：24c256中要写入数据的首地址
nbyte：写入的字节数
返回值:  0：执行完毕；1：执行出现错误
形参中：C02只有一个地址adr；C256中有高位地址hadr和低位地址ladr
***************************************************************************/
uint8_t IIC_EE_BufferWrite(uint8_t *psrc_data,uint8_t adr,uint8_t nbyte)
{
    uint8_t i;

    for(; nbyte!=0; nbyte--)
    {
        i = TWI_Start();
        if(i)
            return i;

        TWI_SendByte( ADDR_24CXX & 0xFE);//写器件地址

        if(!TWI_WaitAck())
        {
            TWI_Stop();
            return 0;
        }

        TWI_SendByte(adr);   //设置起始地址
        TWI_WaitAck();
        TWI_SendByte(*psrc_data);           //写数据
        TWI_WaitAck();
        psrc_data++;    //指向待写数据的指针加1
        adr++;    //对24C08的操作地址加1
        TWI_Stop();
        //注意：因为这里要等待EEPROM写完，可以采用查询或延时方式(10ms)
        delay_ms(12); //写入延时 12ms  写周期大于10ms即可

    }
    return 0;
}


/***************************************************************************
从24c02读多个字节
pdin_data：指向要保存读出数据的数组的指针
adr：24c02中要读出数据的首地址
nbyte：读出的字节数
返回值:  0：执行完毕；1：执行出现错误
***************************************************************************/
uint8_t IIC_EE_BufferRead(uint8_t *pdin_data,uint8_t adr,uint8_t nbyte)
{
    uint8_t i;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte((ADDR_24CXX & 0xFE));//写器件地址，先执行一次伪写操作
    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(adr);   //设置起始地址
    TWI_WaitAck();
    TWI_Start();
    TWI_SendByte((ADDR_24CXX & 0xFE)|0x01);    //读器件地址    写入：地址最低位是0，读取：地址最低位是1
    TWI_WaitAck();

    while(nbyte!=1)                 //读入前(nbyte-1)个字节
    {
        *pdin_data = TWI_ReceiveByte(); //循环从24C02中读数据，存入pdin_data所指的存储器中
        TWI_Ack();   //IIC应答
        pdin_data++;  //指向存储读入数据的存储器指针加1
        nbyte--;  //剩余要读入的字节减1
    };

    *pdin_data = TWI_ReceiveByte();  //读入最后一个字节
    TWI_NoAck();      //IIC无应答操作

    TWI_Stop();

    return 0;
}
