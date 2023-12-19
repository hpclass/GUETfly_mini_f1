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
    uint8_t i=10; //i=10��ʱ1.5us//��������Ż��ٶ� ����������͵�5����д��
    while(i--);

}
/**************************************************************************
��ʱ
ms����ʱ�ĺ�����
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
    if(!SDAread)return TWI_BUS_BUSY; //SDA��Ϊ�͵�ƽ������æ,�˳�
    SDAL;
    TWI_delay();
    if(SDAread) return TWI_BUS_ERROR; //SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
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
uint8_t TWI_WaitAck(void)   //����Ϊ:=1��ACK,=0��ACK
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
// return 1;��






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
void TWI_SendByte(uint8_t SendByte) //���ݴӸ�λ����λ//
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


uint8_t TWI_ReceiveByte(void)  //���ݴӸ�λ����λ//
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
//���أ�3д��ɹ���0д������ַ����1����æ��2����
//д��1�ֽ�����           SendByte����д������    WriteAddress����д���ַ
uint8_t TWI_WriteByte(uint8_t WriteAddress,uint8_t SendByte)
{
    uint8_t i;
    uint16_t j;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte( ADDR_24CXX & 0xFE);//д������ַ  д�룺��ַ���λ��0����ȡ����ַ���λ��1

    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(WriteAddress);   //������ʼ��ַ
    TWI_WaitAck();
    TWI_SendByte(SendByte);           //д����
    TWI_WaitAck();
    TWI_Stop();
//ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
    //delay_ms(12); //д����ʱ 12ms  д���ڴ���10ms����
    return 3;
}




//���أ�0д������ַ����1����æ��2����,
//����1�ֽ�����
//ReadAddress����������ַ
uint8_t TWI_ReadByte( uint8_t ReadAddress)
{
    uint8_t i,temp;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte((ADDR_24CXX & 0xFE));//д������ַ����ִ��һ��αд����
    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(ReadAddress);   //������ʼ��ַ
    TWI_WaitAck();
    TWI_Start();
    TWI_SendByte((ADDR_24CXX & 0xFE)|0x01);    //��������ַ    д�룺��ַ���λ��0����ȡ����ַ���λ��1
    TWI_WaitAck();

    //*pDat = TWI_ReceiveByte();
    temp = TWI_ReceiveByte();

    TWI_NoAck();

    TWI_Stop();
    return temp;//���ص������0��1��2������������ͬ�ˣ��ٿ���һ��
}


u8 Single_WriteI2C2(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
    uint8_t i,temp;
    i = TWI_Start();
    if(i)
        return 0;              //��ʼ�ź�

    TWI_SendByte(Slave_Address);   //�����豸��ַ+д�ź�
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_Stop();                   //����ֹͣ�ź�
    return 1;
}
//**************************************
//��IIC�豸��ȡһ���ֽ�����
//**************************************
u8 Single_ReadI2C2(u8 Slave_Address,u8 REG_Address)
{
    u8 REG_data;
    TWI_Start();                   //��ʼ�ź�

    TWI_SendByte(Slave_Address);    //�����豸��ַ+д�ź�
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    TWI_Start();                   //��ʼ�ź�

    TWI_SendByte(Slave_Address+1);  //�����豸��ַ+���ź�
    if(!TWI_WaitAck()) {
        TWI_Stop();
        return 0;
    }

    REG_data=TWI_ReceiveByte();       //�����Ĵ�������

    TWI_NoAck();                //����ֹͣ�����ź�

    TWI_Stop();                    //ֹͣ�ź�
    return REG_data;
}

/***************************************************************************
��24c256��д����ֽ�
psrc_data��ָ��Ҫд�����������ָ��
adr��24c256��Ҫд�����ݵ��׵�ַ
nbyte��д����ֽ���
����ֵ:  0��ִ����ϣ�1��ִ�г��ִ���
�β��У�C02ֻ��һ����ַadr��C256���и�λ��ַhadr�͵�λ��ַladr
***************************************************************************/
uint8_t IIC_EE_BufferWrite(uint8_t *psrc_data,uint8_t adr,uint8_t nbyte)
{
    uint8_t i;

    for(; nbyte!=0; nbyte--)
    {
        i = TWI_Start();
        if(i)
            return i;

        TWI_SendByte( ADDR_24CXX & 0xFE);//д������ַ

        if(!TWI_WaitAck())
        {
            TWI_Stop();
            return 0;
        }

        TWI_SendByte(adr);   //������ʼ��ַ
        TWI_WaitAck();
        TWI_SendByte(*psrc_data);           //д����
        TWI_WaitAck();
        psrc_data++;    //ָ���д���ݵ�ָ���1
        adr++;    //��24C08�Ĳ�����ַ��1
        TWI_Stop();
        //ע�⣺��Ϊ����Ҫ�ȴ�EEPROMд�꣬���Բ��ò�ѯ����ʱ��ʽ(10ms)
        delay_ms(12); //д����ʱ 12ms  д���ڴ���10ms����

    }
    return 0;
}


/***************************************************************************
��24c02������ֽ�
pdin_data��ָ��Ҫ����������ݵ������ָ��
adr��24c02��Ҫ�������ݵ��׵�ַ
nbyte���������ֽ���
����ֵ:  0��ִ����ϣ�1��ִ�г��ִ���
***************************************************************************/
uint8_t IIC_EE_BufferRead(uint8_t *pdin_data,uint8_t adr,uint8_t nbyte)
{
    uint8_t i;
    i = TWI_Start();
    if(i)
        return i;

    TWI_SendByte((ADDR_24CXX & 0xFE));//д������ַ����ִ��һ��αд����
    if(!TWI_WaitAck())
    {
        TWI_Stop();
        return 0;
    }

    TWI_SendByte(adr);   //������ʼ��ַ
    TWI_WaitAck();
    TWI_Start();
    TWI_SendByte((ADDR_24CXX & 0xFE)|0x01);    //��������ַ    д�룺��ַ���λ��0����ȡ����ַ���λ��1
    TWI_WaitAck();

    while(nbyte!=1)                 //����ǰ(nbyte-1)���ֽ�
    {
        *pdin_data = TWI_ReceiveByte(); //ѭ����24C02�ж����ݣ�����pdin_data��ָ�Ĵ洢����
        TWI_Ack();   //IICӦ��
        pdin_data++;  //ָ��洢�������ݵĴ洢��ָ���1
        nbyte--;  //ʣ��Ҫ������ֽڼ�1
    };

    *pdin_data = TWI_ReceiveByte();  //�������һ���ֽ�
    TWI_NoAck();      //IIC��Ӧ�����

    TWI_Stop();

    return 0;
}
