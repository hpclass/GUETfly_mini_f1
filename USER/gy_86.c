#include "GY_86.h"
#include "stm32f10x.h"
/*Ӳ��IIC*/
#if defined(USE_STM32_I2C1)
void I2C_GPIO_Config(void)
{

    RCC->APB1ENR |=1<<21;//����I2C1
    //RCC->APB1ENR |=1<<22;//����I2C1
    RCC->APB2ENR |=1<<3; //����GPIOB
    GPIOB->CRL&=0X00FFFFFF; //PB6 PB7
    GPIOB->CRL|=0XFF000000; //���ÿ�©���

    RCC->APB1RSTR  |= 1<<21;           //��λI2C1
    RCC->APB1RSTR  &= ~(1<<21);            //��λ����I2C1

    //RCC->APB1RSTR  |= 1<<22;         //��λI2C2

    //I2C1 -> CR1 |=  1<<15;               //��λ�Ĵ���

    //I2Cģ��ʱ��Ƶ��,2��36MHz֮��
    I2C1 -> CR2 |=   36 ;                //000000������ 000001������ 000010��2MHz ... 100100��36MHz


    I2C1 -> CCR |= 0<<15;              //I2C��ģʽ  0����׼ģʽ��I2C    1������ģʽ��I2C
    I2C1 -> CCR |= 0<<14;                //����ģʽʱ��ռ�ձ� 0 Tlow/Thigh = 2    1   Tlow/Thigh = 16/9

    //�õ�200kHzƵ��
    I2C1 -> CCR |= 90<<0;              //ʱ�ӿ��Ʒ�Ƶϵ��  = PCLK1 /2/f    f Ϊ��õ���Ƶ��
    //I2C1 -> CCR |= 45<<0;
    //��ģʽ�������ʱ��
    //I2C1 -> TRISE |= 37;             //�������SCL����ʱ��Ϊ1000ns����TRISE[5:0]�б���д��(1us/(1/36)us = 36+1)��
    I2C1 -> TRISE |= 10;
    I2C1 -> CR1 |=  1<<10;             //��ACKӦ��,�ڽ��յ�һ���ֽں󷵻�һ��Ӧ��
    I2C1 -> CR1 |= 1<<6;               //�㲥����ʹ��

    I2C1 -> OAR1 |= 0<<15;             //Ѱַģʽ   1 ��Ӧ10λ��ַ  0  ��Ӧ7λ��ַ

    I2C1 -> OAR1 |= 1<<14;             //����ʼ�����������Ϊ 1

    I2C1 -> OAR1 |=  0x7f <<1 ;            //���ýӿڵ�ַ�� 7~1λ

    //I2C1 -> OAR1 |=  0 ;           //����10λ��ַģʽʱ��ַ��0λ
    //I2C1 -> OAR1 |= 0<<8;                //����10λ��ַģʽʱ��ַ��9~8λ

    //I2C1 -> CR2 |=  1<<10;               //�������ж�ʹ��
    I2C1 -> CR2 |=  1<<9;              //�¼��ж�ʹ��
    I2C1 -> CR2 |=  1<<8;              //�����ж�ʹ��

    I2C1 -> CR1 |=   1<<0;             //����I2C1
}
void  i2c_Start()
{
    uint16_t i=50;
    I2C1 -> CR1 |=   1<<8;             //I2C1������ʼ����
    while(!(I2C1 -> SR1& 1<<0)&&i)
        i--;
    if(i==0)
    {
        //iicerror
    } else {

    }
}

void  i2c_stop()
{
    I2C1 -> CR1 |=   1<<9;             //I2C1����ֹͣ����
}


void  i2c_write(u8 data)
{
    uint16_t i=50;
    I2C1 -> DR = data;
    while(!(I2C1 -> SR1& 1<<1)&&i)//��ַ���ͳɹ�
        i--;
    if(i==0)
    {
        //iicerror
    } else {
        i = I2C1 -> SR2;
    }

}

u8  i2c_readAck()
{
    uint16_t i=50;
    I2C1 -> CR1 |=  1<<10;             //��ACKӦ��,�ڽ��յ�һ���ֽں󷵻�һ��Ӧ��

    while(!(I2C1 -> SR1 & 1<<6)&&i)      //���յ����ݱ�־λ
    {
        i--;
    }

    return I2C1 -> DR;
}

u8  i2c_readNak()
{
    uint16_t i=50;
    I2C1 -> CR1 &=  ~(1<<10);             //�ر�ACKӦ��,�ڽ��յ�һ���ֽں󷵻�һ��Ӧ��
    while(!(I2C1 -> SR1 & 1<<6)&&i)      //���յ����ݱ�־λ
    {
        i--;
    }
    return I2C1 -> DR;
}
void  I2c_End()                         //�ر�I2C
{
    I2C1 -> CR1 &=~(1<<0);
}
bool Single_WriteI2C(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{


}
void I2C1_EV_IRQHandler() {//�¼��ж�

}

void I2C1_ER_IRQHandler() {//�����ж�

}


#else
/*******************************************
 ģ��IIC
*****************************************/
void I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;

#if defined(GUET_FLY_V1)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOD, ENABLE);
#if defined(EXTERN_IIC1)//ʹ��������IIC
//    RCC->APB2ENR|=1<<2;     //ʹ��PORTBʱ��
//    GPIOB->CRL&=0X00FFFFFF; //������Ӧλ
//    GPIOB->CRL|=0X88000000; //�������룬PB6��PB7
//    GPIOB->CRH&=0X000FFFFF; //������Ӧλ
//    GPIOB->CRH|=0X71700000; //�������룬PB14����©���13,15
//    GPIOB->ODR= 0X000000C0;
//		    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
//    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
//    GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6| GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
#endif


    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
//  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
//  GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1|GPIO_Pin_0|GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);
    PDout(10)=1;
#else
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
#endif
}
///////////IIC��ʼ��//////////////

////////////������ʱ����//////////
void Delay_1us(u16 n)//Լ1us,1100k
{
    uint8_t i=10; //i=10��ʱ1.5us//��������Ż��ٶ� ����������͵�5����д��
    while(i--);
}
void EX_Delay_1us(u16 n)//Լ1us,1100k
{
    uint8_t i=10; //i=10��ʱ1.5us//��������Ż��ٶ� ����������͵�5����д��
    while(i--);
}
////////IIC��������//////////
void I2C_Start(void)
{
    SDA_H;
    SCL_H;
    Delay_1us(1);
    if(!SDA_read) return;//SDA��Ϊ�͵�ƽ������æ,�˳�
    SDA_L;
    Delay_1us(1);
    if(SDA_read) return;//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
    SDA_L;
    Delay_1us(1);
    SCL_L;
    Delay_1us(1);
}
//**************************************
//IICֹͣ�ź�
//**************************************
void I2C_Stop(void)
{
    SDA_L;
    SCL_L;
    Delay_1us(1);
    SCL_H;
    Delay_1us(1);
    SDA_H;
    Delay_1us(1);                 //��ʱ

}
//**************************************
//IIC����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(u8 i)
{
    if(1==i)SDA_H;                  //дӦ���ź�
    else SDA_L;
    SCL_H;                    //����ʱ����
    Delay_1us(1);                 //��ʱ
    SCL_L ;                  //����ʱ����
    Delay_1us(1);
}
//**************************************
//IIC�ȴ�Ӧ��
//����ֵ��ack (1:ACK 0:NAK)
//**************************************
bool I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
    unsigned int i;
    SDA_H;
    Delay_1us(1);
    SCL_H;
    Delay_1us(1);
    while(SDA_read) {
        i++;
        if(i>=5000)break;
    }
    if(SDA_read)
    {   SCL_L;
        Delay_1us(1);
        return false;
    }

    SCL_L;
    Delay_1us(1);
    return true;
}

//**************************************
//��IIC���߷���һ���ֽ�����
//**************************************
void I2C_SendByte(u8 dat)
{
    unsigned int i;
//	unsigned char ack=1;

    SCL_L;
		Delay_1us(1); 
    for (i=0; i<8; i++)         //8λ������
    {
        if(dat&0x80) {
            SDA_H;   //�����ݿ�
        }
        else SDA_L;
        SCL_H;                //����ʱ����
        Delay_1us(1);             //��ʱ
        SCL_L;                //����ʱ����
        Delay_1us(1); 		  //��ʱ
        dat <<= 1;          //�Ƴ����ݵ����λ
    }
}

//**************************************
//��IIC���߽���һ���ֽ�����
//**************************************
u8 I2C_RecvByte()
{
    u8 i;
    u8 dat = 0;
    SDA_H;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {

        dat <<= 1;
        SCL_H;                //����ʱ����
        Delay_1us(1);            //��ʱ
        if(SDA_read) //������
        {
            dat |=0x01;
        }
        SCL_L;                //����ʱ����
        Delay_1us(1);
    }

    return dat;
}
//**************************************
//��IIC�豸д��һ���ֽ�����
//**************************************
bool Single_WriteI2C(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
    I2C_Start();              //��ʼ�ź�

    I2C_SendByte(Slave_Address);   //�����豸��ַ+д�ź�
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    I2C_Stop();                   //����ֹͣ�ź�
    return true;
}
//**************************************
//��IIC�豸��ȡһ���ֽ�����
//**************************************
u8 Single_ReadI2C(u8 Slave_Address,u8 REG_Address)
{
    u8 REG_data;
    I2C_Start();                   //��ʼ�ź�

    I2C_SendByte(Slave_Address);    //�����豸��ַ+д�ź�
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    I2C_Start();                   //��ʼ�ź�

    I2C_SendByte(Slave_Address+1);  //�����豸��ַ+���ź�
    if(!I2C_WaitAck()) {
        I2C_Stop();
        return false;
    }

    REG_data=I2C_RecvByte();       //�����Ĵ�������

    I2C_SendACK(1);                //����ֹͣ�����ź�

    I2C_Stop();                    //ֹͣ�ź�
    return REG_data;
}

#if defined(EXTERN_IIC1)//ʹ��������IIC


#endif
#endif
