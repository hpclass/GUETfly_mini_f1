#include <string.h>
#include "config.h"
#include "def.h"
#include "stm32f10x_flash.h"
#include "EEPROM.h"


#if !defined(USE_EX_EEPROM)//��ʹ������EEPROM��ʹ��STM32��flash
volatile FLASH_Status FLASHStatus;
ErrorStatus HSEStartUpStatus;

#define FLASH_SIZE 8
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif


#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (u32)(0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage
#if defined(GUET_FLY_V1)
#define STM32_FLASH_SIZE 512 	 		//��ѡSTM32��FLASH������С(��λΪK)
#else
#define STM32_FLASH_SIZE 64 	 		//0��ѡSTM32��FLASH������С(��λΪK)
#endif

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH����ʼ��ַ


#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else
#define STM_SECTOR_SIZE	2048
#endif


#define FLASH_EEPROM_BASS_ADDR STM32_FLASH_BASE+1024*(STM32_FLASH_SIZE-USE_FLASH_SIZE)  //ʹ�����1k
/*******************************************************************************
* Function Name  : Readflash
* Description    : �����ݣ���FLASH�ж�����Ҫ������
*
* Input          : None
* Output         : Data���Ҫȡ��������
* Return         : None
*******************************************************************************/


void Readflash(u32 *p,u8 start,u8 end) //
{
    int j = start;
    // FlashAddress = FLASH_WRITE_ADDR+4*start;
    //������
    u32 FlashAddress =STM32_FLASH_BASE+1024*(STM32_FLASH_SIZE-1);
    while(j<end)
    {
        *(p+j) = *(u32*)(FlashAddress+4*j);
        j++;
    }
}

//��ȡָ����ַ�İ���(16λ����)
//faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
//����ֵ:��Ӧ����.

u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}
//��ָ����ַ��ʼ����ָ�����ȵ�����
//ReadAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
    u16 i;
    u16 temp_data=0,loop_times_=0;
    u8 *temp_P=0;
    temp_P=(u8*)pBuffer;//ת����ַ
    if(NumToRead%2!=0)
        loop_times_=NumToRead/2+1;
    else
        loop_times_=NumToRead/2;
    if(ReadAddr%2!=0)//������ַ
    {
        //��ַ������

        ReadAddr-=1;//��ǰһ���ֽ�
        temp_data=STMFLASH_ReadHalfWord(ReadAddr);
        *temp_P=temp_data>>8;//ȡ�Ͱ�λ
        temp_P++;
        NumToRead--;
        loop_times_--;//��ȥһ��˫��λ��
        ReadAddr+=2;
        for(i=0; i<(loop_times_); i++)
        {
            temp_data=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
            *temp_P=temp_data;
            temp_P++;
            NumToRead--;
            if(NumToRead==0)return;
            *temp_P=temp_data>>8;
            temp_P++;
            NumToRead--;
            if(NumToRead==0)return;
            ReadAddr+=2;//ƫ��2���ֽ�.
        }
    } else {
        if(NumToRead%2==0) {
            for(i=0; i<(loop_times_); i++)//ż��������ʱ��16λ��ȡ
            {
                pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
                ReadAddr+=2;//ƫ��2���ֽ�.
            }
        } else { //�������밴8λ��ȡ
            for(i=0; i<(loop_times_); i++)//ż��������ʱ��16λ��ȡ
            {
                temp_data=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
                *temp_P=temp_data;
                temp_P++;
                NumToRead--;
                if(NumToRead==0)return;
                *temp_P=temp_data>>8;
                temp_P++;
                NumToRead--;
                if(NumToRead==0)return;
                ReadAddr+=2;//ƫ��2���ֽ�.
            }
        }

    }
//    for(i=0; i<(NumToRead); i++)
//    {
//        pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//��ȡ2���ֽ�.
//        ReadAddr+=2;//ƫ��2���ֽ�.
//    }
}
/*******************************************************************************
* Function Name  : Readflash
* Description    : �����ݣ���FLASH�ж�����Ҫ������
*
* Input          : None
* Output         : Data���Ҫȡ��������
* Return         : None
*******************************************************************************/

/*******************************************************************************
* Function Name  : Readflash
* Description    : �����ݣ���FLASH�ж�����Ҫ������
*
* Input          : None
* Output         : Data���Ҫȡ��������
* Return         : None
*******************************************************************************/
//������д��
//WriteAddr:��ʼ��ַ
//pBuffer:����ָ��
//NumToWrite:����(16λ)��
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 i;
    for(i=0; i<NumToWrite; i++)
    {
        FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr+=2;//��ַ����2.
    }
}
void STMFLASH_Write_b(u32 WriteAddr,u16 *pBuffer,int32_t NumToWrite)
{
    u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
    u32 secpos;	   //������ַ
    u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
    if(NumToWrite%2!=0)
        NumToWrite=NumToWrite/2+1;
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
    FLASH_Unlock();						//����
    offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
    secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С
    if(NumToWrite<=secremain)secremain=NumToWrite;//�����ڸ�������Χ
    while(1)//���ɹ��ϴ���Ƭ��
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
        for(i=0; i<secremain; i++) //У������
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����
        }
        if(i<secremain)//��Ҫ����
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
            for(i=0; i<secremain; i++) //����
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
        } else {
            STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
        }
        if(NumToWrite==secremain)break;//д�������
        else//д��δ����
        {
            secpos++;				//������ַ��1
            secoff=0;				//ƫ��λ��Ϊ0
            pBuffer+=secremain;  	//ָ��ƫ��
            WriteAddr+=secremain;	//д��ַƫ��
            NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
            if(NumToWrite>(STM_SECTOR_SIZE/2))
                secremain=STM_SECTOR_SIZE/2;//��һ����������д����
            else
                secremain=NumToWrite;//��һ����������д����
        }
    };
    FLASH_Lock();//����
}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,int32_t NumToWrite)
{
    u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//�����2K�ֽ�
    u32 secpos;	   //������ַ
    u16 secoff;	   //������ƫ�Ƶ�ַ(16λ�ּ���)
    u16 secremain; //������ʣ���ַ(16λ�ּ���)
    u16 i;
    u32 offaddr;   //ȥ��0X08000000��ĵ�ַ
    if(NumToWrite%2!=0)
        NumToWrite=NumToWrite/2+1;
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//�Ƿ���ַ
    FLASH_Unlock();						//����
    offaddr=WriteAddr-STM32_FLASH_BASE;		//ʵ��ƫ�Ƶ�ַ.
    secpos=offaddr/STM_SECTOR_SIZE;			//������ַ  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//����ʣ��ռ��С


    if(NumToWrite<=secremain)
        secremain=NumToWrite;//�����ڸ�������Χ,һ�������Ϳ������

    while(1)//���ɹ��ϴ���Ƭ��
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//������������������
        for(i=0; i<secremain; i++) //У������
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//��Ҫ����
        }
        if(i<secremain)//��Ҫ����
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//�����������
            for(i=0; i<secremain; i++) //����
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//д����������
        } else {
            STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//д�Ѿ������˵�,ֱ��д������ʣ������.
        }
        if(NumToWrite==secremain)break;//д�������
        else//д��δ����
        {
            secpos++;				//������ַ��1
            secoff=0;				//ƫ��λ��Ϊ0
            pBuffer+=secremain;  	//ָ��ƫ��
            WriteAddr+=secremain;	//д��ַƫ��
            NumToWrite-=secremain;	//�ֽ�(16λ)���ݼ�
            if(NumToWrite>(STM_SECTOR_SIZE/2))
                secremain=STM_SECTOR_SIZE/2;//��һ����������д����
            else
                secremain=NumToWrite;//��һ����������д����
        }
        //			delay_ms(100);
    };
    FLASH_Lock();//����
}
void eeprom_write_block (void *buf, void *addr, size_t n)
{
    u32 addr_=(u32)addr;
    if((u32)addr_%2!=0)
        addr_+=1;
    LED1_ON
    //delay_ms(500);
    STMFLASH_Write(FLASH_EEPROM_BASS_ADDR+(u32)addr_,(u16 *)buf,n);
    LED1_OFF
}
void eeprom_read_block (void *buf,void *addr, size_t n)//��ȡ��ָ����ַ��ʼ��ָ�����ȵ�EEPROM����
{
    u32 addr_=(u32)addr;
    if((u32)addr_%2!=0)//����ż����ַ
        addr_+=1;
    LED2_ON
    STMFLASH_Read(FLASH_EEPROM_BASS_ADDR+(u32)addr_,(u16 *)buf,n);//����flash
    LED2_OFF
}

#else
/*******************************************
 ģ��IIC
PA4 ->SDA
PA5 ->SCL
*****************************************/

#define EEPROM_ADDR
#define EEPROM_SCL_H         GPIOB->BSRR |= GPIO_Pin_12
#define EEPROM_SCL_L         GPIOB->BRR  |= GPIO_Pin_12

#define EEPROM_SDA_H         GPIOB->BSRR |= GPIO_Pin_13
#define EEPROM_SDA_L         GPIOB->BRR  |= GPIO_Pin_13

#define EEPROM_SCL_read      GPIOB->IDR  & GPIO_Pin_12
#define EEPROM_SDA_read      GPIOB->IDR  & GPIO_Pin_13
//��С����




void EEPROM_I2C_GPIO_Config(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    // Configure I2C1 pins: SCL and SDA
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
}
///////////IIC��ʼ��//////////////

////////////������ʱ����//////////
void EEPROM_Delay_1us(u16 n)//Լ1us,1100k
{
    uint8_t i=6; //i=10��ʱ1.5us//��������Ż��ٶ� ����������͵�5����д��
    while(n--) {
        i=6;
        while(i--);
    }

}
////////IIC��������//////////
void EEPROM_I2C_Start(void)
{
    EEPROM_SDA_H;
    EEPROM_SCL_H;
    EEPROM_Delay_1us(1);
    if(!EEPROM_SDA_read) return;//SDA��Ϊ�͵�ƽ������æ,�˳�
    EEPROM_SDA_L;
    EEPROM_Delay_1us(1);
    if(EEPROM_SDA_read) return;//SDA��Ϊ�ߵ�ƽ�����߳���,�˳�
    EEPROM_SDA_L;
    EEPROM_Delay_1us(1);
    EEPROM_SCL_L;
    EEPROM_Delay_1us(1);
}
//**************************************
//IICֹͣ�ź�
//**************************************
void EEPROM_I2C_Stop(void)
{
    EEPROM_SDA_L;
    EEPROM_SCL_L;
    EEPROM_Delay_1us(1);
    EEPROM_SCL_H;
    EEPROM_Delay_1us(1);
    EEPROM_SDA_H;
    EEPROM_Delay_1us(1);                 //��ʱ
}
//**************************************
//IIC����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void EEPROM_I2C_SendACK(u8 i)
{
    if(1==i)EEPROM_SDA_H;                  //дӦ���ź�
    else EEPROM_SDA_L;
    EEPROM_SCL_H;                    //����ʱ����
    EEPROM_Delay_1us(1);                 //��ʱ
    EEPROM_SCL_L ;                  //����ʱ����
    EEPROM_Delay_1us(1);
}
//**************************************
//IIC�ȴ�Ӧ��
//����ֵ��ack (1:ACK 0:NAK)
//**************************************
bool EEPROM_I2C_WaitAck(void) 	 //����Ϊ:=1��ACK,=0��ACK
{
    unsigned int i;
    EEPROM_SDA_H;
    EEPROM_Delay_1us(1);
    EEPROM_SCL_H;
    EEPROM_Delay_1us(1);
    while(EEPROM_SDA_read) {
        i++;
        if(i>=5000)break;
    }
    if(EEPROM_SDA_read)
    {   EEPROM_SCL_L;
        EEPROM_Delay_1us(1);
        return false;
    }

    EEPROM_SCL_L;
    EEPROM_Delay_1us(1);
    return true;
}

//**************************************
//��IIC���߷���һ���ֽ�����
//**************************************
void EEPROM_I2C_SendByte(u8 dat)
{
    unsigned int i;
//	unsigned char ack=1;

    EEPROM_SCL_L;
    for (i=0; i<8; i++)         //8λ������
    {
        if(dat&0x80) {
            EEPROM_SDA_H;   //�����ݿ�
        }
        else EEPROM_SDA_L;
        EEPROM_SCL_H;                //����ʱ����
        EEPROM_Delay_1us(1);             //��ʱ
        EEPROM_SCL_L;                //����ʱ����
        EEPROM_Delay_1us(1); 		  //��ʱ
        dat <<= 1;          //�Ƴ����ݵ����λ
    }
}

//**************************************
//��IIC���߽���һ���ֽ�����
//**************************************
u8 EEPROM_I2C_RecvByte()
{
    u8 i;
    u8 dat = 0;
    EEPROM_SDA_H;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {

        dat <<= 1;
        EEPROM_SCL_H;                //����ʱ����
        EEPROM_Delay_1us(1);            //��ʱ
        if(EEPROM_SDA_read) //������
        {
            dat |=0x01;
        }
        EEPROM_SCL_L;                //����ʱ����
        EEPROM_Delay_1us(1);
    }

    return dat;
}
//**************************************
//��IIC�豸д��һ���ֽ�����
//**************************************
bool EEPROM_Single_WriteI2C(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
    EEPROM_I2C_Start();              //��ʼ�ź�

    EEPROM_I2C_SendByte(Slave_Address);   //�����豸��ַ+д�ź�
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_Stop();                   //����ֹͣ�ź�
    return true;
}
//**************************************
//��IIC�豸��ȡһ���ֽ�����
//**************************************
u8 EEPROM_Single_ReadI2C(u8 Slave_Address,u8 REG_Address)
{
    u8 REG_data;
    EEPROM_I2C_Start();                   //��ʼ�ź�

    EEPROM_I2C_SendByte(Slave_Address);    //�����豸��ַ+д�ź�
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_Start();                   //��ʼ�ź�

    EEPROM_I2C_SendByte(Slave_Address+1);  //�����豸��ַ+���ź�
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    REG_data=EEPROM_I2C_RecvByte();       //�����Ĵ�������

    EEPROM_I2C_SendACK(1);                //����ֹͣ�����ź�

    EEPROM_I2C_Stop();                    //ֹͣ�ź�
    return REG_data;
}
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
    u8 temp=0;
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	   //����д����
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(ReadAddr>>8);//���͸ߵ�ַ
    } else {
        EEPROM_I2C_SendByte(0XA0+((ReadAddr/256)<<1));   //����������ַ0XA0,д����
    }
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(ReadAddr%256);   //���͵͵�ַ
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Start();
    EEPROM_I2C_SendByte(0XA1);           //�������ģʽ
    EEPROM_I2C_WaitAck();
    temp=EEPROM_I2C_RecvByte();
    EEPROM_I2C_Stop();//����һ��ֹͣ����
    return temp;
}
//��AT24CXXָ����ַд��һ������
//WriteAddr  :д�����ݵ�Ŀ�ĵ�ַ
//DataToWrite:Ҫд�������
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	    //����д����
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(WriteAddr>>8);//���͸ߵ�ַ
    } else {
        EEPROM_I2C_SendByte(0XA0+((WriteAddr/256)<<1));   //����������ַ0XA0,д����
    }
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(WriteAddr%256);   //���͵͵�ַ
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(DataToWrite);     //�����ֽ�
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Stop();//����һ��ֹͣ����
    EEPROM_Delay_1us(10);
}
uint8_t AT24CXX_Check(void)
{
    static uint8_t temp=0,i;
    temp=AT24CXX_ReadOneByte(E2END);//����ÿ�ο�����дAT24CXX
    if(temp==0XA3)return 0;
    else//�ų���һ�γ�ʼ�������
    {
        AT24CXX_WriteOneByte(E2END,0XA3);
        temp=AT24CXX_ReadOneByte(E2END);
        if(temp==0XA3)return 0;
    }
    return 1;
}
u8 AT24CXX_write_Page(u8 *buf, u16 addr, size_t n)
{
    //http://ww1.microchip.com/downloads/en/devicedoc/21081G.pdf
    //24C08��ҳ��дֻҪ2ms�ĵȴ�ʱ�䣬16�ֽ�һҳ
    uint16_t i,j,k,h,p1,p2,last_n;
    k=0;
    p2=0;
    p1=addr/E2PAGESIZE;//ǰ�澭����ҳ��
    h=addr%E2PAGESIZE;//���һҳ��ȱ
    if(h!=0)//������һҳ
    {
        p2=E2PAGESIZE-h;//����һҳ
        if(p2>n)
            p2=n;//����һҳ
        for(i=0; i<p2; i++) //��ǰ��ʣ���ҳд��
        {
            //AT24CXX_WriteOneByte(addr+i,*(buf++));
            //delay_ms(7);
            EEPROM_I2C_Start();
            if(E2END>2047)
            {
                EEPROM_I2C_SendByte(0XA0);	    //����д����
                EEPROM_I2C_WaitAck();
                EEPROM_I2C_SendByte(addr>>8);//���͸ߵ�ַ
            } else {
                EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //����������ַ0XA0,д����
            }
            EEPROM_I2C_WaitAck();
            EEPROM_I2C_SendByte(addr%256);   //���͵͵�ַ
            EEPROM_I2C_WaitAck();
            for(; h<E2PAGESIZE; h++)
            {
                EEPROM_I2C_SendByte(*(buf));//��������
                EEPROM_I2C_WaitAck();
                buf++;
            }
            EEPROM_I2C_Stop();//����һ��ֹͣ����
            delay_ms(5);
        }
    }
    last_n=n;
    n=n-p2;//д����ɼ�ȥ
    if(n>1000)
        n=n;
    addr=addr+p2;
//
//
//	i=n/16;//����ҳ��
//	j=n%16;//����ʣ���ֽ���
//
//	for(k=0;k<i;k++)//д���ҳ
//	{
//		EEPROM_I2C_Start();
//		EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));
//		EEPROM_I2C_WaitAck();
//		EEPROM_I2C_SendByte(addr%256);   //���͵͵�ַ
//		EEPROM_I2C_WaitAck();
//		for(h=0;h<16;h++)
//		{
//			EEPROM_I2C_SendByte(*(buf));//��������
//			EEPROM_I2C_WaitAck();
//			buf=buf+1;
//			addr++;
//		}
//		//buf=buf+16;
//		EEPROM_I2C_Stop();//����һ��ֹͣ����
//		delay_ms(10);
//	}

//	for(i=0;i<j;i++)//��ǰ��ʣ���ҳ���ֽ�д��
//	{
//		AT24CXX_WriteOneByte(addr++,*(buf++));
//		delay_ms(7);
//	}

    while(k<n)//д���ҳ
    {
        EEPROM_I2C_Start();
        if(E2END>2047)
        {
            EEPROM_I2C_SendByte(0XA0);	    //����д����
            EEPROM_I2C_WaitAck();
            EEPROM_I2C_SendByte(addr>>8);//���͸ߵ�ַ
        } else {
            EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //����������ַ0XA0,д����
        }
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(addr%256);   //���͵͵�ַ
        EEPROM_I2C_WaitAck();
        for(h=0; h<E2PAGESIZE; h++)
        {
            if(k>n)
                break;
            EEPROM_I2C_SendByte(*(buf));//��������
            EEPROM_I2C_WaitAck();
            buf=buf+1;
            addr++;
            k++;
        }
        EEPROM_I2C_Stop();//����һ��ֹͣ����
        delay_ms(5);//�����ֲ�д5ms����ʱ�䣬5ms֮�ڲ���Ӧ
    }

}
/*
������ȡ:
���Ϳ�ʼ�ź�
��������ѡ��+д�ź�
�����ڴ��ַ
���Ϳ�ʼ�ź�
��������ѡ��+���ź�
��ȡ����
����Ӧ���ź�
��ȡ��һ�ֽڵ�����
����Ӧ���ź�
......
��ȡ���һ�ֽڵ�����
������Ӧ���ź�
����ֹͣ�ź�
*/
void eeprom_Continuous_reading(u8 *buf,u16 addr, size_t n)//������ȡ
{
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	   //����д����
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(addr>>8);//���͸ߵ�ַ
    } else EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //����������ַ0XA0,д����

    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(addr%256);   //���͵͵�ַ
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Start();
    EEPROM_I2C_SendByte(0XA1);           //�������ģʽ
    EEPROM_I2C_WaitAck();
    while(--n)
    {
        *buf=EEPROM_I2C_RecvByte();
        EEPROM_I2C_SendACK(0);//ACK
        buf++;
    }
    *buf=EEPROM_I2C_RecvByte();
    EEPROM_I2C_SendACK(1);//ACK
    EEPROM_I2C_Stop();//����һ��ֹͣ����
    //EEPROM_Delay_1us(1);

}
//u8 temp_wp[4096]=0;
void eeprom_write_block (void *buf, void *addr, size_t n)
{
    
    AT24CXX_write_Page((u8*)buf,(u16)addr,n);
    

//		uint16_t i;
//	u8 *temp=(u8*)buf;
//		LED1_ON
//	for(i=0;i<=n;i++)
//	{
//		AT24CXX_WriteOneByte((u16)addr+i,*temp);
//		temp++;
//		delay_ms(7);
//	}
//		LED1_OFF

}
void eeprom_read_block (void *buf,void *addr, size_t n)//��ȡ��ָ����ַ��ʼ��ָ�����ȵ�EEPROM����
{



    
    eeprom_Continuous_reading((u8*)buf,(u16)addr,n);//������ȡ
    


//		uint16_t i;
//		u8 *temp=(u8*)buf;
//		LED2_ON
//		for(i=0;i<=n;i++)
//	{
//		*temp=(u8)AT24CXX_ReadOneByte((u16)addr+i);//���ֽڶ�ȡ
//		temp++;
//	}
//		LED2_OFF
}
#endif

