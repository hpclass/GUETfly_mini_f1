#include <string.h>
#include "config.h"
#include "def.h"
#include "stm32f10x_flash.h"
#include "EEPROM.h"


#if !defined(USE_EX_EEPROM)//不使用外置EEPROM而使用STM32的flash
volatile FLASH_Status FLASHStatus;
ErrorStatus HSEStartUpStatus;

#define FLASH_SIZE 8
#ifndef FLASH_PAGE_COUNT
#define FLASH_PAGE_COUNT 128
#endif


#define FLASH_PAGE_SIZE                 ((uint16_t)0x400)
#define FLASH_WRITE_ADDR                (u32)(0x08000000 + (uint32_t)FLASH_PAGE_SIZE * (FLASH_PAGE_COUNT - 1))       // use the last KB for storage
#if defined(GUET_FLY_V1)
#define STM32_FLASH_SIZE 512 	 		//所选STM32的FLASH容量大小(单位为K)
#else
#define STM32_FLASH_SIZE 64 	 		//0所选STM32的FLASH容量大小(单位为K)
#endif

#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH的起始地址


#if STM32_FLASH_SIZE<256
#define STM_SECTOR_SIZE 1024 //字节
#else
#define STM_SECTOR_SIZE	2048
#endif


#define FLASH_EEPROM_BASS_ADDR STM32_FLASH_BASE+1024*(STM32_FLASH_SIZE-USE_FLASH_SIZE)  //使用最后1k
/*******************************************************************************
* Function Name  : Readflash
* Description    : 读数据，从FLASH中读出需要的数据
*
* Input          : None
* Output         : Data输出要取出的数据
* Return         : None
*******************************************************************************/


void Readflash(u32 *p,u8 start,u8 end) //
{
    int j = start;
    // FlashAddress = FLASH_WRITE_ADDR+4*start;
    //读数据
    u32 FlashAddress =STM32_FLASH_BASE+1024*(STM32_FLASH_SIZE-1);
    while(j<end)
    {
        *(p+j) = *(u32*)(FlashAddress+4*j);
        j++;
    }
}

//读取指定地址的半字(16位数据)
//faddr:读地址(此地址必须为2的倍数!!)
//返回值:对应数据.

u16 STMFLASH_ReadHalfWord(u32 faddr)
{
    return *(vu16*)faddr;
}
//从指定地址开始读出指定长度的数据
//ReadAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Read(u32 ReadAddr,u16 *pBuffer,u16 NumToRead)
{
    u16 i;
    u16 temp_data=0,loop_times_=0;
    u8 *temp_P=0;
    temp_P=(u8*)pBuffer;//转换地址
    if(NumToRead%2!=0)
        loop_times_=NumToRead/2+1;
    else
        loop_times_=NumToRead/2;
    if(ReadAddr%2!=0)//奇数地址
    {
        //地址是奇数

        ReadAddr-=1;//向前一个字节
        temp_data=STMFLASH_ReadHalfWord(ReadAddr);
        *temp_P=temp_data>>8;//取低八位
        temp_P++;
        NumToRead--;
        loop_times_--;//减去一个双字位置
        ReadAddr+=2;
        for(i=0; i<(loop_times_); i++)
        {
            temp_data=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
            *temp_P=temp_data;
            temp_P++;
            NumToRead--;
            if(NumToRead==0)return;
            *temp_P=temp_data>>8;
            temp_P++;
            NumToRead--;
            if(NumToRead==0)return;
            ReadAddr+=2;//偏移2个字节.
        }
    } else {
        if(NumToRead%2==0) {
            for(i=0; i<(loop_times_); i++)//偶数，对齐时按16位读取
            {
                pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
                ReadAddr+=2;//偏移2个字节.
            }
        } else { //奇数对齐按8位读取
            for(i=0; i<(loop_times_); i++)//偶数，对齐时按16位读取
            {
                temp_data=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
                *temp_P=temp_data;
                temp_P++;
                NumToRead--;
                if(NumToRead==0)return;
                *temp_P=temp_data>>8;
                temp_P++;
                NumToRead--;
                if(NumToRead==0)return;
                ReadAddr+=2;//偏移2个字节.
            }
        }

    }
//    for(i=0; i<(NumToRead); i++)
//    {
//        pBuffer[i]=STMFLASH_ReadHalfWord(ReadAddr);//读取2个字节.
//        ReadAddr+=2;//偏移2个字节.
//    }
}
/*******************************************************************************
* Function Name  : Readflash
* Description    : 读数据，从FLASH中读出需要的数据
*
* Input          : None
* Output         : Data输出要取出的数据
* Return         : None
*******************************************************************************/

/*******************************************************************************
* Function Name  : Readflash
* Description    : 读数据，从FLASH中读出需要的数据
*
* Input          : None
* Output         : Data输出要取出的数据
* Return         : None
*******************************************************************************/
//不检查的写入
//WriteAddr:起始地址
//pBuffer:数据指针
//NumToWrite:半字(16位)数
void STMFLASH_Write_NoCheck(u32 WriteAddr,u16 *pBuffer,u16 NumToWrite)
{
    u16 i;
    for(i=0; i<NumToWrite; i++)
    {
        FLASH_ProgramHalfWord(WriteAddr,pBuffer[i]);
        WriteAddr+=2;//地址增加2.
    }
}
void STMFLASH_Write_b(u32 WriteAddr,u16 *pBuffer,int32_t NumToWrite)
{
    u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
    u32 secpos;	   //扇区地址
    u16 secoff;	   //扇区内偏移地址(16位字计算)
    u16 secremain; //扇区内剩余地址(16位字计算)
    u16 i;
    u32 offaddr;   //去掉0X08000000后的地址
    if(NumToWrite%2!=0)
        NumToWrite=NumToWrite/2+1;
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
    FLASH_Unlock();						//解锁
    offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
    secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小
    if(NumToWrite<=secremain)secremain=NumToWrite;//不大于该扇区范围
    while(1)//可疑故障代码片段
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
        for(i=0; i<secremain; i++) //校验数据
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除
        }
        if(i<secremain)//需要擦除
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
            for(i=0; i<secremain; i++) //复制
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区
        } else {
            STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间.
        }
        if(NumToWrite==secremain)break;//写入结束了
        else//写入未结束
        {
            secpos++;				//扇区地址增1
            secoff=0;				//偏移位置为0
            pBuffer+=secremain;  	//指针偏移
            WriteAddr+=secremain;	//写地址偏移
            NumToWrite-=secremain;	//字节(16位)数递减
            if(NumToWrite>(STM_SECTOR_SIZE/2))
                secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
            else
                secremain=NumToWrite;//下一个扇区可以写完了
        }
    };
    FLASH_Lock();//上锁
}

void STMFLASH_Write(u32 WriteAddr,u16 *pBuffer,int32_t NumToWrite)
{
    u16 STMFLASH_BUF[STM_SECTOR_SIZE/2];//最多是2K字节
    u32 secpos;	   //扇区地址
    u16 secoff;	   //扇区内偏移地址(16位字计算)
    u16 secremain; //扇区内剩余地址(16位字计算)
    u16 i;
    u32 offaddr;   //去掉0X08000000后的地址
    if(NumToWrite%2!=0)
        NumToWrite=NumToWrite/2+1;
    if(WriteAddr<STM32_FLASH_BASE||(WriteAddr>=(STM32_FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//非法地址
    FLASH_Unlock();						//解锁
    offaddr=WriteAddr-STM32_FLASH_BASE;		//实际偏移地址.
    secpos=offaddr/STM_SECTOR_SIZE;			//扇区地址  0~127 for STM32F103RBT6
    secoff=(offaddr%STM_SECTOR_SIZE)/2;		//在扇区内的偏移(2个字节为基本单位.)
    secremain=STM_SECTOR_SIZE/2-secoff;		//扇区剩余空间大小


    if(NumToWrite<=secremain)
        secremain=NumToWrite;//不大于该扇区范围,一个扇区就可以完成

    while(1)//可疑故障代码片段
    {
        STMFLASH_Read(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//读出整个扇区的内容
        for(i=0; i<secremain; i++) //校验数据
        {
            if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//需要擦除
        }
        if(i<secremain)//需要擦除
        {
            FLASH_ErasePage(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE);//擦除这个扇区
            for(i=0; i<secremain; i++) //复制
            {
                STMFLASH_BUF[i+secoff]=pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos*STM_SECTOR_SIZE+STM32_FLASH_BASE,STMFLASH_BUF,STM_SECTOR_SIZE/2);//写入整个扇区
        } else {
            STMFLASH_Write_NoCheck(WriteAddr,pBuffer,secremain);//写已经擦除了的,直接写入扇区剩余区间.
        }
        if(NumToWrite==secremain)break;//写入结束了
        else//写入未结束
        {
            secpos++;				//扇区地址增1
            secoff=0;				//偏移位置为0
            pBuffer+=secremain;  	//指针偏移
            WriteAddr+=secremain;	//写地址偏移
            NumToWrite-=secremain;	//字节(16位)数递减
            if(NumToWrite>(STM_SECTOR_SIZE/2))
                secremain=STM_SECTOR_SIZE/2;//下一个扇区还是写不完
            else
                secremain=NumToWrite;//下一个扇区可以写完了
        }
        //			delay_ms(100);
    };
    FLASH_Lock();//上锁
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
void eeprom_read_block (void *buf,void *addr, size_t n)//读取由指定地址开始的指定长度的EEPROM数据
{
    u32 addr_=(u32)addr;
    if((u32)addr_%2!=0)//补齐偶数地址
        addr_+=1;
    LED2_ON
    STMFLASH_Read(FLASH_EEPROM_BASS_ADDR+(u32)addr_,(u16 *)buf,n);//读出flash
    LED2_OFF
}

#else
/*******************************************
 模拟IIC
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
//大小定义




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
///////////IIC初始化//////////////

////////////粗略延时函数//////////
void EEPROM_Delay_1us(u16 n)//约1us,1100k
{
    uint8_t i=6; //i=10延时1.5us//这里可以优化速度 ，经测试最低到5还能写入
    while(n--) {
        i=6;
        while(i--);
    }

}
////////IIC启动函数//////////
void EEPROM_I2C_Start(void)
{
    EEPROM_SDA_H;
    EEPROM_SCL_H;
    EEPROM_Delay_1us(1);
    if(!EEPROM_SDA_read) return;//SDA线为低电平则总线忙,退出
    EEPROM_SDA_L;
    EEPROM_Delay_1us(1);
    if(EEPROM_SDA_read) return;//SDA线为高电平则总线出错,退出
    EEPROM_SDA_L;
    EEPROM_Delay_1us(1);
    EEPROM_SCL_L;
    EEPROM_Delay_1us(1);
}
//**************************************
//IIC停止信号
//**************************************
void EEPROM_I2C_Stop(void)
{
    EEPROM_SDA_L;
    EEPROM_SCL_L;
    EEPROM_Delay_1us(1);
    EEPROM_SCL_H;
    EEPROM_Delay_1us(1);
    EEPROM_SDA_H;
    EEPROM_Delay_1us(1);                 //延时
}
//**************************************
//IIC发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void EEPROM_I2C_SendACK(u8 i)
{
    if(1==i)EEPROM_SDA_H;                  //写应答信号
    else EEPROM_SDA_L;
    EEPROM_SCL_H;                    //拉高时钟线
    EEPROM_Delay_1us(1);                 //延时
    EEPROM_SCL_L ;                  //拉低时钟线
    EEPROM_Delay_1us(1);
}
//**************************************
//IIC等待应答
//返回值：ack (1:ACK 0:NAK)
//**************************************
bool EEPROM_I2C_WaitAck(void) 	 //返回为:=1有ACK,=0无ACK
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
//向IIC总线发送一个字节数据
//**************************************
void EEPROM_I2C_SendByte(u8 dat)
{
    unsigned int i;
//	unsigned char ack=1;

    EEPROM_SCL_L;
    for (i=0; i<8; i++)         //8位计数器
    {
        if(dat&0x80) {
            EEPROM_SDA_H;   //送数据口
        }
        else EEPROM_SDA_L;
        EEPROM_SCL_H;                //拉高时钟线
        EEPROM_Delay_1us(1);             //延时
        EEPROM_SCL_L;                //拉低时钟线
        EEPROM_Delay_1us(1); 		  //延时
        dat <<= 1;          //移出数据的最高位
    }
}

//**************************************
//从IIC总线接收一个字节数据
//**************************************
u8 EEPROM_I2C_RecvByte()
{
    u8 i;
    u8 dat = 0;
    EEPROM_SDA_H;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {

        dat <<= 1;
        EEPROM_SCL_H;                //拉高时钟线
        EEPROM_Delay_1us(1);            //延时
        if(EEPROM_SDA_read) //读数据
        {
            dat |=0x01;
        }
        EEPROM_SCL_L;                //拉低时钟线
        EEPROM_Delay_1us(1);
    }

    return dat;
}
//**************************************
//向IIC设备写入一个字节数据
//**************************************
bool EEPROM_Single_WriteI2C(u8 Slave_Address,u8 REG_Address,u8 REG_data)
{
    EEPROM_I2C_Start();              //起始信号

    EEPROM_I2C_SendByte(Slave_Address);   //发送设备地址+写信号
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_Address);    //内部寄存器地址，
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_data);       //内部寄存器数据，
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_Stop();                   //发送停止信号
    return true;
}
//**************************************
//从IIC设备读取一个字节数据
//**************************************
u8 EEPROM_Single_ReadI2C(u8 Slave_Address,u8 REG_Address)
{
    u8 REG_data;
    EEPROM_I2C_Start();                   //起始信号

    EEPROM_I2C_SendByte(Slave_Address);    //发送设备地址+写信号
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    EEPROM_I2C_Start();                   //起始信号

    EEPROM_I2C_SendByte(Slave_Address+1);  //发送设备地址+读信号
    if(!EEPROM_I2C_WaitAck()) {
        EEPROM_I2C_Stop();
        return false;
    }

    REG_data=EEPROM_I2C_RecvByte();       //读出寄存器数据

    EEPROM_I2C_SendACK(1);                //发送停止传输信号

    EEPROM_I2C_Stop();                    //停止信号
    return REG_data;
}
u8 AT24CXX_ReadOneByte(u16 ReadAddr)
{
    u8 temp=0;
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	   //发送写命令
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(ReadAddr>>8);//发送高地址
    } else {
        EEPROM_I2C_SendByte(0XA0+((ReadAddr/256)<<1));   //发送器件地址0XA0,写数据
    }
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(ReadAddr%256);   //发送低地址
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Start();
    EEPROM_I2C_SendByte(0XA1);           //进入接收模式
    EEPROM_I2C_WaitAck();
    temp=EEPROM_I2C_RecvByte();
    EEPROM_I2C_Stop();//产生一个停止条件
    return temp;
}
//在AT24CXX指定地址写入一个数据
//WriteAddr  :写入数据的目的地址
//DataToWrite:要写入的数据
void AT24CXX_WriteOneByte(u16 WriteAddr,u8 DataToWrite)
{
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	    //发送写命令
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(WriteAddr>>8);//发送高地址
    } else {
        EEPROM_I2C_SendByte(0XA0+((WriteAddr/256)<<1));   //发送器件地址0XA0,写数据
    }
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(WriteAddr%256);   //发送低地址
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(DataToWrite);     //发送字节
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Stop();//产生一个停止条件
    EEPROM_Delay_1us(10);
}
uint8_t AT24CXX_Check(void)
{
    static uint8_t temp=0,i;
    temp=AT24CXX_ReadOneByte(E2END);//避免每次开机都写AT24CXX
    if(temp==0XA3)return 0;
    else//排除第一次初始化的情况
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
    //24C08的页读写只要2ms的等待时间，16字节一页
    uint16_t i,j,k,h,p1,p2,last_n;
    k=0;
    p2=0;
    p1=addr/E2PAGESIZE;//前面经过的页数
    h=addr%E2PAGESIZE;//最后一页残缺
    if(h!=0)//不满足一页
    {
        p2=E2PAGESIZE-h;//补足一页
        if(p2>n)
            p2=n;//补足一页
        for(i=0; i<p2; i++) //将前方剩余的页写入
        {
            //AT24CXX_WriteOneByte(addr+i,*(buf++));
            //delay_ms(7);
            EEPROM_I2C_Start();
            if(E2END>2047)
            {
                EEPROM_I2C_SendByte(0XA0);	    //发送写命令
                EEPROM_I2C_WaitAck();
                EEPROM_I2C_SendByte(addr>>8);//发送高地址
            } else {
                EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //发送器件地址0XA0,写数据
            }
            EEPROM_I2C_WaitAck();
            EEPROM_I2C_SendByte(addr%256);   //发送低地址
            EEPROM_I2C_WaitAck();
            for(; h<E2PAGESIZE; h++)
            {
                EEPROM_I2C_SendByte(*(buf));//发送数据
                EEPROM_I2C_WaitAck();
                buf++;
            }
            EEPROM_I2C_Stop();//产生一个停止条件
            delay_ms(5);
        }
    }
    last_n=n;
    n=n-p2;//写入完成减去
    if(n>1000)
        n=n;
    addr=addr+p2;
//
//
//	i=n/16;//计算页数
//	j=n%16;//计算剩余字节数
//
//	for(k=0;k<i;k++)//写入多页
//	{
//		EEPROM_I2C_Start();
//		EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));
//		EEPROM_I2C_WaitAck();
//		EEPROM_I2C_SendByte(addr%256);   //发送低地址
//		EEPROM_I2C_WaitAck();
//		for(h=0;h<16;h++)
//		{
//			EEPROM_I2C_SendByte(*(buf));//发送数据
//			EEPROM_I2C_WaitAck();
//			buf=buf+1;
//			addr++;
//		}
//		//buf=buf+16;
//		EEPROM_I2C_Stop();//产生一个停止条件
//		delay_ms(10);
//	}

//	for(i=0;i<j;i++)//将前方剩余的页单字节写入
//	{
//		AT24CXX_WriteOneByte(addr++,*(buf++));
//		delay_ms(7);
//	}

    while(k<n)//写入多页
    {
        EEPROM_I2C_Start();
        if(E2END>2047)
        {
            EEPROM_I2C_SendByte(0XA0);	    //发送写命令
            EEPROM_I2C_WaitAck();
            EEPROM_I2C_SendByte(addr>>8);//发送高地址
        } else {
            EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //发送器件地址0XA0,写数据
        }
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(addr%256);   //发送低地址
        EEPROM_I2C_WaitAck();
        for(h=0; h<E2PAGESIZE; h++)
        {
            if(k>n)
                break;
            EEPROM_I2C_SendByte(*(buf));//发送数据
            EEPROM_I2C_WaitAck();
            buf=buf+1;
            addr++;
            k++;
        }
        EEPROM_I2C_Stop();//产生一个停止条件
        delay_ms(5);//数据手册写5ms操作时间，5ms之内不响应
    }

}
/*
连续读取:
发送开始信号
发送器件选择+写信号
发送内存地址
发送开始信号
发送器件选择+读信号
读取数据
发送应答信号
读取下一字节的数据
发送应答信号
......
读取最后一字节的数据
不发送应答信号
发送停止信号
*/
void eeprom_Continuous_reading(u8 *buf,u16 addr, size_t n)//连续读取
{
    EEPROM_I2C_Start();
    if(E2END>2047)
    {
        EEPROM_I2C_SendByte(0XA0);	   //发送写命令
        EEPROM_I2C_WaitAck();
        EEPROM_I2C_SendByte(addr>>8);//发送高地址
    } else EEPROM_I2C_SendByte(0XA0+((addr/256)<<1));   //发送器件地址0XA0,写数据

    EEPROM_I2C_WaitAck();
    EEPROM_I2C_SendByte(addr%256);   //发送低地址
    EEPROM_I2C_WaitAck();
    EEPROM_I2C_Start();
    EEPROM_I2C_SendByte(0XA1);           //进入接收模式
    EEPROM_I2C_WaitAck();
    while(--n)
    {
        *buf=EEPROM_I2C_RecvByte();
        EEPROM_I2C_SendACK(0);//ACK
        buf++;
    }
    *buf=EEPROM_I2C_RecvByte();
    EEPROM_I2C_SendACK(1);//ACK
    EEPROM_I2C_Stop();//产生一个停止条件
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
void eeprom_read_block (void *buf,void *addr, size_t n)//读取由指定地址开始的指定长度的EEPROM数据
{



    
    eeprom_Continuous_reading((u8*)buf,(u16)addr,n);//连续读取
    


//		uint16_t i;
//		u8 *temp=(u8*)buf;
//		LED2_ON
//		for(i=0;i<=n;i++)
//	{
//		*temp=(u8)AT24CXX_ReadOneByte((u16)addr+i);//单字节读取
//		temp++;
//	}
//		LED2_OFF
}
#endif

