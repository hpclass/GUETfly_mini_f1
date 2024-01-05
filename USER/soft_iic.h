/* Define to prevent recursive inclusion ------------------------------------ */

#ifdef STM32F10X_MD
#include "stm32f10x.h"
#endif
#include "stdint.h"
/* Includes ------------------------------------------------------------------*/

typedef unsigned short int uint;

/* Exported macro ------------------------------------------------------------*/
#define ADDR_24CXX 0x78
// #define ADDR_24CXX        0xD0

// #define SCLH         GPIOB->BSRR = GPIO_Pin_6
// #define SCLL         GPIOB->BRR  = GPIO_Pin_6
//
// #define SDAH         GPIOB->BSRR = GPIO_Pin_7
// #define SDAL         GPIOB->BRR  = GPIO_Pin_7

// #define SCLread      GPIOB->IDR  & GPIO_Pin_6
// #define SDAread      GPIOB->IDR  & GPIO_Pin_7

#define SCLH GPIOA->BSRR = GPIO_Pin_5
#define SCLL GPIOA->BRR = GPIO_Pin_5

#define SDAH GPIOA->BSRR = GPIO_Pin_4
#define SDAL GPIOA->BRR = GPIO_Pin_4

#define SCLread GPIOA->IDR &GPIO_Pin_5
#define SDAread GPIOA->IDR &GPIO_Pin_4

// IO方向设置

#define SDA_IN()                    \
    {                               \
        GPIOB->CRL &= 0X0FFFFFFF;   \
        GPIOB->CRL |= (u32)8 << 28; \
    }
#define SDA_OUT()                   \
    {                               \
        GPIOB->CRL &= 0X0FFFFFFF;   \
        GPIOB->CRL |= (u32)3 << 28; \
    }

// IO操作函数
// #define IIC_SCL    PBout(6) //SCL
// #define IIC_SDA    PBout(7) //SDA
// #define READ_SDA   PBin(7)  //输入SDA

#define IIC_SCL PAout(5) // SCL
#define IIC_SDA PAout(4) // SDA
#define READ_SDA PAin(4) // 输入SDA
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
/*
void IIC_Init(void);
u8 IIC_WaitAck(void);
void IIC_Start(void);
void IIC_Stop(void);
void IIC_Ack(void);
void IIC_SendACK(u8 i);
void IIC_NAck(void);
void IIC_Send_Byte(u8 txd);
void IIC_SendByte(u8 txd);
u8 IIC_Read_Byte(unsigned char ack);
u8 IIC_RecvByte();
*/

enum ENUM_TWI_REPLY
{
    TWI_NACK = 0,
    TWI_ACK = 1
};

enum ENUM_TWI_BUS_STATE
{
    TWI_READY = 0,
    TWI_BUS_BUSY = 1,
    TWI_BUS_ERROR = 2
};
void IIC_EE_Init(void);
uint8_t I2C_EE_BufferWrite(uint8_t *psrc_data, uint8_t adr, uint8_t nbyte);
uint8_t I2C_EE_BufferRead(uint8_t *pdin_data, uint8_t adr, uint8_t nbyte);
uint8_t TWI_WriteByte(uint8_t SendByte, uint8_t WriteAddress);

uint8_t TWI_WaitAck(void);
void TWI_NoAck(void);
void TWI_Ack(void);
void TWI_Stop(void);
uint8_t TWI_Start(void);
// u8 TWI_Start(void);
void TWI_SendByte(uint8_t SendByte); // 数据从高位到低位//
uint8_t TWI_ReadByte(uint8_t ReadAddress);
uint8_t TWI_ReceiveByte(void); // 数据从高位到低位//
