#include "sys.h"
#include "config.h"
#include "usart.h"
#include "def.h"
#include "Serial.h"
//////////////////////////////////////////////////////////////////
//����1�жϷ������
void USART1_IRQHandler(void)
{

    u8 res;
    if(USART1->SR&(1<<5))//���յ�����
    {
        res=USART1->DR;
        store_uart_in_buf(res,1);
        //GPS_newFrame(res);
    }

}
void USART2_IRQHandler(void)
{
    u8 res;
    if(USART2->SR&(1<<5))//���յ�����
    {

        res=USART2->DR;
        store_uart_in_buf(res,2);

    }

}
//#if 0
#if !defined(SBUS__)
void USART3_IRQHandler(void)
{

    u8 res;
    if(USART3->SR&(1<<5))//���յ�����
    {
        res=USART3->DR;
        store_uart_in_buf(res,3);
    }

}
#else
static uint8_t num_cap=0;
u16 SBUSBuf[30]= {0};
uint8_t SBUS_FLAG_=0;
u8 testdata[256]={0};
void USART3_IRQHandler(void)
{
    u8 res;
    static int i=0;
    static u8 startflag=0;
    static u8 lastres=0,lastlastres=0;
    if(USART3->SR&(1<<5))//���յ�����
    {
        res=USART3->DR;
				testdata[testdata[0]+1]=res;
				testdata[0]++;
			
        if(startflag==1)
        {
            SBUSBuf[i]=res;
            i++;
            if(i==22)
            {
                startflag=0;
								SBUS_FLAG_=1;
                //Capture();
            }
        }
//#if defined(FRSHY_SBUS)
#if 1
     if(res==0x0F&&lastres==0x00)
        {
					SBUSBuf[0]=res;
            startflag=1;
            i=1;
        }  
#endif
#if 0
 if(res==0x00&&lastres==0x00&&lastlastres==0x80)
        {
            startflag=1;
            i=0;
        }
#endif

        lastlastres=lastres;
        lastres=res;

    }
}
uint16_t Cap_CH[13];
//uint16_t Capture_CH[13];
void Capture()
{
    int i=0;
    if(SBUSBuf[0]==0x0F)
    {
        Cap_CH[1]=((SBUSBuf[i+2]&0x07)<<8)|(SBUSBuf[i+1]&0xff);
        Cap_CH[2]=((SBUSBuf[i+3]&0x3f)<<5)|((SBUSBuf[i+2]&0xf8)>>3);
        Cap_CH[3]=((SBUSBuf[i+5]&0x01)<<10)|((SBUSBuf[i+4]&0xff)<<2)|(((SBUSBuf[i+3]&0xc0)>>6)&0xff);
        Cap_CH[4]=((SBUSBuf[i+6]&0x0f)<<7)|((SBUSBuf[i+5]&0xfe)>>1);
        Cap_CH[5]=((SBUSBuf[i+7]&0x7f)<<4)|((SBUSBuf[i+6]&0xf0)>>4);
        Cap_CH[6]=((SBUSBuf[i+9]&0x03)<<9)|((SBUSBuf[i+8])<<1)|(SBUSBuf[i+7]&0x80)>>7;
        Cap_CH[7]=((SBUSBuf[i+10]&0x1f)<<6)|(SBUSBuf[i+9]&0xfc)>>2;
        Cap_CH[8]=((SBUSBuf[i+11]&0xff)<<3)|(SBUSBuf[i+10]&0xe0)>>5;
        Cap_CH[9]=((SBUSBuf[i+13]&0x07)<<8)|(SBUSBuf[i+12]&0xff);
        Cap_CH[10]=((SBUSBuf[i+14]&0x3f)<<5)|(SBUSBuf[i+13]&0xf8)>>3;

        SBUSBuf[20]=0;
        SBUSBuf[21]=0;
    }
}
#if 0
void Capture()
{
    int i=0;
    if(SBUSBuf[0]==0x0F&&SBUSBuf[20]==0x02&&SBUSBuf[21]==0x10)
    {
        Cap_CH[1]=((SBUSBuf[i+2]&0x07)<<8)|(SBUSBuf[i+1]&0xff);
        Cap_CH[2]=((SBUSBuf[i+3]&0x3f)<<5)|((SBUSBuf[i+2]&0xf8)>>3);
        Cap_CH[3]=((SBUSBuf[i+5]&0x01)<<10)|((SBUSBuf[i+4]&0xff)<<2)|(((SBUSBuf[i+3]&0xc0)>>6)&0xff);
        Cap_CH[4]=((SBUSBuf[i+6]&0x0f)<<7)|((SBUSBuf[i+5]&0xfe)>>1);
        Cap_CH[5]=((SBUSBuf[i+7]&0x7f)<<4)|((SBUSBuf[i+6]&0xf0)>>4);
        Cap_CH[6]=((SBUSBuf[i+9]&0x03)<<9)|((SBUSBuf[i+8])<<1)|(SBUSBuf[i+7]&0x80)>>7;
        Cap_CH[7]=((SBUSBuf[i+10]&0x1f)<<6)|(SBUSBuf[i+9]&0xfc)>>2;
        Cap_CH[8]=((SBUSBuf[i+11]&0xff)<<3)|(SBUSBuf[i+10]&0xe0)>>5;
        Cap_CH[9]=((SBUSBuf[i+13]&0x07)<<8)|(SBUSBuf[i+12]&0xff);
        Cap_CH[10]=((SBUSBuf[i+14]&0x3f)<<5)|(SBUSBuf[i+13]&0xf8)>>3;

        SBUSBuf[20]=0;
        SBUSBuf[21]=0;
    }
}
#endif

#endif
//��ʼ��IO ����1
//pclk2:PCLK2ʱ��Ƶ��(Mhz)
//bound:������
//CHECK OK
//091209
void uart_init(u32 pclk2,u32 bound)
{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
//	temp=(float)(pclk2*1000000)/(bound*16);//�õ�USARTDIV
//	mantissa=temp;				 //�õ���������
//	fraction=(temp-mantissa)*16; //�õ�С������	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
//	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
//	GPIOA->CRH&=0XFFFFF00F; 
//	GPIOA->CRH|=0X000008B0;//IO״̬����
//		  
//	RCC->APB2RSTR|=1<<14;   //��λ����1
//	RCC->APB2RSTR&=~(1<<14);//ֹͣ��λ	   	   
//	//����������
// 	USART1->BRR=mantissa; // ����������	 
//	USART1->CR1|=0X200C;  //1λֹͣ,��У��λ.
//#ifdef EN_USART1_RX		  //���ʹ���˽���
//	//ʹ�ܽ����ж�
//	USART1->CR1|=1<<8;    //PE�ж�ʹ��
//	USART1->CR1|=1<<5;    //���ջ������ǿ��ж�ʹ��	    	
//	MY_NVIC_Init(3,3,USART1_IRQn,2);//��2��������ȼ� 
//#endif
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

//  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);
  
  //  USART2_TX -> PA2 , USART2_RX ->	PA3
  			

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	         
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;	        
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
//  GPIO_Init(GPIOA, &GPIO_InitStructure);
	RCC->APB2ENR|=1<<2;   //ʹ��PORTA��ʱ��  
	RCC->APB2ENR|=1<<14;  //ʹ�ܴ���ʱ�� 
	GPIOA->CRH&=0XFFFFF00F; 
	GPIOA->CRH|=0X000008B0;//IO״̬����
	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //�շ�ģʽ

	USART_Init(USART1, &USART_InitStructure); //��ʼ������
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���   
}


void USART2_Configuration(u32 bound)
{ 
	/*
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure; 

  RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB1Periph_USART2,ENABLE);
  
  //  USART2_TX -> PA2 , USART2_RX ->	PA3
  			

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;	         
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);		   

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;	        
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;  
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz; 
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

  USART_Init(USART2, &USART_InitStructure); 
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
  USART_ClearFlag(USART2,USART_FLAG_TC);
  USART_Cmd(USART2, ENABLE);
	*/
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ��USART3��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //ʹ��USART3��GPIOBʱ��
	//USART_DeInit(USART3);  //��λ����1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA10

		 //Usart1 NVIC ����

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
		 //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //�շ�ģʽ

	USART_Init(USART2, &USART_InitStructure); //��ʼ������
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���  
}
void USART3_Configuration(u32 bound)
{ 
//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ��USART3��GPIOBʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //ʹ��USART3��GPIOBʱ��
	USART_DeInit(USART3);  //��λ����1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //�����������
	GPIO_Init(GPIOB, &GPIO_InitStructure); //��ʼ��PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //��ʼ��PA10

		 //Usart1 NVIC ����

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure); //����ָ���Ĳ�����ʼ��VIC�Ĵ���
		
		 //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//һ������Ϊ9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //�շ�ģʽ

	USART_Init(USART3, &USART_InitStructure); //��ʼ������
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�����ж�
	USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���   
}

