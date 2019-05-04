#include "sys.h"
#include "config.h"
#include "usart.h"
#include "def.h"
#include "Serial.h"
//////////////////////////////////////////////////////////////////
//串口1中断服务程序
void USART1_IRQHandler(void)
{

    u8 res;
    if(USART1->SR&(1<<5))//接收到数据
    {
        res=USART1->DR;
        store_uart_in_buf(res,1);
        //GPS_newFrame(res);
    }

}
void USART2_IRQHandler(void)
{
    u8 res;
    if(USART2->SR&(1<<5))//接收到数据
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
    if(USART3->SR&(1<<5))//接收到数据
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
    if(USART3->SR&(1<<5))//接收到数据
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
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void uart_init(u32 pclk2,u32 bound)
{  	 
//	float temp;
//	u16 mantissa;
//	u16 fraction;	   
//	temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
//	mantissa=temp;				 //得到整数部分
//	fraction=(temp-mantissa)*16; //得到小数部分	 
//    mantissa<<=4;
//	mantissa+=fraction; 
//	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
//	RCC->APB2ENR|=1<<14;  //使能串口时钟 
//	GPIOA->CRH&=0XFFFFF00F; 
//	GPIOA->CRH|=0X000008B0;//IO状态设置
//		  
//	RCC->APB2RSTR|=1<<14;   //复位串口1
//	RCC->APB2RSTR&=~(1<<14);//停止复位	   	   
//	//波特率设置
// 	USART1->BRR=mantissa; // 波特率设置	 
//	USART1->CR1|=0X200C;  //1位停止,无校验位.
//#ifdef EN_USART1_RX		  //如果使能了接收
//	//使能接收中断
//	USART1->CR1|=1<<8;    //PE中断使能
//	USART1->CR1|=1<<5;    //接收缓冲区非空中断使能	    	
//	MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级 
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
	RCC->APB2ENR|=1<<2;   //使能PORTA口时钟  
	RCC->APB2ENR|=1<<14;  //使能串口时钟 
	GPIOA->CRH&=0XFFFFF00F; 
	GPIOA->CRH|=0X000008B0;//IO状态设置
	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //收发模式

	USART_Init(USART1, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART1, ENABLE);                    //使能串口   
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
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能USART3，GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART3，GPIOB时钟
	//USART_DeInit(USART3);  //复位串口1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA10

		 //Usart1 NVIC 配置

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
		
		 //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //收发模式

	USART_Init(USART2, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART2, ENABLE);                    //使能串口  
}
void USART3_Configuration(u32 bound)
{ 
//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
		
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能USART3，GPIOB时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART3，GPIOB时钟
	USART_DeInit(USART3);  //复位串口1
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //PB.10
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_Init(GPIOB, &GPIO_InitStructure); //初始化PA9

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);  //初始化PA10

		 //Usart1 NVIC 配置

	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道使能
	NVIC_Init(&NVIC_InitStructure); //根据指定的参数初始化VIC寄存器
		
		 //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//一般设置为9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;; //收发模式

	USART_Init(USART3, &USART_InitStructure); //初始化串口
	USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//开启中断
	USART_Cmd(USART3, ENABLE);                    //使能串口   
}

