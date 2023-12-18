#include "sys.h"
#include "config.h"
#include "usart.h"
#include "def.h"
#include "Serial.h"
#if 1
#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
    /* Whatever you require here. If the only file you are using is */
    /* standard output using printf() for debugging, no file handling */
    /* is required. */
};
/* FILE is typedef’ d in stdio.h. */
FILE __stdout;
int _sys_exit(int x)
{
    x = x;
}
int fputc(int ch, FILE *f)
{
    while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
    USART1->DR = (u8) ch;
    return ch;
}
#endif
//end
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
void USART3_IRQHandler(void)
{
    u8 res;
    static int i=0;
    static u8 startflag=0;
    static u8 lastres=0,lastlastres=0;
    if(USART3->SR&(1<<5))//接收到数据
    {
        res=USART3->DR;

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

        if(res==0x00&&lastres==0x00&&lastlastres==0x80)
        {
            startflag=1;
            i=0;
        }



        lastlastres=lastres;
        lastres=res;

    }
}
uint16_t Cap_CH[13];
//uint16_t Capture_CH[13];


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

//        if((Cap_CH[1]<1900&&Cap_CH[1]>200)&&(Cap_CH[2]<1900&&Cap_CH[2]>200)&&(Cap_CH[3]<1900&&Cap_CH[3]>200)&&(Cap_CH[4]<1900&&Cap_CH[4]>200)&&(Cap_CH[5]<1900&&Cap_CH[5]>200)&&
//                (Cap_CH[6]<1900&&Cap_CH[6]>200)&&(Cap_CH[7]<1900&&Cap_CH[7]>200)&&(Cap_CH[8]<1900&&Cap_CH[8]>200))

//        {
//            Capture_CH[1]=(int)((Cap_CH[1]-300)/1.4);
//            Capture_CH[2]=(int)((Cap_CH[2]-300)/1.4);
//            Capture_CH[3]=(int)((Cap_CH[3]-300)/1.4);
//            Capture_CH[4]=(int)((Cap_CH[4]-300)/1.4);
//            Capture_CH[5]=(int)((Cap_CH[5]-300)/1.4);
//            Capture_CH[6]=(int)((Cap_CH[6]-300)/1.4);
//            Capture_CH[7]=(int)((Cap_CH[7]-300)/1.4);
//            Capture_CH[8]=(int)((Cap_CH[8]-300)/1.4);
//            if(Cap_CH[9]>=1000)Capture_CH[9]=1000;
//            else if(Cap_CH[9]<=300)Capture_CH[9]=0;
//            if(Cap_CH[10]>=1000)Capture_CH[10]=1000;
//            else if(Cap_CH[10]<=300)Capture_CH[10]=0;

//        }


    }
//				for(i=1;i<12;i++)
//		{
//			Cap_CH[i]=Cap_CH[i]>>2;
//		}
//		i=0;

}
#endif
void UART4_IRQHandler(void)
{

    u8 res;
    if(UART4->SR&(1<<5))//接收到数据
    {
        res=UART4->DR;
        store_uart_in_buf(res,4);
    }

}
void UART5_IRQHandler(void)
{

    u8 res;
    if(UART5->SR&(1<<5))//接收到数据
    {
        res=UART5->DR;
        store_uart_in_buf(res,5);
    }

}
//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void uart_init(u32 pclk2,u32 bound)
{
    float temp;
    u16 mantissa;
    u16 fraction;
    temp=(float)(pclk2*1000000)/(bound*16);//得到USARTDIV
    mantissa=temp;				 //得到整数部分
    fraction=(temp-mantissa)*16; //得到小数部分
    mantissa<<=4;
    mantissa+=fraction;
    RCC->APB2ENR|=1<<2;   //使能PORTA口时钟
    RCC->APB2ENR|=1<<14;  //使能串口时钟
    GPIOA->CRH&=0XFFFFF00F;
    GPIOA->CRH|=0X000008B0;//IO状态设置

    RCC->APB2RSTR|=1<<14;   //复位串口1
    RCC->APB2RSTR&=~(1<<14);//停止复位
    //波特率设置
    USART1->BRR=mantissa; // 波特率设置
    USART1->CR1|=0X200C;  //1位停止,无校验位.
    //使能接收中断
    USART1->CR1|=1<<8;    //PE中断使能
    USART1->CR1|=1<<5;    //接收缓冲区非空中断使能
    MY_NVIC_Init(3,3,USART1_IRQn,2);//组2，最低优先级
}


void USART2_Configuration(u32 bound)
{

    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_AFIO, ENABLE); //使能USART3，GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART3，GPIOB时钟
    GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);  //初始化PA10

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

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD| RCC_APB2Periph_AFIO, ENABLE); //使能USART3，GPIOB时钟,复用时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART3，GPIOB时钟
    //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);  //部分重映射
    GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE);
//	USART_DeInit(USART3);  //复位串口1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);

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

void USART4_Configuration(u32 bound)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能UART4，GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOC, &GPIO_InitStructure);  //初始化PA10
    //NVIC 配置
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
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

    USART_Init(UART4, &USART_InitStructure); //初始化串口
    USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(UART4, ENABLE);                    //使能串口
}

void USART5_Configuration(u32 bound)
{

    //GPIO端口设置
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD|RCC_APB2Periph_GPIOC|RCC_APB2Periph_AFIO, ENABLE); //使能USART3，GPIOB时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

    //GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_UART5);
    //GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_UART5);

    //GPIO_PinRemapConfig(GPIO_Remap_USART2,ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
    GPIO_Init(GPIOD, &GPIO_InitStructure);  //初始化PA10

    //Usart1 NVIC 配置

    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
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

    USART_Init(UART5, &USART_InitStructure); //初始化串口
    USART_ITConfig(UART5, USART_IT_RXNE, ENABLE);//开启中断
    USART_Cmd(UART5, ENABLE);                    //使能串口
}
