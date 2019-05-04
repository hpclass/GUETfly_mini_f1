/************************************

���ֵ��ӿƼ���ѧ
ʱ�䣺2016-07-25 

�ļ�˵����

�δ�ʱ��
************************************/

#include "timer.h"
#include "def.h"
extern unsigned int delays;
extern unsigned int SysTick_Handler_times;
extern unsigned int delays_u;
extern unsigned int runtime_us;
extern unsigned int runtime_ms;
extern unsigned long	micros_time;
unsigned long micros(void) 
{
	
	return micros_time*10;
}
unsigned long millis(void) 
{
	
	return micros_time/100;
}

void  SysTick_Handler_time_rise(void)//10us
{                                                      
		////////////////////////////////	
			micros_time++;
			if(delays_u)
		{
			
			delays_u--;
		}else{
			
			delays_u=100;
			if(delays)
			{
				delays--;
			}
		}
		
////////////////////////////
////////////////////////////
		
					
		
}
void time_init()
{
	SysTick_Handler_times=0;
	
	
}
void TIM3_PWM_Init(u16 arr,u16 psc)
{  
	
 RCC->APB1ENR|=1<<1;  //TIM3ʱ��ʹ��
 RCC->APB2ENR|=1<<2;     //ʹ��PORTAʱ��    
 RCC->APB2ENR|=1<<3;     //ʹ��PORTBʱ�� 

 GPIOA->CRL&=0X00FFFFFF; //PA6 CH1 PA7 CH2
 GPIOA->CRL|=0XBB000000; //���ù������
 GPIOB->CRL&=0XFFFFFF00; //PB0 CH3 PB1 CH4
 GPIOB->CRL|=0X000000BB; //���ù������     
   

 TIM3->ARR=arr;   //�趨�������Զ���װֵ
 TIM3->PSC=psc;   //Ԥ��Ƶ��


 TIM3->CCMR1|=6<<4;   //CH1 PWM1ģʽ  
 TIM3->CCMR1|=1<<3;      //CH1Ԥװ��ʹ��   
 TIM3->CCER|=1<<0;    //OC1 ���ʹ��
     
 
 TIM3->CCMR1|=6<<12;   //CH2 PWM1ģʽ  
 TIM3->CCMR1|=1<<11;  //CH2Ԥװ��ʹ��   
 TIM3->CCER|=1<<4;    //OC2 ���ʹ��
  

 TIM3->CCMR2|=6<<4;   //CH3 PWM1ģʽ  
 TIM3->CCMR2|=1<<3;      //CH3Ԥװ��ʹ��   
 TIM3->CCER|=1<<8;    //OC3 ���ʹ��
  
 TIM3->CCMR2|=6<<12;   //CH4 PWM1ģʽ  
 TIM3->CCMR2|=1<<11;  //CH4Ԥװ��ʹ��   
 TIM3->CCER|=1<<12;    //OC4 ���ʹ��

 TIM3->CCR1=1000;           
 TIM3->CCR2=1000;       

 TIM3->CCR3=1000;       

 TIM3->CCR4=1000;        
      
 TIM3->CR1|=1<<7;        //ARPEʹ��
 TIM3->CR1|=0x01;     //ʹ�ܶ�ʱ��3  
TIM_SetCompare1(TIM3,1000);
TIM_SetCompare2(TIM3,1000);
TIM_SetCompare3(TIM3,1000);
TIM_SetCompare4(TIM3,1000);
}
void TIM4_PWM_Init(u16 arr,u16 psc)
{  
//	GPIO_InitTypeDef GPIO_InitStructure;
//	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);	//ʹ��TIM2ʱ��
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  //ʹ��GPIOBʱ��
//	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8|GPIO_Pin_9; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //���ù������ 
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
   
 RCC->APB1ENR|=1<<2;  //TIM4ʱ��ʹ�� 
 RCC->APB2ENR|=1<<3;     //ʹ��PORTBʱ�� 

 GPIOB->CRL&=0X00FFFFFF; //PB6 PB7
 GPIOB->CRL|=0XBB000000; //���ù������
 GPIOB->CRH&=0XFFFFFF00; //PB0 CH3 PB1 CH4
 GPIOB->CRH|=0X000000BB; //���ù������     
 TIM4->ARR=arr;   //�趨�������Զ���װֵ
 TIM4->PSC=psc;   //Ԥ��Ƶ��


 TIM4->CCMR1|=6<<4;   //CH1 PWM1ģʽ  
 TIM4->CCMR1|=1<<3;      //CH1Ԥװ��ʹ��   
 TIM4->CCER|=1<<0;    //OC1 ���ʹ��
     
 
 TIM4->CCMR1|=6<<12;   //CH2 PWM1ģʽ  
 TIM4->CCMR1|=1<<11;  //CH2Ԥװ��ʹ��   
 TIM4->CCER|=1<<4;    //OC2 ���ʹ��
  

 TIM4->CCMR2|=6<<4;   //CH3 PWM1ģʽ  
 TIM4->CCMR2|=1<<3;      //CH3Ԥװ��ʹ��   
 TIM4->CCER|=1<<8;    //OC3 ���ʹ��
  
 TIM4->CCMR2|=6<<12;   //CH4 PWM1ģʽ  
 TIM4->CCMR2|=1<<11;  //CH4Ԥװ��ʹ��   
 TIM4->CCER|=1<<12;    //OC4 ���ʹ��

// TIM4->CCR1=1000;           
// TIM4->CCR2=1000;       

// TIM4->CCR3=1000;       

// TIM4->CCR4=1000;        
      
 TIM4->CR1|=1<<7;        //ARPEʹ��
 TIM4->CR1|=0x01;     //ʹ�ܶ�ʱ��3  
TIM_SetCompare1(TIM4,1500);
TIM_SetCompare2(TIM4,1500);
TIM_SetCompare3(TIM4,1500);
TIM_SetCompare4(TIM4,1500);
}
//��ʱ��1ͨ��1���벶������


void TIM1_Cap_Init(u16 arr,u16 psc)
{	 
	TIM_ICInitTypeDef  TIM1_ICInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ��TIM2ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIOAʱ��

	GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_8;  //PA0 ���֮ǰ����  
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD; //PA0 ����  
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA,GPIO_Pin_8);						 //PA0 ����
	TIM_DeInit(TIM1);   //������TIM1�Ĵ�������ΪĬ��ֵ
	//��ʼ����ʱ��1
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; 	//Ԥ��Ƶ��   
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM2���벶�����
	TIM1_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
	TIM1_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//�����ز���
	TIM1_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //ӳ�䵽TI1��
	TIM1_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	 //���������Ƶ,����Ƶ 
	TIM1_ICInitStructure.TIM_ICFilter = 0x00;//IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM1_ICInitStructure);

	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;  //��ռ���ȼ�2��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	TIM_ITConfig(TIM1,TIM_IT_Update|TIM_IT_CC1,ENABLE);//��������ж� ,����CC1IE�����ж�	

	TIM_Cmd(TIM1,ENABLE ); 	//ʹ�ܶ�ʱ��1




}

u32 temp=0;
u8  TIM1CH1_CAPTURE_STA=0,ppm_rx_sta=0,ppm_rx_num=0;	//���벶��״̬		    				
u16	TIM1CH1_CAPTURE_VAL;	//���벶��ֵ
u16 ppm_rx[12];//ppm_rx[0]   1   ���յ�ppm����
//extern volatile uint16_t rcValue[RC_CHANS];
//��ʱ��1�жϷ������	 
void TIM1_CC_IRQHandler(void)
{ 

 	if((TIM1CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
	{	  
		if (TIM_GetITStatus(TIM1, TIM_IT_Update) != RESET)
		 
		{	    
			if(TIM1CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
			{
				if((TIM1CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
				{
					TIM1CH1_CAPTURE_STA|=0X80;//��ǳɹ�������һ��
					TIM1CH1_CAPTURE_VAL=0XFFFF;
				}else TIM1CH1_CAPTURE_STA++;
			}	 
		}
	if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//����1���������¼�
		{	
			if(TIM1CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
			{	  			
				TIM1CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
				TIM1CH1_CAPTURE_VAL=TIM_GetCapture1(TIM1);
		   		TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 ����Ϊ�����ز���
			}else  								//��δ��ʼ,��һ�β���������
			{
				TIM1CH1_CAPTURE_STA=0;			//���
				TIM1CH1_CAPTURE_VAL=0;
	 			TIM_SetCounter(TIM1,0);
				TIM1CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
		   		TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 ����Ϊ�½��ز���
			}		    
		}			     	    					   
 	}
	
	//����֡����
		if(TIM1CH1_CAPTURE_STA&0X80)//�ɹ�������һ��������
		{
			if(ppm_rx_sta==1) {ppm_rx[ppm_rx_num+1]=TIM1CH1_CAPTURE_VAL;ppm_rx_num++;}
			//printf("TIM1CH1_CAPTURE_VAL:%d\r\n",TIM1CH1_CAPTURE_VAL);
			if(4>TIM1CH1_CAPTURE_STA&0X3F>0||TIM1CH1_CAPTURE_VAL>3000) ppm_rx_sta++;//�͵�ƽʱ�����3000usΪ��ʼ֡
			if(ppm_rx_sta==2) {ppm_rx[ppm_rx_num]=0;ppm_rx_sta=0;ppm_rx[0]=1;ppm_rx_num=0;}//printf("receive\r\n");//ppm_rx_sta   1 ��ʾ���յ�ͬ��֡/ 2���յ�����һ��ʼ֡ ppm���ݽ������
			
			TIM1CH1_CAPTURE_STA=0;//������һ�β���
			
		}
			
			
		
		
    TIM_ClearITPendingBit(TIM1, TIM_IT_CC1|TIM_IT_Update); //����жϱ�־λ
 
}
 
