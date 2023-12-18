#include "stm32f10x.h"
#include "optic_.h"
uint8_t Mini_Flow_SSI,Mini_Flow_SSI_CNT;
uint8_t mini_flow_flag;
_flow flow;
//�������ݽ���
void Player_Flow_Receive(uint8_t data)
{
    static uint8_t RxBuffer[32];
    static uint8_t _data_cnt = 0;
    static uint8_t state = 0;
    uint8_t sum = 0;

    switch(state)
    {
    case 0:
        if(data==0xFE)//��ͷ
        {
            state=1;
            RxBuffer[_data_cnt++]=data;
        } else state = 0;
        break;

    case 1:
        if(data==0x04)//�ֽ���
        {
            state=2;
            RxBuffer[_data_cnt++]=data;
        } else state = 0;
        break;

    case 2:
        RxBuffer[_data_cnt++]=data;

        if(_data_cnt==9)
        {
            state = 0;
            _data_cnt = 0;

            sum =  (RxBuffer[2] + RxBuffer[3] + RxBuffer[4] + RxBuffer[5]);

            if((0xAA == data) && (sum == RxBuffer[6])) //��У��
            {
                Mini_Flow_SSI_CNT++;//���Թ�������֡��

                flow.x = ( (s16)(*(RxBuffer+3)<<8)|*(RxBuffer+2) );
                flow.y = ( (s16)(*(RxBuffer+5)<<8)|*(RxBuffer+4) );
                mini_flow_flag=1;
            }
        }
        break;

    default:
        state = 0;
        _data_cnt = 0;
        break;
    }
}

