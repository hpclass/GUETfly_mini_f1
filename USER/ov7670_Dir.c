#include "stm32f10x.h"
#include "types.h"
#include "config.h"
#include "def.h"
#include "ov7670_Dir.h"
#include "sys.h"


double XPosition,YPosition;//λ����Ϣ
double DirectionXSpeed,DirectionYSpeed;//λ����Ϣ
double HIGH_CURRENT_CSB=0;//�߶���Ϣ
double DOWNSPEED_CURRENT_CSB=0;//�Ե��ٶ���Ϣ
uint8_t ov7670_flag=0;//֡��־
int16_t angle_ov7670[2];
extern int16_t  debug[4];
//���ڽ���Э��
void ov7670_get_one_byte(uint8_t c)
{
    static uint8_t state=0;
    static short temp[2];
    state++;
    if(state==1)
    {
        if(c!=0x01)//У��ͷ����һ���ֽ�
            state=0;
    } else if(state==2) {
        if(c!=0xfe)//У��ͷ���ڶ����ֽ�
            state=0;
    } else if(state>=3) { //У����ͷ��������������������
        switch(state)
        {
        case 3:
            XPosition=(double)((int)(c-120));
            break;
        case 4:
            YPosition=(double)((int)(c-120));
            break;
        case 5:
            DirectionXSpeed=(double)((int)(c-120));
            break;
        case 6:
            DirectionYSpeed=(double)((int)(c-120));
            break;
        case 7:
            temp[0]=c<<8;//�߰�λ
            break;
        case 8:
            temp[0]|=c;//�Ͱ�λ
            HIGH_CURRENT_CSB=(double)((short)temp[0]);
            break;
        case 9:
            temp[1]=c<<8;//�߰�λ
            break;
        case 10:
            temp[1]|=c;//�Ͱ�λ
            DOWNSPEED_CURRENT_CSB=(double)((short)temp[1]);
            break;

        default:
            state=0;//һ֡�������
            ov7670_flag=1;//��־֡

            break;

        }

        //		if(state>3)

    }


}
/*
XPosition=(double)((int)(PositiondataRX_BUF[2]-120));
YPosition=(double)((int)(PositiondataRX_BUF[3]-120));
DirectionXSpeed=(double)((int)(PositiondataRX_BUF[4]-120));
DirectionYSpeed=(double)((int)(PositiondataRX_BUF[5]-120));
HIGH_CURRENT_CSB=(double)((short)(PositiondataRX_BUF[6]<<8|PositiondataRX_BUF[7])/1000.0);
DOWNSPEED_CURRENT_CSB=(double)((short)(PositiondataRX_BUF[8]<<8|PositiondataRX_BUF[9])/100.0)-10.0;
rx_flag=1;
new_uart2dataflag=0;


*/
static double head_error_P,head_error_I,head_error_I_SUM[2];
static double error,error_P,error_I,error_I_SUM[2],error_D,err_D_last[2];
void c_flow()
{
    uint8_t axis=0;
    double temp_POSITION[2];
    double temp_SPEED[2];
    double head_P[2],head_I[2];


    double K_P[2],K_I[2],K_D[2];

    temp_POSITION[ROLL]=YPosition;
    temp_POSITION[PITCH]=-XPosition;

    temp_SPEED[ROLL]=DirectionYSpeed;
    temp_SPEED[PITCH]=-DirectionXSpeed;

    head_P[ROLL]=0.05;
    head_I[ROLL]=0;

    head_P[PITCH]=0.05;
    head_I[PITCH]=0;

    K_P[ROLL]=6;
    K_I[ROLL]=0.001;
    K_D[ROLL]=8;

    K_P[PITCH]=7;
    K_I[PITCH]=0.001;
    K_D[PITCH]=8;


    if(!ov7670_flag)//δ����֡����������
        return ;
    ov7670_flag=0;

    GPS_TOGGLE;
    for(axis=0; axis<2; axis++)
    {
        //ǰ������
        //P
        head_error_P=constrain(head_P[axis]*temp_POSITION[axis],-3,+3);
        debug[axis]=head_error_P;
        //I
        head_error_I_SUM[axis]=constrain(head_error_I_SUM[axis]+YPosition,-20,+20);
        head_error_I=head_I[axis]*temp_POSITION[axis];
        //debug[axis]=temp_SPEED[axis];



        //������
        error=temp_SPEED[axis] + head_error_P + head_error_I;//ǰ�������������
        //P
        error_P=K_P[axis]*error;

        //I
        error_I_SUM[axis]=constrain(error_I_SUM[axis]+error,-20,+20);
        error_I=K_I[axis]*error_I_SUM[axis];

        //D
        error_D=K_D[axis]*(error - err_D_last[axis]);
        err_D_last[axis]=error;

        //OUTPUT=P+I+D
        angle_ov7670[axis]=constrain(error_P+error_I-error_D,-50,+50);//���������С
    }


}
void init_c_flow()
{
    //��������
    head_error_P=head_error_I=head_error_I_SUM[0]=head_error_I_SUM[1]=0;
    error=error_P=error_I=error_I_SUM[0]=error_I_SUM[1]=error_D=err_D_last[0]=err_D_last[1]=0;

}
/*
#if defined(MS561101BA)

#else
void Baro_update()
{

}
void Baro_init()
{

}
#endif

*/














