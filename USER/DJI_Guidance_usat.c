#include "stm32f10x.h"
#include "DJI_Guidance_usat.h"
/*
桂林电子科技大学--黄鹏
于2018-7-21

*/
obstacle_distance DJI_Guidance_obstacle_distance;//障碍距离
ultrasonic_data DJI_Guidance_ultrasonic;//5路距离
velocity DJI_Guidance_vo;//3路速度，单位mm
protocal_sdk_uart_header DJI_Guidance_head;//协议头
soc2pc_vo_can_output output;//临时缓存区，通用缓存区
uint8_t DJI_Guidance_flag_alt;
uint8_t DJI_Guidance_flag_vel;
//101110
//101110 46
//AA 80 0B 00 00 00 00 00 D3 30 17 9C 00 02 D9 42 00 00 EA D0 01 00 5B 00 CF 04 FF FF 5B 00 F3 09 01 00 01 00 00 00 01 00 01 00 00 00 00 00 AA
//AA800B0000000000D330179C0002D9420000EAD001005B00CF04FFFF5B00F3090100010000000100010000000000AA
//SOF	LEN	VER	RES	SEQ	CRC16	DATA	CRC32
//8   10  6   40  16  16    *     32

//AA 80 0B 00 00 00 00 00 25 30 CF B4 00 02 E2 41 00 00 44 CA 01 00 4A 00 DE 01 9E 01 8D 01 FF FF 01 00 01 00 01 00 01 00 00 00 00 00 00 00
//AA 80 23 00 00 00 00 00 27 30 33 1C 00 03 00 00 3F FF AC FD DB FF 82 14 DB C0 9D 1C D9 C1 0A D7 23 3C 17 B7 D1 39 00 00 00 00 00 00 00 00 17 B7 D1 39 00 00 00 00 02 2B 07 3D 68 EB 87 3A 00 00 00 00 00 00 00 00 68 EB 87 3A 00 00 00 00 02 2B 07 3D 00 00 00 00 00 00 80 3F 33 00 00 00 F0 41 00 00 BC EF 01 00 4B 00 00 7D 10 01 00 00 8C 7D D3 70 BC EF 01 00 EF 41 00 00 00 00 00 00 7D 0C 6E C0 FA 18 9C 3F 90 C1 79 3E 00 00 00 00

static /*
**   const variable define
*/
const uint16_t wCRC_Table[256] =
{
    0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
    0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
    0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
    0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
    0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
    0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
    0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
    0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
    0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
    0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
    0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
    0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
    0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
    0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
    0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
    0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
    0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
    0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
    0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
    0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
    0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
    0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
    0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
    0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
    0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
    0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
    0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
    0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
    0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
    0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
    0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
    0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};
const uint16_t CRC_INIT = 0x3692; //0x7000;	 //dji naza

static uint8_t DJI_UART_temp_buf[256];
uint16_t CRC16_(uint16_t wCRC,uint8_t chData)//CRC16断点计算,每次计算一个字节
{
    (wCRC) = ((uint16_t)(wCRC) >> 8)  ^ wCRC_Table[((uint16_t)(wCRC) ^ (uint16_t)(chData)) & 0x00ff];
    return (wCRC);
}
void DJI_Analysis_vel()//解析三轴速度
{
    //memcpy( &output,DJI_UART_temp_buf, sizeof(DJI_Guidance_vo) );
    //DJI_Guidance_vo.frame_index;
    //DJI_Guidance_vo.time_stamp=output.m_time_stamp;
    DJI_Guidance_vo.vx=DJI_UART_temp_buf[3]<<8|DJI_UART_temp_buf[2];
    DJI_Guidance_vo.vy=DJI_UART_temp_buf[5]<<8|DJI_UART_temp_buf[4];
    DJI_Guidance_vo.vz=DJI_UART_temp_buf[7]<<8|DJI_UART_temp_buf[6];
    DJI_Guidance_flag_vel=1;
}
void DJI_Analysis_distance()//解析五向距离
{
    uint8_t d;
    for ( d = 0; d < CAMERA_PAIR_NUM; ++d )
    {
        DJI_Guidance_ultrasonic.ultrasonic[d]=DJI_UART_temp_buf[9+2*d]<<8|DJI_UART_temp_buf[8+2*d];
        DJI_Guidance_ultrasonic.reliability[d]=(int)DJI_UART_temp_buf[0x13+2*d]<<8|DJI_UART_temp_buf[0x12+2*d];
    }
    DJI_Guidance_flag_alt=1;
}
void DJI_Analysis_obstacle_distance()//解析障碍距离
{
    //memcpy( &output,DJI_UART_temp_buf, sizeof(DJI_Guidance_obstacle_distance) );
    uint8_t d;
    for ( d = 0; d < CAMERA_PAIR_NUM; ++d )
    {
        DJI_Guidance_obstacle_distance.distance[d]=DJI_UART_temp_buf[9+2*d]<<8|DJI_UART_temp_buf[8+2*d];
    }

}

void DJI_Guidance_uart_protocl(uint8_t c)
{
    static uint16_t size_len=0,step_flag=0,CRC16=0xffff;//最大字节可达1007
    static uint32_t CRC32=0xffffffff;
    static uint8_t cmd_id;//	e_image: 0x00; e_imu: 0x01; e_ultrasonic: 0x02; e_velocity: 0x03; e_obstacle_distance: 0x04
    if(step_flag==0&&c==0xaa)//协议头
    {
        CRC16=CRC_INIT;//初始值
        CRC32=0xffffffff;
        CRC16=CRC16_(CRC16,c);//计算CRC
        size_len=0;
        step_flag++;
    } else if(step_flag==1)//80  高2位属于长度的低二位
    {
        DJI_Guidance_head.m_length=0x0003&(c>>6);//取高八位做低二位,同时清零
        DJI_Guidance_head.m_version=0x3f&c;//取低八位
        CRC16=CRC16_(CRC16,c);//计算CRC
        step_flag++;
    } else if(step_flag==2) { //0bLEN的3-10位
        DJI_Guidance_head.m_length|=0xffc&(c<<2);//8+2位
        CRC16=CRC16_(CRC16,c);//计算CRC
        step_flag++;
    } else if(step_flag>=3&&step_flag<=7) { //跳过RES
        CRC16=CRC16_(CRC16,c);//计算CRC
        step_flag++;
    } else if(step_flag==8)
    {
        CRC16=CRC16_(CRC16,c);//计算CRC
        DJI_Guidance_head.m_seq_num=0x00ff&c;
        step_flag++;
    } else if(step_flag==9)
    {
        CRC16=CRC16_(CRC16,c);//计算CRC
        DJI_Guidance_head.m_seq_num|=0xff00&c;
        step_flag++;
    } else if(step_flag==10)
    {
        CRC16=CRC16_(CRC16,c);//计算CRC
        DJI_Guidance_head.m_header_checksum=0xff00&(c<<8);
        step_flag++;
    } else if(step_flag==11)
    {
        CRC16=CRC16_(CRC16,c);//计算CRC
        DJI_Guidance_head.m_header_checksum|=0x00ff&c;
        if(CRC16!=0)//RCR16 error
            step_flag=0;
        else
            step_flag++;
    } else if(step_flag>=12&&step_flag<=DJI_Guidance_head.m_length-5)//接收数据
    {
        //接收数据帧

        if(step_flag==12&&c!=0)step_flag=0;//数据第一个字节为0，固定
        else if(step_flag==13)
            cmd_id=c;
        else {
            if(size_len==0) {
                if(cmd_id==e_velocity) {
                    size_len=sizeof(DJI_Guidance_vo);
                } else if(cmd_id==e_obstacle_distance) { //障碍距离
                    size_len=sizeof(DJI_Guidance_obstacle_distance);
                } else if(cmd_id==e_ultrasonic) { //超声波距离
                    size_len=sizeof(DJI_Guidance_ultrasonic);
                } else {
                    size_len=0;
                }
            }
            //计算得到数据长度，开始拷贝数据到缓存区
            if(size_len>step_flag-14)
                DJI_UART_temp_buf[step_flag-14]=c;//将需要的数据放入缓存中
            else
                1==1;

        }
        step_flag++;
    } else if(step_flag==DJI_Guidance_head.m_length-4) { //校验码
        DJI_Guidance_head.m_data_checksum=0x000000ff&(c<<0);
        step_flag++;
    } else if(step_flag==DJI_Guidance_head.m_length-3) { //校验码
        DJI_Guidance_head.m_data_checksum|=0x0000ff00&(c<<8);
        step_flag++;
    } else if(step_flag==DJI_Guidance_head.m_length-2) { //校验码
        DJI_Guidance_head.m_data_checksum|=0x00ff0000&(c<<16);
        step_flag++;
    } else if(step_flag==DJI_Guidance_head.m_length-1) { //校验码
        DJI_Guidance_head.m_data_checksum|=0xff000000&(c<<24);
        switch(cmd_id)
        {
        case e_velocity:
            DJI_Analysis_vel();
            break;
        case e_ultrasonic:
            DJI_Analysis_distance();
            break;
        case e_obstacle_distance:

            DJI_Analysis_obstacle_distance();
            break;
        }
        step_flag=0;
    } else {
        step_flag=0;//接收完成
    }
}



