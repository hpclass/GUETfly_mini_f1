#include "stm32f10x.h"
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MultiWii.h"
#include "usart.h"
#include "usb_ch341.h"

extern conf_t conf;
extern flags_struct_t f;
static  uint16_t serialHeadRX[UART_NUMBER],serialTailRX[UART_NUMBER];
uint8_t serialBufferRX[UART_NUMBER][RX_BUFFER_SIZE];
static  uint16_t serialHeadTX[UART_NUMBER],serialTailTX[UART_NUMBER];
static  uint8_t serialBufferTX[UART_NUMBER][TX_BUFFER_SIZE];


// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
//STM32�Ĵ��ڷ��Ͳ�������
// *******************************************************
void test_Uart()
{
    u16 i=TX_BUFFER_SIZE;
    u8 j=1;

    //serialHeadTX[1] =i;

    serialHeadTX[j] =0;
    serialTailTX[j] =1;
    i=TX_BUFFER_SIZE;
    while(--i)
    {
        serialBufferTX[j][i]=i;

    }
    UartSendData(j);

    1==1;
}



void UartSendData(uint8_t port) {
    u8 res;
    //stm32 add
    switch (port) {
    case 0:

//            while(serialHeadTX[0] != serialTailTX[0]) {//��ε���Ч�ʵ���
//                if (++serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
//                USB_send((u8 *)&serialBufferTX[0][serialTailTX[0]],1);
//            }

        /*
        	USB_send((u8 *)&serialBufferTX[0][serialTailTX[0]+1],serialHeadTX[0]-serialTailTX[0]);
        	serialTailTX[0]=serialHeadTX[0];
        	if (serialTailTX[0] >= TX_BUFFER_SIZE) serialTailTX[0] = 0;
        */
        if(serialHeadTX[0] != serialTailTX[0])
        {
            if(serialHeadTX[0] > serialTailTX[0])//ͷ��βǰ�棬һ�ο��Է���
            {
                USB_send((u8 *)&serialBufferTX[0][serialTailTX[0]],serialHeadTX[0]-serialTailTX[0]);
                serialTailTX[0]=serialHeadTX[0];
            } else {															//ͷ��β���棬������
                USB_send((u8 *)&serialBufferTX[0][serialTailTX[0]],TX_BUFFER_SIZE-serialTailTX[0]);//β�������β
                USB_send((u8 *)&serialBufferTX[0][0],serialHeadTX[0]);//����ͷ��ͷ
                serialTailTX[0]=serialHeadTX[0];//�������
            }

        }
        break;
    case 1:
        while(serialHeadTX[1] != serialTailTX[1]) {
            if (++serialTailTX[1] >= TX_BUFFER_SIZE) serialTailTX[1] = 0;
            while((USART1->SR&0X40)==0);//ѭ������,ֱ���������
            res = (u8) serialBufferTX[1][serialTailTX[1]];
            USART1->DR=res;
        }
        break;
    case 2:
        while(serialHeadTX[2] != serialTailTX[2]) {
            if (++serialTailTX[2] >= TX_BUFFER_SIZE) serialTailTX[2] = 0;
            while((USART2->SR&0X40)==0);//ѭ������,ֱ���������
            res = (u8) serialBufferTX[2][serialTailTX[2]];
            USART2->DR=res;
        }
        break;
    case 3:
        while(serialHeadTX[3] != serialTailTX[3]) {
            if (++serialTailTX[3] >= TX_BUFFER_SIZE) serialTailTX[3] = 0;
            while((USART3->SR&0X40)==0);//ѭ������,ֱ���������
            res = (u8) serialBufferTX[3][serialTailTX[3]];
            USART3->DR=res;
        }
        break;
    case 4:
        while(serialHeadTX[4] != serialTailTX[4]) {
            if (++serialTailTX[4] >= TX_BUFFER_SIZE) serialTailTX[4] = 0;
            while((UART4->SR&0X40)==0);//ѭ������,ֱ���������
            res = (u8) serialBufferTX[4][serialTailTX[4]];
            UART4->DR=res;
        }

        break;
    case 5:
        while(serialHeadTX[5] != serialTailTX[5]) {
            if (++serialTailTX[5] >= TX_BUFFER_SIZE) serialTailTX[5] = 0;
            while((UART5->SR&0X40)==0);//ѭ������,ֱ���������
            res = (u8) serialBufferTX[5][serialTailTX[5]];
            UART5->DR=res;
        }
        break;


    }
}

#if defined(GPS_SERIAL)
bool SerialTXfree(uint8_t port) {
    return (serialHeadTX[port] == serialTailTX[port]);
}
#endif
void SerialEnd(uint8_t port) {
    switch (port) {

    }
}

// we don't care about ring buffer overflow (head->tail) to avoid a test condition : data is lost anyway if it happens
void store_uart_in_buf(uint8_t data, uint8_t portnum) {
    serialBufferRX[portnum][serialHeadRX[portnum]++] = data;
    if (serialHeadRX[portnum] >= RX_BUFFER_SIZE)
        serialHeadRX[portnum] = 0;


    //serialHeadRX[portnum] = h;
}




uint8_t SerialRead(uint8_t port) {
    /*
    uint8_t t = serialTailRX[port];
    uint8_t c = serialBufferRX[t][port];
    if (serialHeadRX[port] != t) {
    if (++t >= RX_BUFFER_SIZE) t = 0;
    serialTailRX[port] = t;
    }
    */
    uint8_t c = serialBufferRX[port][serialTailRX[port]];
    serialTailRX[port]++;
    //serialBufferRX[port][t]=0;// �������㣬ԭ��û��
    if (serialHeadRX[port] != serialTailRX[port]) {
        if (serialTailRX[port] >= RX_BUFFER_SIZE)
            serialTailRX[port] = 0;


    }

    return c;
}

#if defined(SERIAL_RX)
uint8_t SerialPeek(uint8_t port) {
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port])) return c;
    else return 0;
}
#endif

uint16_t SerialAvailable(uint8_t port) {//stm32 add

    if(serialHeadRX[port] >= serialTailRX[port])//δ���껺��
    {
        return serialHeadRX[port] - serialTailRX[port];
    } else { //�Ѵ��껺��
        return RX_BUFFER_SIZE-serialTailRX[port]+serialHeadRX[port];

    }
    //  return ((uint16_t)( serialHeadRX[port] - serialTailRX[port] ))%RX_BUFFER_SIZE;//��ֵ����
}

uint16_t SerialUsedTXBuff(uint8_t port) {
    if(serialHeadTX[port] >= serialTailTX[port])//δ���껺��
    {
        return serialHeadTX[port] - serialTailTX[port];
    } else { //�Ѵ��껺��
        return TX_BUFFER_SIZE-serialTailTX[port]+serialHeadTX[port];
    }

    //return ((uint16_t)(ABS(serialHeadTX[port] - serialTailTX[port]) ))%TX_BUFFER_SIZE;
}

void SerialSerialize(uint8_t port,uint8_t a) {
    uint8_t t = serialHeadTX[port];
    if (++t >= TX_BUFFER_SIZE) t = 0;
    serialBufferTX[port][t] = a;
    serialHeadTX[port] = t;
}

void SerialWrite(uint8_t port,uint8_t c) {
    SerialSerialize(port,c);
    UartSendData(port);
}