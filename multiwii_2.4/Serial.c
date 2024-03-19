
#ifdef STM32F10X_MD
#include "stm32f10x.h"

#include "USB_CH341.H"
#include "usart.h"
#else
#include "gd32f3x0.h"
#include "guetfly_data_types.h"
#endif
#include "config.h"
#include "def.h"
#include "Serial.h"
#include "MultiWii.h"

extern conf_t conf;
extern flags_struct_t f;
static uint16_t serialHeadRX[UART_NUMBER], serialTailRX[UART_NUMBER];
uint8_t serialBufferRX[UART_NUMBER][RX_BUFFER_SIZE];
static uint16_t serialHeadTX[UART_NUMBER], serialTailTX[UART_NUMBER];
static uint8_t serialBufferTX[UART_NUMBER][TX_BUFFER_SIZE];

// *******************************************************
// Interrupt driven UART transmitter - using a ring buffer
// STM32的串口发送不在这里
// *******************************************************
void test_Uart()
{
    uint16_t i = TX_BUFFER_SIZE;
    uint8_t j = 1;

    // serialHeadTX[1] =i;

    serialHeadTX[j] = 0;
    serialTailTX[j] = 1;
    i = TX_BUFFER_SIZE;
    while (--i)
    {
        serialBufferTX[j][i] = i;
    }
    UartSendData(j);
}

void UartSendData(uint8_t port)
{
    uint8_t res;
    // stm32 add
    switch (port)
    {
#ifdef STM32F10X_MD
#if UART_NUMBER > 0
    case 0:
        if (serialHeadTX[0] != serialTailTX[0])
        {
            if (serialHeadTX[0] > serialTailTX[0]) // 头在尾前面，一次可以发完
            {
                USB_send((uint8_t *)&serialBufferTX[0][serialTailTX[0]], serialHeadTX[0] - serialTailTX[0]);
                serialTailTX[0] = serialHeadTX[0];
            }
            else
            {                                                                                               // 头在尾后面，发两次
                USB_send((uint8_t *)&serialBufferTX[0][serialTailTX[0]], TX_BUFFER_SIZE - serialTailTX[0]); // 尾到数组结尾
                USB_send((uint8_t *)&serialBufferTX[0][0], serialHeadTX[0]);                                // 数组头到头
                serialTailTX[0] = serialHeadTX[0];                                                          // 发送完成
            }
        }
        break;
#endif
#if UART_NUMBER > 1
    case 1:
        while (serialHeadTX[1] != serialTailTX[1])
        {
            if (++serialTailTX[1] >= TX_BUFFER_SIZE)
                serialTailTX[1] = 0;
            while ((USART1->SR & 0X40) == 0)
                ; // 循环发送,直到发送完毕
            res = (uint8_t)serialBufferTX[1][serialTailTX[1]];
            USART1->DR = res;
        }
        break;
#endif
#if UART_NUMBER > 2
    case 2:
        while (serialHeadTX[2] != serialTailTX[2])
        {
            if (++serialTailTX[2] >= TX_BUFFER_SIZE)
                serialTailTX[2] = 0;
            while ((USART2->SR & 0X40) == 0)
                ; // 循环发送,直到发送完毕
            res = (uint8_t)serialBufferTX[2][serialTailTX[2]];
            USART2->DR = res;
        }
        break;

#endif
#if UART_NUMBER > 3
    case 3:
        while (serialHeadTX[3] != serialTailTX[3])
        {
            if (++serialTailTX[3] >= TX_BUFFER_SIZE)
                serialTailTX[3] = 0;
            while ((USART3->SR & 0X40) == 0)
                ; // 循环发送,直到发送完毕
            res = (uint8_t)serialBufferTX[3][serialTailTX[3]];
            USART3->DR = res;
        }
        break;
#endif
#if UART_NUMBER > 4
    case 4:
        while (serialHeadTX[4] != serialTailTX[4])
        {
            if (++serialTailTX[4] >= TX_BUFFER_SIZE)
                serialTailTX[4] = 0;
            while ((UART4->SR & 0X40) == 0)
                ; // 循环发送,直到发送完毕
            res = (uint8_t)serialBufferTX[4][serialTailTX[4]];
            UART4->DR = res;
        }

        break;
#endif
#if UART_NUMBER > 5
    case 5:
        while (serialHeadTX[5] != serialTailTX[5])
        {
            if (++serialTailTX[5] >= TX_BUFFER_SIZE)
                serialTailTX[5] = 0;
            while ((UART5->SR & 0X40) == 0)
                ; // 循环发送,直到发送完毕
            res = (uint8_t)serialBufferTX[5][serialTailTX[5]];
            UART5->DR = res;
        }
        break;
#endif
#else
#if UART_NUMBER > 0
    case 0:
        while (serialHeadTX[port] != serialTailTX[port])
        {
            if (++serialTailTX[port] >= TX_BUFFER_SIZE)
                serialTailTX[port] = 0;
            res = (uint8_t)serialBufferTX[port][serialTailTX[port]];
            uart_send_buff(HANDLE_usart_gps, res);
        }
        break;
#endif
#if UART_NUMBER > 1
    case 1:
        while (serialHeadTX[port] != serialTailTX[port])
        {
            if (++serialTailTX[port] >= TX_BUFFER_SIZE)
                serialTailTX[port] = 0;
            res = (uint8_t)serialBufferTX[port][serialTailTX[port]];
            uart_send_buff(HANDLE_usart_radio, res);
        }
        break;
        
#endif
#endif
    }
}

#if defined(GPS_SERIAL)
bool SerialTXfree(uint8_t port)
{
    return (serialHeadTX[port] == serialTailTX[port]);
}
#endif
void SerialEnd(uint8_t port)
{
    switch (port)
    {
    }
}

// we don't care about ring buffer overflow (head->tail) to avoid a test condition : data is lost anyway if it happens
void store_uart_in_buf(uint8_t data, uint8_t portnum)
{
    serialBufferRX[portnum][serialHeadRX[portnum]++] = data;
    if (serialHeadRX[portnum] >= RX_BUFFER_SIZE)
        serialHeadRX[portnum] = 0;

    // serialHeadRX[portnum] = h;
}

uint8_t SerialRead(uint8_t port)
{
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
    // serialBufferRX[port][t]=0;// 读完清零，原来没有
    if (serialHeadRX[port] != serialTailRX[port])
    {
        if (serialTailRX[port] >= RX_BUFFER_SIZE)
            serialTailRX[port] = 0;
    }

    return c;
}

#if defined(SERIAL_RX)
uint8_t SerialPeek(uint8_t port)
{
    uint8_t c = serialBufferRX[serialTailRX[port]][port];
    if ((serialHeadRX[port] != serialTailRX[port]))
        return c;
    else
        return 0;
}
#endif

uint16_t SerialAvailable(uint8_t port)
{ // stm32 add

    if (serialHeadRX[port] >= serialTailRX[port]) // 未存完缓冲
    {
        return serialHeadRX[port] - serialTailRX[port];
    }
    else
    { // 已存完缓冲
        return RX_BUFFER_SIZE - serialTailRX[port] + serialHeadRX[port];
    }
    //  return ((uint16_t)( serialHeadRX[port] - serialTailRX[port] ))%RX_BUFFER_SIZE;//差值求余
}

uint16_t SerialUsedTXBuff(uint8_t port)
{
    if (serialHeadTX[port] >= serialTailTX[port]) // 未存完缓冲
    {
        return serialHeadTX[port] - serialTailTX[port];
    }
    else
    { // 已存完缓冲
        return TX_BUFFER_SIZE - serialTailTX[port] + serialHeadTX[port];
    }

    // return ((uint16_t)(ABS(serialHeadTX[port] - serialTailTX[port]) ))%TX_BUFFER_SIZE;
}

void SerialSerialize(uint8_t port, uint8_t a)
{
    uint8_t t = serialHeadTX[port];
    if (++t >= TX_BUFFER_SIZE)
        t = 0;
    serialBufferTX[port][t] = a;
    serialHeadTX[port] = t;
}

void SerialWrite(uint8_t port, uint8_t c)
{
    SerialSerialize(port, c);
    UartSendData(port);
}