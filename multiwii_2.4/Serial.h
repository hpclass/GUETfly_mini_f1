#ifndef SERIAL_H_
#define SERIAL_H_
#include "sys.h"
#if defined(MEGA)
#define UART_NUMBER 4
#elif defined(PROMICRO)
#define UART_NUMBER 2
#elif defined(GUET_FLY_V1)
#define UART_NUMBER 6  //·É¿Ø°å6¸ö´®¿Ú
#else
//#define UART_NUMBER 1
//stm32 add
#define UART_NUMBER 4
#endif
#define RX_BUFFER_SIZE 512 // 256 RX buffer is needed for GPS communication (64 or 128 was too short)
#define TX_BUFFER_SIZE 128

void    SerialOpen(uint8_t port, uint32_t baud);
uint8_t SerialRead(uint8_t port);
void    SerialWrite(uint8_t port,uint8_t c);
uint16_t SerialAvailable(uint8_t port);
void    SerialEnd(uint8_t port);
uint8_t SerialPeek(uint8_t port);
bool    SerialTXfree(uint8_t port);
uint16_t SerialUsedTXBuff(uint8_t port);
void    SerialSerialize(uint8_t port,uint8_t a);
void    UartSendData(uint8_t port);

void SerialWrite16(uint8_t port, int16_t val);
void store_uart_in_buf(uint8_t data, uint8_t portnum);
#endif /* SERIAL_H_ */
