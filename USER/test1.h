#ifndef __serial_H__
#define __serial_H__

void Init_SerialComm(void);
void UartSendChar(unsigned char ch);
void UartSendString(unsigned char *p,unsigned int strlong);
//static void UartIntrruptService(void);



#endif