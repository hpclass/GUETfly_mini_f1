#include <stm32f10x_lib.h>
#include "sys.h"
#include "usart.h"		
#include "delay.h"		   
#include "usb_ch341.h"
//This demo will send two string every 700ms and send any string it receive.(Receive function is in USB_CH341.c Line 44.)
int main6(void)
{	
    char string1[]="hello,world -w-\r\n";
    char string2[]="----by blackmiaool\r\n\r\n";



	while(1)
	{
        USB_send((u8 *)string1,sizeof(string1)-1);//"-1" to remove last 0 in C's string
        USB_send((u8 *)string2,sizeof(string2)-1); 
        delay_ms(700);
	}
}




