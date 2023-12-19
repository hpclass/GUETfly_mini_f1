
#include "sys.h"
#include "delay.h"
unsigned long delays=0;
void delay_ms(unsigned long Counter)
{
    delays=Counter;
    while(delays);

}

void delay(unsigned long Counter)
{
    delays=Counter;
    while(delays);

}
void delay_us(unsigned int Counter)
{
    while(Counter--)
    {
        delay_1us();
    }

}

void delay_1us(void)
{   //122-45=77 1个大概7.7ns,1000个需要
    unsigned char i;
    for(i=0; i<10; i++)
    {
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
        __nop();
    }
}
