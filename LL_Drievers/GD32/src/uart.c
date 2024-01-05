/*
Copyright 2021 huangpeng

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/
#include "gd32f3x0_libopt.h"
#include "Protocol.h"
#include "uart.h"

void init_uarts(type_usart_handle handle,uint32_t bound)
{
    switch(handle)
    {
    case HANDLE_usart_gps:
        /* enable USART and GPIOA clock */
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_USART0);
        /* configure the USART0 Tx pin and USART1 Tx pin */
        gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_10);
        gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_9);
        /* configure USART0 Tx as alternate function push-pull */
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_9);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);
        /* configure USART1 Tx as alternate function push-pull */
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_10);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_10);
        /* USART0 and USART1 baudrate configuration */
        usart_baudrate_set(USART0, bound);
        /* Enable USART0 Half Duplex Mode*/
       // usart_halfduplex_enable(USART0);
        /* configure USART transmitter and receiver*/
        usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
        usart_receive_config(USART0, USART_RECEIVE_ENABLE);
				nvic_irq_enable(USART0_IRQn, 1, 0);
				usart_interrupt_enable(USART0, USART_INT_RBNE);
        /* enable USART */
        usart_enable(USART0);
        usart_data_receive(USART0);
        break;
    case HANDLE_usart_radio:
        /* enable USART and GPIOA clock */
        rcu_periph_clock_enable(RCU_GPIOA);
        rcu_periph_clock_enable(RCU_USART1);
        /* configure the USART1 Tx pin  */
        gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_2);//tx1
        gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_3);//rx1
        /* configure USART1 Tx as alternate function push-pull */
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_2);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);
        /* configure USART1 Rx as alternate function push-pull */
        gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO_PIN_3);
        gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_3);
        /* USART1 and USART1 baudrate configuration */
        usart_baudrate_set(USART1, bound);
        /* Enable USART1 Half Duplex Mode*/
       // usart_halfduplex_enable(USART1);
        /* configure USART transmitter and receiver*/
        usart_transmit_config(USART1, USART_TRANSMIT_ENABLE);
        usart_receive_config(USART1, USART_RECEIVE_ENABLE);
				nvic_irq_enable(USART1_IRQn, 0, 0);
				usart_interrupt_enable(USART1, USART_INT_RBNE);
        /* enable USART */
        usart_enable(USART1);
        usart_data_receive(USART1);
        break;
    case HANDLE_usart_rx:
        //这里预留后面再写
        break;
    default :
        break;
    }

}


void USART0_IRQHandler(void)//串口接收中断
{
		 usart_interrupt_flag_clear(USART0, USART_INT_FLAG_ERR_ORERR); 
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)) {
        /* clear IDLE flag */
        store_uart_in_buf(usart_data_receive(USART0), HANDLE_usart_gps);
    }
}

void USART1_IRQHandler(void)//串口接收中断
{
	usart_interrupt_flag_clear(USART1, USART_INT_FLAG_ERR_ORERR); 
    if(RESET != usart_interrupt_flag_get(USART1, USART_INT_FLAG_RBNE)) {
        /* clear IDLE flag */
        store_uart_in_buf(usart_data_receive(USART1), HANDLE_usart_radio);
    }
}
//底层串口输出函数
void uart_send_buff(type_usart_handle handle,uint8_t ch)
{
		switch(handle)
    {
    case HANDLE_usart_gps:
			
				usart_data_transmit(USART0, (uint8_t) ch);
				while(RESET == usart_flag_get(USART0, USART_FLAG_TC));
				break;
		case HANDLE_usart_radio:
				usart_data_transmit(USART1,ch);
				while(RESET == usart_flag_get(USART1, USART_FLAG_TC));
			break;
		case HANDLE_usart_rx:
        //这里预留后面再写
        break;
    default :
        break;
		}
}

/* retarget the C library printf function to the USART */
#if 1
int fputc(int ch, FILE *f)
{
    usart_data_transmit(USART1, (uint8_t)ch);
    while(RESET == usart_flag_get(USART1, USART_FLAG_TC));

    return ch;
}
#endif
