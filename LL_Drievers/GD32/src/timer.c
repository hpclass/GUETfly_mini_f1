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
#include "guetfly_data_types.h"
#include "timer.h"
#include "RX.h"
static uint16_t sys_tick_base_div=0;
int16_t tmp_buff[256];
uint8_t tmp_flag=0;
void systick_config(void)//嘀嗒时钟
{
    sys_tick_base_div=(SystemCoreClock/ 1000U/ 1000U);
    /* setup systick timer for 1000Hz interrupts */
    if(SysTick_Config(SystemCoreClock / 1000U)) {
        /* capture error */
        while (1) {
        }
    }
    /* configure the systick handler priority */
    /* Function successful */
    NVIC_SetPriority(SysTick_IRQn, 0x00U);
}
void init_timer2(uint16_t period,uint16_t psc)
{
    timer_oc_parameter_struct timer_ocintpara;
    timer_parameter_struct timer_initpara;
    //打开RCC
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_GPIOB);
    //配置GPIO
    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0);

    gpio_mode_set(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_1);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_1);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);

    gpio_mode_set(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOA, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);
    //配置功能复用
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_0);
    gpio_af_set(GPIOB, GPIO_AF_1, GPIO_PIN_1);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_6);
    gpio_af_set(GPIOA, GPIO_AF_1, GPIO_PIN_7);

    rcu_periph_clock_enable(RCU_TIMER2);//打开timer2时钟

    timer_deinit(TIMER2);

    timer_initpara.prescaler         = psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER2,&timer_initpara);

    /* CH1,CH2 and CH3 configuration in PWM mode */
    timer_ocintpara.outputstate  = TIMER_CCX_ENABLE;
    timer_ocintpara.outputnstate = TIMER_CCXN_DISABLE;
    timer_ocintpara.ocpolarity   = TIMER_OC_POLARITY_HIGH;
    timer_ocintpara.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
    timer_ocintpara.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
    timer_ocintpara.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;

    timer_channel_output_config(TIMER2,TIMER_CH_0,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_1,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_2,&timer_ocintpara);
    timer_channel_output_config(TIMER2,TIMER_CH_3,&timer_ocintpara);
    //ch0
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,1000);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_0,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_0,TIMER_OC_SHADOW_DISABLE);
    //ch1
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,1000);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_1,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_1,TIMER_OC_SHADOW_DISABLE);
    //ch2
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_2,1000);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_2,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_2,TIMER_OC_SHADOW_DISABLE);
    //ch3
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,1000);
    timer_channel_output_mode_config(TIMER2,TIMER_CH_3,TIMER_OC_MODE_PWM0);
    timer_channel_output_shadow_config(TIMER2,TIMER_CH_3,TIMER_OC_SHADOW_DISABLE);

    /* auto-reload preload enable */
    timer_auto_reload_shadow_enable(TIMER2);
    /* auto-reload preload enable */
    timer_enable(TIMER2);

}


static uint16_t soft_pwm[4];
static uint16_t timer_int_count;
void soft_pwm_init(uint16_t period,uint16_t psc)//软件PWM不是很准的，驱动舵机还行电机就不稳了
{
    timer_parameter_struct timer_initpara;
    soft_pwm[0]=soft_pwm[1]=soft_pwm[2]=soft_pwm[3]=100;
    timer_int_count=0;

    rcu_periph_clock_enable(RCU_GPIOB);
    //配置GPIO
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_6);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_6);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_7);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_7);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_8);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_8);

    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_PIN_9);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_9);

    rcu_periph_clock_enable(RCU_TIMER13);//打开timer2时钟

    timer_deinit(TIMER13);

    timer_initpara.prescaler         = psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER13,&timer_initpara);
    timer_interrupt_flag_clear(TIMER13, TIMER_INT_FLAG_UP);

    timer_interrupt_enable(TIMER13,TIMER_INT_UP);
    nvic_irq_enable(TIMER13_IRQn,0,2);
    timer_enable(TIMER13);
}

void pwm_output()//把缓存的数据直接输出
{
    //硬件PWM输出
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_0,0);
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_1,0);
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_2,0);
    timer_channel_output_pulse_value_config(TIMER2,TIMER_CH_3,0);

}
void servos_output(uint16_t *buff)//把缓存的数据直接输出
{
    //模拟PWM输出
    soft_pwm[0]=buff[0];
    soft_pwm[1]=buff[1];
    soft_pwm[2]=buff[2];
    soft_pwm[3]=buff[3];
}

void TIMER13_IRQHandler()//软件PWM
{
    //1us的中断单片机很难处理的过来的，10us还行，精度自行计算
    if(RESET != timer_interrupt_flag_get(TIMER13,TIMER_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER13,TIMER_FLAG_UP);
        timer_int_count++;
        if(timer_int_count>soft_pwm[0])
        {
            gpio_bit_reset(GPIOB, GPIO_PIN_6);
        }
        if(timer_int_count>soft_pwm[1])
        {
            gpio_bit_reset(GPIOB, GPIO_PIN_7);
        }
        if(timer_int_count>soft_pwm[2])
        {
            gpio_bit_reset(GPIOB, GPIO_PIN_8);
        }
        if(timer_int_count>soft_pwm[3])
        {
            gpio_bit_reset(GPIOB, GPIO_PIN_9);
        }
        if(timer_int_count>2000)//2000*10us=20ms
        {
            timer_int_count=0;
            if(0!=soft_pwm[0])
            {
                gpio_bit_set(GPIOB, GPIO_PIN_6);
            }
            if(0!=soft_pwm[1])
            {
                gpio_bit_set(GPIOB, GPIO_PIN_7);
            }
            if(0!=soft_pwm[2])
            {
                gpio_bit_set(GPIOB, GPIO_PIN_8);
            }
            if(0!=soft_pwm[3])
            {
                gpio_bit_set(GPIOB, GPIO_PIN_9);
            }
        }
    }
}

/*******计时器部分**********/
static uint32_t jiffies=0;//ms
static uint64_t time_t_us=0;//us
void msleep(uint32_t sleep)
{
    uint64_t this_time_t=millis();
    while(millis()-this_time_t<sleep)
    {
        //do sth when wait
    }
}
void usleep(uint32_t sleep)
{
    uint64_t this_time_t=micros();
    uint64_t t2=0;
    do {
        t2=micros();
    } while(t2-sleep<this_time_t);
}
void SysTick_Handler()//1ms中断累加
{
    jiffies++;
    time_t_us+=1000;
}
uint32_t millis()
{
    return jiffies;//直接返回毫秒
}
uint64_t micros()
{
    return time_t_us+(1000-SysTick->VAL/sys_tick_base_div);//这里还要加上systick的值,
    /*
    这个单片机的滴答时钟不能分频，只能重载，所以按照主频来重载，每84个计数为1us，注意，这个是向下计数的。递减的。
    */
}
/*************S.bus捕获函数*************/
void init_timer_help_sbus(uint16_t period,uint16_t psc)
{
    //正常来说是不需要这个函数来捕获Sbus的，但是GD32F330串口太少，且考虑到10000波特率也不快，
    //所以用定时器捕获SBUS也是可行的，
    //其实也是个软串口吧
    /*

    */
    timer_parameter_struct timer_initpara;
    rcu_periph_clock_enable(RCU_TIMER14);//打开timer2时钟

    timer_deinit(TIMER14);

    timer_initpara.prescaler         = psc;
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;
    timer_initpara.period            = period;
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;
    timer_initpara.repetitioncounter = 0;
    timer_init(TIMER14,&timer_initpara);
    timer_interrupt_flag_clear(TIMER14, TIMER_INT_FLAG_UP);

    timer_interrupt_enable(TIMER14,TIMER_INT_UP);
    nvic_irq_enable(TIMER14_IRQn,0,2);
    timer_enable(TIMER14);
}

/*************PPM捕获部分***************/

void init_capture_RX()
{
    configs.flag.RX_data_type=SBUS;
    rcu_periph_clock_enable(RCU_GPIOA);
    rcu_periph_clock_enable(RCU_CFGCMP);
    //配置GPIO
    gpio_mode_set(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, GPIO_PIN_8);
    nvic_irq_enable(EXTI4_15_IRQn, 2U, 1U);
    syscfg_exti_line_config(EXTI_SOURCE_GPIOA, EXTI_SOURCE_PIN8);
    //配置ppm
    switch(configs.flag.RX_data_type)
    {
    case PPM:
        exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_RISING);//上升沿采样
        break;
    case SBUS:
        exti_init(EXTI_8, EXTI_INTERRUPT, EXTI_TRIG_BOTH);//双边采样
        break;
    default:
        break;
    }
    exti_interrupt_flag_clear(EXTI_8);

}
RX_data_t RX_data;
void EXTI4_15_IRQHandler(void)
{
    uint16_t this_time=micros();
    uint16_t count_time;

    if (RESET != exti_interrupt_flag_get(EXTI_8)) {//接收到中断
        exti_interrupt_flag_clear(EXTI_8);

        switch(configs.flag.RX_data_type)
        {
        case PPM:
            //PPM 10通道的话所用时间 20ms一帧，包含IDEL.
            count_time=this_time-RX_data.PPM.RX_PPM_last_timer;
            if(count_time>3000)//收到同步帧
            {
                RX_data.PPM.count=0;
            } else {
                RX_data.PPM.RX_DATA[RX_data.PPM.count++]=count_time;//数据存入数组之中
                if(RX_data.PPM.count>=16)
                    RX_data.PPM.count=0;
            }
            RX_data.PPM.RX_PPM_last_timer=this_time;
            break;
        case SBUS:
            //看少了个0，是1000’00，所以还是sbus快
            count_time=this_time-RX_data.SBUS.RX_SBUS_last_timer;//计算与上次的时间差


            if(gpio_input_bit_get(GPIOA,GPIO_PIN_8)==RESET)
            {   //下降沿
                tmp_buff[tmp_flag++]=count_time;
                if(RX_data.SBUS.bit_count==0)//第一个下降沿
                {
                    count_time-=10;//删除起始位
                    RX_data.SBUS.bit_count=1;//表示收到起始位
                }
                if(RX_data.SBUS.bit_count<9) {
                    uint8_t this_count=count_time/9;//计算高电平的个数
                    switch(this_count)//用空间换时间，中断里面尽快做事
                    {
                    case 0:
                        RX_data.SBUS.byte|=0;
                        break;
                    case 1:
                        RX_data.SBUS.byte|=0x01<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=1;
                        break;
                    case 2:
                        RX_data.SBUS.byte|=0x03<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=2;
                        break;
                    case 3:
                        RX_data.SBUS.byte|=0x07<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=3;
                        break;
                    case 4:
                        RX_data.SBUS.byte|=0x0f<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=4;
                        break;
                    case 5:
                        RX_data.SBUS.byte|=0x1f<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=5;
                        break;
                    case 6:
                        RX_data.SBUS.byte|=0x3f<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=6;
                        break;
                    case 7:
                        RX_data.SBUS.byte|=0x7f<<(RX_data.SBUS.bit_count-1);
                        RX_data.SBUS.bit_count+=7;
                        break;
                    case 8:
                        RX_data.SBUS.byte|=0xff;
                        RX_data.SBUS.bit_count+=8;
                        break;
                    default :
                        break;
                    }
                }
            } else {
                //上升沿
                if(count_time>6000)//比快速Sbus时间长意味着已经结束了一帧，归零
                {
                    RX_data.SBUS.bit_count=0;//bit计数归零
                    RX_data.SBUS.byte_count=0;//byte计数归零
                    RX_data.SBUS.byte=0;//byte归零
                    tmp_flag=0;
                    //RX_data.SBUS.RX_SBUS_last_timer=RX_data.SBUS.RX_SBUS_last_trig_timer=this_time;
                } 
                    if(RX_data.SBUS.bit_count==0)
                    {
                        init_timer_help_sbus(85,84);
                    } else {
                        if(RX_data.SBUS.bit_count<9) {
                            uint8_t this_count=count_time/9;//
                            RX_data.SBUS.bit_count+=this_count;
                        }
                        tmp_buff[tmp_flag++]=-count_time;
                    }

                
            }
            RX_data.SBUS.RX_SBUS_last_timer=this_time;
            break;
        default :
            break;

        }
    }
}

void TIMER14_IRQHandler()
{
    uint16_t this_time=micros();
    uint16_t count_time;
    if(RESET != timer_interrupt_flag_get(TIMER14,TIMER_FLAG_UP))
    {
        timer_interrupt_flag_clear(TIMER14,TIMER_FLAG_UP);
        timer_disable(TIMER14);
        count_time=this_time-RX_data.SBUS.RX_SBUS_last_timer;//计算与上次的时间差
        tmp_buff[tmp_flag++]=-count_time;
        if(RX_data.SBUS.bit_count<9) {
            uint8_t this_count=count_time/9;//
            RX_data.SBUS.bit_count+=this_count;
        }
        RX_data.SBUS.bit_count=0;//bit计数归零
        RX_data.SBUS.buff[RX_data.SBUS.byte_count++]=~RX_data.SBUS.byte;//把数据存入队列中,数据是反相的
        RX_data.SBUS.byte=0;//byte归零
				if(RX_data.SBUS.byte_count==25)//25 byte == one frame
				{
					RX_data.SBUS.byte_count=0;
					cover_sbus_buff_to_ch(RX_data.SBUS.buff);
				}

    }
}
void HAL_delay(uint32_t time)
{
    msleep(time);

}
