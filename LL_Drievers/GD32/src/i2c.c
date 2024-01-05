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

#include "i2c.h"
//关于I2C的可以浏览此链接：
//http://news.eeworld.com.cn/mcu/article_2017022033883.html
static void i2c_delay_us(uint16_t delays)
{
    while(delays--)
    {
        //Nop
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
        __ASM volatile ("nop");
    }
}
void i2c_init(type_i2c_handle handle)
{
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        rcu_periph_clock_enable(RCU_GPIOB);
        gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_14|GPIO_PIN_15);
        gpio_bit_set(GPIOB, GPIO_PIN_14|GPIO_PIN_15);


        break;

    case HANDLE_I2C_EEPROM:
        rcu_periph_clock_enable(RCU_GPIOB);
        gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_12|GPIO_PIN_13);
        gpio_bit_set(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

        break;
    default://未定义

        break;
    }
}
static void i2c_send_ack(type_i2c_handle handle,uint8_t ack)
{
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        ack==0?I2C_SDA1_L:I2C_SDA1_H;//ack=0；NACK=1;
        I2C_SCL1_H;
        i2c_delay_us(10);
        I2C_SCL1_L;
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
}
static uint8_t i2c_wait_ack(type_i2c_handle handle)//ack=0,NAK=1;
{
    uint8_t ack;
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        //i2c_delay_us(10);
				I2C_SDA1_L;
        I2C_SDA_MODE_IN;
        I2C_SCL1_H;
        i2c_delay_us(10);
        ack=I2C_READ_SDA;
        I2C_SDA_MODE_OUT;
        //I2C_SDA1_H;
        i2c_delay_us(10);
        I2C_SCL1_L;
        //i2c_delay_us(10);
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
    return ack;
}
static uint8_t i2c_start(type_i2c_handle handle)
{
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        I2C_SCL1_H;
        I2C_SDA1_H;
        i2c_delay_us(10);
        I2C_SDA1_L;
        i2c_delay_us(10);
        I2C_SCL1_L;
        i2c_delay_us(10);
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
    return 0;
}
static uint8_t i2c_stop(type_i2c_handle handle)
{
    switch(handle)
    {
    case HANDLE_I2C_MPU:
				I2C_SDA_MODE_OUT;
        I2C_SCL1_L;
        I2C_SDA1_L;
        i2c_delay_us(10);
        I2C_SCL1_H;
        i2c_delay_us(10);
        I2C_SDA1_H;
        i2c_delay_us(10);
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
    return 0;
}
static uint8_t i2c_send_byte(type_i2c_handle handle,uint8_t data)
{
    uint8_t i=8;
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        while(i--)
        {
            ((data&(0x01<<i))!=0)?(I2C_SDA1_H):(I2C_SDA1_L);//逐位比对
            I2C_SCL1_H;
            i2c_delay_us(10);
            I2C_SCL1_L;
            i2c_delay_us(10);
        }
        I2C_SCL1_H;
        I2C_SDA1_H;
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
    return 0;
}
static uint8_t i2c_read_byte(type_i2c_handle handle)
{
    uint8_t i=8,data=0;
    switch(handle)
    {
    case HANDLE_I2C_MPU:
        I2C_SDA_MODE_IN;
        while(i--)
        {
            I2C_SCL1_H;
            i2c_delay_us(10);//高电平期间数据会保持稳定，等延时结束再取样
            ((I2C_READ_SDA)!=0)?(data|=0x01<<i):(data&=~(0x01<<i));//逐位比对
            I2C_SCL1_L;
            i2c_delay_us(10);
        }
        I2C_SDA_MODE_OUT;
        I2C_SCL1_L;
        I2C_SDA1_L;
        i2c_delay_us(10);
        break;
    case HANDLE_I2C_EEPROM:
        break;
    default:
        break;
    }
    return data;
}
uint8_t  test_i2c(type_i2c_handle handle,uint8_t addr)
{
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_WRITE);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
		i2c_stop(handle);//结束传输
		return 0;
	}
uint8_t i2c_read_reg(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t *data)//单字读取
{
    //S->send addr W->wait ACK->send REG ->wait ACK ->S->send addr R ->Read byte ->Send NACK ->STOP
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_WRITE);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }

    i2c_send_byte(handle,reg);//发送寄存器地址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }

    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_READ);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    *data=i2c_read_byte(handle);//接收数据
    i2c_send_ack(handle,NACK);//发送ACK
    i2c_stop(handle);//结束传输
    return 0;
}
uint8_t i2c_write_reg(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t	data)//单字写入
{
    //S->Send addr W->wait ACK->send reg->wait ack->send data ->wait ACK ->stop
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_WRITE);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    i2c_send_byte(handle,reg);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    i2c_send_byte(handle,data);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    i2c_stop(handle);//结束传输
    return 0;
}

uint8_t i2c_read_buff(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t * buff,uint16_t size)//连续读取
{
//S->send addr W ->wait ACK ->send REG ->wait ACK->S->send addr R->read data->send ACK->...->send NACK ->STOP
    if(!size||!buff)
        return 0xff;
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_WRITE);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }

    i2c_send_byte(handle,reg);//发送寄存器地址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_READ);//发送I2C地址，寻址
    if(i2c_wait_ack(handle))//等待ack
    {
        i2c_stop(handle);//结束传输
        return 0xff;
    }
    while(--size)
    {
        *(buff++)=i2c_read_byte(handle);
        i2c_send_ack(handle,ACK);//发送ACK
    }
    *buff=i2c_read_byte(handle);
    i2c_send_ack(handle,NACK);//发送ACK
    i2c_stop(handle);//结束传输
    return 0;
}
uint8_t i2c_write_buff(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t * buff,uint16_t size)//连续写入
{
//S->addr W ->wait ACK->send REG ->wait ACK->send data->wait ACK ->...->wait ack->stop
    i2c_start(handle);//开始传输

    i2c_send_byte(handle,(addr<<1)|I2C_WRITE);//发送I2C地址，寻址
    if(!i2c_wait_ack(handle))//等待ack
    {
        return 0xff;
    }
    i2c_send_byte(handle,reg);//发送I2C地址，寻址
    if(!i2c_wait_ack(handle))//等待ack
    {
        return 0xff;
    }
    while(size--)
    {
        i2c_send_byte(handle,*(buff++));//发送I2C地址，寻址
        if(!i2c_wait_ack(handle))//等待ack
        {
            return 0xff;
        }
    }
    i2c_stop(handle);//结束传输
    return 0;
}
