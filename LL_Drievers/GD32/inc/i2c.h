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

#ifndef __HP_I2C_H__
#define __HP_I2C_H__

#include "guetfly_data_types.h"
#include "gd32f3x0_gpio.h"
#include "stdint.h"

#define I2C_READ 	1
#define I2C_WRITE 0
#define ACK				0
#define NACK			1

/**********************/
//IMU I2C
#define I2C_SCL1_Pin 		GPIO_PIN_15
#define I2C_SDA1_Pin		GPIO_PIN_14
#define I2C_SCL1_Port 	GPIOB
#define I2C_SDA1_Port 	GPIOB
//以上部分匹配引脚时修改
#define I2C_SCL1_L  				gpio_bit_reset(I2C_SCL1_Port,I2C_SCL1_Pin)
#define I2C_SCL1_H					gpio_bit_set(I2C_SCL1_Port,I2C_SCL1_Pin)
#define I2C_SDA1_L					gpio_bit_reset(I2C_SDA1_Port,I2C_SDA1_Pin)
#define I2C_SDA1_H					gpio_bit_set(I2C_SDA1_Port,I2C_SDA1_Pin)
#define I2C_READ_SDA				gpio_input_bit_get(I2C_SDA1_Port,I2C_SDA1_Pin)
#define I2C_SDA_MODE_OUT		gpio_mode_set(I2C_SDA1_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,I2C_SDA1_Pin)
#define I2C_SDA_MODE_IN			gpio_mode_set(I2C_SDA1_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,I2C_SDA1_Pin)

//EEPROM I2C
#define I2C_SCL2_Pin 		GPIO_PIN_12
#define I2C_SDA2_Pin		GPIO_PIN_13
#define I2C_SCL2_Port 	GPIOB
#define I2C_SDA2_Port 	GPIOB

#define I2C_SCL2_L  				gpio_bit_reset(I2C_SCL2_Port,I2C_SCL2_Pin)
#define I2C_SCL2_H					gpio_bit_set(I2C_SCL2_Port,I2C_SCL2_Pin)
#define I2C_SDA2_L					gpio_bit_reset(I2C_SDA2_Port,I2C_SDA2_Pin)
#define I2C_SDA2_H					gpio_bit_set(I2C_SDA2_Port,I2C_SDA2_Pin)
#define I2C_READ2_SDA				gpio_input_bit_get(I2C_SDA2_Port,I2C_SDA2_Pin)
#define I2C_SDA2_MODE_OUT		    gpio_mode_set(I2C_SDA2_Port, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP,I2C_SDA2_Pin)
#define I2C_SDA2_MODE_IN			gpio_mode_set(I2C_SDA2_Port, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP,I2C_SDA2_Pin)


void LL_i2c_init(type_i2c_handle handle);
uint8_t LL_i2c_read_reg(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t *data);
uint8_t LL_i2c_write_reg(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t	data);
uint8_t LL_i2c_read_buff(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t * buff,uint16_t size);
uint8_t LL_i2c_write_buff(type_i2c_handle handle,uint8_t addr,uint8_t reg,uint8_t * buff,uint16_t size);
uint8_t Single_ReadI2C(uint8_t , uint8_t );
bool Single_WriteI2C(uint8_t , uint8_t , uint8_t );
#define I2C_Start 
#endif
