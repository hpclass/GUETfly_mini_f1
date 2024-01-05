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
#ifndef __HP_LEDS_H__
#define __HP_LEDS_H__
#include "gd32f3x0_libopt.h"

#define LED1_ON  			gpio_bit_reset(GPIOA, GPIO_PIN_0);
#define LED1_OFF 			gpio_bit_set(GPIOA, GPIO_PIN_0);
#define LED2_ON  	gpio_bit_reset(GPIOA, GPIO_PIN_1);
#define LED2_OFF 	gpio_bit_set(GPIOA, GPIO_PIN_1);
#define LED1_TOGOGLE gpio_bit_toggle(GPIOA, GPIO_PIN_0);
#define LED2_TOGOGLE gpio_bit_toggle(GPIOA, GPIO_PIN_1);


void init_ws2812(void);
void init_leds(void);
void ws2812_write_byte(uint8_t byte);
#endif
