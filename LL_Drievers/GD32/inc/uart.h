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
#ifndef __HP_USART_H__
#define __HP_USART_H__
#include "guetfly_data_types.h"
#include "stdint.h"
#include "stdio.h"
void init_uarts(type_usart_handle handle,uint32_t bound);
void uart_send_buff(type_usart_handle handle,uint8_t ch);
#endif

