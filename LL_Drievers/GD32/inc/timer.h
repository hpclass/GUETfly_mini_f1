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
#ifndef __HP_TIMER_H__
#define __HP_TIMER_H__
#include "stdint.h"
typedef struct {
    uint8_t bit_count;
    uint8_t byte_count;
    uint8_t byte;
    uint16_t RX_SBUS_last_timer;
    uint16_t RX_SBUS_last_trig_timer;
    uint8_t buff[256];
} SBUS_data_t;
typedef struct {
    uint8_t count;
    uint8_t flag;
    uint16_t RX_PPM_last_timer;
    uint16_t RX_DATA[16];
} PPM_data_t;
typedef  union {
    PPM_data_t PPM;
    SBUS_data_t SBUS;
} RX_data_t;
extern RX_data_t RX_data;
void systick_config(void);
void init_timer2(uint16_t period,uint16_t psc);
void soft_pwm_init(uint16_t period,uint16_t psc);
void HAL_delay(uint32_t time);
uint64_t micros(void);
uint32_t millis(void);
void msleep(uint32_t sleep);
void usleep(uint32_t sleep);
void init_capture_RX(void);
void servos_output(uint16_t *buff);

void Capture();
void cover_sbus_buff_to_ch(uint8_t *buff);
#define delay(X) HAL_delay(X)
#define delay_ms(X) HAL_delay(X)
#endif
