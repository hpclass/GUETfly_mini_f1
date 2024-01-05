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
#ifndef __HP_GUETFLY_DATA_TYPES_H__
#define __HP_GUETFLY_DATA_TYPES_H__
#include "stdint.h"
#define typecheck(type,x)\
({        type __dummy; \
        typeof(x) __dummy2; \
        (void)(&__dummy == &__dummy2);\
        1; \
})

/*
 *  These inlines deal with timer wrapping correctly. You are 
 *  strongly encouraged to use them
 *  1. Because people otherwise forget
 *  2. Because if the timer wrap changes in future you won't have to
 *     alter your driver code.
 *
 * time_after(a,b) returns true if the time a is after time b.
 *
 * Do this with "<0" and ">=0" to only test the sign of the result. A
 * good compiler would generate better code (and a really good compiler
 * wouldn't care). Gcc is currently neither.
 */
#define time_after(a,b)  \
 (typecheck(unsigned long, a) && \
  typecheck(unsigned long, b) && \
  ((long)(b) - (long)(a) < 0))
#define time_before(a,b)  time_after(b,a)


#define time_after_eq(a,b)  \
 (typecheck(unsigned long, a) && \
  typecheck(unsigned long, b) && \
  ((long)(a) - (long)(b) >= 0))
#define time_before_eq(a,b)  time_after_eq(b,a)


/*
 * Calculate whether a is in the range of [b, c].
 */
#define time_in_range(a,b,c) \
 (time_after_eq(a,b) && \
  time_before_eq(a,c))


/*
 * Calculate whether a is in the range of [b, c).
 */
#define time_in_range_open(a,b,c) \
 (time_after_eq(a,b) && \
  time_before(a,c))


/* Same as above, but does so with platform independent 64bit types.
 * These must be used when utilizing jiffies_64 (i.e. return value of
 * get_jiffies_64() */
#define time_after64(a,b)  \
 (typecheck(__u64, a) &&  \
  typecheck(__u64, b) && \
  ((__s64)(b) - (__s64)(a) < 0))
#define time_before64(a,b)  time_after64(b,a)


#define time_after_eq64(a,b)  \
 (typecheck(__u64, a) && \
  typecheck(__u64, b) && \
  ((__s64)(a) - (__s64)(b) >= 0))
#define time_before_eq64(a,b)  time_after_eq64(b,a)


/*
 * These four macros compare jiffies and 'a' for convenience.
 */


/* time_is_before_jiffies(a) return true if a is before jiffies */
#define time_is_before_jiffies(a) time_after(jiffies, a)


/* time_is_after_jiffies(a) return true if a is after jiffies */
#define time_is_after_jiffies(a) time_before(jiffies, a)


/* time_is_before_eq_jiffies(a) return true if a is before or equal to jiffies*/
#define time_is_before_eq_jiffies(a) time_after_eq(jiffies, a)


/* time_is_after_eq_jiffies(a) return true if a is after or equal to jiffies*/
#define time_is_after_eq_jiffies(a) time_before_eq(jiffies, a)


/*
 * Have the 32 bit jiffies value wrap 5 minutes after boot
 * so jiffies wrap bugs show up earlier.
 */
#define INITIAL_JIFFIES ((unsigned long)(unsigned int) (-300*HZ))


/*
 * Change timeval to jiffies, trying to avoid the
 * most obvious overflows..
 *
 * And some not so obvious.
 *
 * Note that we don't want to return LONG_MAX, because
 * for various timeout reasons we often end up having
 * to wait "jiffies+1" in order to guarantee that we wait
 * at _least_ "jiffies" - so "jiffies+1" had better still
 * be positive.
 */
typedef enum {
    task_OUTPUT=0,
    task_RX,
    task_GPS,
    task_IMU__,
    task_MAX_NUM___

} type_task_loop;

typedef enum {
    HANDLE_I2C_MPU=0,
    HANDLE_I2C_EEPROM,
    HANDLE_I2C_EX
} type_i2c_handle;

typedef enum {
    HANDLE_usart_gps=0,
    HANDLE_usart_radio,
    HANDLE_usart_rx
} type_usart_handle;
typedef enum {
    PPM=0,
    SBUS,
    END
} RX_types;
typedef struct {
    uint8_t RX_data_type;
    uint8_t mag_calibrate_count;//磁力计校准次数，0为校准完成
    uint8_t acc_gyro_calibrate_count;//角速度度、重力校准次数，0校准结束
} flags;
typedef struct {
    flags flag;
    struct {
        int16_t acc_calibrate[3];
        int16_t gyro_calibrate[3];
        int16_t mag_calibrate[6];//六面校准
    } imu_calibrate;

} all_config_type;
extern all_config_type configs;
#define SERIER_TIME_OUT_US 350
#endif
