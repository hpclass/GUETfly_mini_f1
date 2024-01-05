#include "guetfly_data_types.h"
all_config_type configs;
void BSP_init(void)
{

    SystemInit();
    systick_config();
    init_leds();
    init_uarts(HANDLE_usart_gps, 115200);   // 初始化GPS的串口
    init_uarts(HANDLE_usart_radio, 115200); // 初始化电台串口
    i2c_init(HANDLE_I2C_MPU);    // 初始化内置传感器I2C总线
    i2c_init(HANDLE_I2C_EEPROM); // 初始化外置传感器I2C总线
    // init_timer2(1999, 84);
    // soft_pwm_init(9, 84);
    init_capture_RX();
}