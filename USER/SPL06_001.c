#ifndef SPL06_01_C
#define SPL06_01_C
#if defined(STM32F10X_MD)
#include "soft_iic.h"
#include "delay.h"
#include "gy_86.h"
#endif
#if defined(GD32F330)
#include "gd32f3x0.h"
#include "i2c.h"
#endif
#include "SPL06_001.h"
// #include "i2c.h"
#include "types.h"

#define PRESSURE_REG 0X00
#define TEMP_REG 0X03
#define PRS_CFG 0x06
#define TMP_CFG 0x07
#define MEAS_CFG 0x08

#define SPL06_REST_VALUE 0x09
#define SPL06_REST_REG 0x0C

#define PRODUCT_ID 0X0D

#define uint32 unsigned int

static struct spl0601_t spl0601;
static struct spl0601_t *p_spl0601;

void spl0601_write(uint8 hwadr, uint8 regadr, uint8 val);
uint8 spl0601_read(uint8 hwadr, uint8 regadr);
void spl0601_get_calib_param(void);

/*****************************************************************************
 函 数 名  : spl0601_write
 功能描述  : I2C 寄存器写入子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
             uint8 val     值
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_write(unsigned char hwadr, unsigned char regadr, unsigned char val)
{
    //	hwI2C0_Device_Addr = hwadr;
    //	bI2C0_TxM_Data[0] = regadr;
    //	bI2C0_TxM_Data[1] = val;
    //
    //	I2C0_Engine(2,0,0);
    // IIC_Write_1Byte(hwadr,regadr,val);
    Single_WriteI2C(hwadr << 1, regadr, val);
}

/*****************************************************************************
 函 数 名  : spl0601_read
 功能描述  : I2C 寄存器读取子函数
 输入参数  : uint8 hwadr   硬件地址
             uint8 regadr  寄存器地址
 输出参数  :
 返 回 值  : uint8 读出值
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
uint8 spl0601_read(unsigned char hwadr, unsigned char regadr)
{
    uint8_t reg_data;

    //	hwI2C0_Device_Addr = hwadr;
    //	bI2C0_TxM_Data[0] = regadr; //
    //	I2C0_Engine(1,1,1);

    //	reg_data = bI2C0_RxM_Data[0];
    reg_data = Single_ReadI2C(hwadr << 1, regadr);
    // IIC_Read_1Byte(hwadr,regadr,&reg_data);
    return reg_data;
}

/*****************************************************************************
 函 数 名  : spl0601_init
 功能描述  : SPL06-01 初始化函数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_init(void)
{
    p_spl0601 = &spl0601; /* read Chip Id */
    p_spl0601->i32rawPressure = 0;
    p_spl0601->i32rawTemperature = 0;
    p_spl0601->chip_id = 0x34;
    spl0601_get_calib_param();

    //    // sampling rate = 32Hz; Pressure oversample = 8;
    //    spl0601_rateset(PRESSURE_SENSOR,32, 8);
    //    // sampling rate = 32Hz; Temperature oversample = 8;
    //    spl0601_rateset(TEMPERATURE_SENSOR,32, 8);

    spl0601_rateset(PRESSURE_SENSOR, 128, 32);
    // sampling rate = 1Hz; Temperature oversample = 1;
    spl0601_rateset(TEMPERATURE_SENSOR, 32, 8);

    // Start background measurement
    spl0601_start_continuous(CONTINUOUS_P_AND_T);
}

/*****************************************************************************
 函 数 名  : spl0601_rateset
 功能描述  :  设置温度传感器的每秒采样次数以及过采样率
 输入参数  : uint8 u8OverSmpl  过采样率         Maximal = 128
             uint8 u8SmplRate  每秒采样次数(Hz) Maximal = 128
             uint8 iSensor     0: Pressure; 1: Temperature
 输出参数  : 无
 返 回 值  : 无
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月24日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_rateset(uint8 iSensor, uint8 u8SmplRate, uint8 u8OverSmpl)
{
    uint8 reg = 0;
    int32 i32kPkT = 0;
    switch (u8SmplRate)
    {
    case 2:
        reg |= (1 << 5);
        break;
    case 4:
        reg |= (2 << 5);
        break;
    case 8:
        reg |= (3 << 5);
        break;
    case 16:
        reg |= (4 << 5);
        break;
    case 32:
        reg |= (5 << 5);
        break;
    case 64:
        reg |= (6 << 5);
        break;
    case 128:
        reg |= (7 << 5);
        break;
    case 1:
    default:
        break;
    }
    switch (u8OverSmpl)
    {
    case 2:
        reg |= 1;
        i32kPkT = 1572864;
        break;
    case 4:
        reg |= 2;
        i32kPkT = 3670016;
        break;
    case 8:
        reg |= 3;
        i32kPkT = 7864320;
        break;
    case 16:
        i32kPkT = 253952;
        reg |= 4;
        break;
    case 32:
        i32kPkT = 516096;
        reg |= 5;
        break;
    case 64:
        i32kPkT = 1040384;
        reg |= 6;
        break;
    case 128:
        i32kPkT = 2088960;
        reg |= 7;
        break;
    case 1:
    default:
        i32kPkT = 524288;
        break;
    }

    if (iSensor == 0)
    {
        p_spl0601->i32kP = i32kPkT;
        spl0601_write(HW_ADR, 0x06, reg);
        if (u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x04);
        }
    }
    if (iSensor == 1)
    {
        p_spl0601->i32kT = i32kPkT;
        spl0601_write(HW_ADR, 0x07, reg | 0x80); // Using mems temperature
        if (u8OverSmpl > 8)
        {
            reg = spl0601_read(HW_ADR, 0x09);
            spl0601_write(HW_ADR, 0x09, reg | 0x08);
        }
    }
}

/*****************************************************************************
 函 数 名  : spl0601_get_calib_param
 功能描述  : 获取校准参数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_calib_param(void)
{
    uint32 h;
    uint32 m;
    uint32 l;
    h = spl0601_read(HW_ADR, 0x10);
    l = spl0601_read(HW_ADR, 0x11);
    p_spl0601->calib_param.c0 = (int16)h << 4 | l >> 4;
    p_spl0601->calib_param.c0 = (p_spl0601->calib_param.c0 & 0x0800) ? (0xF000 | p_spl0601->calib_param.c0) : p_spl0601->calib_param.c0;
    h = spl0601_read(HW_ADR, 0x11);
    l = spl0601_read(HW_ADR, 0x12);
    p_spl0601->calib_param.c1 = (int16)(h & 0x0F) << 8 | l;
    p_spl0601->calib_param.c1 = (p_spl0601->calib_param.c1 & 0x0800) ? (0xF000 | p_spl0601->calib_param.c1) : p_spl0601->calib_param.c1;
    h = spl0601_read(HW_ADR, 0x13);
    m = spl0601_read(HW_ADR, 0x14);
    l = spl0601_read(HW_ADR, 0x15);
    p_spl0601->calib_param.c00 = (int32)h << 12 | (int32)m << 4 | (int32)l >> 4;
    p_spl0601->calib_param.c00 = (p_spl0601->calib_param.c00 & 0x080000) ? (0xFFF00000 | p_spl0601->calib_param.c00) : p_spl0601->calib_param.c00;
    h = spl0601_read(HW_ADR, 0x15);
    m = spl0601_read(HW_ADR, 0x16);
    l = spl0601_read(HW_ADR, 0x17);
    p_spl0601->calib_param.c10 = (int32)h << 16 | (int32)m << 8 | l;
    p_spl0601->calib_param.c10 = (p_spl0601->calib_param.c10 & 0x080000) ? (0xFFF00000 | p_spl0601->calib_param.c10) : p_spl0601->calib_param.c10;
    h = spl0601_read(HW_ADR, 0x18);
    l = spl0601_read(HW_ADR, 0x19);
    p_spl0601->calib_param.c01 = (int16)h << 8 | l;
    h = spl0601_read(HW_ADR, 0x1A);
    l = spl0601_read(HW_ADR, 0x1B);
    p_spl0601->calib_param.c11 = (int16)h << 8 | l;
    h = spl0601_read(HW_ADR, 0x1C);
    l = spl0601_read(HW_ADR, 0x1D);
    p_spl0601->calib_param.c20 = (int16)h << 8 | l;
    h = spl0601_read(HW_ADR, 0x1E);
    l = spl0601_read(HW_ADR, 0x1F);
    p_spl0601->calib_param.c21 = (int16)h << 8 | l;
    h = spl0601_read(HW_ADR, 0x20);
    l = spl0601_read(HW_ADR, 0x21);
    p_spl0601->calib_param.c30 = (int16)h << 8 | l;
}

/*****************************************************************************
 函 数 名  : spl0601_start_temperature
 功能描述  : 发起一次温度测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_temperature(void)
{
    spl0601_write(HW_ADR, 0x08, 0x02);
}

/*****************************************************************************
 函 数 名  : spl0601_start_pressure
 功能描述  : 发起一次压力值测量
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_pressure(void)
{
    spl0601_write(HW_ADR, 0x08, 0x01);
}

/*****************************************************************************
 函 数 名  : spl0601_start_continuous
 功能描述  : Select node for the continuously measurement
 输入参数  : uint8 mode  1: pressure; 2: temperature; 3: pressure and temperature
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月25日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_start_continuous(uint8 mode)
{
    spl0601_write(HW_ADR, 0x08, mode + 4);
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_temp
 功能描述  : 获取温度的原始值，并转换成32Bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_temp(void)
{
    uint8 h[3] = {0};

    h[0] = spl0601_read(HW_ADR, 0x03);
    h[1] = spl0601_read(HW_ADR, 0x04);
    h[2] = spl0601_read(HW_ADR, 0x05);

    p_spl0601->i32rawTemperature = (int32)h[0] << 16 | (int32)h[1] << 8 | (int32)h[2];
    p_spl0601->i32rawTemperature = (p_spl0601->i32rawTemperature & 0x800000) ? (0xFF000000 | p_spl0601->i32rawTemperature) : p_spl0601->i32rawTemperature;
}

/*****************************************************************************
 函 数 名  : spl0601_get_raw_pressure
 功能描述  : 获取压力原始值，并转换成32bits整数
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
void spl0601_get_raw_pressure(void)
{
    uint8 h[3];

    h[0] = spl0601_read(HW_ADR, 0x00);
    h[1] = spl0601_read(HW_ADR, 0x01);
    h[2] = spl0601_read(HW_ADR, 0x02);

    p_spl0601->i32rawPressure = (int32)h[0] << 16 | (int32)h[1] << 8 | (int32)h[2];
    p_spl0601->i32rawPressure = (p_spl0601->i32rawPressure & 0x800000) ? (0xFF000000 | p_spl0601->i32rawPressure) : p_spl0601->i32rawPressure;
}

/*****************************************************************************
 函 数 名  : spl0601_get_temperature
 功能描述  : 在获取原始值的基础上，返回浮点校准后的温度值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_temperature(void)
{
    float fTCompensate;
    float fTsc;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fTCompensate = p_spl0601->calib_param.c0 * 0.5 + p_spl0601->calib_param.c1 * fTsc;
    return fTCompensate;
}

/*****************************************************************************
 函 数 名  : spl0601_get_pressure
 功能描述  : 在获取原始值的基础上，返回浮点校准后的压力值
 输入参数  : void
 输出参数  : 无
 返 回 值  :
 调用函数  :
 被调函数  :

 修改历史      :
  1.日    期   : 2015年11月30日
    作    者   : WL
    修改内容   : 新生成函数

*****************************************************************************/
float spl0601_get_pressure(void)
{
    float fTsc, fPsc;
    float qua2, qua3;
    float fPCompensate;

    fTsc = p_spl0601->i32rawTemperature / (float)p_spl0601->i32kT;
    fPsc = p_spl0601->i32rawPressure / (float)p_spl0601->i32kP;
    qua2 = p_spl0601->calib_param.c10 + fPsc * (p_spl0601->calib_param.c20 + fPsc * p_spl0601->calib_param.c30);
    qua3 = fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);
    // qua3 = 0.9f *fTsc * fPsc * (p_spl0601->calib_param.c11 + fPsc * p_spl0601->calib_param.c21);

    fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + fTsc * p_spl0601->calib_param.c01 + qua3;
    // fPCompensate = p_spl0601->calib_param.c00 + fPsc * qua2 + 0.9f *fTsc  * p_spl0601->calib_param.c01 + qua3;
    return fPCompensate;
}

extern alt_t alt;
float temperature;
float pressure;
extern uint16_t calibratingB;
float logBaroGroundPressureSum;
float baroGroundTemperatureScale;
int16_t BaroAlt;
float Tempbaro;
float user_spl0601_get()
{

    spl0601_get_raw_temp();
    temperature = spl0601_get_temperature();

    spl0601_get_raw_pressure();
    pressure = spl0601_get_pressure() * 10;

    if (calibratingB > 0)
    {
        logBaroGroundPressureSum = pressure;
        calibratingB--;
    }
    BaroAlt = BaroAlt * 6 + (logBaroGroundPressureSum - pressure) * 2;
    BaroAlt >>= 3;
    alt.EstAlt = BaroAlt;
    // Tempbaro=(float)(pressure/logBaroGroundPressureSum)*1.0;
    // BaroAlt = 4433000.0f * (1 - powf((float)(Tempbaro),0.190295f));

    // printf("%f,%f,%d\r\n",temperature,pressure,BaroAlt);
    return 0;
}
#endif
void SPL06_001_StateMachine(void)
{
}
