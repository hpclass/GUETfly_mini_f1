/*
********************************************************************************
*                            Library.c
*
* File          : Library.c
* Version       : V1.0
* Author        : whq
* Mode          : Thumb2
* Toolchain     :                      
* Description   : �⺯��
*
* Date          : 2013/12/11
* History		: 
* 
*******************************************************************************/
//#include "stdafx.h"

#include <time.h>
#include <string.h>


#include "Library.h"



/*******************************************************************************
* Function Name :RECORD_BcdToInt
* Description   :BCD�ֽ�תΪ����
* Input         :uint8_t val    BCDֵ
* Output        :uint8_t        INT8ֵ
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_BcdToInt8(uint8_t val)
{
    return ((val >> 4) * 10 + (val & 0xF));
}

/*******************************************************************************
* Function Name :RECORD_Int8ToBcd
* Description   :
* Input         :uint8_t val
* Output        :uint8_t
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_Int8ToBcd(uint8_t val)
{
    return (((val / 10) << 4) + (val % 10));
}

/*******************************************************************************
* Function Name :RECORD_StringBcdToInt
* Description   :�޷���BCD�ַ���תΪ����    (BCD���ֽ���ǰ)
* Input         :uint8_t * str  BCD�ַ���
* Input         :uint16_t len   BCD�ַ�������
* Output        :uint32_t
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint32_t LIB_StringBcdToInt(uint8_t *str, uint16_t len)
{
    uint32_t res = 0;

    while (len--)
    {
        res *= 100;
        res += LIB_BcdToInt8(*str++);
    }
    return res;
}

/*******************************************************************************
* Function Name :RECORD_IntToStringBcd
* Description   :����תΪBCD�ַ������ַ����Ҷ��� (BCD���ֽ���ǰ)
* Input         :uint32_t val   ����
* Input         :uint8_t * str  ת���󻺳���
* Input         :uint16_t size  �������ռ��С
* Other         :
* Date          :2013/12/11
*******************************************************************************/
void LIB_IntToStringBcd( uint8_t *str, uint16_t size, uint32_t val )
{
    while (size--)
    {
        *(str + size)  = LIB_Int8ToBcd(val % 100);
        val /= 100;
    }
}

/*******************************************************************************
* Function Name :RECORD_Int16ByteReversed
* Description   :16λ���� ��С�˻�ת
* Input         :uint16_t val
* Output        :uint16_t
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint16_t LIB_Int16ByteReversed(uint16_t val)
{
    uint16_t res = 0;

    res = val << 8;
    res += val >> 8;

    return res;
}

/*******************************************************************************
* Function Name :RECORD_Int32ByteReversed
* Description   :32λ��С�˻�ת 
* Input         :uint32_t val
* Output        :uint32_t
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint32_t LIB_Int32ByteReversed(uint32_t val)
{
    uint8_t *p = (uint8_t *)&val;
    uint8_t arr[4] = {0};

    arr[0] = p[3];
    arr[1] = p[2];
    arr[2] = p[1];
    arr[3] = p[0];

    return *(uint32_t *)arr;
}

/*******************************************************************************
* Function Name :RECORD_CheckXOR
* Description   :���Ч��
* Input         :uint8_t * pBuf ���Ч������
* Input         :uint16_t len   ���ݳ���
* Output        :uint8_t        Ч����
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_CheckXOR(uint8_t *pBuf, uint16_t len)
{
    uint8_t cXor = 0;

    while (len--)
    {
        cXor ^= *pBuf++;
    }

    return cXor;
}

/*******************************************************************************
* Function Name :RECORD_CheckXOR
* Description   :��չ���Ч��
* Input         :uint8_t xor    Ч���ʼֵ
* Input         :uint8_t * pBuf ���Ч������
* Input         :uint16_t len   ���ݳ���
* Output        :uint8_t        Ч����
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_CheckXORExt(uint8_t cXor, uint8_t *pBuf, uint16_t len)
{
    while (len--)
    {
        cXor ^= *pBuf++;
    }

    return cXor;
}

/*******************************************************************************
* Function Name :RECORD_CheckSum
* Description   :����Ч��
* Input         :uint8_t * pBuf ����Ч������
* Input         :uint16_t len   ���ݳ���
* Output        :uint8_t        ���ܺ�
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_CheckSum(uint8_t *pBuf, uint16_t len)
{
    uint8_t sum = 0;

    while (len--)
    {
        sum += *pBuf++;
    }

    return sum;
}

/*******************************************************************************
* Function Name :RECORD_CheckSumExt
* Description   :��չ����Ч��
* Input         :uint8_t sum    ���ܳ�ʼֵ
* Input         :uint8_t * pBuf ����Ч������
* Input         :uint16_t len   ���ݳ���
* Output        :uint8_t        ���ܺ�
* Other         :
* Date          :2013/12/11
*******************************************************************************/
uint8_t LIB_CheckSumExt(uint8_t sum, uint8_t *pBuf, uint16_t len)
{
    while (len--)
    {
        sum += *pBuf++;
    }

    return sum;
}

/*******************************************************************************
* Function Name :LIB_BCDTime2Sec
* Description   :BCDתΪ����ʱ�� 
* Input         :BCDTIME * time BCDʱ��
* Output        :uint32_t       ����ʱ��
* Other         :
* Date          :2013/12/12
*******************************************************************************/
uint32_t LIB_BCDTime2Sec(BCDTIME_t *time)
{
    uint32_t lTime;
    struct tm conv = {0};

    conv.tm_year = LIB_BcdToInt8(time->year) + 2000 - 1900;
    conv.tm_mon = LIB_BcdToInt8(time->month)-1;
    conv.tm_mday = LIB_BcdToInt8(time->day);
    conv.tm_hour = LIB_BcdToInt8(time->hour);
    conv.tm_min = LIB_BcdToInt8(time->minute);
    conv.tm_sec = LIB_BcdToInt8(time->seconds);

    lTime = (uint32_t)mktime(&conv);
    //lTime -= g_TimeZone * 60 * 60;	    //convert +8:00 time to UTC from 1970-01-01 00:00:00

    return lTime;
}

/*******************************************************************************
* Function Name :LIB_Sec2BCDTime
* Description   :����ʱ��תΪBCD
* Input         :BCDTIME * tim  BCD������
* Input         :uint32_t sec   ����ʱ��
* Output        :void
* Other         :
* Date          :2013/12/12
*******************************************************************************/
void LIB_Sec2BCDTime(BCDTIME_t *tim, uint32_t sec)
{
    struct tm conv;
    time_t cTim = sec;

    //cTim += g_TimeZone * 60 * 60;	    //convert UTC time to +8:00 time zone

#ifdef _OS_WINDOWS_

    localtime_s(&conv, &cTim);

#elif defined(_OS_UCOSII_) || defined(_OS_UCOS_III_) || defined(_OS_LINUX_)

    localtime_r(&cTim, &conv);

#else

    localtime_r(&cTim, &conv);
    
#endif

    tim->year = LIB_Int8ToBcd((uint8_t)(conv.tm_year + 1900 - 2000));
    tim->month = LIB_Int8ToBcd((uint8_t)conv.tm_mon+1);
    tim->day = LIB_Int8ToBcd((uint8_t)conv.tm_mday);
    tim->hour = LIB_Int8ToBcd((uint8_t)conv.tm_hour);
    tim->minute = LIB_Int8ToBcd((uint8_t)conv.tm_min);
    tim->seconds = LIB_Int8ToBcd((uint8_t)conv.tm_sec);
}



/*******************************************************************************
* Function Name : int32_t LIB_StatusFilter(STATE_JUDGE_t *pState, int8_t newState, int16_t judgeCount)
* Description   : �źŹ����ж�����  
* Input         :   pState:     �ź��ж�����
                    newState:   �µ�״̬
                    judgeCount: �ж�����
* Output        : �ж����״̬
* Other         : 
* Date          : 2014.05.23
*******************************************************************************/
int32_t LIB_StatusFilter(STATE_JUDGE_t *pState, int8_t newState, int16_t judgeCount)
{
    if (pState->state != newState)
    {
        pState->judgeCount++;

        if (pState->judgeCount >= judgeCount)
        {
            pState->state = newState;

            pState->judgeCount = 0;
        }
    }
    else
    {
        pState->judgeCount = 0;
    }

    return pState->state;
}


/******************************END*********************************************/
