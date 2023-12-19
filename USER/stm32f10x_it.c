/**
  ******************************************************************************
  * @file    Project/STM32F10x_StdPeriph_Template/stm32f10x_it.c
  * @author  MCD Application Team
  * @version V3.4.0
  * @date    10/15/2010
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2010 STMicroelectronics</center></h2>
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
/** @addtogroup STM32F10x_StdPeriph_Template
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
    //关闭马达输出
    TIM3->CCR1=1;
    TIM3->CCR2=1;
    TIM3->CCR3=1;
    TIM3->CCR4=1;
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
    /* Go to infinite loop when Hard Fault exception occurs */
    //关闭马达输出
    TIM3->CCR1=1;
    TIM3->CCR2=1;
    TIM3->CCR3=1;
    TIM3->CCR4=1;
    if(CoreDebug->DHCSR & 1) {  //check C_DEBUGEN == 1 -> Debugger Connected
        __breakpoint(0);  // halt program execution here
    }
    while (1)
    {

    }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    /* Go to infinite loop when Memory Manage exception occurs */
    while (1)
    {
        //关闭马达输出
        TIM3->CCR1=1;
        TIM3->CCR2=1;
        TIM3->CCR3=1;
        TIM3->CCR4=1;
    }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    /* Go to infinite loop when Bus Fault exception occurs */
    while (1)
    {
        //关闭马达输出
        TIM3->CCR1=1;
        TIM3->CCR2=1;
        TIM3->CCR3=1;
        TIM3->CCR4=1;
    }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    /* Go to infinite loop when Usage Fault exception occurs */
    while (1)
    {
        //关闭马达输出
        TIM3->CCR1=1;
        TIM3->CCR2=1;
        TIM3->CCR3=1;
        TIM3->CCR4=1;
    }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}


/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */


//void TIM2_IRQHandler() {}
void TIM3_IRQHandler() {}
void TIM4_IRQHandler() {}
void WWDG_IRQHandler() {}
void PVD_IRQHandler() {}
void TAMPER_IRQHandler() {}
void RTC_IRQHandler() {}
void FLASH_IRQHandler() {}
void RCC_IRQHandler() {}
void EXTI0_IRQHandler() {}
//void EXTI1_IRQHandler() {}
void EXTI2_IRQHandler() {}
void EXTI3_IRQHandler() {}
//void EXTI4_IRQHandler(){}
void DMA1_Channel1_IRQHandler() {}
void DMA1_Channel2_IRQHandler() {}
void DMA1_Channel3_IRQHandler() {}
void DMA1_Channel4_IRQHandler() {}
void DMA1_Channel5_IRQHandler() {}
void DMA1_Channel6_IRQHandler() {}
void DMA1_Channel7_IRQHandler() {}
void ADC1_2_IRQHandler() {}
void USB_HP_CAN1_TX_IRQHandler() {}
//void USB_LP_CAN1_RX0_IRQHandler(){}
void CAN1_RX1_IRQHandler() {}
void CAN1_SCE_IRQHandler() {}
void EXTI9_5_IRQHandler() {}
void TIM1_BRK_IRQHandler() {}
void TIM1_UP_IRQHandler() {}
void TIM1_TRG_COM_IRQHandler() {}
//void TIM1_CC_IRQHandler(){}
//void TIM2_IRQHandler(){}
//void TIM3_IRQHandler(){}
//void TIM4_IRQHandler(){}
//void I2C1_EV_IRQHandler() {}
//void I2C1_ER_IRQHandler() {}
void I2C2_EV_IRQHandler() {}
void I2C2_ER_IRQHandler() {}
void SPI1_IRQHandler() {}
void SPI2_IRQHandler() {}
//void USART1_IRQHandler(){}
//void USART2_IRQHandler(){}
//void USART3_IRQHandler(){}
void EXTI15_10_IRQHandler() {}
void RTCAlarm_IRQHandler() {}
void USBWakeUp_IRQHandler() {}
void TIM8_BRK_IRQHandler() {}
void TIM8_UP_IRQHandler() {}
void TIM8_TRG_COM_IRQHandler() {}
void TIM8_CC_IRQHandler() {}
void ADC3_IRQHandler() {}
void FSMC_IRQHandler() {}
void SDIO_IRQHandler() {}

//void TIM5_IRQHandler() {}
void SPI3_IRQHandler() {}
//void UART4_IRQHandler(){}
//void UART5_IRQHandler(){}
void TIM6_IRQHandler() {}
void TIM7_IRQHandler() {}
void DMA2_Channel1_IRQHandler() {}
void DMA2_Channel2_IRQHandler() {}
void DMA2_Channel3_IRQHandler() {}
void DMA2_Channel4_5_IRQHandler() {}




/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
