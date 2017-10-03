/**
  ******************************************************************************
  * @file    stm32f2xx_it.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    07-October-2011
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "main.h"
#include "my_time.h"
#include "can.h"
#include "ade.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/******************************************************************************
 * 	!!! ОПРЕДЕЛЕНЫ В ./system/src/stm32f1-std/stm32f10x_it.c !!!
 ******************************************************************************/


void NMI_Handler(void)
{
}

void HardFault_Handler(void)
{
  // Go to infinite loop when Hard Fault exception occurs
  while (1)
  {
  }
}

void MemManage_Handler(void)
{
  // Go to infinite loop when Memory Manage exception occurs
  while (1)
  {
  }
}

void BusFault_Handler(void)
{
  // Go to infinite loop when Bus Fault exception occurs
  while (1)
  {
  }
}

void UsageFault_Handler(void)
{
  // Go to infinite loop when Usage Fault exception occurs
  while (1)
  {
  }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void PPP_IRQHandler(void)
{
}

// ----- SysTick_Handler() ----------------------------------------------------

void
SysTick_Handler (void) {
#if defined(USE_HAL_DRIVER)
  HAL_IncTick();
#endif
  sTick++;
  timersHandler();
}

#if 0   //Выключаем обработку лишних прерываний
/**
  * @brief  This function handles External line 3 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI3_IRQHandler(void){
}

/**
  * @brief  This function handles External line 10 interrupt request.
  * @param  None
  * @retval None
  */
void EXTI15_10_IRQHandler(void){
}
#endif

/******************************************************************************/
/*                 STM32F2xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f2xx.s).                                               */
/******************************************************************************/
void TIM4_IRQHandler( void ){
	TIM4->SR &= ~TIM_SR_UIF;
	if(usDelFlag){
		usDelFlag = FALSE;
	}
}

void RTC_IRQHandler(void){
	if( RTC->CRL & RTC_CRL_SECF ){
		uxTime++;
		sysRtc.SecFlag = SET;
		RTC->CRL &= ~RTC_CRL_SECF;
		// Секундный таймер
		utime2Tm( &sysRtc, uxTime );
			// Выставляем флаги минут, часов, дней, недель и месяцев
		if( sysRtc.sec == 0 ){
			sysRtc.MinFlag = SET;
			if( sysRtc.min == 0){
				sysRtc.HourFlag = SET;
				if( sysRtc.hour == 0){
					sysRtc.DayFlag = SET;
					if( sysRtc.wday == 1){
						sysRtc.WeekFlag = SET;
					}
					if( sysRtc.mday == 1){
						sysRtc.MonthFlag = SET;
					}
				}
			}
		}
	}
	if( RTC->CRL & RTC_CRL_ALRF ){
		RTC->CRL &= ~RTC_CRL_ALRF;
	}
	// Переполнение счетного регистра
	if(RTC->CRL & RTC_CRL_OWF) {
     RTC->CRL &= ~RTC_CRL_OWF;     //сбросить флаг (обязательно!!!)
     //выполняем какие-то действия
	}
}

// Прерывание от ADE
void EXTI9_5_IRQHandler(void){
  if (EXTI_GetITStatus(ISR_EXTI_LINE)) {
    adeIrqHandler();
    EXTI_ClearITPendingBit(ISR_EXTI_LINE);
  }
}

void USB_LP_CAN1_RX0_IRQHandler(void)
{
	canRx0IrqHandler();
}

void USB_HP_CAN1_TX_IRQHandler( void ){
	canTxIrqHandler();
}

void CAN1_RX1_IRQHandler( void ){
	canRx1IrqHandler();
}

void CAN1_SCE_IRQHandler( void ){
	canSceIrqHandler();
}

/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
