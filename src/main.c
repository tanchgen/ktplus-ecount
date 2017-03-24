//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "main.h"
#include "spi.h"
#include "buffer.h"
#include "firmata.h"

uint8_t rxCplt = FALSE;
uint8_t txCplt = FALSE;

extern BUFFER_t rxBuffer;
extern uint8_t rxBuf[];

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

// Сообщение, передаваемое роутером на сервер по команде "Init"
char * initMsg = "UID=EngineControl27182:ou=EngineControl27182:"\
		  "description=All people in organisation:objectclass=deviceModule:"\
		  "wr=FALSE:stat=FALSE:";
char * initMsg2 = "UID=EngineControl27182:ou=EngineControl27182:parameterName=Fuel_Vol:"\
		  "description=Volume of fuel level:value=100:"\
		  "prevValue=0:power=0:wr=FALSE:stat=TRUE:"\
		  "recverAdd=FALSE:recverName=EngineControl27182:recverName=EngineControl27183:"\
		  "modifyTime=20141220171526Z:"\
		  "recverName=EngineControl27184;";

int main(int argc, char* argv[]) {
  uint8_t * ptxBuf = NULL;

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  timer_start();

  spiInit();
  BUFFER_Init( &rxBuffer, rxBuf, BUF_SIZE );

  timer_sleep( 1000 );
  
  while(rxCplt == 0)
  {}
  ptxBuf = (uint8_t *)"\xF0\x79ver.energo-0.2.4\xF7";
  SPI_TransmitData( ptxBuf, strlen((char*)ptxBuf) );

  while(txCplt == 0)
  {}
  // Infinite loop
  while(1) {

    if(rxCplt){
      uint8_t tmpBuf[BUF_SIZE];
      if ( BUFFER_ReadString( &rxBuffer, (char*)tmpBuf, BUF_SIZE ) ){
        rxCplt = FALSE;
//        trace_puts(" Receive 1 message.\n");

        if( tmpBuf[1] == FIRM_CMD_VERS ){
          // Получена команда "Запрос версии роутера"
          trace_puts(" Receive message: Version number query.\n");
          ptxBuf = (uint8_t *)"ver.energo-0.2.4";
        }
        else if( ((tmpBuf[1] << 8) | tmpBuf[2]) == FIRM_CMD_INIT ){
          // Получена команда "Инициализация роутера"
          trace_puts(" Receive message: Init command.\n");
          ptxBuf = (uint8_t *)initMsg ;
        }

        ptxBuf = (uint8_t *)"\xF0\x79ver.energo-0.2.4\xF7";
        SPI_TransmitData( ptxBuf, strlen((char*)ptxBuf) );
      }

    }

    if( txCplt ){
      txCplt = FALSE;
      while( (SPI2->SR & SPI_SR_BSY) != 0) // Ждем окончания передачи
      {}
      GPIOA->BSRR |= ISR_PIN; //Снимаем прерывание - Выставляем высокий уровень

      if(ptxBuf == (uint8_t *)initMsg) {
        SPI_TransmitData( (uint8_t *)initMsg2, strlen(initMsg2) );
      }
    }

  }
  // Infinite loop, never return.
}

/**
  * @brief  Sets System clock frequency to 48MHz and configure HCLK, PCLK2
  *         and PCLK1 prescalers.
  * @note   This function should be used only after reset.
  * @param  None
  * @retval None
  */
static void SetSysClockTo48(void)
{
  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
  /* Enable HSE */
  RCC->CR |= ((uint32_t)RCC_CR_HSEON);

  /* Wait till HSE is ready and if Time out is reached exit */
  do
  {
    HSEStatus = RCC->CR & RCC_CR_HSERDY;
    StartUpCounter++;
  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
  {
    HSEStatus = (uint32_t)0x01;
  }
  else
  {
    HSEStatus = (uint32_t)0x00;
  }

  if (HSEStatus == (uint32_t)0x01)
  {

    /* HCLK = SYSCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;

    /* PCLK2 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;

    /* PCLK1 = HCLK */
    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;

#ifdef STM32F10X_CL
    /* Configure PLLs ------------------------------------------------------*/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */

    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);

    /* Enable PLL2 */
    RCC->CR |= RCC_CR_PLL2ON;
    /* Wait till PLL2 is ready */
    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
    {
    }


    /* PLL configuration: PLLCLK = PREDIV1 * 6 = 48 MHz */
    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 |
                            RCC_CFGR_PLLMULL6);
#else
    /*  PLL configuration: PLLCLK = HSE * 6 = 48 MHz */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL6);
#endif /* STM32F10X_CL */

    /* Enable PLL */
    RCC->CR |= RCC_CR_PLLON;

    /* Wait till PLL is ready */
    while((RCC->CR & RCC_CR_PLLRDY) == 0)
    {
    }

    /* Select PLL as system clock source */
    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;

    /* Wait till PLL is used as system clock source */
    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
    {
    }

    /* Flash 1 wait state */
    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_1;

  }
  else
  { /* If HSE fails to start-up, the application will have wrong clock
         configuration. User can add here some code to deal with this error */
  }
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
