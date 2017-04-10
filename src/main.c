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
#include "ade.h"
#include "can.h"
#include "buffer.h"
#include "my_time.h"

uint8_t rxCplt = FALSE;
uint8_t txCplt = FALSE;

// ----- Timing definitions -------------------------------------------------

// ----- main() ---------------------------------------------------------------


int main(int argc, char* argv[]) {
  (void)argc;
  (void)argv;

  sTick = 0;
  uxTime = 1491037200;      // Unix Time = 01.04.2017г., 09:00:00

  // Send a greeting to the trace device (skipped on Release).
  trace_puts("Hello ARM World!");

  // At this stage the system clock should have already been configured
  // at high speed.
  trace_printf("System clock: %u Hz\n", SystemCoreClock);

  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);

  rtcSetup();

 // canInit();

  adeInit();

  myDelay( 1000 );
  
  // Infinite loop
  while(1) {
    timersProcess();
    canProcess();
//  TODO: UART: Обработка принятых и передача подготовленных сообщений
//    uartProcess();
  }
  // Infinite loop, never return.
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
