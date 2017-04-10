/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 * unix_time.c
 *      Author: Jet <g.tanchin@yandex.ru>
 *  Created on: 08 апр. 2016 г.
 */

#include <main.h>
#include "stm32f10x.h"
#include "stm32f10x_it.h"
#include "my_time.h"

volatile timer_ticks_t timer_delayCount;
volatile uTime_t uxTime = 0;
volatile uint32_t sTick;
volatile uint32_t usDelFlag;

tRTC sysRtc;

extern RCC_ClocksTypeDef RCC_Clocks;

uint32_t sendTout;
uint32_t adeReadCount;


/*******************************************************************************
* Function Name  : rtc_init
* Description    : initializes HW RTC,
*                  sets default time-stamp if RTC has not been initialized before
* Input          : None
* Output         : None
* Return         : not used
*  Based on code from a STM RTC example in the StdPeriph-Library package
*******************************************************************************/
int rtcInit( eClkSrc src, uTime_t t) {
	volatile uint16_t i;
	uint16_t psc;				// Прескалер тактирования RTC

	// Enable PWR and BKP clocks
  RCC->APB1ENR |= (RCC_APB1Periph_PWR | RCC_APB1Periph_BKP);

	/* LSI clock stabilization time */
	for(i=0;i<5000;i++) { ; }


	/* Allow access to BKP Domain */
	PWR_BackupAccessCmd(ENABLE);

	/* Reset Backup Domain */
	BKP_DeInit();

	switch( src ){
		case SRC_LSE:
			// Enable LSE
			RCC_LSEConfig(RCC_LSE_ON);
			// Wait till LSE is ready
			while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET) { ; }
			// Select LSE as RTC Clock Source
		  RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;
			psc = 32767; // RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1)
			break;
		case SRC_LSI:
			RCC->CSR |= RCC_CSR_LSION;
			while ( (RCC->CSR & RCC_CSR_LSIRDY) == RESET)
			{}
			// Select LSI as RTC Clock Source
		  RCC->BDCR |= RCC_BDCR_RTCSEL_LSI;
			psc = 39999; // RTC period = RTCCLK/RTC_PR = (40 KHz)/(39999+1)
			break;
		case SRC_HSE128:
			// Select HSE/128 as RTC Clock Source
		  RCC->BDCR |= RCC_BDCR_RTCSEL_HSE;
			psc = (HSE_VALUE/128) - 1; // RTCCLK = (HSE_VALUE)/(128), RTC_PR = (RTCCLK / RTC period) - 1
//			PWR->CR |= PWR_CR_DBP;
			break;
		default:
			//Недопустимое значение
			return -1;
			break;
	}
	/* Enable RTC Clock */
	RCC_RTCCLKCmd(ENABLE);

	/* Wait for RTC registers synchronization */
	RTC_WaitForSynchro();

	// Wait until last write operation on RTC registers has finished
	RTC_WaitForLastTask();

	/* Set RTC prescaler: set RTC period to 1sec */
	RTC_SetPrescaler(psc);

	// Wait until last write operation on RTC registers has finished
	RTC_WaitForLastTask();

	/* Set initial value */
	RTC_SetCounter( (uint32_t)t ); // here: 1st January 2000 11:55:00

	// Wait until last write operation on RTC registers has finished
	RTC_WaitForLastTask();

//		BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);

	if( (RCC->BDCR & RCC_BDCR_RTCSEL ) != RCC_BDCR_RTCSEL ){
		//RTC тактируется не от HSE
		PWR_BackupAccessCmd(DISABLE);
	}

	return 0;
}


/*!
 * @brief Configures the RTC timer
 * @note The timer is based on the RTC
 * @param none
 * @retval none
 */
void rtcSetup( void ) {
  if( uxTime == 0) {
    sysRtc.year = 2017;
    sysRtc.month = 1;
    sysRtc.mday = 1;
    sysRtc.hour = 9;
    sysRtc.min = 0;
    sysRtc.sec = 0;
    uxTime = tm2Utime( &sysRtc );
  }
  sysRtc.SecFlag = RESET;
  sysRtc.MinFlag = RESET;
  sysRtc.HourFlag = RESET;
  sysRtc.DayFlag = RESET;
  sysRtc.WeekFlag = RESET;
  sysRtc.MonthFlag = RESET;
  sysRtc.YearFlag = RESET;

  // Для перерасчета дня недели
  utime2Tm( &sysRtc, uxTime );

  rtcInit( SRC_HSE128, uxTime );
	RTC_ITConfig( RTC_IT_SEC, ENABLE);
	NVIC_EnableIRQ( RTC_IRQn );
	NVIC_SetPriority( RTC_IRQn, 1 );
}


// Получение системного мремени
uint32_t getTick( void ) {
	// Возвращает количество тиков
	return sTick;
}

uint32_t sys_now( void ){
	return sTick;
}
#define _TBIAS_DAYS		((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		1900
#define MONTAB(year)		((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

uTime_t tm2Utime( const tRTC *mrtc ){
	/* convert time structure to scalar time */
int32_t		days;
int32_t		secs;
int32_t		mon, year;

	/* Calculate number of days. */
	mon = mrtc->month - 1;
	year = mrtc->year - _TBIAS_YEAR;
	days  = Daysto32(year, mon) - 1;
	days += 365 * year;
	days += mrtc->mday;
	days -= _TBIAS_DAYS;

	/* Calculate number of seconds. */
	secs  = 3600 * mrtc->hour;
	secs += 60 * mrtc->min;
	secs += mrtc->sec;

	secs += (days * (uTime_t)86400);

	return (secs);
}

/////////////////////////////////////////////////////////////////////

void utime2Tm( tRTC * mrtc, uTime_t secsarg){
	uint32_t		secs;
	int32_t		days;
	int32_t		mon;
	int32_t		year;
	int32_t		i;
	const int16_t *	pm;

	#ifdef	_XT_SIGNED
	if (secsarg >= 0) {
			secs = (uint32_t)secsarg;
			days = _TBIAS_DAYS;
		} else {
			secs = (uint32_t)secsarg + _TBIAS_SECS;
			days = 0;
		}
	#else
		secs = secsarg;
		days = _TBIAS_DAYS;
	#endif

		/* days, hour, min, sec */
	days += secs / 86400;
	secs = secs % 86400;
	mrtc->hour = secs / 3600;
	secs %= 3600;
	mrtc->min = secs / 60;
	mrtc->sec = secs % 60;

	mrtc->wday = (days + 1) % 7;

	/* determine year */
	for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
	days -= i;
	mrtc->year = year + _TBIAS_YEAR;

		/* determine month */
	pm = MONTAB(year);
	for (mon = 12; days < pm[--mon]; );
	mrtc->month = mon + 1;
	mrtc->mday = days - pm[mon] + 1;
}

void timersHandler( void ) {

	// Decrement to zero the counter used by the delay routine.
  if (timer_delayCount != 0u) {
    --timer_delayCount;
  }

/*	// Таймаут для логгирования температуры
	if ( toLogCount > 1) {
		toLogCount--;
	}
*/
	// Таймаут для считывания температуры
	if ( adeReadCount > 1) {
		adeReadCount--;
	}



}

void timersProcess( void ) {
/*
	// Таймаут для логгирования температуры
	if ( toLogCount == 1 ) {
		toLogCount = toLogTout+1;
		toLogWrite();
	}
*/

	// Секундный таймер
	if ( sysRtc.SecFlag ) {
		sysRtc.SecFlag = RESET;
		// Симуляция водомера
//		flowGetVolume();
//		flowSecondProcess();
	}

	// Таймаут для считывания температуры
	if ( adeReadCount == 1 ) {
	  // TODO: Читаем данные из ADE
	}
}

// Инициализация таймера микросекундных задержек
void delayUsInit( void ) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM_DeInit(TIM4);
	TIM4->CR1 &= ~TIM_CR1_CEN;
	// Выставляем счетчик на 0,5 мкс
	TIM4->PSC = (RCC_Clocks.PCLK2_Frequency/2000000) - 1;
	TIM4->ARR = 0xFFFF;
	TIM4->CR1 &= ~TIM_CR1_CKD;
	TIM4->CR1 &= ~TIM_CR1_DIR;		// Считаем на возрастание

/*
	TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);
	DELAY_NVIC_InitStructure.NVIC_IRQChannel = DELAY_NVIC_IRQCHANNEL;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DELAY_NVIC_InitStructure);
*/
}

// Задержка в 0,5 мкс

void usDelay( uint32_t usDel ){

	TIM4->ARR = usDel-1;
	TIM4->CNT = 0;
	TIM4->CR1 |= TIM_CR1_CEN;

	while( !(TIM4->SR & TIM_SR_UIF) )
	{}
	TIM4->CR1 &= ~TIM_CR1_CEN;
	TIM4->SR &= ~TIM_SR_UIF;
}

// Задержка в мс
void myDelay( uint32_t del ){
	uint32_t finish = sTick + del;
	while ( sTick < finish)
	{}
}

