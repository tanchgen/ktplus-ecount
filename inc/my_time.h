/*
 * my_time.h
 *
 *  Created on: 08 апр. 2016 г.
 *      Author: Jet <g.tanchin@yandex.ru>
 */

#ifndef MY_TIME_H_
#define MY_TIME_H_

#include <stdint.h>
#include "stm32f10x.h"


#define DEBOUNCE_RCC_TIM 					RCC_APB1Periph_TIM4
#define DEBOUNCE_TIM 							TIM4
#define DEBOUNCE_NVIC_IRQCHANNEL 	TIM4_IRQn
#define DEBOUNCE_NVIC_IRQHANDLER 	TIM4_IRQHandler

#define TIMER_FREQUENCY_HZ (1000u)
typedef uint32_t timer_ticks_t;


#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

#define TIMEZONE_MSK			(+3)

	// DEF: standard signed format
	// UNDEF: non-standard unsigned format
//#define	_XT_SIGNED

#ifndef	_XT_SIGNED

typedef	uint32_t                           uTime_t;

#else
typedef	int32_t                          time_t;
#endif

typedef enum {
	SRC_LSE,
	SRC_LSI,
	SRC_HSE128
} eClkSrc;

typedef struct {
	uint16_t year;	/* 1..4095 */
	uint8_t  month;	/* 1..12 */
	uint8_t  mday;	/* 1.. 31 */
	uint8_t  wday;	/* 0..6, Sunday = 0*/
	uint8_t  hour;	/* 0..23 */
	uint8_t  min;	/* 0..59 */
	uint8_t  sec;	/* 0..59 */
	uint8_t  dst;	/* 0 Winter, !=0 Summer */
	volatile uint8_t SecFlag;
	volatile uint8_t MinFlag;
	volatile uint8_t HourFlag;
	volatile uint8_t DayFlag;
	volatile uint8_t WeekFlag;
	volatile uint8_t MonthFlag;
	volatile uint8_t YearFlag;
} tRTC;

extern volatile uTime_t uxTime;

extern tRTC sysRtc;

extern volatile uint32_t sTick;
extern volatile uint32_t usDelFlag;

extern uint32_t sendTout;
extern uint32_t toReadCount;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
// Получение системного мремени
uint32_t getTick( void );
uTime_t tm2Utime( const tRTC *srtc );
void utime2Tm( tRTC * srtc, uTime_t ut);
void timersProcess( void );
void timeToStr( uTime_t ut, uint8_t *str );

void timersHandler( void );

extern void rtcSetup( void );
extern volatile timer_ticks_t timer_delayCount;
extern void timer_start (void);
extern void timer_sleep (timer_ticks_t ticks);

void debounceInit( void );
void myDelay( uint32_t del );
void delayUsInit( void );
void usDelay( uint32_t usDel );

#endif /* UNIX_TIME_H_ */
