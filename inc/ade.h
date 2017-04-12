/*
 * ade.h
 *
 *  Created on: 04 апр. 2017 г.
 *      Author: G.Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ADE_H_
#define ADE_H_

#ifdef ADE7878
  #include "ade7878.h"
#elif defined (ADE7953)
    #include "ade7953.h"
#endif
//#define ISR_PIN     GPIO_Pin_12
//#define ISR_PORT    GPIOB

#define ISR_PIN         GPIO_Pin_9
#define ISR_PORT        GPIOA
#define ISR_PIN_NUM     9
#define ISR_PORT_NUM    0 // GPIOA
#define ISR_EXTI_LINE   EXTI_Line9

#define CTRL_PIN         GPIO_Pin_8
#define CTRL_PORT        GPIOA
#define CTRL_PIN_NUM     8

typedef enum {
  ADE_OK,
  ADE_STOP,
  ADE_BUSY,
  ADE_READY,
  ADE_IRQ,
  ADE_WATT_READY,
  ADE_DATA_ERR,
  ADE_ERR,
} eAdeState;

typedef struct {
  // Установочные данные
  uint32_t awgain;
  uint32_t ovlvl;
  uint32_t oilvl;
  // Максимально допустимое значение потребляемой мощности
  uint32_t maxWatt;
  // Измерения
  int32_t wattDay;        // Потребленное количество энергии за сутки
  int32_t wattDaySum;     // Потребленное количество энергии ежесуточно с нарастющим итогом
  int32_t watt;           // Потребляемая мощность
  int32_t maxWattMon;     // Махимальная мощность за месяц
  uint32_t tPerWatt;      // Интервал измерения мощности
  uint32_t tPerSend;      // Интервал передачи данных на сервер
} tAde;

//extern BUFFER_t uartRxBuffer;
extern uint8_t rxBuf[];
extern uint8_t txBuf[];

eAdeState adeInit( void );
eAdeState sendAde( uint16_t addr, uint8_t data[], uint8_t len );
eAdeState recvAde( uint16_t addr, uint8_t * data, uint8_t len );
eAdeState adeIrqHandler( void );
#endif /* ADE_H_ */
