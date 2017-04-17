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

// ADE constants
#define FULL_SCALE_I	9032007	    // Максимальное значение регистра IRMS
#define FULL_SCALE_V	9032007	    // Максимальное значение регистра VRMS
#define FULL_SCALE_W	4862401	    // Максимальное значение регистра IRMS    

// Коэффициент пересчета значения IRMS в Амперы (50А - ток полной шкалы)
#define K_I	(FULL_SCALE_I / 50) 

// Коэффициент пересчета значения VRMS в Вольты (500В - напряжение полной шкалы)
#define K_V	(FULL_SCALE_V / 500) 

// Коэффициент пересчета значения AWATT в Ватты
#define K_W (FULL_SCALE_W / (500*50) )

//#define ISR_PIN     GPIO_Pin_12
//#define ISR_PORT    GPIOB

#define ISR_PIN         GPIO_Pin_12
#define ISR_PORT        GPIOA
#define ISR_PIN_NUM     12
#define ISR_PORT_NUM    0 // GPIOA
#define ISR_EXTI_LINE   EXTI_Line12

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
  int32_t enrgDay;        // Потребленное количество энергии с начала суток
  int32_t enrgDaySum;     // Потребленное количество энергии ежесуточно с нарастющим итогом
  int32_t enrgMon;        // Потребленное количество энергии с начала месяца
  int32_t enrgMonSum;     // Потребленное количество энергии ежесуточно с нарастющим итогом
  int32_t enrgYear;       // Потребленное количество энергии с начала года
  int32_t watt;           // Потребляемая мощность
  int32_t maxWattMon;     // Махимальная мощность за месяц
  uint32_t tWatt;      // Интервал измерения мощности
  uint32_t tWattCount;
  uint32_t tSend;      // Интервал передачи данных на сервер
  uint32_t tSendCount;
  uint32_t tEnergy;
  uint32_t tEnergyCount;
} tAde;

//extern BUFFER_t uartRxBuffer;
extern uint8_t rxBuf[];
extern uint8_t txBuf[];

extern tAde ade;

eAdeState adeInit( void );
eAdeState sendAde( uint16_t addr, uint8_t data[], uint8_t len );
eAdeState recvAde( uint16_t addr, uint8_t * data, uint8_t len );
// Блокирующий режим: пока не примет len байт из SPI - из функции не выйдет (таймаут = 100мс)
eAdeState recvAde_s( uint16_t addr, uint8_t * data, uint8_t len );
eAdeState adeIrqHandler( void );
eAdeState adeSecondProcess( void );
uint32_t recvV( void );
uint32_t recvI( void );
uint32_t recvW( void );
uint32_t recvE( void );

#endif /* ADE_H_ */
