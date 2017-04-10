/*
 * ade.c
 *
 *  Created on: 04 апр. 2017 г.
 *      Author: G.Tanchin <g.tanchin@yandex.ru>
 */

#include "stm32f10x.h"

#include "my_time.h"
#include "buffer.h"
#include "spi.h"
#include "ade.h"

volatile eAdeState adeState = ADE_STOP;

static eAdeState adeSetup( void );

tAde ade;

int isrInit( void ){
  int rc = -1;

  // Инициируем ISR

  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

  // ISR Pin - PA9
  EXTI_InitTypeDef isr_EXTI_InitStructure;
  GPIO_InitTypeDef isr_InitStructure;

  isr_InitStructure.GPIO_Pin = ISR_PIN;
  isr_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  isr_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_Init(ISR_PORT, &isr_InitStructure);

  GPIO_EXTILineConfig(ISR_PORT_NUM, ISR_PIN_NUM);
  isr_EXTI_InitStructure.EXTI_Line = ISR_PIN;
  isr_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  isr_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  isr_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&isr_EXTI_InitStructure);

  NVIC_EnableIRQ( EXTI9_5_IRQn );
  NVIC_SetPriority( EXTI9_5_IRQn, 4);


  return rc;
}

eAdeState adeInit( void ){
  uint32_t tout = sTick+100;

  // Инициируем SPI
  spiInit();
  isrInit();
  while( adeState != ADE_READY){
    if( sTick > tout ){
      return adeState;
    }
  }
  memset( (uint8_t*)&ade, sizeof(tAde) );
  ade.tPerWatt = 1;
  ade.tPerSend = 60;

  adeState = adeSetup();

  return adeState;
}

eAdeState adeSetup( void ){
  eAdeState rc = ADE_READY;

  union {
    uint32_t u32d;
    uint16_t u16d;
    uint8_t u8d;
  } data;

  // Power-Up setup (from Datasheet)
  data.u8d = 0xAD;
  sendAde( UNLOCK_REG_8, &(data.u8d), 1 );
  data.u16d = 0x30;
  sendAde( SETUP_REG_16, &(data.u16d), 1 );

  // Установка предварительного усиления
  data.u32d = 0;
  sendAde( AIGAIN_32, &(data.u32d), 4 );
  sendAde( AVGAIN_32, &(data.u32d), 4 );
  ade.awgain = 0;
  sendAde( AWGAIN_32, &(ade.awgain), 4 );

  // TODO: Установка MAX тока и напряжения
  recvAde( RSTVPEAK_32, &(data.u32d), 4 );
  recvAde( RSTIAPEAK_32, &(data.u32d), 4);
  for( uint8_t i = 0; i < 10; i++ ) {
    myDelay(100);
    recvAde( VPEAK_32, &(data.u32d), 4 );
    if( ade.ovlvl < data.u32d ){
      ade.ovlvl = data.u32d;
    }
    recvAde( IAPEAK_32, &(data.u32d), 4 );
    if( ade.oilvl < data.u32d ){
      ade.oilvl = data.u32d;
    }
  }
  ade.ovlvl = (ade.ovlvl * 12) / 10;
  ade.oilvl = (ade.oilvl * 12) / 10;

  /* ----- Установка IRQ --------
   * IOA - Максимальный ток
   * OV  - Максимальное напряжение
   * AEHFA - Регистр подсчета Активной энергией наполовину заполнен
   */
  data.u32d = IRQENA_OIA | IRQENA_OV | IRQENA_AEHFA;
  sendAde( IRQENA_32, &(data.u32d), 4);

  // TODO: Запуск DSP (Digital Signal Processor)

  return rc;
}

// TODO: Отправка значения региства в ADE
eAdeState sendAde( uint16_t addr, uint8_t data[], uint8_t len ){
  uint8_t byteLen = (addr & 0x300) >> 8;

  if( len != byteLen ){
    return ADE_DATA_ERR;
  }
  uint8_t j = 0;
  // Send buffer packing
  spiTxBuf[j++] = (addr & 0xFF00) >> 8;
  spiTxBuf[j++] = addr & 0xFF;
  spiTxBuf[j++] = 0;    // Write data to ADE
  for( int8_t i = byteLen; i >= 0; i-- ){
    spiTxBuf[j++] = data[i];
  }

  SPI_Transmit( data, j );
  return ADE_OK;
}

int32_t i32ToAde( int32_t vol ){

  vol &= ~(0xFF000000);
  if( vol & 0x800000 ){
    // 24-ч битное отрицательное
    vol &= ~0x800000;
    vol = -vol;
  }

  return vol;
}

eAdeState adeIrqHandler( void ){
  uint32_t irqState;
  // Чтение регистра флагов прерывания ADE
  recvAde( IRQSTATA_32, &irqState, 4 );
  while( rxCplt != SET )
  {}

  // TODO: Обработка всех прерываний ADE
  if( irqState & IRQENA_Reset ) {
    adeState = ADE_READY;
  }
  else if( irqState & IRQENA_OIA ) {
    // Отправить сообщение на сарвер
    // XXX: Пересчитать величину ade.oilvl в Амперы
    canSendMsg( CUR_MAX, ade.oilvl );
    // TODO: Выставить управляющий защитой пин в 1
  }
  else if( irqState & IRQENA_OV ) {
    // Отправить сообщение на сарвер
    // XXX: Пересчитать величину ade.ovlvl в Вольты
    canSendMsg( VOLT_MAX, ade.ovlvl );
    // TODO: Выставить управляющий защитой пин в 1
  }
  else if( irqState & IRQENA_AEHFA ) {
    int32_t aenrg;
    // Отправить сообщение на сарвер
    // XXX: Пересчитать величину ade.oilvl в Амперы
    recvAde( AENERGYA_32, spiRxBuf );
  }

  // Сброс флагов прерываний
  recvAde( RSTIRQSTATA_32, irqState);

  return ADE_READY;
}
