/*
 * ade.c
 *
 *  Created on: 04 апр. 2017 г.
 *      Author: G.Tanchin <g.tanchin@yandex.ru>
 */

#include <spi.h>
#include "string.h"
#include "stm32f10x.h"

#include "main.h"
#include "my_time.h"
#include "buffer.h"
#include "can.h"
#include "ade.h"

volatile eAdeState adeState = ADE_STOP;

static eAdeState adeSetup( void );

tAde ade;

int adeIrqInit( void ){
  int rc = -1;

  // Initialization ADE IRQ Input

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

int adeCtrlInit( void ) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

#if (CTRL_PIN_NUM > 7)
  CTRL_PORT->CRH &= ~(0xF << ((CTRL_PIN_NUM - 8) * 4));
  CTRL_PORT->CRH |= 0x3 << ((CTRL_PIN_NUM - 8)* 4);
#else
  CTRL_PORT->CRL &= ~(0xF << (CTRL_PIN_NUM * 4));
  CTRL_PORT->CRL |= 0x3 << (CTRL_PIN_NUM * 4);

#endif
  // Выставляем в 0
  CTRL_PORT->BRR |= CTRL_PIN;
  return 0;
}

eAdeState adeInit( void ){
  uint32_t tout = sTick+100;

  // Инициируем SPI
  spiInit();
  adeIrqInit();
  while( adeState != ADE_READY){
    if( sTick > tout ){
      break;
// TODO: Вернуть "Выход с ошибкой по тайматуту"
//      return adeState;
    }
  }
  memset( (uint8_t*)&ade, 0, sizeof(tAde) );
  ade.tWatt = 1;
  ade.tWattCount = ade.tWatt;
  ade.tSend = 60;
  ade.tSendCount = ade.tSend;
  ade.tEnergy = 15;
  ade.tEnergyCount = ade.tEnergy;
  adeCtrlInit();
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
  sendAde( SETUP_REG_16, (uint8_t *)&(data.u16d), 2 );

  // Установка предварительного усиления
  data.u8d = 1;         // Коэффициент уселителя на входе (1, 2, 4, 8, 16, 22)
  sendAde( PGA_V_8, (uint8_t *)&(data.u8d), 1 );
  sendAde( PGA_IA_8, (uint8_t *)&(data.u8d), 1 );
//  ade.awgain = data.u32d;        // No gain calibration (Minimum = 0x2000000 = -50%)
//  sendAde( AWGAIN_32, (uint8_t *)&(ade.awgain), 4 );

  // Установка MAX тока и напряжения
  recvAde( RSTVPEAK_32, (uint8_t *)&(data.u32d), 4 );
  recvAde( RSTIAPEAK_32, (uint8_t *)&(data.u32d), 4);
  for( uint8_t i = 0; i < 10; i++ ) {
    myDelay(100);
    recvAde_s( VPEAK_32, (uint8_t *)&(data.u32d), 4 );
    if( ade.ovlvl < data.u32d ){
      ade.ovlvl = data.u32d;
    }
    recvAde_s( IAPEAK_32, (uint8_t *)&(data.u32d), 4 );
    if( ade.oilvl < data.u32d ){
      ade.oilvl = data.u32d;
    }
  }
  ade.ovlvl = (ade.ovlvl * 12) / 10;
  sendAde( OVLVL_32, (uint8_t *)ade.ovlvl, 4);
  ade.oilvl = (ade.oilvl * 12) / 10;
  sendAde( OILVL_32, (uint8_t *)ade.oilvl, 4);

  /* ----- Установка IRQ --------
   * IOA - Максимальный ток
   * OV  - Максимальное напряжение
   * AEHFA - Регистр подсчета Активной энергией наполовину заполнен
   */
  data.u32d = IRQENA_OIA | IRQENA_OV | IRQENA_AEHFA;
  sendAde( IRQENA_32, (uint8_t *)&(data.u32d), 4);

  // TODO: Запуск DSP (Digital Signal Processor)

  return rc;
}

// Отправка значения региства в ADE
eAdeState sendAde( uint16_t addr, uint8_t data[], uint8_t len ){
  uint8_t byteLen = ((addr & 0x300) >> 8)+1;

  if( len != byteLen ){
    return ADE_DATA_ERR;
  }
  uint8_t j = 0;
  // Send buffer packing
  spiTxBuf[j++] = (addr & 0xFF00) >> 8;
  spiTxBuf[j++] = addr & 0xFF;
  spiTxBuf[j++] = 0x00;    // Write data to ADE
  for( int8_t i = byteLen; i > 0; ){
    spiTxBuf[j++] = data[--i];
  }

  SPI_Transmit( spiTxBuf, j );
  return ADE_OK;
}

// Отправка значения региства в ADE
eAdeState recvAde( uint16_t addr, uint8_t * data, uint8_t len ){
  uint8_t byteLen = ((addr & 0x300) >> 8)+1;

  if( len != byteLen ){
    return ADE_DATA_ERR;
  }
  uint8_t j = 0;
  // Send buffer packing
  spiTxBuf[j++] = (addr & 0xFF00) >> 8;
  spiTxBuf[j++] = addr & 0xFF;
  spiTxBuf[j++] = 0x80;    // Read data from ADE
  j += len;

  SPI_TransRecv( spiTxBuf, data, j );
  return ADE_OK;
}

eAdeState recvAde_s( uint16_t addr, uint8_t * data, uint8_t len ){
  eAdeState rc;
  uint32_t tout = sTick + 100;

  if( (rc = recvAde( addr, data, len )) != ADE_OK ){
    return rc;
  }

  while( rxCplt != SET ){
    if( sTick > tout ){
      return ADE_ERR;
    }
  }
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
  recvAde_s( IRQSTATA_32, (uint8_t *)&irqState, 4 );

  // Обработка всех прерываний ADE
  if( irqState & IRQENA_Reset ) {
    adeState = ADE_READY;
  }
  else {
    if( irqState & IRQENA_OIA ) {
      uint32_t tmpI;
      // Отправить сообщение на сарвер
      tmpI = recvI();
      canSendMsg( CUR_MAX, tmpI );
      // Выставить управляющий защитой пин в 1
      CTRL_PORT->BSRR |= CTRL_PIN;
    }
    if( irqState & IRQENA_OV ) {
      uint32_t tmpV;
      // Отправить сообщение на сарвер
      tmpV = recvV();
      canSendMsg( VOLT_MAX, tmpV );
      // Выставить управляющий защитой пин в 1
      CTRL_PORT->BSRR |= CTRL_PIN;
    }
    if( irqState & IRQENA_AEHFA ) {
      // Отправить сообщение на сарвер
      ade.tEnergyCount = ade.tEnergy;
      ade.enrgDay += recvE();
    }
  }

  // Сброс флагов прерываний
  recvAde( RSTIRQSTATA_32, (uint8_t *)&irqState, 4);

  return ADE_READY;
}

eAdeState adeSecondProcess( void ) {
  if( --ade.tWattCount == 0 ){
    ade.tWattCount = ade.tWatt;
    // Получаем значение мощности
    ade.watt = recvW();
    if(ade.watt > ade.maxWattMon){
      ade.maxWattMon = ade.watt;
    }
  }
  if( --ade.tSendCount == 0 ){
    ade.tSendCount = ade.tSend;
    // Отправляем в S207 значение мощности
    canSendMsg( AWATT_NOW, ade.watt );
  }
  if( --ade.tEnergyCount == 0 ){
    ade.tEnergyCount = ade.tEnergy;
    ade.enrgDay += recvE();
  }
  if( sysRtc.DayFlag == SET ) {
    // TODO: Отправить значение потребленной энергии сначала за сутки
    canSendMsg( AENRG_DAY, ade.enrgDay );
    ade.enrgMon += ade.enrgDay;
    ade.enrgDay = 0;
    canSendMsg( AENRG_MON, ade.enrgMon );
    if( sysRtc.MonthFlag == SET ){
      canSendMsg( AENRG_MON, ade.enrgMon );
      ade.enrgYear += ade.enrgMon;
      ade.enrgMon = 0;
      canSendMsg( AWATT_MON_MAX, ade.maxWattMon);
      ade.maxWattMon = 0;
    }
  }

  return ADE_READY;
}

uint32_t recvV( void ){
  uint32_t tmpV;
  if( recvAde_s( VRMS_32, (uint8_t *)&tmpV, 4 ) != ADE_OK ){
    return 0;
  }
  tmpV /= K_V;
  return tmpV;
}

uint32_t recvI( void ){
  uint32_t tmpI;
  if( recvAde_s( IRMSA_32, (uint8_t *)&tmpI, 4 ) != ADE_OK ){
    return 0;
  }
  tmpI /= K_I;
  return tmpI;
}

uint32_t recvW( void ){
  uint32_t tmpW;
  if( recvAde_s( AWATT_32, (uint8_t *)&tmpW, 4 ) != ADE_OK ){
    return 0;
  }
  tmpW /= K_W;
  return tmpW;
}

uint32_t recvE( void ){
  uint32_t tmpE;
  if( recvAde_s( AENERGYA_32, (uint8_t *)&tmpE, 4 ) != ADE_OK ){
    return 0;
  }
  tmpE /= K_W;
  return tmpE;
}

