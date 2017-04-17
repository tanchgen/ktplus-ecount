/*
 * my_uart.c
 *
 *  Created on: 06 дек. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
#include "buffer.h"
#include "my_time.h"
#include "can.h"
#include "fmt_translate.h"

USART_InitTypeDef usart;
tCanBuf	txUartBuf;
tCanBuf	rxUartBuf;

uint8_t txUart[512];
uint8_t rxUart[512];

uint8_t rxFin = 0;

//extern volatile time_t uxTime;

#define BAUDRATE 115200

#define PARAM_NB		15
const struct _param {
	uint8_t name[13];
	uint8_t len;
} param[PARAM_NB] = {
						{"TIME", 4},
					  {"WATT", 4},            // Действующее значение потребляемой мощности
					  {"ENERGYDAY", 9},       // Значение потребляемой мощности за сутки
					  {"ENRGDAYSUM", 10},     // Значение потребляемой мощности за сутки с нарастающим итогом
					  {"ENERGYMON", 9},       // Значение потребляемой мощности за месяц
					  {"ENRGMONSUM", 10},     // Значение потребляемой мощности за месяц с нарастающим итогом
					  {"WATTMONMAX", 10},     // Пиковое значение потребляемой мощности за месяц
					  // Сообщения от S207
					  {"WGAINA", 6},          // Коэффициент коррекции мощности - Фаза A
            {"WGAINN", 6},          // Коэффициент коррекции мощности - Фаза B
            {"WGAINN", 6},          // Коэффициент коррекции мощности - Фаза C
            {"WGAINN", 6},          // Коэффициент коррекции мощности - N
					  {"WATTINTERVAL", 12},   // Интервал отправки данных

					  // От S207 - задаются параметры, к S207 - превышены значения параметров
					  {"WATTMAX", 7},         // Максимум потребляемой мощности
					  {"CURMAX", 6},          // Максимальный ток в цепи
					  {"VOLTMAX", 7}          // Максимальное напряжение в цепи
};

void canRecvSimMsg( eMessId msgId, uint32_t data );
uint8_t mqttTopCoder( uint8_t * top, CanTxMsg * can );
uint8_t mqttMsgCoder( uint8_t * msg, CanTxMsg *can);

void uartInit( void ){
	GPIO_InitTypeDef port;

	//Включаем тактирование
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    //Пины PA9 и PA10 в режиме альтернативных функций –
    //Rx и Tx USART’а
    GPIO_StructInit(&port);
    port.GPIO_Mode = GPIO_Mode_AF_PP;
    port.GPIO_Pin = GPIO_Pin_9;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);

    port.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    port.GPIO_Pin = GPIO_Pin_10;
    port.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &port);

    //Настройка USART, все поля оставляем дефолтными, кроме скорости обмена
    USART_StructInit(&usart);
    usart.USART_BaudRate = BAUDRATE;
    USART_Init(USART1, &usart);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    USART_ITConfig(USART1, USART_IT_TC, ENABLE);
    //Запускаем сам USART
    USART_Cmd(USART1, ENABLE);
    NVIC_EnableIRQ(USART1_IRQn);

    txUartBuf.begin = 0;
    txUartBuf.end = 0;
    txUartBuf.full = 0;
    txUartBuf.len = 512;
    txUartBuf.size = 1;
    txUartBuf.bufAddr = txUart;

    rxUartBuf.begin = 0;
    rxUartBuf.end = 0;
    rxUartBuf.full = 0;
    rxUartBuf.len = 512;
    rxUartBuf.size = 1;
    rxUartBuf.bufAddr = rxUart;

}

void USART1_IRQHandler()
{
    if (USART_GetITStatus(USART1, USART_IT_TC) != RESET)
    {
    	uint8_t ch;
    	if( readBuff( &txUartBuf, &ch ) ){
        USART_SendData(USART1, ch);
    	}
      USART_ClearITPendingBit(USART1, USART_IT_TC);
    }
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
    	uint8_t ch = USART1->DR;
    	if( ch == '\n'){
    		rxFin = 1;
    	}
    	writeBuff( &rxUartBuf, &ch );
      USART_ClearITPendingBit(USART1, USART_IT_RXNE);
    }
}

void uartProcess( void ){
	uint8_t in[512];

	if( rxFin ){
		in[0] = 0;
		rxFin = 0;
		for( uint8_t i=0; i< 32; i++){
			readBuff( &rxUartBuf, &in[i]);
			if( in[i] == '\n'  ){
				break;
			}
		}
		// TODO: Декодирование принятого по UART сообщения
//		newTime = (uTime_t)atol( (char *)in );
//		canRecvSimMsg( TIME, newTime );
		// TODO: Отправка CAN-сообщения в приемный буфер CAN
	}

}

void sendMqttToUart( CanTxMsg * tmp ){

		uint8_t top[256];
		uint8_t msg[256];
		uint8_t len;

		mqttTopCoder( top, tmp );
		len = strlen((char*)top);
		for(uint8_t i = 0; i<len; i++ ){
			writeBuff( &txUartBuf, &top[i] );
		}
		writeBuff( &txUartBuf, (uint8_t*)"\t" );

		mqttMsgCoder( msg, tmp );
		len = strlen((char*)msg);
		for(uint8_t i = 0; i<len; i++ ){
			writeBuff( &txUartBuf, &msg[i] );
		}
		writeBuff( &txUartBuf, (uint8_t*)"\n" );
    if ( (USART1->SR & USART_SR_TXE) != RESET) {
    	uint8_t ch;
    	if( readBuff( &txUartBuf, &ch ) ){
        USART_SendData(USART1, ch);
    	}
    }

}

uint32_t getDevId( uint32_t canExtId ){
	return (canExtId & DEV_ID_MASK);
}

/* Формирует топик MQTT-сообщения в формате <S207_ID>/<DEVICE_ID>
 *
 * возвращает длину строки топика
 */
uint8_t mqttTopCoder( uint8_t * top, CanTxMsg * can ){
	uint8_t len, pos;
	uint8_t tmp[5], *ptmp;
	uint8_t msgId;

	pos = 0;
	ptmp = tmp;
	*top++ = '/';
	pos++;

	// Идентификатор устройства, передавшего сообщение - в топик
	len = hlToStr( getDevId(can->ExtId), &ptmp );
	memcpy( top, tmp, len);
	top += len;
	*top++ = '/';
	pos += 9;
	msgId = (can->ExtId & MSG_ID_MASK) >> 22;
	if(msgId >= 14) {
	  memcpy( top, param[msgId].name, param[msgId].len );
	  pos += param[msgId].len;

	  *(top + param[msgId].len) = '\0';
	}
	return (pos);
}


uint8_t mqttMsgCoder( uint8_t * msg, CanTxMsg *can) {
	eMessId msgId;
	uint8_t tmpMsg[80];
	uint8_t * pTmp = tmpMsg;
	uint8_t * tmpdata;

	// Дабавляем таймстамп
	timeToStr( uxTime, tmpMsg);

	sprintf((char *)msg, "{\"datetime\":\"%s\", \"payload\":\"", tmpMsg );

	msgId = (can->ExtId & MSG_ID_MASK) >> 22;

	tmpdata = can->Data+4;
	if( (msgId == TO_IN_MSG) || (msgId == TO_OUT_MSG) ) {
		// Теперь название и значение параметра
		fToStr( (float)*((int16_t *)(tmpdata))/16, tmpMsg, 6 );
	}
	else if ( msgId == IMP_EXEC ) {
		lToStr( *((uint32_t *)tmpdata), pTmp );
	}
	else if (msgId == TO_DELTA_HOUR) {
		fToStr( (float)*((int32_t *)(tmpdata))/16, tmpMsg, 9 );
	}
	else {
		ulToStr( *((uint32_t *)tmpdata), &pTmp );
	}

	strcat((char *)msg, (char *)tmpMsg);
	strcat((char *)msg, "\"}");

	return strlen((char *)msg);
}

int getFild( char *dst, char ** msg, char termin ){
  uint8_t i;

/*
  if( (**msg == ':') || (**msg == '=') ){
    (*msg)++;
  }
*/
  for( i=0; (**msg != '\0') && (**msg != ';') ; i++ ){
    if( **msg == termin ){
      (*msg)++;
      break;
    }
    dst[i] = *(*msg)++;
  }
  dst[i] = '\0';

  return strlen(dst);
}

int oneRec( char **pMsg, char * pat, char * pva ){
  *pat = '\0';
  *pva = '\0';

  if( getFild( pat, pMsg, '=' ) ){
    if( getFild( pva, pMsg, ':' ) ){
      return 2;
    }
  }
  return 0;
}

int msgParse( char * pMsg ){
  char atBuf[32];
//  char *patBuf = atBuf;
  char vaBuf[16];

  while( (*pMsg != 0) && (*pMsg != ';') ){
    uint8_t fildNum = 0;

    fildNum = oneRec( &pMsg, atBuf, vaBuf );
    if( fildNum == 2 ){
      uint8_t i;
      for( i = 0; i < PARAM_NB; i++ ){
        if( memcmp( atBuf, param[i].name, param[i].len ) == 0 ){
          break;
        }
      }
      if( i == PARAM_NB ){
        continue;
      }
      i += 15;

      canRecvSimMsg( i, atol(vaBuf) );

    }
  }

  return 0;
}

