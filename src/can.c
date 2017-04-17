/*
 * can.c
 *
 *  Created on: 30 июля 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

//#include <main.h>
#include "stm32f10x_conf.h"
#include "my_time.h"
#include "buffer.h"
#include "ade.h"
#include "can.h"

	uint32_t selfDevId;
	uint32_t valveDevId;

#define CIRCUIT			COLD

// Преобразование 32-х битное со знаком в 24-х битное со знаком
//int32_t	i32ToAde( int32_t );
void getIdList( tCanId *canid, uint32_t extId);

void canCoreInit( void ){
	CAN_InitTypeDef CAN_InitStruct;

	CAN_DeInit(CAN_CAN);
	CAN_InitStruct.CAN_Prescaler = 18;
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStruct.CAN_BS1 = CAN_BS1_4tq;
	CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStruct.CAN_TTCM = DISABLE;
	CAN_InitStruct.CAN_ABOM = DISABLE;
	CAN_InitStruct.CAN_AWUM = DISABLE;
	CAN_InitStruct.CAN_NART = DISABLE;
	CAN_InitStruct.CAN_RFLM = DISABLE;
	CAN_InitStruct.CAN_TXFP = DISABLE;
	CAN_Init(CAN_CAN, &CAN_InitStruct);

	canFilterInit();

	CAN_ITConfig(CAN_CAN, CAN_IT_FMP0, ENABLE);
	CAN_ITConfig(CAN_CAN, CAN_IT_TME, ENABLE);
	CAN_ITConfig(CAN_CAN, CAN_IT_ERR, ENABLE);
	CAN_ITConfig(CAN_CAN, CAN_IT_BOF, ENABLE);

}
void canInit(void)
{
	NVIC_InitTypeDef CAN_NVIC_InitStruct;

#define DEV_SIGNATURE			(0x1FFFF7E8+8)
	selfDevId = (*(uint32_t *)DEV_SIGNATURE) & 0xFFFFF;

	RCC->APB1ENR |= RCC_APB1Periph_CAN1;

	canBspInit();

	canCoreInit();

	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX1_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_TX_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_SCE_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);

	canBufferInit();
}

void canBspInit( void ){
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

#if ( CAN_RX_PIN_NUM > 7)
	// CAN RX GPIO configuration
	CAN_RX_PORT->CRH &= ~((uint32_t)0xF << ((CAN_RX_PIN_NUM-8)*4));
	CAN_RX_PORT->CRH |= (uint32_t)0x8 << ((CAN_RX_PIN_NUM-8)*4);			// Input PullUp-PullDown

	// CAN TX GPIO configuration
	CAN_RX_PORT->CRH &= ~((uint32_t)0xF << ((CAN_TX_PIN_NUM-8)*4));
	CAN_RX_PORT->CRH |= (uint32_t)0xB << ((CAN_TX_PIN_NUM-8)*4);			// AF Output PullUp-PullDown 50MHz
#else
	// CAN RX GPIO configuration
	CAN_RX_PORT->CRL &= ~((uint32_t)0xF << ((CAN_RX_PIN_NUM)*4));
	CAN_RX_PORT->CRL |= (uint32_t)0x8 << ((CAN_RX_PIN_NUM)*4);			// Input PullUp-PullDown

	// CAN TX GPIO configuration
	CAN_RX_PORT->CRL &= ~((uint32_t)0xF << ((CAN_TX_PIN_NUM)*4));
	CAN_RX_PORT->CRL |= (uint32_t)0xB << ((CAN_TX_PIN_NUM)*4);			// AF Output PullUp-PullDown 50MHz

#endif

	CAN_RX_PORT->ODR |= CAN_RX_PIN;

	AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

}

void canFilterInit( void ){
	tFilter filter;
	tCanId canId;
	uint8_t  filterNum;

// Формируем фильтр для приема пакетов от S207
	canId.adjCur = ADJ;
	canId.coldHot = 1;
	canId.msgId = VALVE_DEG;
	canId.s207 = S207_DEV;
	canId.devId = selfDevId;
	// Фильтр принимаемых устройств
#if CAN_TEST
// Для тестирования в колбцевом режиме - маска = 0x00000000
	filter.idList = 0;
	filter.idMask = 0;
#else
	filter.idList = setIdList( &canId );
	filter.idMask = CUR_ADJ_MASK | MSG_ID_MASK | DEV_ID_MASK;
#endif

	filter.ideList = 0;
	filter.ideMask = 0;
	filter.rtrList = 0;
	filter.rtrMask = 0;
	filterNum = 0;

	canFilterUpdate( &filter, filterNum );

	// Для сообщения TIME
	canId.adjCur = ADJ;
	canId.msgId = TIME;
	// Фильтр принимаемых устройств
#if CAN_TEST
// Для тестирования в колбцевом режиме - маска = 0x00000000
	filter.idList = 0;
	filter.idMask = 0;
#else
	filter.idList = setIdList( &canId );
	filter.idMask = CUR_ADJ_MASK | MSG_ID_MASK;
#endif

	filterNum = 1;

	canFilterUpdate( &filter, filterNum );

}

void canFilterUpdate( tFilter * filter, uint8_t filterNum ) {
	CAN_FilterInitTypeDef CAN_FilterInitStruct;

	filter->idList <<= 0x3;
	CAN_FilterInitStruct.CAN_FilterIdHigh = (filter->idList >> 16) & 0xFFFF;
	CAN_FilterInitStruct.CAN_FilterIdLow = (filter->idList & 0xFFFF) | (filter->ideList << 2) | (filter->rtrList << 1);
	filter->idMask <<= 0x3;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (filter->idMask >> 16) & 0xFFFF;
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = (filter->idMask & 0xFFFF) | (filter->ideMask << 2) | (filter->rtrMask << 1);

	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber = filterNum;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);
}

/*
void canFilterUpdate( tFilter * filter, uint8_t filterNum ) {
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	uint16_t stdId;
	uint32_t extId;

	stdId = (filter->idList & 0x7FF);
	extId = (filter->idList >> 11) & 0x3FFFF;
	CAN_FilterInitStruct.CAN_FilterIdHigh = (stdId << 5) | (extId >> 13);
	CAN_FilterInitStruct.CAN_FilterIdLow = (extId << 3) | (filter->ideList << 2) | (filter->rtrList << 1);
	stdId = (filter->idMask & 0x7FF);
	extId = (filter->idMask >> 11) & 0x3FFFF;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (stdId << 5) | (extId >> 13);
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = (extId << 3) | (filter->ideMask << 2) | (filter->rtrMask << 1);

	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber = filterNum;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);

}
*/

void canRx0IrqHandler(void) {
	CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0))
	{
//		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&RxMessage);

		writeBuff( &canRxBuf, (uint8_t *)&RxMessage );
	}
}

void canRx1IrqHandler(void) {
	CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP1))
	{
//		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
		CAN_Receive(CAN1, CAN_FIFO1, (CanRxMsg *)&RxMessage);

		writeBuff( &canRxBuf, (uint8_t *)&RxMessage );
	}
}

void canTxIrqHandler(void) {
	CanTxMsg TxMessage;

	if ((CAN_GetITStatus(CAN1, CAN_IT_TME)))
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
		// Если есть сообщение для отправки - отправить его
		if (  readBuff( &canTxBuf, (uint8_t *)&TxMessage) ) {
	    // Если есть сообщение для отправки - отправить его
			CAN_Transmit(CAN1, &TxMessage);
		}
	}
}

void canSceIrqHandler(void) {
	if (CAN_GetITStatus(CAN1, CAN_IT_BOF)){
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
		canCoreInit();
	}
	if (CAN_GetITStatus(CAN1, CAN_IT_ERR)){
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
		canCoreInit();
	}
}

void canProcess( void ){
	CanRxMsg rxMessage;
	CanTxMsg txMessage;

  // Select one empty transmit mailbox
	if( canTxBuf.begin != canTxBuf.end ){
		if( ((CAN1->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) ||
  		((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) ||
			((CAN1->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) ){
//Читаем предназначенные для отправки сообщения, если они есть, и запихиваем его в буфер отправки.
			if (  readBuff( &canTxBuf, (uint8_t *)&txMessage) ) {
				CAN_Transmit(CAN1, (CanTxMsg *)&txMessage);
			}
		}
  }

  if( readBuff( &canRxBuf, (uint8_t *)&rxMessage) ) {
  	tCanId canid;
  	getIdList( &canid, rxMessage.ExtId );
  	switch( canid.msgId ){
      int32_t regVal;
      uint16_t regAddr;

  		case WGAIN_A:           // Коэффициент усиления вход тока - Фаза A
#ifdef ADE7878
// Переделать адреса для регистров 3-фазного
      case IGAIN_B:           // Коэффициент усиления вход тока - Фаза B
      case VGAIN_B:           // Коэффициент усиления вход напряжения - Фаза B
      case IGAIN_C:           // Коэффициент усиления вход тока - Фаза C
      case VGAIN_C:           // Коэффициент усиления вход напряжения - Фаза C
#endif
        regAddr = AWGAIN_32;
        // 0.1% = 4194 ед,
        // 0x400000 - WGAIN = 1, 0x200000 - WGAIN = -50%, 0x600000 - WGAIN = +50%,
  		  regVal = ((*((int32_t *)&rxMessage.Data) ) * 4194) + 0x400000;
  		  sendAde( regAddr, (uint8_t *)&regVal, 4 );
  		  break;
      case WATT_SEND_TOUT:    // Интервал отправки данных
        sendTout = *((uint32_t *)&rxMessage.Data);
  			break;
      case WATT_MAX:         // Максимум потребляемой мощности
        // TODO: Установка границы максимальной потребляемой мощности
        // Читаем Предел напряжения
        recvAde_s( OVLVL_32, (uint8_t *)&regVal, 4);
        ade.maxWatt = *((uint32_t *)&rxMessage.Data);
        // Вычисляем Максимальный ток
        regVal = ade.maxWatt/( regVal/1.2);
        regVal *= K_W;
        sendAde( OILVL_32, (uint8_t *)&regVal, 4);
        break;

      case TIME:
  			uxTime = *((uint32_t *)&rxMessage.Data);
  			// Allow access to BKP Domain
  			PWR_BackupAccessCmd(ENABLE);
  			// Wait until last write operation on RTC registers has finished
  			RTC_WaitForLastTask();
  			// Set initial value
  			RTC_SetCounter( uxTime );
  			// Wait until last write operation on RTC registers has finished
  			RTC_WaitForLastTask();
  			// Lock access to BKP Domain
  			if( (RCC->BDCR & RCC_BDCR_RTCSEL ) != RCC_BDCR_RTCSEL ){
  				//RTC тактируется не от HSE
  				PWR_BackupAccessCmd(DISABLE);
  			}
  			break;
  		default:
  			break;
  	}
  }

}

void canSendMsg( eMsgId msgId, uint32_t data ) {
	CanTxMsg canTxMsg;
	tCanId canId;
	// Формируем структуру canId

	canId.adjCur = CUR;
	canId.devId = selfDevId;
	canId.msgId = msgId;
	canId.coldHot = CIRCUIT;
	canId.s207 = nS207_DEV;

	// Включаем системное время
	*((uint32_t *)canTxMsg.Data) = (uint32_t)uxTime;
	// Для всех, кроме температуры, беззнаковое 32-х битное целое
	*((uint32_t *)(canTxMsg.Data+4)) = data;
	canTxMsg.DLC = 8;

	canTxMsg.ExtId = setIdList( &canId );
	canTxMsg.IDE = CAN_Id_Extended;
	canTxMsg.RTR = 0;
	canTxMsg.StdId = 0;

	writeBuff( &canTxBuf, (uint8_t *)&canTxMsg );
}

uint32_t setIdList( tCanId *canid ){
 return 	( (((canid->adjCur)<<28) & CUR_ADJ_MASK)	|
		 	 	 	 	(((canid->msgId)<<22) & MSG_ID_MASK)		|
						(((canid->coldHot)<<21) & COLD_HOT_MASK)|
						(((canid->s207)<<20) & S207_MASK)				|
						((canid->devId) & DEV_ID_MASK) );
}

void getIdList( tCanId *canid, uint32_t extId){
	canid->adjCur = (extId & CUR_ADJ_MASK) >> 28;
	canid->msgId =  (extId & MSG_ID_MASK) >> 22;
	canid->coldHot = (extId & COLD_HOT_MASK) >> 21;
	canid->s207 = (extId & S207_MASK) >> 20;
	canid->devId = (extId & DEV_ID_MASK);
}

// Для тестов
void canRecvSimMsg( eMsgId msgId, uint32_t data ) {
  CanTxMsg canTxMsg;
  tCanId canId;
  // Формируем структуру canId

  canId.devId = selfDevId;

  canId.adjCur = ADJ;
  canId.msgId = msgId;
  canId.coldHot = 1;
  canId.s207 = S207_DEV;

  // Для всех, кроме температуры, беззнаковое 32-х битное целое
  *((uint32_t *)canTxMsg.Data) = data;
  canTxMsg.DLC = 4;

  canTxMsg.ExtId = setIdList( &canId );
  canTxMsg.IDE = CAN_Id_Extended;
  canTxMsg.RTR = 0;
  canTxMsg.StdId = 0;

  writeBuff( &canRxBuf, (uint8_t *)&canTxMsg );
}
