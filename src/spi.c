#include "stm32f10x.h"

#include "main.h"
#include "spi.h"
#include "buffer.h"


#define DMA_TX_ON 1
#define DMA_RX_ON 1

extern uint8_t rxCplt;
extern uint8_t txCplt;
uint8_t testBuf[BUF_SIZE];

uint8_t spiTxBuf[SPI_LEN];
uint8_t spiRxBuf[SPI_LEN];

int spiInit(void) {
 
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef SPI_InitStructure;  

  RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;


/* SPI2_NSS - PB12,
 * SPI2_SCK - PB13,
 * SPI2_MISO - PB14,
 * SPI2_MOSI - PB15
 */
//  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 ;
/*
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;

  GPIO_Init(GPIOB, &GPIO_InitStructure);
*/
  // For SPI2 - MISO: PB14
  // Reset Pin
  GPIOB->CRH &= ~(0xF << ((14-8)*4));
  // Output AF Pull-Push 50MHz Pin
  GPIOB->CRH |= 0x4 << ((14-8)*4);

  // SPI_NSS, SPI_CLK, SPI_MOSI
  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //Заполняем структуру с параметрами SPI модуля
  SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex; //полный дуплекс
  SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b; // передаем по 8 бит
  SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low; // Полярность и
  SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge; // фаза тактового сигнала
  SPI_InitStructure.SPI_NSS = SPI_NSS_Hard; // Управлять состоянием сигнала NSS аппаратно
  SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2; // Предделитель SCK
  SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB; // Первым отправляется старший бит
  SPI_InitStructure.SPI_Mode = SPI_Mode_Master; // Режим - слейв
  SPI_InitStructure.SPI_CRCPolynomial = 7;

  SPI_Init(SPI2, &SPI_InitStructure); //Настраиваем SPI1

#if DMA_TX_ON
  // Передаем через DMA
  dmaTxInit( spiTxBuf, SPI_LEN );
#endif

#if DMA_RX_ON
  // Принимаем через DMA
  dmaRxInit( spiRxBuf, SPI_LEN );
#else
  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта

  SPI_Cmd(SPI2, ENABLE); // Включаем модуль SPI1....

  NVIC_EnableIRQ(SPI2_IRQn); //Разрешаем прерывания от SPI1
#endif

  return 0;
}
 
//Обработчик прерываний от SPI1

void SPI2_IRQHandler (void) {

#if ! ( DMA_RX_ON )
  if( (SPI2->SR & SPI_SR_RXNE) && (SPI2->CR2 & SPI_CR2_RXNEIE) ) {
    // Прерывание вызвано приемом байта ?
    uint8_t ch;

    ch = SPI2->DR; //Читаем то что пришло
    if( ch ){
      BUFFER_Write( &rxBuffer, &ch, 1);
    }
  }
#endif

#if ! DMA_TX_ON
  if( (SPI2->SR & SPI_SR_TXE) && (SPI2->CR2 & SPI_CR2_TXEIE) ) {
    if( (iTx == 0x200) || (txBuf[iTx-1] == 0xF7) ){
      txCplt = SET;
      iTx = 0;
      // For SPI2 - MISO: PB14
      ISR_PORT->BSRR |= ISR_PIN;
      // Reset Pin
      GPIOB->CRH &= ~(0xF << ((14-8)*4));
      // Input Floating Pin
      GPIOB->CRH |= 0x4 << ((14-8)*4);
      SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта
      SPI_I2S_ITConfig( SPI2, SPI_I2S_IT_TXE, DISABLE ); //Выключаем прерывание по прередаче байта
    }
    else {
      SPI2->DR = txBuf[iTx];
      testBuf[iTx] = txBuf[iTx];
    // Не больше 511-и
      iTx++;
    }
  }
#endif // DMA_TX_ON
}


void dmaTxInit(uint8_t * txdata, uint16_t size) {
  DMA_InitTypeDef dma;
 
  // Тактируем DMA
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
  // Адрес периферии
  dma.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
  // Адрес того, что передавать
  dma.DMA_MemoryBaseAddr = (uint32_t)txdata;
  // Направление передачи: тут - в периферию
  dma.DMA_DIR = DMA_DIR_PeripheralDST;
  // Размер данных. Сколько передавать...
  dma.DMA_BufferSize = size;
  dma.DMA_M2M = DMA_M2M_Disable;
  // Побайтно...
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  // Увеличение адреса в памяти
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // Увеличение адреса периферии, у нас не меняем
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // От начала передаем
  dma.DMA_Mode = DMA_Mode_Normal;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_Priority = DMA_Priority_Medium;
 
  // 5й канал - это SPI2_TX
  DMA_Init(DMA1_Channel5, &dma);

  // Связываем DMA с событием окончания передачи
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Tx, ENABLE);
 
  // Прерывание по окончанию передачи DMA
  DMA_ITConfig(DMA1_Channel5, DMA_IT_TC, ENABLE);
  // Включаем прерывание
  NVIC_EnableIRQ(DMA1_Channel5_IRQn);
}

void dmaRxInit(uint8_t * rxdata, uint16_t size) {
  DMA_InitTypeDef dma;
 
  // Тактируем DMA
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
 
  // Адрес периферии
  dma.DMA_PeripheralBaseAddr = (uint32_t)(&(SPI2->DR));
  // Адрес того, что передавать
  dma.DMA_MemoryBaseAddr = (uint32_t)rxdata;
  // Направление передачи: тут - в периферию
  dma.DMA_DIR = DMA_DIR_PeripheralSRC;
  // Размер данных. Сколько передавать...
  dma.DMA_BufferSize = size;
  dma.DMA_M2M = DMA_M2M_Disable;
  // Побайтно...
  dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  // Увеличение адреса в памяти
  dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
  // Увеличение адреса периферии, у нас не меняем
  dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  // От начала передаем
  dma.DMA_Mode = DMA_Mode_Normal;
  dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  dma.DMA_Priority = DMA_Priority_Medium;
 
  // 4й канал - это SPI2_RX
  DMA_Init(DMA1_Channel4, &dma);
 
  // Связываем DMA с событием окончания передачи
  SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx, DISABLE);
 
  // Прерывание по окончанию передачи DMA
  DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
  // Включаем прерывание
  NVIC_EnableIRQ(DMA1_Channel4_IRQn);
}

void DMA1_Channel4_IRQHandler(void) {
  if (DMA_GetFlagStatus(DMA1_IT_TC4) == SET) {
 
    // Останавливаем запросы DMA
    SPI2->CR2 &= ~SPI_CR2_RXDMAEN;

    // Disable the selected DMAy Channelx
    DMA1_Channel4->CCR &= ~DMA_CCR1_EN;

    // Clear Transfer Complete IT Flag
    DMA1->IFCR = DMA1_IT_TC4;
    rxCplt = SET;
  }
 
}

void DMA1_Channel5_IRQHandler(void) {
  if (DMA_GetFlagStatus(DMA1_IT_TC5) == SET) {

    // Ждем последний байт
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_TXE) == RESET);
    while(SPI_I2S_GetFlagStatus(SPI2,SPI_I2S_FLAG_BSY) == SET);

    // Останавливаем запросы DMA
    SPI2->CR2 &= ~SPI_CR2_TXDMAEN;

    DMA_Cmd(DMA1_Channel5, DISABLE);
    DMA_ClearITPendingBit(DMA1_IT_TC5);
    txCplt = SET;
//    SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,ENABLE); //Включаем прерывание по приему байта
//    GPIO_SetBits(GPIOA, GPIO_Pin_4);
  }

}


void SPI_Receive(uint8_t *data, uint16_t size) {
//  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
  rxCplt = RESET;
  DMA1_Channel4->CMAR = (uint32_t)data;
  DMA1_Channel4->CNDTR = size;
  DMA_Cmd(DMA1_Channel4, ENABLE);
  // Начинаем прием
  SPI2->CR2 &= ~SPI_CR2_RXDMAEN;
}

void SPI_Transmit(uint8_t *data, uint16_t size) {
//  GPIO_ResetBits(GPIOA, GPIO_Pin_4);
//  SPI_I2S_ITConfig(SPI2,SPI_I2S_IT_RXNE,DISABLE); //Выключаем прерывание по приему байта
  // Для начала инициализируем прием через DMA
  txCplt = RESET;
#if DMA_TX_ON
  DMA1_Channel5->CMAR = (uint32_t)data;
  DMA1_Channel5->CNDTR = size;
  DMA_Cmd(DMA1_Channel5, ENABLE);
  // Начинаем передачу
  SPI2->CR2 &= ~SPI_CR2_TXDMAEN;
#else
  memcpy( txBuf, data, size);
  txBuf[size] = '\0';
  // Старт передачи
  SPI2->DR = txBuf[iTx];
  testBuf[iTx] = txBuf[iTx];
  iTx++;

  // For SPI2 - MISO: PB14
  // Reset Pin
  GPIOB->CRH &= ~(0xF << ((14-8)*4));
  // Output AF Pull-Push 50MHz Pin
  GPIOB->CRH |= 0xB << ((14-8)*4);

  SPI_I2S_ITConfig( SPI2, SPI_I2S_IT_TXE, ENABLE ); //Включаем прерывание по передаче байта
#endif
}

void SPI_TransRecv( uint8_t * txData, uint8_t * rxData, uint8_t len ){
  rxCplt = RESET;
  DMA1_Channel4->CMAR = (uint32_t)rxData;
  DMA1_Channel4->CNDTR = len;
  DMA_Cmd(DMA1_Channel4, ENABLE);
  txCplt = RESET;
  DMA1_Channel5->CMAR = (uint32_t)txData;
  DMA1_Channel5->CNDTR = len;
  DMA_Cmd(DMA1_Channel5, ENABLE);
  // Начинаем передачу-прием
  SPI2->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}
