/*
 * spi.h
 *
 *  Created on: 13 февр. 2017 г.
 *      Author: jet G.V.Tanchin <g.tanchin@yandex.ru>
 */

#ifndef SPI_H_
#define SPI_H_

#define BUF_SIZE    511
#define RX_LEN      127

//#define ISR_PIN     GPIO_Pin_12
//#define ISR_PORT    GPIOB

#define ISR_PIN     GPIO_Pin_9
#define ISR_PORT    GPIOA

extern uint8_t rxBuf[];

int spiInit(void);
void dmaRxInit( uint8_t * rxdata, uint16_t size );
void dmaTxInit( uint8_t * txdata, uint16_t size );
void SPI_ReceiveData(uint8_t *data, uint16_t size);
void SPI_TransmitData(uint8_t *data, uint16_t size);

#endif /* SPI_H_ */
