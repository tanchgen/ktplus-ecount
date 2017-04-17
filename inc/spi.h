/*
 * spi.h
 *
 *  Created on: 13 февр. 2017 г.
 *      Author: jet G.V.Tanchin <g.tanchin@yandex.ru>
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>

#define BUF_SIZE    511
#define SPI_LEN      7

#define SPI SPI2
/* SPI2_NSS - PB12,
 * SPI2_SCK - PB13,
 * SPI2_MISO - PB14,
 * SPI2_MOSI - PB15
*/
#define SPI_MOSI_PIN    GPIO_Pin_15
#define SPI_MISO_PIN    GPIO_Pin_14
#define SPI_SCK_PIN     GPIO_Pin_13
#define SPI_NSS_PIN     GPIO_Pin_12

#define SPI_PORT    GPIOB

extern uint8_t spiTxBuf[];
extern uint8_t spiRxBuf[];

int spiInit(void);
void dmaRxInit( uint8_t * rxdata, uint16_t size );
void dmaTxInit( uint8_t * txdata, uint16_t size );
void SPI_Receive(uint8_t *data, uint16_t size);
void SPI_Transmit(uint8_t *data, uint16_t size);
void SPI_TransRecv( uint8_t * txData, uint8_t * rxData, uint8_t len );

#endif /* SPI_H_ */
