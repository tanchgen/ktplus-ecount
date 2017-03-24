/*
 * firmata.h
 *
 *  Created on: Dec 27, 2016
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef SRC_FIRMATA_H_
#define SRC_FIRMATA_H_

#define START_SYSEX									0xF0
#define END_SYSEX										0xF7

// FARMATA SYSEX ID
#define EXTENDED_ID                 0x00 // A value of 0x00 indicates the next 2 bytes define the extended ID
//#define RESERVED               0x01-0x0F // IDs 0x01 - 0x0F are reserved for user defined commands
#define ANALOG_MAPPING_QUERY        0x69 // ask for mapping of analog to pin numbers
#define ANALOG_MAPPING_RESPONSE     0x6A // reply with mapping info
#define CAPABILITY_QUERY            0x6B // ask for supported modes and resolution of all pins
#define CAPABILITY_RESPONSE         0x6C // reply with supported modes and resolution
#define PIN_STATE_QUERY             0x6D // ask for a pin's current mode and state (different than value)
#define PIN_STATE_RESPONSE          0x6E // reply with a pin's current mode and state (different than value)
#define EXTENDED_ANALOG             0x6F // analog write (PWM, Servo, etc) to any pin
#define STRING_DATA                 0x71 // a string message with 14-bits per char
#define REPORT_FIRMWARE             0x79 // report name and version of the firmware
#define SAMPLEING_INTERVAL          0x7A // the interval at which analog input is sampled (default = 19ms)
#define SYSEX_NON_REALTIME          0x7E // MIDI Reserved for non-realtime messages
#define SYSEX_REALTIME              0X7F // MIDI Reserved for realtime messages

#define FIRM_CMD_INIT							0x7F01	// Инициализация
#define FIRM_CMD_STAT_CMD					0x7F02	// Перевод в режим Сбор статистики
#define FIRM_CMD_WAIT							0x7F03	// Перевод в режим Ожидания
#define FIRM_CMD_STAT_RCV					0x7F04	// Сбор статистики c конкретного ФМ (работает только в режиме Ожидания)
#define FIRM_CMD_PARAM_SEND				0x7E05	// Запись переменной
#define FIRM_CMD_CMD							0x7E06	// Команда
#define FIRM_CMD_STAT_QUER				0x7E07	// Запрос статистики по параметру
#define FIRM_CMD_STAT_RESP				0x7F07	// Отклик на запрос статистики по параметру
#define FIRM_CMD_VERS							0x79

int firmataCod( uint16_t cmd, uint8_t * fbuf, uint8_t * mbuf, uint8_t len );
int firmataDecod( uint8_t * buf, int len, int iface );
int firmCmdSend( uint16_t cmd, uint8_t * mbuf, uint8_t mlen, int iface );
int getFild( char *dst, char ** msg, char termin );
int forwardMsg( uint8_t * buf, int len );

#endif /* SRC_FIRMATA_H_ */
