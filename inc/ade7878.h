/*
 * ade7878.h
 *
 *  Created on: 06 апр. 2017 г.
 *      Author: G.Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ADE7878_H_
#define ADE7878_H_


// REgister Addresses
#define RUN    0xE228 // Start Digital Signal Processor

#define AIGAIN 0x4380
#define AVGAIN 0x4381
#define BIGAIN 0x4382
#define BVGAIN 0x4383
#define CIGAIN 0x4384
#define CVGAIN 0x4385
#define NIGAIN 0x4386

#define ISUM   0x43BF
#define AIRMS  0x43C0
#define AVRMS  0x43C1
#define BIRMS  0x43C2
#define BVRMS  0x43C3
#define CIRMS  0x43C4
#define CVRMS  0x43C5
#define NIRMS  0x43C6

#define WTHR1  0x43AB       // Регистр максимально допустимой мощности
#define WTHR0  0x43AC       // Регистр минимально допустимой мощности





#endif /* ADE7878_H_ */
