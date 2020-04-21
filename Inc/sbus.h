/*
 * sbus.h
 *
 *  Created on: Jan 23, 2020
 *      Author: dsayo
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "main.h"

#define START_BYTE 0x0F
#define END_BYTE   0x00
#define TIMEOUT_MS 7
#define PACKET_SZ 25
#define FAILSAFE   0x08
#define LOST_FRAME  0x04
#define DEFAULT_MIN 172
#define DEFAULT_MID 992
#define DEFAULT_MAX 1811

/* Calculates packet index with offset in case of misalignment */
#define SHFT(i) ((si + i) % PACKET_SZ)

/* TODO: Channel scaling */

#define CHAN_LONG 0  /* Longitudinal axis movement */
#define CHAN_TRAN 1  /* Transverse axis movement   */
#define CHAN_ROT  2  /* Normal axis rotation       */

typedef struct RXData {
	uint16_t channels[16];       /* Channel data */
	uint8_t failsafe;            /* Failsafe     */
	uint8_t lost_frame;          /* Lost frame   */
} RXData;

#define DELTA_THRESH 20

void sbus_format(uint8_t *pkt, RXData *data); /* Read a packet, return true if complete, else false */
uint8_t ctrl_delta(RXData *old, RXData *new); /* */

#endif /* INC_SBUS_H_ */
