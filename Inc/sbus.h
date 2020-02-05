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
#define DEFAULT_MAX 1811

/* TODO: Channel scaling */
/* TODO: packet aligning */

#define CHAN_LONG 0  /* Longitudinal axis movement */
#define CHAN_TRAN 1  /* Transverse axis movement   */
#define CHAN_ROT  2  /* Normal axis rotation       */

typedef struct RXData {
	uint16_t channels[16];       /* Channel data */
	uint8_t failsafe;            /* Failsafe     */
	uint8_t lost_frame;          /* Lost frame   */
} RXData;

void sbus_format(uint8_t *pkt, RXData *data); /* Read a packet, return true if complete, else false */

#endif /* INC_SBUS_H_ */
