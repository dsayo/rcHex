/*
 * sbus.h
 *
 *  Created on: Jan 23, 2020
 *      Author: dsayo
 */

#ifndef INC_SBUS_H_
#define INC_SBUS_H_

#include "main.h"
#include <stdlib.h>

/* SBUS protocol specs */
#define START_BYTE 0x0F
#define END_BYTE   0x00
#define PACKET_SZ 25
#define FAILSAFE   0x08
#define LOST_FRAME  0x04

/* Radio-dependent min/mid/max values. */
#define DEFAULT_MIN 172 /* [Taranis Q X7] */
#define DEFAULT_MID 992
#define DEFAULT_MAX 1811

/* Calculates packet index with offset in case of misalignment */
#define SHFT(i) ((si + i) % PACKET_SZ)

/* Parsed receiver data format */
typedef struct RXData {
	uint16_t channels[16];       /* Channel data */
	uint8_t failsafe;            /* Failsafe     */
	uint8_t lost_frame;          /* Lost frame   */
} RXData;

void sbus_format(uint8_t *pkt, RXData *data); /* Read a packet, return true if complete, else false */

#endif /* INC_SBUS_H_ */
