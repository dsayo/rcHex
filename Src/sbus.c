/*
 * sbus.c
 *
 *  Created on: Jan 23, 2020
 *      Author: dsayo
 */
#include "sbus.h"
#include "main.h"

/* Reads SBUS data and formats it into channel data.
 */
void sbus_format(uint8_t *pkt, RXData *data)
{
	data->channels[0] = ((uint16_t)pkt[1] >> 0) | ((uint16_t)pkt[2] << 8);
	data->channels[1] = (pkt[2] >> 3) | ((uint16_t)pkt[3] << 5);
	data->channels[2] = (pkt[3] >> 6) | ((uint16_t)pkt[4] << 2) | ((uint16_t)pkt[5] << 10);
	data->channels[3] = (pkt[5] >> 1) | ((uint16_t)pkt[6] << 7);
	data->channels[4] = (pkt[6] >> 4) | ((uint16_t)pkt[7] << 4);
	/* ... */

	data->failsafe = FAILSAFE & pkt[23];
	data->lost_frame = LOST_FRAME & pkt[23];
	return;
}
