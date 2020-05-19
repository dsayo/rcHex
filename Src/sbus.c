/*
 * sbus.c
 *
 *  Created on: Jan 23, 2020
 *      Author: dsayo
 */
#include "sbus.h"

uint8_t sync = 0;
uint8_t si = 0;   /* Sync index */

/* Finds the start of the SBUS packet in the buffer received through UART.
 */
void sync_bytes(uint8_t *pkt)
{
	int i;

	if (pkt[0] == START_BYTE && pkt[PACKET_SZ - 1] == END_BYTE)
	{
		/* Normal case */
		si = 0;
		return;
	}
	for (i = 1; i < PACKET_SZ; i++)
	{
		/* If read started in middle of pkt transmission */
		if (pkt[i] == START_BYTE && pkt[i - 1] == END_BYTE)
		{
			si = i;
			return;
		}
	}
}

/* Reads SBUS data and formats it into channel data.
 */
void sbus_format(uint8_t *pkt, RXData *data)
{
	if (!sync)
	{
		/* Synchronize UART buffer and SBUS packet */
		sync_bytes(pkt);
		sync = 1;
	}

	/* Extract 11-bit channel data from packet bytes */
	data->channels[0] = 0x7FF & (((uint16_t)pkt[SHFT(1)] >> 0) | ((uint16_t)pkt[SHFT(2)] << 8));
	data->channels[1] = 0x7FF & (((uint16_t)pkt[SHFT(2)] >> 3) | ((uint16_t)pkt[SHFT(3)] << 5));
	data->channels[2] = 0x7FF & (((uint16_t)pkt[SHFT(3)] >> 6) | ((uint16_t)pkt[SHFT(4)] << 2) | ((uint16_t)pkt[SHFT(5)] << 10));
	data->channels[3] = 0x7FF & (((uint16_t)pkt[SHFT(5)] >> 1) | ((uint16_t)pkt[SHFT(6)] << 7));
	data->channels[4] = 0x7FF & (((uint16_t)pkt[SHFT(6)] >> 4) | ((uint16_t)pkt[SHFT(7)] << 4));
	data->channels[5] = 0x7FF & (((uint16_t)pkt[SHFT(7)] >> 7) | ((uint16_t)pkt[SHFT(8)] << 1) | ((uint16_t)pkt[SHFT(9)] << 9));
	data->channels[6] = 0x7FF & (((uint16_t)pkt[SHFT(9)] >> 2) | ((uint16_t)pkt[SHFT(10)] << 6));
	data->channels[7] = 0x7FF & (((uint16_t)pkt[SHFT(10)] >> 5) | ((uint16_t)pkt[SHFT(11)] << 3));
	/* Add channels here */

	data->failsafe = FAILSAFE & pkt[23];
	data->lost_frame = LOST_FRAME & pkt[23];
	return;
}
