/*
 * controls.c
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */
#include "controls.h"
#include "ssc.h"
#include "main.h"

uint8_t armed = 0;

void arm()
{
	armed = 1;
}

void disarm()
{
	armed = 0;
}

void init_stance()
{
	uint8_t i;

	for (i = 0; i < 32; i++)
	{
		servo_move(i, CENTER_PW, 0);
	}
	ssc_cmd_cr();
}

void leg_forward()
{

}
