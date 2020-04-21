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

void set_all_angles(float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG])
{
	int leg;
	int servo;
	uint32_t pw;

	for (leg = 0; leg < NUM_LEGS; leg++)
	{
		for (servo = 0; servo < NUM_SERVO_PER_LEG; servo++)
		{
			switch(leg)
			{
				case LEG_1:
				case LEG_2:
				case LEG_3:
					pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
					break;

				default:  /* Mirror for left legs */
					pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
					break;
			}

			servo_move(ssc_channel[leg][servo], pw, NO_ARG);
		}
	}
}
