/*
 * controls.c
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */
#include "controls.h"
#include "ssc.h"
#include "sbus.h"

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
		servo_move(i, CENTER_PW, NO_SPD, NO_TIME);
	}
	ssc_cmd_cr();
}

/* Compares changes in control data to a threshold.
 * Returns 0 if no change, 1 if there is.
 */
uint8_t ctrl_delta(RXData *old, RXData *new)
{
   int i;

   for (i = 0; i < MAX_CHAN_USED + 1; i++)
   {
      if (abs(old->channels[i] - new->channels[i]) > DELTA_THRESH)
      {
         return 1;
      }
   }
   return 0;
}

Command to_command(RXData rxdata)
{
	Command cmd;

	cmd.pos_x = 0;
	cmd.pos_y = 0;
	cmd.pos_z = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_Z]) / RXDATA_SCALE_MM;
	cmd.rot_x = ((int16_t)rxdata.channels[CHAN_PITCH] - DEFAULT_MID) / RXDATA_SCALE_DEG;
	cmd.rot_y = ((int16_t)rxdata.channels[CHAN_ROLL] - DEFAULT_MID) / RXDATA_SCALE_DEG;
	cmd.rot_z = ((int16_t)rxdata.channels[CHAN_YAW] - DEFAULT_MID) / RXDATA_SCALE_DEG;

	return cmd;
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
			switch(servo)
			{
				case COXA:
					pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
					break;

				case FEMUR:  /* Mirror left leg femur servos */
				   switch(leg)
				   {
				      case LEG_1:
				      case LEG_2:
				      case LEG_3:
				         pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;

				      case LEG_4:
				      case LEG_5:
				      case LEG_6:
				         pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;
				   }
				   break;

            case TIBIA:
				   switch(leg)
				   {
				      case LEG_1:
				      case LEG_2:
				      case LEG_3:
				         pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;

				      case LEG_4:
				      case LEG_5:
				      case LEG_6:
				         pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;
				   }

			}

			servo_move(ssc_channel[leg][servo], pw, NO_SPD, 200);
		}
	}
}
