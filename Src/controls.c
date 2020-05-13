/*
 * controls.c
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */
#include "controls.h"
#include "ssc.h"
#include "sbus.h"
#include "ik.h"

extern Phase max_phase;
extern float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG];

void init_stance()
{
   uint8_t i;

   for (i = 0; i < 32; i++)
   {
      servo_move(i, CENTER_PW, NO_SPD, NO_TIME);
   }
   ssc_cmd_cr();
}

uint8_t get_arm(RXData rx_data)
{
   if (rx_data.channels[CHAN_ARM] > DEFAULT_MID)
   {
      GPIOB->ODR |= GPIO_PIN_5;
      return 1;
   }
   else
   {
      GPIOB->ODR &= ~GPIO_PIN_5;
      return 0;
   }
}

Mode get_mode(RXData rx_data)
{
   if (rx_data.channels[CHAN_MODE] > DEFAULT_MID)
   {
      return MODE_XY;
   }
   else if (rx_data.channels[CHAN_MODE] == DEFAULT_MID)
   {
      return MODE_CRAWL;
   }
   else
   {
      return MODE_RPY;
   }
}

CrawlMode get_cmod(RXData rx_data)
{
   if (rx_data.channels[CHAN_CMOD] > DEFAULT_MID)
   {
      max_phase = F2;
      return CMOD_6;
   }
   else if (rx_data.channels[CHAN_CMOD] == DEFAULT_MID)
   {
      max_phase = C2;
      return CMOD_3;
   }
   else
   {
      max_phase = B2;
      return TRIPOD;
   }
}

uint16_t get_speed(RXData rx_data, Mode mode)
{
   if (mode == MODE_CRAWL)
   {
      /* Calculate speed */
      return SPEED_SCALAR * MAX(abs(rx_data.channels[CHAN_PITCH] - DEFAULT_MID),
            abs(rx_data.channels[CHAN_ROLL] - DEFAULT_MID));
   }
   else
   {
      return 0;
   }
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

Command to_command(RXData rxdata, Mode mode)
{
	Command cmd;

	cmd.pos_x = 0;
	cmd.pos_y = 0;
   cmd.pos_z = ((int16_t)rxdata.channels[CHAN_Z] - DEFAULT_MID) / RXDATA_SCALAR_MM;
	cmd.rot_x = 0;
	cmd.rot_y = 0;
   cmd.rot_z = ((int16_t)rxdata.channels[CHAN_YAW] - DEFAULT_MID) / RXDATA_SCALAR_DEG;

	switch (mode)
	{
	   case MODE_XY:
	      cmd.pos_x = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_ROLL]) / RXDATA_SCALAR_MM;
	      cmd.pos_y = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_PITCH]) / RXDATA_SCALAR_MM;
	      break;

	   case MODE_RPY:
	      cmd.rot_x = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_PITCH]) / RXDATA_SCALAR_DEG;
	      cmd.rot_y = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_ROLL]) / RXDATA_SCALAR_DEG;

	      break;

	   default:
	      break;
	}

	return cmd;
}

void set_angles(uint8_t leg_bitmap, float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG],
      uint16_t speed)
{
	int leg;
	int servo;
	uint32_t pw;

	for (leg = 0; leg < NUM_LEGS; leg++)
	{
	   if (!(leg_bitmap & (1 << leg)))
	   {
	      /* Leg not set in bitmap, no change */
	      continue;
	   }

		for (servo = 0; servo < NUM_SERVO_PER_LEG; servo++)
		{
			switch(servo)
			{
				case COXA:
					pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
					break;

				case FEMUR:  /* Mirror left leg femur/tibia servos */
				case TIBIA:
				   switch(leg)
				   {
				      case LEG_1:
				      case LEG_2:
				      case LEG_3:
				         pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;

				      default:
				         pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
				         break;
				   }
				   break;

//            case TIBIA: /* Only mirror in rot mode BUG: rot and pos contribute in opposite directions to tibia*/
//				   switch(leg)
//				   {
//				      case LEG_1:
//				      case LEG_2:
//				      case LEG_3:
//				         pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
//				         break;
//
//				      default:
//				         pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
//				         break;
//				   }
//				   break;

			}

			servo_move(ssc_channel[leg][servo], pw, speed, NO_TIME);
		}
	}
}

/* Tripod phase
 */
void tripod_phase(Phase phase, uint16_t servo_speed)
{
   uint8_t leg_group_1 = L2 | L4 | L6;
   uint8_t leg_group_2 = L1 | L3 | L5;

   /* Mirrored commands for each subgroup */
   Command cmd_1 = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };
   Command cmd_2  = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };

   /* Form commands */
   switch (phase)
   {
      case A1:
      case B1:
         cmd_1.pos_x = 0;
         cmd_1.pos_y = 10;
         cmd_1.pos_z = LEG_GROUND;

         cmd_2.pos_x = 0;
         cmd_2.pos_y = -30;
         cmd_2.pos_z = LEG_RAISED;
         break;

      case A2:
      case B2:
         cmd_1.pos_x = 0;
         cmd_1.pos_y = -10;
         cmd_1.pos_z = LEG_GROUND;

         cmd_2.pos_x = 0;
         cmd_2.pos_y = 30;
         cmd_2.pos_z = LEG_RAISED;
         break;

      case A3:
      case B3:
         cmd_1.pos_x = 0;
         cmd_1.pos_y = -30;
         cmd_1.pos_z = LEG_GROUND;

         cmd_2.pos_x = 0;
         cmd_2.pos_y = 30;
         cmd_2.pos_z = LEG_GROUND;
         break;

      default:
         break;
   }

   /* Issue commands */
   switch(phase)
   {
      /* Phase A: 1st half cycle */
      case A1:
      case A2:
      case A3:
         ik(cmd_1, leg_group_1, angle_delta);
         ik(cmd_2, leg_group_2, angle_delta);
         set_angles(ALL_LEGS, angle_delta, servo_speed);
         break;

      /* Phase B: Switch commands, complete 2nd half cycle */
      case B1:
      case B2:
      case B3:
         ik(cmd_1, leg_group_2, angle_delta);
         ik(cmd_2, leg_group_1, angle_delta);
         set_angles(ALL_LEGS, angle_delta, servo_speed);
         break;

      default:
         break;
   }

   ssc_cmd_cr();
}

uint16_t to_servo_speed(uint16_t seq_speed)
{
   return (seq_speed >> 6) + 500;
}

void exec_phase(Phase phase, CrawlMode cmod, uint16_t seq_speed)
{
   uint16_t servo_speed;

   switch (phase)
   {
      case A1:
      case A2:
      case A3:
         GPIOF->ODR |= GPIO_PIN_1;
         GPIOB->ODR &= ~GPIO_PIN_4;
         GPIOF->ODR &= ~GPIO_PIN_0;
         break;

      case B1:
      case B2:
      case B3:
         GPIOF->ODR &= ~GPIO_PIN_1;
         GPIOB->ODR |= GPIO_PIN_4;
         GPIOF->ODR &= ~GPIO_PIN_0;
         break;

      case C1:
      case C2:
      case C3:
         GPIOF->ODR |= GPIO_PIN_1;
         GPIOB->ODR |= GPIO_PIN_4;
         GPIOF->ODR &= ~GPIO_PIN_0;
         break;

      case D1:
      case D2:
      case D3:
         GPIOF->ODR &= ~GPIO_PIN_1;
         GPIOB->ODR &= ~GPIO_PIN_4;
         GPIOF->ODR |= GPIO_PIN_0;
         break;

      case E1:
      case E2:
      case E3:
         GPIOF->ODR |= GPIO_PIN_1;
         GPIOB->ODR &= ~GPIO_PIN_4;
         GPIOF->ODR |= GPIO_PIN_0;
         break;

      case F1:
      case F2:
      case F3:
         GPIOF->ODR &= ~GPIO_PIN_1;
         GPIOB->ODR |= GPIO_PIN_4;
         GPIOF->ODR |= GPIO_PIN_0;
         break;
   }

   servo_speed = to_servo_speed(seq_speed);

   switch (cmod)
   {
      case TRIPOD:
         tripod_phase(phase, servo_speed);
         break;

      default:
         break;
   }
}
