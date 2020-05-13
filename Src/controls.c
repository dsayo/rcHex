/*
 * controls.c
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */
#include <math.h>
#include "controls.h"
#include "ssc.h"
#include "sbus.h"
#include "ik.h"
#include "gait.h"

extern Phase max_phase;
extern float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG];

/* Powerup stance: Coxa 90 deg, femur 45 deg, tibia 45 deg.
 * Because SSC ignorees speed commands, this is done first before slowly
 * transitioning to init stance.
 */
void powerup_stance()
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
               pw = CENTER_PW;
               break;

            case FEMUR:  /* Mirror left leg femur/tibia servos */
            case TIBIA:
               switch(leg)
               {
                  case LEG_1:
                  case LEG_2:
                  case LEG_3:
                     pw = CENTER_PW + PW_PER_DEGREE * 45;
                     break;

                  default:
                     pw = CENTER_PW - PW_PER_DEGREE * 45;
                     break;
               }
               break;
         }
         servo_move(ssc_channel[leg][servo], pw, NO_SPD, NO_TIME);
      }
   }
   ssc_cmd_cr();
}

void init_stance()
{
   int leg;
   int servo;

   for (leg = 0; leg < NUM_LEGS; leg++)
   {
      for (servo = 0; servo < NUM_SERVO_PER_LEG; servo++)
      {
         servo_move(ssc_channel[leg][servo], CENTER_PW, 500, NO_TIME);
      }
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
      return MODE_CRAWL;
   }
   else if (rx_data.channels[CHAN_MODE] == DEFAULT_MID)
   {
      return MODE_XY;
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
      max_phase = F3;
      return WAVE;
   }
   else if (rx_data.channels[CHAN_CMOD] == DEFAULT_MID)
   {
      max_phase = F3;
      return RIPPLE;
   }
   else
   {
      max_phase = B3;
      return TRIPOD;
   }
}

uint16_t get_speed(RXData rx_data)
{
   /* Calculate speed */
   return SPEED_SCALAR * MAX(abs(rx_data.channels[CHAN_PITCH] - DEFAULT_MID),
         abs(rx_data.channels[CHAN_ROLL] - DEFAULT_MID));

}

float get_angle(RXData rx_data)
{
   return atan2f(rx_data.channels[CHAN_PITCH] - DEFAULT_MID,
                 rx_data.channels[CHAN_ROLL] - DEFAULT_MID);
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
			}

			servo_move(ssc_channel[leg][servo], pw, speed, NO_TIME);
		}
	}
}

uint16_t to_servo_speed(uint16_t seq_speed)
{
   uint16_t ctrl_val = seq_speed >> 6;

   /* Piecewise function: steeper for higher magnitude */
   if (ctrl_val < 600)
   {
      return 5 * ctrl_val / 4 + 50;
   }
   else
   {
      return 2 * ctrl_val - 400;
   }
}

void exec_phase(Phase phase, CrawlMode cmod, uint16_t seq_speed, float crawl_angle)
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

      default:
         return;
   }

   servo_speed = to_servo_speed(seq_speed);

   switch (cmod)
   {
      case TRIPOD:
         tripod_phase(phase, servo_speed, crawl_angle);
         break;

      case RIPPLE:
         ripple_phase(phase, servo_speed, crawl_angle);
         break;

      case WAVE:
         break;

      default:
         break;
   }
}


