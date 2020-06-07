/*******************************************************************************
 * controls.c
 *
 * General control subroutines and functions for the hexapod.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 11 Feb 2020
 *******************************************************************************
 */
#include <math.h>
#include "controls.h"
#include "ssc.h"
#include "sbus.h"
#include "ik.h"
#include "gait.h"

extern Phase max_phase;
extern float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG];

/* Powerup stance command: Coxa 0 deg, femur 45 deg, tibia 45 deg
 * Because SSC ignores speed commands for the very first command,
 * this is done first before slowly transitioning to the neutral stance.
 */
void powerup_stance()
{
   int leg;
   int servo;
   uint32_t pw;  /* Pulse width */

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

/* Neutral stance: all servos at 0 degrees.
*/
void neutral_stance()
{
   int leg;
   int servo;

   for (leg = 0; leg < NUM_LEGS; leg++)
   {
      for (servo = 0; servo < NUM_SERVO_PER_LEG; servo++)
      {
         servo_move(ssc_channel[leg][servo], CENTER_PW, NEUTRAL_SERVO_SPEED,
               NO_TIME);
      }
   }

   ssc_cmd_cr();
}

/* Poll the arm switch and return whether it is on or not.
*/
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

/* Poll the rotation and mode switch and return the corresponding mode.
 * rot HIGH -> Crawl mode
 * mod HIGH -> Crawl mode
 * mod MID  -> X-Y control mode
 * mod LOW  -> Roll-Pitch-Yaw mode
 */
Mode get_mode(RXData rx_data)
{
   if (rx_data.channels[CHAN_ROT] > DEFAULT_MID)
   {
      return MODE_CRAWL;
   }

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

/* Poll the rotation and crawl mode switch and return the corresponding gait.
 * The rotation switch takes priority, as it can be switched to from any gait
 * or any stationary mode.
 * rot HIGH -> Rotation gait
 * cmo HIGH -> Wave gait
 * cmo MID  -> Ripple gait
 * cmd LOW  -> Tripod gait
 */
CrawlMode get_cmod(RXData rx_data)
{
   /* Prioritize rotation switch */
   if (rx_data.channels[CHAN_ROT] > DEFAULT_MID)
   {
      max_phase = B3;
      return ROTATE;
   }

   /* Check crawl mode switch */
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

/* Get the sequencer speed from control data. If in rotation mode, use yaw
 * channel. In other crawl modes, use the maximum of x-y channels.
 */
uint16_t get_speed(RXData rx_data, CrawlMode cmod)
{
   if (cmod == ROTATE)
   {
      return SPEED_SCALAR * abs(rx_data.channels[CHAN_YAW] - DEFAULT_MID);
   }
   /* Calculate speed */
   return SPEED_SCALAR * MAX(abs(rx_data.channels[CHAN_PITCH] - DEFAULT_MID),
         abs(rx_data.channels[CHAN_ROLL] - DEFAULT_MID));

}

/* Return the angle of direction of travel (in radians)
 * using the x and y channels.
 */
float get_angle(RXData rx_data)
{
   return atan2f(rx_data.channels[CHAN_PITCH] - DEFAULT_MID,
         rx_data.channels[CHAN_ROLL] - DEFAULT_MID);
}

/* Return the direction of rotation (+CW -CCW) from yaw channel.
*/
int16_t get_rot_dir(RXData rx_data)
{
   return DEFAULT_MID - rx_data.channels[CHAN_YAW];
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

/* Convert rx data into a command to be used in stationary mode.
*/
Command to_command(RXData rxdata, Mode mode)
{
   Command cmd;

   cmd.pos_x = 0;
   cmd.pos_y = 0;
   cmd.pos_z = ((int16_t)rxdata.channels[CHAN_Z] - DEFAULT_MID) /
      RXDATA_SCALAR_MM;
   cmd.rot_x = 0;
   cmd.rot_y = 0;
   cmd.rot_z = ((int16_t)rxdata.channels[CHAN_YAW] - DEFAULT_MID) /
      RXDATA_SCALAR_DEG;

   switch (mode)
   {
      case MODE_XY:
         cmd.pos_x = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_ROLL]) /
            RXDATA_SCALAR_MM;
         cmd.pos_y = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_PITCH]) /
            RXDATA_SCALAR_MM;
         break;

      case MODE_RPY:
         cmd.rot_x = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_PITCH]) /
            RXDATA_SCALAR_DEG;
         cmd.rot_y = (DEFAULT_MID - (int16_t)rxdata.channels[CHAN_ROLL]) /
            RXDATA_SCALAR_DEG;
         break;

      default:
         break;
   }

   return cmd;
}

/* Send the servo movement commands to the SSC32 for the legs in leg_bitmap
 * using the angles in angle_delta and the given speed. For leg_bitmap,
 * 0b00000001 represents LEG_1, 0b00000010 LEG_2, and so on. Use the macros L1
 * thru L6 and bitwise OR to command subsets of legs.
 */
void set_angles(uint8_t leg_bitmap,
      float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG], uint16_t speed)
{
   int leg;
   int servo;
   uint32_t pw;  /* Pulse width */

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
                     pw = CL(CENTER_PW + PW_PER_DEGREE *
                           angle_delta[leg][servo]);
                     break;

                  default:
                     pw = CL(CENTER_PW - PW_PER_DEGREE *
                           angle_delta[leg][servo]);
                     break;
               }
               break;
         }

         servo_move(ssc_channel[leg][servo], pw, speed, NO_TIME);
      }
   }
}

/* Tuned function to convert sequencer speed to servo movement speeds to be as
 * fluid as possible i.e. not too slow such that commands seem choppy at each
 * phase, and servos move fast enough to not cut corners between commands and
 * the final positions are reached.
 */
uint16_t to_servo_speed(uint16_t seq_speed)
{
   uint16_t ctrl_val = seq_speed / SPEED_SCALAR; /* Revert to raw ctrl data */

   /* Piecewise function from tuning */
   if (ctrl_val < SPEED_PIECEWISE_THRESH)
   {
      return SPEED_FX_1(ctrl_val);
   }

   /* Need steeper slope to keep up with sequence speed */
   return SPEED_FX_2(ctrl_val);
}

/* Execute the phase based on the gait type.
*/
void exec_phase(Phase phase, CrawlMode cmod, uint16_t seq_speed,
      float crawl_angle, int16_t rot_dir)
{
   uint16_t servo_speed = to_servo_speed(seq_speed);

   switch (cmod)
   {
      case TRIPOD:
         tripod_phase(phase, servo_speed, crawl_angle);
         break;

      case RIPPLE:
         ripple_phase(phase, servo_speed, crawl_angle);
         break;

      case WAVE:
         wave_phase(phase, servo_speed, crawl_angle);
         break;

      case ROTATE:
         rotate_phase(phase, servo_speed, rot_dir);
         break;

      default:
         break;
   }
}

