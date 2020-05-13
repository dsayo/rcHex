/*
 * gait.c
 *
 *  Created on: May 13, 2020
 *      Author: dsayo
 */
#include <math.h>
#include "gait.h"
#include "ssc.h"
#include "ik.h"

extern float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG];

/* Generate power stroke command according to the division.
 * For example, giving a div of (float)1/3 would calculate a command to move
 * to length 1/3 of the power stroke vector.
 */
Command p_stroke(float x_dir, float y_dir, float div)
{
   Command cmd = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };

   cmd.pos_x = x_dir - 2 * x_dir * div;
   cmd.pos_y = y_dir - 2 * y_dir * div;
   cmd.pos_z = LEG_GROUND; /* Leg is on the ground */

   return cmd;
}

/* Start return stroke, raise leg (1/3)
 */
Command start_r_stroke(float x_dir, float y_dir)
{
   Command cmd = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };

   cmd.pos_x = -x_dir;
   cmd.pos_y = -y_dir;
   cmd.pos_z = LEG_RAISED;

   return cmd;
}

/* Mid return stroke (2/3)
 */
Command mid_r_stroke(float x_dir, float y_dir)
{
   Command cmd = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };

   cmd.pos_x = x_dir;
   cmd.pos_y = y_dir;
   cmd.pos_z = LEG_RAISED;

   return cmd;
}

/* End return stroke (3/3)
 */
Command end_r_stroke(float x_dir, float y_dir)
{
   Command cmd = {
         .rot_x = 0,
         .rot_y = 0,
         .rot_z = 0
   };

   cmd.pos_x = x_dir;
   cmd.pos_y = y_dir;
   cmd.pos_z = LEG_GROUND;

   return cmd;
}

/* Tripod phase
 */
void tripod_phase(Phase phase, uint16_t servo_speed, float crawl_angle)
{
   uint8_t leg_group_1 = L2 | L4 | L6;
   uint8_t leg_group_2 = L1 | L3 | L5;

   /* 2 phases: Mirrored commands for each subgroup */
   Command cmd_1;
   Command cmd_2;

   /* Calculate stroke vector */
   int16_t x_dir;
   int16_t y_dir;

   x_dir = (TRIPOD_STROKE_LEN / 2) * cosf(crawl_angle);
   y_dir = (TRIPOD_STROKE_LEN / 2) * sinf(crawl_angle);

   /* Form commands */
   switch (phase)
   {
      case A1:
      case B1:
         cmd_1 = p_stroke(x_dir, y_dir, (float)1/3);
         cmd_2 = start_r_stroke(x_dir, y_dir);
         break;

      case A2:
      case B2:
         cmd_1 = p_stroke(x_dir, y_dir, (float)2/3);
         cmd_2 = mid_r_stroke(x_dir, y_dir);
         break;

      case A3:
      case B3:
         cmd_1 = p_stroke(x_dir, y_dir, (float)3/3);
         cmd_2 = mid_r_stroke(x_dir, y_dir);
         break;

      default:
         return;
   }

   /* Issue mirrored commands */
   switch(phase)
   {
      /* Phase A: 1st half cycle */
      case A1:
      case A2:
      case A3:
         ik(cmd_1, leg_group_1, angle_delta);
         ik(cmd_2, leg_group_2, angle_delta);
         break;

      /* Phase B: 2nd half cycle, switch leg subgroups */
      case B1:
      case B2:
      case B3:
         ik(cmd_1, leg_group_2, angle_delta);
         ik(cmd_2, leg_group_1, angle_delta);
         break;

      default:
         return;
   }

   set_angles(ALL_LEGS, angle_delta, servo_speed);
   ssc_cmd_cr();
}

/* Ripple phase
 */
void ripple_phase(Phase phase, uint16_t servo_speed, float crawl_angle)
{
   /* 3 phases */
   Command cmd_1;
   Command cmd_2;
   Command cmd_3;

   /* Calculate stroke angles */
   int16_t x_dir;
   int16_t y_dir;

   y_dir = (TRIPOD_STROKE_LEN / 2) * sinf(crawl_angle);
   x_dir = (TRIPOD_STROKE_LEN / 2) * cosf(crawl_angle);

   /* Form and calculate command */
   switch (phase)
   {
      /* A1, A3, B2: LEG_4 return stroke */
      case A1:
         cmd_1 = start_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)4/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)1/6);
         break;

      case A3:
         cmd_1 = mid_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)5/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)2/6);
         break;

      case B2:
         cmd_1 = end_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)6/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)3/6);
         break;

      /* C1, C3, D2: LEG_5 return stroke */
      case C1:
         cmd_1 = p_stroke(x_dir, y_dir, (float)1/6);
         cmd_2 = start_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)4/6);
         break;

      case C3:
         cmd_1 = p_stroke(x_dir, y_dir, (float)2/6);
         cmd_2 = mid_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)5/6);
         break;

      case D2:
         cmd_1 = p_stroke(x_dir, y_dir, (float)3/6);
         cmd_2 = end_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)6/6);
         break;

      /* E1, E3, F2: LEG_6 return stroke */
      case E1:
         cmd_1 = p_stroke(x_dir, y_dir, (float)4/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)1/6);
         cmd_3 = start_r_stroke(x_dir, y_dir);
         break;

      case E3:
         cmd_1 = p_stroke(x_dir, y_dir, (float)5/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)2/6);
         cmd_3 = mid_r_stroke(x_dir, y_dir);
         break;

      case F2:
         cmd_1 = p_stroke(x_dir, y_dir, (float)6/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)3/6);
         cmd_3 = end_r_stroke(x_dir, y_dir);
         break;

      /* B1, B3, C2: LEG_1 return stroke */
      case B1:
         cmd_1 = start_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)1/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)4/6);
         break;

      case B3:
         cmd_1 = mid_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)2/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)5/6);
         break;

      case C2:
         cmd_1 = end_r_stroke(x_dir, y_dir);
         cmd_2 = p_stroke(x_dir, y_dir, (float)3/6);
         cmd_3 = p_stroke(x_dir, y_dir, (float)6/6);
         break;

      /* D1, D3, E2: LEG_3 return stroke */
      case D1:
         cmd_1 = p_stroke(x_dir, y_dir, (float)1/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)4/6);
         cmd_3 = start_r_stroke(x_dir, y_dir);
         break;

      case D3:
         cmd_1 = p_stroke(x_dir, y_dir, (float)2/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)5/6);
         cmd_3 = mid_r_stroke(x_dir, y_dir);
         break;

      case E2:
         cmd_1 = p_stroke(x_dir, y_dir, (float)3/6);
         cmd_2 = p_stroke(x_dir, y_dir, (float)6/6);
         cmd_3 = end_r_stroke(x_dir, y_dir);
         break;

      /* F1, F3, A2: LEG_2 return stroke */
      case F1:
         cmd_1 = p_stroke(x_dir, y_dir, (float)4/6);
         cmd_2 = start_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)1/6);
         break;

      case F3:
         cmd_1 = p_stroke(x_dir, y_dir, (float)5/6);
         cmd_2 = mid_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)2/6);
         break;

      case A2:
         cmd_1 = p_stroke(x_dir, y_dir, (float)6/6);
         cmd_2 = end_r_stroke(x_dir, y_dir);
         cmd_3 = p_stroke(x_dir, y_dir, (float)3/6);
         break;

      default:
         break;
   }

   /* Issue command, alternate command initiations every subphase */
   switch (phase)
   {
      case A1:
      case A3:
      case B2:
      case C1:
      case C3:
      case D2:
      case E1:
      case E3:
      case F2:
         /* Only left leg commands */
         ik(cmd_1, L4, angle_delta);
         ik(cmd_2, L5, angle_delta);
         ik(cmd_3, L6, angle_delta);
         set_angles(L4 | L5 | L6, angle_delta, servo_speed);
         break;

      case A2:
      case B1:
      case B3:
      case C2:
      case D1:
      case D3:
      case E2:
      case F1:
      case F3:
         /* Only right leg commands */
         ik(cmd_1, L1, angle_delta);
         ik(cmd_2, L2, angle_delta);
         ik(cmd_3, L3, angle_delta);
         set_angles(L1 | L2 | L3, angle_delta, servo_speed);
         break;

      default:
         return;
   }

   ssc_cmd_cr();
}
