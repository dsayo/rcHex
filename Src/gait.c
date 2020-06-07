/*******************************************************************************
 * gait.c
 *
 * Gait coordination for the movement modes.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 13 May 2020
 *******************************************************************************
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
Command p_stroke(int16_t x_dir, int16_t y_dir, float div)
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


/* Start return stroke: raise leg (1/3)
*/
Command start_r_stroke(int16_t x_dir, int16_t y_dir)
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

/* Mid return stroke: move leg in air (2/3)
*/
Command mid_r_stroke(int16_t x_dir, int16_t y_dir)
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

/* End return stroke: drop leg to ground(3/3)
*/
Command end_r_stroke(int16_t x_dir, int16_t y_dir)
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

/* Power stroke (yaw version). Has the same idea of dividing the yaw angle.
*/
Command p_stroke_yaw(int16_t rot_angle, float div)
{
   Command cmd = {
      .pos_x = 0,
      .pos_y = 0,
      .rot_x = 0,
      .rot_y = 0
   };

   cmd.rot_z = rot_angle - 2 * rot_angle * div;
   cmd.pos_z = LEG_GROUND; /* Leg is on the ground */

   return cmd;
}

/* Start return stroke (yaw version): raise leg (1/3)
*/
Command start_r_stroke_yaw(int16_t rot_angle)
{
   Command cmd = {
      .pos_x = 0,
      .pos_y = 0,
      .rot_x = 0,
      .rot_y = 0,
   };

   cmd.rot_z = -rot_angle;
   cmd.pos_z = LEG_RAISED;

   return cmd;
}

/* Mid return stroke (yaw version): move leg in air (2/3)
*/
Command mid_r_stroke_yaw(int16_t rot_angle)
{
   Command cmd = {
      .pos_x = 0,
      .pos_y = 0,
      .rot_x = 0,
      .rot_y = 0,
   };

   cmd.rot_z = rot_angle;
   cmd.pos_z = LEG_RAISED;

   return cmd;
}

/* End return stroke (yaw version): drop leg to ground (3/3)
*/
Command end_r_stroke_yaw(int16_t rot_angle)
{
   Command cmd = {
      .pos_x = 0,
      .pos_y = 0,
      .rot_x = 0,
      .rot_y = 0,
   };

   cmd.rot_z = rot_angle;
   cmd.pos_z = LEG_GROUND;

   return cmd;
}

/* Executes tripod phase. Fast movement, but not as stable. Three feet on ground
 * simultaneously.
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

   x_dir = (STROKE_LEN / 2) * cosf(crawl_angle); /* x-component */
   y_dir = (STROKE_LEN / 2) * sinf(crawl_angle); /* y-component */

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

   /* Send and execute commands */
   set_angles(ALL_LEGS, angle_delta, servo_speed);
   ssc_cmd_cr();
}

/* Executes ripple phase. Less fast, but more stable.
*/
void ripple_phase(Phase phase, uint16_t servo_speed, float crawl_angle)
{
   /* 3 phases */
   Command cmd_1;
   Command cmd_2;
   Command cmd_3;

   /* Calculate stroke vector */
   int16_t x_dir;
   int16_t y_dir;

   y_dir = (STROKE_LEN / 2) * sinf(crawl_angle);
   x_dir = (STROKE_LEN / 2) * cosf(crawl_angle);

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

/* Executes wave phase: slow, but most stable.
*/
void wave_phase(Phase phase, uint16_t servo_speed, float crawl_angle)
{
   /* 6 simultaneous commands; 1 per leg */
   Command cmd[NUM_LEGS];
   int leg;

   /* Static leg phase counters */
   static uint8_t cntr[NUM_LEGS] = WAVE_INIT_PHASES;

   /* Calculate stroke vector */
   int16_t x_dir;
   int16_t y_dir;

   y_dir = (STROKE_LEN / 2) * sinf(crawl_angle);
   x_dir = (STROKE_LEN / 2) * cosf(crawl_angle);

   for (leg = 0; leg < NUM_LEGS; leg++)
   {
      switch (cntr[leg])
      {
         /* 3 subphases of Return stroke */
         case WAVE_RETURN_1:
            cmd[leg] = start_r_stroke(x_dir, y_dir);
            break;

         case WAVE_RETURN_2:
            cmd[leg] = mid_r_stroke(x_dir, y_dir);
            break;

         case WAVE_RETURN_3:
            cmd[leg] = end_r_stroke(x_dir, y_dir);
            break;

         default:
            /* 15 subphases of power stroke */
            cmd[leg] = p_stroke(x_dir, y_dir, (float)cntr[leg]/15);
            break;
      }

      /* Calculate leg angles */
      ik(cmd[leg], (1 << leg), angle_delta);

      /* To next subphase on next function call */
      cntr[leg]++;
      if (cntr[leg] > 18)
      {
         cntr[leg] = 1;
      }
   }

   set_angles(ALL_LEGS, angle_delta, servo_speed);
   ssc_cmd_cr();
}

/* Rotate phase: similar to tripod, but with yaw. Center of body stays in place,
 * but alternating tripod legs rotate the heading of the robot.
 */
void rotate_phase(Phase phase, uint16_t servo_speed, int16_t rot_dir)
{
   uint8_t leg_group_1 = L2 | L4 | L6;
   uint8_t leg_group_2 = L1 | L3 | L5;

   /* 2 phases: Mirrored commands for each subgroup */
   Command cmd_1;
   Command cmd_2;

   /* Calculate rotation direction */
   int16_t rot_angle;

   if (rot_dir > 0)
   {
      /* Rotate clockwise */
      rot_angle = ROTATION_ANGLE;
   }
   else if (rot_dir < 0)
   {
      /* Rotate counter clockwise */
      rot_angle = -ROTATION_ANGLE;

   }
   else
   {
      /* No rotation */
      rot_angle = 0;
   }

   /* Form commands */
   switch (phase)
   {
      case A1:
      case B1:
         cmd_1 = p_stroke_yaw(rot_angle, (float)1/3);
         cmd_2 = start_r_stroke_yaw(rot_angle);
         break;

      case A2:
      case B2:
         cmd_1 = p_stroke_yaw(rot_angle, (float)2/3);
         cmd_2 = mid_r_stroke_yaw(rot_angle);
         break;

      case A3:
      case B3:
         cmd_1 = p_stroke_yaw(rot_angle, (float)3/3);
         cmd_2 = end_r_stroke_yaw(rot_angle);
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

