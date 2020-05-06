#include <math.h>
#include "ik.h"

const float OFFSET_X[NUM_LEGS] =
   {42.5f, 62.5f, 42.5f, -42.5f, -62.5f, -42.5f};
const float OFFSET_Y[NUM_LEGS] =
   {82.614f, 0.0f, -82.614f, -82.614f, 0.0f, 82.614f};

const float INIT_POS_X[NUM_LEGS] =
   {72.125f, 102.0f, 72.125f, -72.125f, -102.0f, -72.125f};
const float INIT_POS_Y[NUM_LEGS] =
   {72.125f, 0, -72.125f, -72.125f, 0, 72.125f};
const float INIT_POS_Z[NUM_LEGS] =
   {107.0f, 107.0f, 107.0f, 107.0f, 107.0f, 107.0f};

void ik(Command command, uint8_t leg_bitmap, float delta[NUM_LEGS][NUM_SERVOS_PER_LEG])
{
   float total_x;
   float total_y;
   float dist_body_foot;
   float theta;
   float roll_z;
   float pitch_z;
   float body_ikx;
   float body_iky;
   float body_ikz;

   float new_pos_x;
   float new_pos_y;
   float new_pos_z;
   float l;
   float hf;
   float a1;
   float a2_arg;
   float a2;
   float b1_arg;
   float b1;
   float alpha1;
   float alpha2;
   float gamma;
   
   float coxa_angle;
   float femur_angle;
   float tibia_angle;
   
   int leg;

   for (leg = 0; leg < NUM_LEGS; leg++)
   {
      if (!(leg_bitmap & (1 << leg)))
      {
         /* Leg not set in bitmap, no change */
         continue;
      }

      /* Body IK */
      total_x = INIT_POS_X[leg] + OFFSET_X[leg] + command.pos_x;
      total_y = INIT_POS_Y[leg] + OFFSET_Y[leg] + command.pos_y;
      dist_body_foot = sqrtf(total_x * total_x + total_y * total_y);
      theta = atan2f(total_y, total_x);
      roll_z = tanf(command.rot_y * PI / 180.0f) * total_x;
      pitch_z = tanf(command.rot_x * PI / 180.0f) * total_y;
      body_ikx = cosf(theta + (command.rot_z * PI / 180.0f)) *
         dist_body_foot - total_x;
      body_iky = sinf(theta + (command.rot_z * PI / 180.0f)) *
         dist_body_foot - total_y;
      body_ikz = roll_z + pitch_z;

      /* Leg IK */
      new_pos_x = INIT_POS_X[leg] + command.pos_x + body_ikx;
      new_pos_y = INIT_POS_Y[leg] + command.pos_y + body_iky;
      new_pos_z = INIT_POS_Z[leg] + command.pos_z + body_ikz;
      l = sqrtf(new_pos_x * new_pos_x + new_pos_y * new_pos_y);
      hf = sqrtf((l - COXA_LEN) * (l - COXA_LEN) + new_pos_z * new_pos_z);
      a1 = atan2f(l - COXA_LEN, new_pos_z) * 180.0f / PI;
      a2_arg = T_CL((FEMUR_LEN * FEMUR_LEN + hf * hf - TIBIA_LEN * TIBIA_LEN) /
            (2 * FEMUR_LEN * hf));
      a2 = acosf(a2_arg) * 180.0f / PI;
      b1_arg = T_CL((FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - hf * hf) /
            (2 * FEMUR_LEN * TIBIA_LEN));
      b1 = acosf(b1_arg) * 180.0f / PI;
      alpha1 = 90 - (a1 + a2);
      alpha2 = 90 - b1;
      gamma = atan2f(new_pos_y, new_pos_x) * 180.0f / PI;

      switch (leg)
      {
         case LEG_1:
            coxa_angle = gamma - 45;
            break;
         case LEG_2:
            coxa_angle = gamma;
            break;
         case LEG_3:
            coxa_angle = gamma + 45;
            break;
         case LEG_4:
            coxa_angle = gamma + 135;
            break;
         case LEG_5:
            coxa_angle = (gamma - 180 > -180) ? (gamma - 180) : (gamma + 180);
            break;
         case LEG_6:
            coxa_angle = gamma - 135;
            break;
      }
      femur_angle = alpha1;
      tibia_angle = alpha2;

      delta[leg][COXA] = coxa_angle;
      delta[leg][FEMUR] = femur_angle;
      delta[leg][TIBIA] = tibia_angle;
   }
}
