#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ik.h"

float angle_delta[NUM_LEGS][NUM_SERVOS_PER_LEG];

void ik(Command command, float delta[NUM_LEGS][NUM_SERVOS_PER_LEG])
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
      /* Body IK */
      total_x = INIT_POS_X[leg] + OFFSET_X[leg] + command.pos_x;
      total_y = INIT_POS_Y[leg] + OFFSET_Y[leg] + command.pos_y;
      dist_body_foot = sqrtf(total_x * total_x + total_y * total_y);
      theta = atan2f(total_y, total_x);
      roll_z = tanf(command.rot_y * PI / 180) * total_x;
      pitch_z = tanf(command.rot_x * PI / 180) * total_y;
      body_ikx = cosf(theta + (command.rot_z * PI / 180)) * 
         dist_body_foot - total_x;
      body_iky = sinf(theta + (command.rot_z * PI / 180)) *
         dist_body_foot - total_y;
      body_ikz = roll_z + pitch_z;

      /* Leg IK */
      new_pos_x = INIT_POS_X[leg] + command.pos_x + body_ikx;
      new_pos_y = INIT_POS_Y[leg] + command.pos_y + body_iky;
      new_pos_z = INIT_POS_Z[leg] + command.pos_z + body_ikz;
      l = sqrtf(new_pos_x * new_pos_x + new_pos_y * new_pos_y);
      hf = sqrtf((l - COXA_LEN) * (l - COXA_LEN) + new_pos_z * new_pos_z);
      a1 = atan2f(l - COXA_LEN, new_pos_z) * 180 / PI;
      a2_arg = T_CL((FEMUR_LEN * FEMUR_LEN + hf * hf - TIBIA_LEN * TIBIA_LEN) /
            (2 * FEMUR_LEN * hf));
      a2 = acosf(a2_arg) * 180 / PI;
      b1_arg = T_CL((FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - hf * hf) / 
            (2 * FEMUR_LEN * TIBIA_LEN));
      b1 = acosf(b1_arg) * 180 / PI;
      alpha1 = (a1 + a2) - 90;
      alpha2 = 90 - b1;
      gamma = atan2f(new_pos_y, new_pos_x) * 180 / PI;

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

int main(int argc, char *argv[])
{
   int leg, servo;

   Command command;
   command.pos_x = 0;
   command.pos_y = 0;
   command.pos_z = 0;
   command.rot_x = 7;
   command.rot_y = 0;
   command.rot_z = 0;

   ik(command, angle_delta);

   uint32_t pw;

   for (leg = 0; leg < NUM_LEGS; leg++)
   {
      printf("Leg %d\n-----\n", leg+1);
      for (servo = 0; servo < NUM_SERVOS_PER_LEG; servo++)
      {
         switch (servo)
         {
            case COXA:
               pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
               break;

            case FEMUR:
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
               break;

            case TIBIA:
               switch(leg)
               {
                  case LEG_1:
                  case LEG_2:
                  case LEG_3:
                     pw = CL(CENTER_PW - PW_PER_DEGREE * angle_delta[leg][servo]);
                     break;

                  default:  /* Mirror for left legs */
                     pw = CL(CENTER_PW + PW_PER_DEGREE * angle_delta[leg][servo]);
                     break;
               }
               break;
         }
         printf("%7f ", angle_delta[leg][servo]);
         printf("%d \n", pw);
      }
      printf("\n");
   }

   return 0;
}
