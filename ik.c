#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "ik.h"


void ik(Command command)
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

   /* Body IK */
   total_x = INIT_POS_X_1 + OFFSET_X_1 + command.pos_x;
   total_y = INIT_POS_Y_1 + OFFSET_Y_1 + command.pos_y;
   dist_body_foot = sqrt(total_x * total_x + total_y * total_y);
   theta = atan2(total_y, total_x);
   roll_z = tan(command.rot_y * M_PI / 180) * total_x;
   pitch_z = tan(command.rot_x * M_PI / 180) * total_y;
   body_ikx = cos(theta + (command.rot_z * M_PI / 180)) * 
              dist_body_foot - total_x;
   body_iky = sin(theta + (command.rot_z * M_PI / 180)) *
              dist_body_foot - total_y;
   body_ikz = roll_z + pitch_z;
  
   printf("%f\n", dist_body_foot); 
   printf("%f\n", theta); 
   printf("%f\n", roll_z); 
   printf("%f\n", pitch_z); 
   printf("%f\n", body_ikx); 
   printf("%f\n", body_iky); 
   printf("%f\n", body_ikz); 


   float new_pos_x;
   float new_pos_y;
   float new_pos_z;
   float l;
   float hf;
   float a1;
   float a2;
   float b1;
   float alpha1;
   float alpha2;
   float gamma;

   /* Leg IK */
   new_pos_x = INIT_POS_X_1 + command.pos_x + body_ikx;
   new_pos_y = INIT_POS_Y_1 + command.pos_y + body_iky;
   new_pos_z = INIT_POS_Z_1 + command.pos_z + body_ikz;
   l = sqrt(new_pos_x * new_pos_x + new_pos_y * new_pos_y);
   hf = sqrt((l - COXA_LEN) * (l - COXA_LEN) + new_pos_z * new_pos_z);
   a1 = atan2(l - COXA_LEN, new_pos_z) * 180 / M_PI;
   a2 = acos((FEMUR_LEN * FEMUR_LEN + hf * hf - TIBIA_LEN * TIBIA_LEN) /
             (2 * FEMUR_LEN * hf)) * 180 / M_PI;
   b1 = acos((FEMUR_LEN * FEMUR_LEN + TIBIA_LEN * TIBIA_LEN - hf * hf) / 
             (2 * FEMUR_LEN * TIBIA_LEN)) * 180 / M_PI;
   alpha1 = 90 - (a1 + a2);
   alpha2 = 90 - b1;
   gamma = atan2(new_pos_y, new_pos_x) * 180 / M_PI;

   printf("\n%f\n", new_pos_x);
   printf("%f\n", new_pos_y);
   printf("%f\n", new_pos_z);
   printf("%f\n", l);
   printf("%f\n", hf);
   printf("%f\n", a1);
   printf("%f\n", a2);
   printf("%f\n", b1);
   printf("%f\n", alpha1);
   printf("%f\n", alpha2);
   printf("%f\n", gamma);

   float coxa_angle;
   float femur_angle;
   float tibia_angle;

   coxa_angle = gamma - 45;
   femur_angle = alpha1;
   tibia_angle = alpha2;

   printf("\nRF_COXA: %d\n", (int)coxa_angle + 90);
   printf("RF_FEMUR: %d\n", (int)femur_angle + 90);
   printf("RF_TIBIA: %d\n", (int)tibia_angle + 90);
}

int main(int argc, char *argv[])
{
   Command command;
   command.pos_x = 4;
   command.pos_y = 2;
   command.pos_z = 15;
   command.rot_x = 6;
   command.rot_y = 8;
   command.rot_z = -2;

   ik(command);

   return 0;
}
