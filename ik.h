#ifndef IK_H
#define IK_H

#define BODY_SIDE_LEN 85
#define BODY_WIDTH 125
#define COXA_LEN 27
#define FEMUR_LEN 75
#define TIBIA_LEN 107

#define OFFSET_X_1 42.500
#define OFFSET_Y_1 82.614

#define INIT_POS_X_1 72.125
#define INIT_POS_Y_1 72.125
#define INIT_POS_Z_1 107.000

typedef struct coord
{
   int16_t x;
   int16_t y;
   int16_t z;
} Coordinates;

typedef struct command
{
   int16_t pos_x; /* mm */
   int16_t pos_y;
   int16_t pos_z;
   int16_t rot_x; /* degrees */
   int16_t rot_y;
   int16_t rot_z;
} Command;

typedef struct leg_deltas
{
   int16_t coxa_delta;
   int16_t femur_delta;
   int16_t tibia_delta;
} Deltas;

#endif /* IK_H */
