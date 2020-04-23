#ifndef IK_H
#define IK_H

#define PW_PER_DEGREE 9
#define MAX_PW 2000
#define CENTER_PW 1500
#define MIN_PW 1000
#define CL(x)  (((x) > (MAX_PW)) ? (MAX_PW) : (((x) < (MIN_PW)) ? (MIN_PW) : (x)))
#define T_CL(x)  (((x) > (1.0f)) ? (1.0f) : (((x) < (-1.0f)) ? (-1.0f) : (x)))

#define NUM_LEGS 6
#define NUM_SERVOS_PER_LEG 3
#define BODY_SIDE_LEN 85
#define BODY_WIDTH 125
#define COXA_LEN 27
#define FEMUR_LEN 75
#define TIBIA_LEN 107

typedef enum
{
   LEG_1, LEG_2, LEG_3, LEG_4, LEG_5, LEG_6
} Leg;

typedef enum
{
   COXA, FEMUR, TIBIA
} Servo;

#define PI 3.14159265358979323846f

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

#endif /* IK_H */
