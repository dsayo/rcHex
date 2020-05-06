#ifndef IK_H
#define IK_H

#include "main.h"
#include "controls.h"

#define NUM_LEGS 6
#define NUM_SERVOS_PER_LEG 3
#define BODY_SIDE_LEN 85
#define BODY_WIDTH 125
#define COXA_LEN 27
#define FEMUR_LEN 75
#define TIBIA_LEN 107

#define PI 3.14159265358979323846f

/* acosf() argument clamp for domain [-1, 1] */
#define T_CL(x)  (((x) > (1.0f)) ? (1.0f) : (((x) < (-1.0f)) ? (-1.0f) : (x)))

typedef struct coord
{
   int16_t x;
   int16_t y;
   int16_t z;
} Coordinates;

void ik(Command command, uint8_t leg_bitmap, float delta[NUM_LEGS][NUM_SERVOS_PER_LEG]);

#endif /* IK_H */
