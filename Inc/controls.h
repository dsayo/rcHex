#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#include "main.h"
#include "sbus.h"

#define CHAN_Z 0
#define CHAN_ROLL 1
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_ARM 4

#define MAX_CHAN_USED 4

#define RXDATA_SCALE_MM 32
#define RXDATA_SCALE_DEG 64

#define NUM_LEGS 6

typedef enum
{
	LEG_1, LEG_2, LEG_3, LEG_4, LEG_5, LEG_6
} Leg;

#define NUM_SERVO_PER_LEG 3

typedef enum
{
	COXA, FEMUR, TIBIA
} Servo;

typedef struct command
{
	int16_t pos_x; /* mm */
	int16_t pos_y;
	int16_t pos_z;
	int16_t rot_x; /* degrees */
	int16_t rot_y;
	int16_t rot_z;
} Command;

void arm();
void disarm();
void init_stance();
uint8_t ctrl_delta(RXData *old, RXData *new); /* */
Command to_command(RXData rxdata);
void set_all_angles(float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG]);

#endif /* INC_CONTROLS_H_ */
