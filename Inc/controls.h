/*
 * controls.h
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */

#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#define CH_ARM 4

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

void arm();
void disarm();
void init_stance();

#endif /* INC_CONTROLS_H_ */
