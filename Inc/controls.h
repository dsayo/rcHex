/*
 * controls.h
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */

#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#define CH_ARM 4

#define RT_FRT_COXA  31
#define RT_FRT_FEMUR 29
#define RT_FRT_TIBIA 27

#define RT_MID_COXA  25
#define RT_MID_FEMUR 23
#define RT_MID_TIBIA 21

#define RT_BCK_COXA  19
#define RT_BCK_FEMUR 17
#define RT_BCK_TIBIA 16

#define LT_FRT_COXA  15
#define LT_FRT_FEMUR 13
#define LT_FRT_TIBIA 11

#define LT_MID_COXA  9
#define LT_MID_FEMUR 7
#define LT_MID_TIBIA 5

#define LT_BCK_COXA  3
#define LT_BCK_FEMUR 1
#define LT_BCK_TIBIA 0

void arm();
void disarm();
void init_stance();

#endif /* INC_CONTROLS_H_ */
