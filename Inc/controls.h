/*
 * controls.h
 *
 *  Created on: Feb 11, 2020
 *      Author: dsayo
 */

#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#define CH_ARM rx_data.channels[4]

#define RT_FRT_COXA  27
#define RT_FRT_FEMUR 29
#define RT_FRT_TIBIA 31

#define RT_MID_COXA  21
#define RT_MID_FEMUR 23
#define RT_MID_TIBIA 25

#define RT_BCK_COXA  16
#define RT_BCK_FEMUR 17
#define RT_BCK_TIBIA 19

#define LT_FRT_COXA  11
#define LT_FRT_FEMUR 13
#define LT_FRT_TIBIA 15

#define LT_MID_COXA  5
#define LT_MID_FEMUR 7
#define LT_MID_TIBIA 9

#define LT_BCK_COXA  0
#define LT_BCK_FEMUR 1
#define LT_BCK_TIBIA 3

void arm();
void disarm();
void init_stance();

#endif /* INC_CONTROLS_H_ */
