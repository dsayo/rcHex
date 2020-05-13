/*
 * gait.h
 *
 *  Created on: May 13, 2020
 *      Author: dsayo
 */
#ifndef INC_GAIT_H_
#define INC_GAIT_H_

#include "main.h"
#include "controls.h"

#define TRIPOD_STROKE_LEN 105  /* (mm) */
#define LEG_RAISED -30 /* (mm) */
#define LEG_GROUND 0   /* (mm) */

void ripple_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void tripod_phase(Phase phase, uint16_t servo_speed, float crawl_angle);


#endif /* INC_GAIT_H_ */
