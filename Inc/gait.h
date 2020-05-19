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

#define STROKE_LEN 100  /* (mm) */
#define LEG_RAISED -40 /* (mm) */
#define LEG_GROUND 0   /* (mm) */
#define ROTATION_ANGLE 8; /* (degrees) */

void ripple_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void tripod_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void wave_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void rotate_phase(Phase phase, uint16_t servo_speed, int16_t rot_dir);

#endif /* INC_GAIT_H_ */
