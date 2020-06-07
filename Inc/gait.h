/*******************************************************************************
 * gait.h
 *
 * Gait coordination for the movement modes.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 13 May 2020
 *******************************************************************************
 */
#ifndef INC_GAIT_H_
#define INC_GAIT_H_

#include "main.h"
#include "controls.h"

#define STROKE_LEN 100  /* (mm) */
#define LEG_RAISED -40 /* (mm) */
#define LEG_GROUND 0   /* (mm) */
#define ROTATION_ANGLE 8; /* (degrees) */

/* Initial leg subphase offsets for wave gait:
 * 1-15 for power stroke phases, 16-18 for return stroke phases */
#define WAVE_INIT_PHASES {16, 10, 4, 1, 7, 13}
#define WAVE_RETURN_1 16
#define WAVE_RETURN_2 17
#define WAVE_RETURN_3 18

void ripple_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void tripod_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void wave_phase(Phase phase, uint16_t servo_speed, float crawl_angle);
void rotate_phase(Phase phase, uint16_t servo_speed, int16_t rot_dir);

#endif /* INC_GAIT_H_ */

