/*******************************************************************************
 * ik.h
 *
 * Inverse kinematics calculations.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 23 Apr 2020
 *******************************************************************************
 */
#ifndef IK_H
#define IK_H

#include "main.h"
#include "controls.h"

/* Physical specifications */
#define BODY_SIDE_LEN 85 /* mm */
#define BODY_WIDTH 125   /* mm */

/* Measured between servo rotation points */
#define COXA_LEN 27      /* mm */
#define FEMUR_LEN 75     /* mm */
#define TIBIA_LEN 107    /* mm */

#define PI 3.14159265358979323846f  /* yum */

/* acosf() argument clamp for domain [-1, 1] */
#define T_CL(x)  (((x) > (1.0f)) ? (1.0f) : (((x) < (-1.0f)) ? (-1.0f) : (x)))

void ik(Command command, uint8_t leg_bitmap,
      float delta[NUM_LEGS][NUM_SERVO_PER_LEG]);

#endif /* IK_H */
