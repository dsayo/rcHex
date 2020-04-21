/*
 * ssc.h
 *
 *  Created on: Feb 9, 2020
 *      Author: dsayo
 */

#ifndef INC_SSC_H_
#define INC_SSC_H_

#include "main.h"
#include "controls.h"

#define PW_PER_DEGREE 9 /* ~9 us per degree */
#define MAX_PW 2000
#define CENTER_PW 1500
#define MIN_PW 1000

/* Max/min limiting function */
#define CL(x)  (((x) > (MAX_PW)) ? (MAX_PW) : (((x) < (MIN_PW)) ? (MIN_PW) : (x)))

#define MAX_ARG_SZ 5
#define NO_ARG 0

extern const uint8_t ssc_channel[NUM_LEGS][NUM_SERVO_PER_LEG];

void servo_move(uint8_t channel, int16_t angle, uint16_t speed);
void ssc_cmd_ch(uint8_t channel);
void ssc_cmd_pw(uint16_t pulse_width);
void ssc_cmd_spd(uint16_t speed);
void ssc_cmd_time(uint16_t time);
void ssc_cmd_cr();

#endif /* INC_SSC_H_ */
