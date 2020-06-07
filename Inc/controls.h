/*******************************************************************************
 * controls.h
 *
 * General control subroutines and functions for the hexapod.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 11 Feb 2020
 *******************************************************************************
 */
#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#include "main.h"
#include "sbus.h"

#define NUM_LEGS 6
#define NUM_SERVO_PER_LEG 3

/* Leg macros for iteration and indexing.
 *               <front>
 *                  /\
 *        LEG_6 .--|  |--. LEG_1
 *<left>  LEG_5 .--|  |--. LEG_2   <right>
 *        LEG_4 .--|__|--. LEG_3
 *
 *                <back>
 */
typedef enum
{
   LEG_1, LEG_2, LEG_3, LEG_4, LEG_5, LEG_6
} Leg;

/* Leg bitmap macros for subgrouping */
typedef enum
{
   L1 = 0x01,
   L2 = 0x02,
   L3 = 0x04,
   L4 = 0x08,
   L5 = 0x10,
   L6 = 0x20
} LegBit;

#define ALL_LEGS (L1 | L2 | L3 | L4 | L5 | L6)

/* Servo macros for iteration and indexing */
typedef enum
{
   COXA, FEMUR, TIBIA
} Servo;

/* TX/RX Channel mapping */
#define CHAN_Z 0
#define CHAN_ROLL 1
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_ARM 4
#define CHAN_MODE 5
#define CHAN_CMOD 6
#define CHAN_ROT 7

#define MAX_CHAN_USED 7

/* Operation modes */
typedef enum
{
   MODE_XY,   /* Stationary: X-Y mode            */
   MODE_RPY,  /* Stationary: Roll-Pitch-Yaw mode */
   MODE_CRAWL /* Moving (utilizes sequencer): crawling or rotating */
} Mode;

/* Gait types */
typedef enum
{
   TRIPOD,  /* 2-phase */
   RIPPLE,  /* 3-phase */
   WAVE,    /* 6-phase */
   ROTATE   /* 2-phase (like tripod but with yaw) */
} CrawlMode;

/* Phases for sequencer state machine */
typedef enum
{
   A1, A2, A3, /* Split into 3 subphases for return stroke */
   B1, B2, B3,
   C1, C2, C3,
   D1, D2, D3,
   E1, E2, E3,
   F1, F2, F3
} Phase;

#define DELTA_THRESH 20 /* rx data threshold to detect a control change */

#define RXDATA_SCALAR_MM 16  /* rx data to mm scalar     */
#define RXDATA_SCALAR_DEG 64 /* rx data to degree scalar */
#define SPEED_SCALAR 64      /* rx data to sequence speed scalar */

#define STATIONARY_SERVO_SPEED 1500  /* Servo speed in stationary mode */
#define NEUTRAL_SERVO_SPEED 500      /* Servo speed to neutral stance  */

/* Piecewise functions tuned for smooth servo command transitions */
#define SPEED_FX_1(x) (5 * (x) / 4 + 50)
#define SPEED_PIECEWISE_THRESH 600
#define SPEED_FX_2(x) (4 * (x) - 1600)

/* Inverse kinematics command format
 * 3D axes from robot's POV:
 * x-axis: to the left
 * y-axis: forwards
 * z-axis: downwards
 */
typedef struct command
{
   int16_t pos_x; /* mm */
   int16_t pos_y; /* mm */
   int16_t pos_z; /* mm */
   int16_t rot_x; /* degrees */
   int16_t rot_y; /* degrees */
   int16_t rot_z; /* degrees */
} Command;

void powerup_stance();
void neutral_stance();
uint8_t get_arm(RXData rx_data);
Mode get_mode(RXData rx_data);
CrawlMode get_cmod(RXData rx_data);
uint16_t get_speed(RXData rx_data, CrawlMode cmod);
float get_angle(RXData rx_data);
int16_t get_rot_dir(RXData rx_data);
uint8_t ctrl_delta(RXData *old, RXData *new);
Command to_command(RXData rxdata, Mode mode);
void set_angles(uint8_t leg_bitmap,
      float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG], uint16_t speed);
void exec_phase(Phase phase, CrawlMode cmod, uint16_t seq_speed,
      float crawl_angle, int16_t rot_dir);

#endif /* INC_CONTROLS_H_ */
