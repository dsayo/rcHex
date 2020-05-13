#ifndef INC_CONTROLS_H_
#define INC_CONTROLS_H_

#include "main.h"
#include "sbus.h"


#define NUM_LEGS 6

typedef enum
{
   LEG_1, LEG_2, LEG_3, LEG_4, LEG_5, LEG_6
} Leg;

/* Leg bitmap */
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

#define NUM_SERVO_PER_LEG 3

typedef enum
{
   COXA, FEMUR, TIBIA
} Servo;

#define CHAN_Z 0
#define CHAN_ROLL 1
#define CHAN_PITCH 2
#define CHAN_YAW 3
#define CHAN_ARM 4
#define CHAN_MODE 5
#define CHAN_CMOD 6

#define MAX_CHAN_USED 6

typedef enum
{
   MODE_XY, MODE_CRAWL, MODE_RPY
} Mode;

typedef enum
{
   TRIPOD, RIPPLE, WAVE
} CrawlMode;

typedef enum
{
   A1, A2, A3,
   B1, B2, B3,
   C1, C2, C3,
   D1, D2, D3,
   E1, E2, E3,
   F1, F2, F3
} Phase;


#define RXDATA_SCALAR_MM 16
#define RXDATA_SCALAR_DEG 64
#define SPEED_SCALAR 64

typedef struct command
{
	int16_t pos_x; /* mm */
	int16_t pos_y;
	int16_t pos_z;
	int16_t rot_x; /* degrees */
	int16_t rot_y;
	int16_t rot_z;
} Command;

void init_stance();
uint8_t get_arm(RXData rx_data);
Mode get_mode(RXData rx_data);
CrawlMode get_cmod(RXData rx_data);
uint16_t get_speed(RXData rx_data);
float get_angle(RXData rx_data);
uint8_t ctrl_delta(RXData *old, RXData *new);
Command to_command(RXData rxdata, Mode mode);
void set_angles(uint8_t leg_bitmap, float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG],
      uint16_t speed);
void exec_phase(Phase phase, CrawlMode cmod, uint16_t seq_speed, float crawl_angle);

#endif /* INC_CONTROLS_H_ */
