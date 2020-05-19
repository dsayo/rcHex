/*
 * ssc.c
 *
 *  Created on: Feb 9, 2020
 *      Author: dsayo
 */
#include "ssc.h"

extern UART_HandleTypeDef huart2;

uint8_t buf[MAX_ARG_SZ];

/* SSC channel mapping */
const uint8_t ssc_channel[NUM_LEGS][NUM_SERVO_PER_LEG] =
{
	{31, 29, 27}, /* LEG_1: Right front  */
	{25, 23, 21}, /*     2: Right middle */
	{19, 17, 16}, /*     3: Right back   */
	{ 3,  1,  0}, /*     4: Left back    */
	{ 9,  7,  5}, /*     5: Left middle  */
	{15, 13, 11}  /*     6:Left front   */
};

/* Convert integer to split UART decimal numbers in a buffer.
 * Returns number of characters to read in buffer.
 */
uint8_t _itoa(uint8_t buf[MAX_ARG_SZ], uint16_t num)
{
	int8_t i, j;
	uint16_t n = num;
	uint16_t rem;
	uint8_t cnt = 0;
	uint8_t rev_buf[MAX_ARG_SZ];

	/* Clear buffer */
	for(i = 0; i < MAX_ARG_SZ; i++){
		rev_buf[i] = '0';
	}

	i = 0;
	/* Write decimal values to buffer */
	while(n != 0){
		rem = n % 10;
		rev_buf[i++] = rem + '0';
		cnt++;
		n /= 10;
	}

	/* Zero case */
	if (cnt == 0)
	{
		buf[0] = '0';
		return 1;
	}

	/* Reverse buffer */
	for (i = 0, j = (cnt - 1); j >= 0; j--)
	{
		buf[i++] = rev_buf[j];
	}

	return cnt;
}

/* Send command to ssc32 buffer to move a servo. Need to call ssc_cmd_cr() to
 * execute buffered commands.
 */
void servo_move(uint8_t channel, uint16_t pulse_width, uint16_t speed, uint16_t time)
{
   ssc_cmd_ch(channel);
	ssc_cmd_pw(pulse_width);
	if (speed)
	{
		ssc_cmd_spd(speed);
	}
	if (time)
	{
	   ssc_cmd_time(time);
	}
}

/* Send channel argument to ssc32.
 * Format: #<ch>
 *    Pin/channel to which the servo is connected (0 to 31) in decimal.
 */
void ssc_cmd_ch(uint8_t channel)
{
	uint8_t hdr = '#';
	uint8_t cnt;

	cnt = _itoa(buf, (uint16_t)channel);

	HAL_UART_Transmit(&huart2, &hdr, 1, 10);
	HAL_UART_Transmit(&huart2, buf, cnt, 10);
}

/* Send pulse width argument to ssc32.
 * Format: P<pw>
 *    Desired pulse width (500 to 2500) in microseconds.
 */
void ssc_cmd_pw(uint16_t pulse_width)
{
	uint8_t hdr = 'P';
	uint8_t cnt;

	cnt = _itoa(buf, pulse_width);

	HAL_UART_Transmit(&huart2, &hdr, 1, 10);
	HAL_UART_Transmit(&huart2, buf, cnt, 10);
}

/* Send servo movement speed argument to ssc32. [Optional]
 * Format: S<spd>
 *    Servo movement speed in microseconds per second.
 *    Ex. A speed value of 100 means the servo will take 10 seconds to
 *    move 90 degrees.
 *    100us/s::10s/90 degrees.
 *    2000us/s::0.5s/90degrees.
 *    Speed is physically limited by hardware.
 */
void ssc_cmd_spd(uint16_t speed)
{
	uint8_t hdr = 'S';
	uint8_t cnt;

	cnt = _itoa(buf, speed);

	HAL_UART_Transmit(&huart2, &hdr, 1, 10);
	HAL_UART_Transmit(&huart2, buf, cnt, 10);
}

/* Send time argument to ssc32. [Optional]
 * Format: T<time>
 *    Time in microseconds to travel from the current position to the
 *    desired position. This affects all servos (65535 us maximum).
 */
void ssc_cmd_time(uint16_t time)
{
	uint8_t hdr = 'T';
	uint8_t cnt;

	cnt = _itoa(buf, time);

	HAL_UART_Transmit(&huart2, &hdr, 1, 10);
	HAL_UART_Transmit(&huart2, buf, cnt, 10);
}

/* Send end of argument character to ssc32.
 * Format: <cr>
 *     Carriage return (ASCII 13). Ends and begins execution of the command
 *     group.
 */
void ssc_cmd_cr()
{
	uint8_t cr = '\r';

	HAL_UART_Transmit(&huart2, &cr, 1, 10);
}
