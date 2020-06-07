/*******************************************************************************
 * term.h
 *
 * Terminal code to test transmit/receive SBUS parsing.
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 24 Jan 2020
 *******************************************************************************
 */
#ifndef INC_TERM_H_
#define INC_TERM_H_

#include "sbus.h"

#define ESC 0x1B
#define BAR_SYMBOL '|'

void init_term(UART_HandleTypeDef *huart);
void print_channels(RXData data);

#endif /* INC_TERM_H_ */

