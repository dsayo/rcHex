/*******************************************************************************
 * main.h
 *
 * RC Hexapod
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 19 May 2020
 *******************************************************************************
 */
#ifndef __MAIN_H
#define __MAIN_H

#include "stm32f3xx_hal.h"

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
void Error_Handler(void);

#endif /* __MAIN_H */

