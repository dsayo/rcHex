/*******************************************************************************
 * stm32f3xx_it.h
 *
 * Interrupt service routines. 
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 23 Jan 2020
 *******************************************************************************
 */
#ifndef __STM32F3xx_IT_H
#define __STM32F3xx_IT_H

void SysTick_Handler(void);
void DMA1_Channel5_IRQHandler(void);
void TIM3_IRQHandler(void);
void USART1_IRQHandler(void);

#endif /* __STM32F3xx_IT_H */

