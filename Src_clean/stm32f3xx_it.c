/*******************************************************************************
 * stm32f3xx_it.c
 *
 * Interrupt service routines. 
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 23 Jan 2020
 *******************************************************************************
 */
#include "main.h"
#include "stm32f3xx_it.h"
#include "controls.h"

extern TIM_HandleTypeDef htim3;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern uint16_t seq_speed;
extern uint8_t phase_ready;

/* Handle System tick timer.
 */
void SysTick_Handler(void)
{
   HAL_IncTick();
}

/* Handle DMA1 channel5 global interrupt.
 */
void DMA1_Channel5_IRQHandler(void)
{
   HAL_DMA_IRQHandler(&hdma_usart1_rx);
}

/* Handle TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
   HAL_TIM_IRQHandler(&htim3);
   if (seq_speed)
   {
      /* Go to next sequence step */
      if (!phase_ready) /* Don't skip a phase if too fast */
      {
         phase_ready = 1;
      }
   }
   TIM3->CCR1 += (0xFFFF - seq_speed);  /* Change sequence speed */
}

/* Handle USART1 global interrupt.
 */
void USART1_IRQHandler(void)
{
   HAL_UART_IRQHandler(&huart1);
}

