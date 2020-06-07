/*******************************************************************************
 * main.c
 *
 * RC Hexapod
 *
 * California Polytechnic State University, San Luis Obispo
 * Dominique Sayo
 * 19 May 2020
 *******************************************************************************
 */
#include <string.h>
#include <math.h>
#include "main.h"
#include "sbus.h"
#include "ssc.h"
#include "term.h"
#include "controls.h"
#include "ik.h"

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;

volatile uint8_t ready = 0;       /* Flag: ready to process RX data    */
volatile uint8_t delta = 0;       /* Flag: change in RX data           */
volatile uint8_t phase_ready = 0; /* Flag: ready for next crawl phase  */
uint16_t seq_speed;               /* CCR subtractor for sequence speed */
Phase max_phase;                  /* Maximum phase in sequence cycle   */
float angle_delta[NUM_LEGS][NUM_SERVO_PER_LEG]; /* Servo degree changes */

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM3_Init(void);

int main(void)
{
   uint8_t packet[PACKET_SZ]; /* SBUS packet data            */
   RXData rx_data;            /* Formatted ctrl data         */
   RXData old_rx_data;        /* Previous ctrl data          */
   Command cmd;               /* Stationary command          */
   uint8_t armed = 0;         /* Flag: is armed              */
   Mode mode = MODE_RPY;      /* Movement mode               */
   CrawlMode cmod = TRIPOD;   /* Gait type / rotate          */
   Phase phase = A1;          /* Current phase in sequence   */
   float crawl_angle;         /* Crawl direction in radians  */
   uint16_t rot_dir;          /* Rotation direction +CW -CCW */

   /* Reset peripherals, Initializes the Flash interface and the Systick. */
   HAL_Init();

   /* Configure the system clock */
   SystemClock_Config();

   /* Initialize all configured peripherals */
   MX_GPIO_Init();
   MX_DMA_Init();
   MX_USART1_UART_Init();
   MX_USART2_UART_Init();
   MX_TIM3_Init();

   HAL_Delay(1000); /* One second startup */

   /* Start reading incoming RX data over UART */
   HAL_TIM_OC_Start_IT(&htim3, TIM_CHANNEL_1);
   __HAL_UART_FLUSH_DRREGISTER(&huart1);
   HAL_UART_Receive_DMA(&huart1, packet, PACKET_SZ);

   powerup_stance();    /* Fast stance on powerup       */
   neutral_stance();    /* Transition to neutral stance */

   while (1)
   {
      /* UART Error checking */
      if (HAL_UART_GetError(&huart1))
      {
         /* Overrun error, flush and restart */
         huart1.ErrorCode = HAL_UART_ERROR_NONE;
         __HAL_UART_FLUSH_DRREGISTER(&huart1);
         HAL_UART_Receive_DMA(&huart1, packet, PACKET_SZ);
      }

      /* Parse control data when ready */
      if (ready)
      {
         ready = 0;

         /* Save prev rx data and get new rx data*/
         memcpy(&old_rx_data, &rx_data, sizeof(RXData));
         sbus_format(packet, &rx_data);

         /* Get armed switch and operating mode */
         armed = get_arm(rx_data);
         mode = get_mode(rx_data);

         if (armed)
         {
            switch (mode)
            {
               case MODE_CRAWL:
                  /* Parse control data into gait and sequencer info */
                  cmod = get_cmod(rx_data);
                  seq_speed = get_speed(rx_data, cmod);
                  crawl_angle = get_angle(rx_data);
                  rot_dir = get_rot_dir(rx_data);
                  break;

               default: /* Stationary modes */
                  /* Check deltas (if rx data changed) */
                  delta = ctrl_delta(&old_rx_data, &rx_data);
                  seq_speed = 0; /* Don't use sequencer */
                  break;
            }
         }
      }

      /* Enable control if armed */
      if (armed)
      {
         switch (mode)
         {
            case MODE_CRAWL:
               /* When sequencer signals next phase */
               if (phase_ready)
               {
                  phase_ready = 0;

                  /* Execute phase movement */
                  exec_phase(phase, cmod, seq_speed, crawl_angle, rot_dir);
                  phase++;
                  if (phase > max_phase)
                  {
                     phase = A1;
                  }
               }
               break;

            default:
               /* Stationary mode */
               if (delta)
               {
                  /* Only run new calculations if rx data changed enough */
                  delta = 0;

                  /* Convert rx data to command & calculate inv. kinematics */
                  cmd = to_command(rx_data, mode);
                  ik(cmd, ALL_LEGS, angle_delta);
                  set_angles(ALL_LEGS, angle_delta, STATIONARY_SERVO_SPEED);
                  ssc_cmd_cr();   /* Send new servo pwm */
               }
               break;
         }
      }
      else
      {
         /* Stand still */
         neutral_stance();
      }

   }
}

/* System Clock Configuration
 */
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

   /* Initializes the CPU, AHB and APB busses clocks */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
   RCC_OscInitStruct.HSIState = RCC_HSI_ON;
   RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
   RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
   if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
   {
      Error_Handler();
   }
   /* Initializes the CPU, AHB and APB busses clocks */
   RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
      |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
   RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

   if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
   {
      Error_Handler();
   }
   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
   PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
   if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
   {
      Error_Handler();
   }
}

/* TIM3 Initialization Function
 */
static void MX_TIM3_Init(void)
{
   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_OC_InitTypeDef sConfigOC = {0};

   htim3.Instance = TIM3;
   htim3.Init.Prescaler = 92;
   htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim3.Init.Period = 0xFFFF;
   htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
   {
      Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
   {
      Error_Handler();
   }
   if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
   {
      Error_Handler();
   }
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
   {
      Error_Handler();
   }
   sConfigOC.OCMode = TIM_OCMODE_ACTIVE;
   sConfigOC.Pulse = 0;
   sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
   sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
   if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
   {
      Error_Handler();
   }
   
   HAL_TIM_MspPostInit(&htim3);
}

/* USART1 Initialization Function
 */
static void MX_USART1_UART_Init(void)
{
   huart1.Instance = USART1;
   huart1.Init.BaudRate = 100000;
   huart1.Init.WordLength = UART_WORDLENGTH_9B;
   huart1.Init.StopBits = UART_STOPBITS_2;
   huart1.Init.Parity = UART_PARITY_EVEN;
   huart1.Init.Mode = UART_MODE_RX;
   huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart1.Init.OverSampling = UART_OVERSAMPLING_16;
   huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXINVERT_INIT | 
      UART_ADVFEATURE_DMADISABLEONERROR_INIT;
   huart1.AdvancedInit.RxPinLevelInvert = UART_ADVFEATURE_RXINV_ENABLE;
   huart1.AdvancedInit.DMADisableonRxError =
      UART_ADVFEATURE_DMA_DISABLEONRXERROR;
   if (HAL_UART_Init(&huart1) != HAL_OK)
   {
      Error_Handler();
   }
}

/* USART2 Initialization Function
 */
static void MX_USART2_UART_Init(void)
{
   huart2.Instance = USART2;
   huart2.Init.BaudRate = 115200;
   huart2.Init.WordLength = UART_WORDLENGTH_8B;
   huart2.Init.StopBits = UART_STOPBITS_1;
   huart2.Init.Parity = UART_PARITY_NONE;
   huart2.Init.Mode = UART_MODE_TX;
   huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
   huart2.Init.OverSampling = UART_OVERSAMPLING_16;
   huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
   huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
   if (HAL_UART_Init(&huart2) != HAL_OK)
   {
      Error_Handler();
   }
}

/* Enable DMA controller clock
 */
static void MX_DMA_Init(void) 
{
   /* DMA controller clock enable */
   __HAL_RCC_DMA1_CLK_ENABLE();

   /* DMA interrupt init */
   /* DMA1_Channel5_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
   HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

}

/* GPIO Initialization Function
 */
static void MX_GPIO_Init(void)
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   /* GPIO Ports Clock Enable */
   __HAL_RCC_GPIOF_CLK_ENABLE();
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();

   /* Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOF, GPIO_PIN_0 | GPIO_PIN_1, GPIO_PIN_RESET);

   /* Configure GPIO pin Output Level */
   HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4 | GPIO_PIN_5, GPIO_PIN_RESET);

   /* Configure GPIO pins : PF0 PF1 */
   GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

   /* Configure GPIO pins : PB4 PB5 */
   GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
   HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/* Callback for complete UART receive
 */ 
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
   if (huart->Instance == USART1)
   {
      ready = 1;
   }
}

