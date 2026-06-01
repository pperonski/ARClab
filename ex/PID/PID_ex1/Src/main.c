/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdio.h>

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// --> include all necessary headers for
// printf() redirection
// FreeRTOS related headers
#include "pid.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

QueueHandle_t mailbox;
QueueHandle_t mailbox_comm;
QueueHandle_t mailbox_setpoint;
QueueHandle_t queue;

static uint32_t measurement;
static uint8_t user_input;

struct {
    uint32_t measurement;
    uint32_t control;
    uint32_t setpoint;
} typedef control_info_t;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 50);
    return len;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

  xQueueSendFromISR(mailbox, &user_input, NULL);

  HAL_UART_Receive_IT(&huart2, (uint8_t *) &user_input, 1);
}

void measureTask(void *args) {
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

	for (;;) {

    xQueueSend(queue, &measurement, portMAX_DELAY);

    vTaskDelay(10/portTICK_PERIOD_MS);
	}
}

void controlTask(void *args) {
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

  uint32_t local_setpoint = 0;
  uint32_t local_measurement = 0;
  control_info_t local_control_info = {0};

	for (;;) {

    xQueuePeek(queue, &local_setpoint, portMAX_DELAY);
    if(xQueueReceive(queue, &local_measurement, portMAX_DELAY))
    {
      int32_t control_signal = pid(local_setpoint, local_measurement);

      if(control_signal > 1000) {
        control_signal = 1000;
      } else if(control_signal < 0) {
        control_signal = 0;
      }

      __HAL_TIM_SET_COMPARE(&htim6, TIM_CHANNEL_3, control_signal);

      local_control_info.measurement = local_measurement;
      local_control_info.setpoint = local_setpoint;
      local_control_info.control = control_signal;
      xQueueOverwrite(mailbox_comm, &local_control_info);
    }
	}
}

void commTask(void *args) {
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

  control_info_t local_control_info = {0};

	for (;;) {

    xQueuePeek(mailbox_comm, &local_control_info, 0);

    printf("time: %lu, measured value: %u, setpoint: %u, control signal: %u\r\n",
          xTaskGetTickCount(), 
          local_control_info.measurement, 
          local_control_info.setpoint, 
          local_control_info.control);

    vTaskDelay(500/portTICK_PERIOD_MS);
	}
}

void userTask(void *args) {
	TickType_t xLastWakeTime;

	xLastWakeTime = xTaskGetTickCount();

  uint8_t local_user_input;

	for (;;) {

    if(xQueueReceive(mailbox, &local_user_input, portMAX_DELAY))
    {
      int32_t local_setpoint = local_user_input - '0';

      local_setpoint = local_setpoint*500;

      xQueueOverwrite(mailbox_setpoint, &local_setpoint);
    }
   
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_DAC1_Init();
  /* USER CODE BEGIN 2 */

	// enable UART receive in interrupt mode

  queue = xQueueCreate(15, sizeof(uint32_t));
  mailbox = xQueueCreate(1, sizeof(int32_t));
  mailbox_comm = xQueueCreate(1, sizeof(control_info_t));
  mailbox_setpoint = xQueueCreate(1, sizeof(uint32_t));

  xTaskCreate(commTask, "comm", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  xTaskCreate(controlTask, "control", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  xTaskCreate(userTask, "user", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);
  xTaskCreate(measureTask, "measure", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);

  HAL_TIM_PWM_Start(&htim6, TIM_CHANNEL_3);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&measurement, 1);
  HAL_UART_Receive_IT(&huart2, (uint8_t *) &user_input, 1);
	// --> create all necessary synchronization mechanisms
	// --> create all necessary tasks
	printf("Starting!\r\n");

	// --> start FreeRTOS scheduler
	vTaskStartScheduler();

	// --> start FreeRTOS scheduler

  /* USER CODE END 2 */

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
