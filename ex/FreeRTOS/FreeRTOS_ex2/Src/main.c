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
#include "stdio.h"
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

// --> include all necessary headers for
// printf() redirection
// FreeRTOS related headers
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef struct {
    uint16_t measurement;
    uint32_t counter;
} queue_data_t;

QueueHandle_t mailbox;

enum QueueStatus {
	QueueOK, QueueWriteProblem, QueueEmpty, QueueCantRead
};

enum QueueMessages {
	QueueMsgNoData, QueueMsgNewData, QueueMsgNewDataChange,
};

uint16_t measurement;
uint8_t queueError = QueueOK;
SemaphoreHandle_t mutex;
QueueHandle_t queue;

void measureTask(void *args) {
	TickType_t xLastWakeTime;
	BaseType_t xStatus;

	xLastWakeTime = xTaskGetTickCount();

	for (;;) {
    BaseType_t xStatus = xQueueSendToBack(queue, &measurement, 0);

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (xStatus == pdPASS) {
            queueError = QueueOK;
        } else {
            queueError = QueueWriteProblem;
        }
        xSemaphoreGive(mutex);
    }

    vTaskDelayUntil(&xLastWakeTime, 300/portTICK_PERIOD_MS);
	}
}

void measureTask_1(void *args) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t internal_counter = 0;
    queue_data_t data_to_send;

    for (;;) {
        internal_counter++;
        
        data_to_send.measurement = measurement;
        data_to_send.counter = internal_counter;

        xQueueOverwrite(mailbox, &data_to_send);

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(1000));
    }
}

void commTask(void *args) {
	TickType_t xLastWakeTime;
  uint16_t measurement_local = 0;
  uint16_t flag_local;
  uint16_t histeresis = 500; // Initialize with the slow period (500ms)
  BaseType_t queue_size;
  BaseType_t xStatus;

  xLastWakeTime = xTaskGetTickCount();

  for (;;) {
    queue_size = uxQueueMessagesWaiting(queue);

    if (queue_size > 12) {
      histeresis = 100; // Switch to fast mode
    } else if (queue_size < 4) {
      histeresis = 500; // Switch to slow mode
    }
    // Note: if queue_size is between 4 and 12, 'histeresis' remains unchanged

    xStatus = xQueueReceive(queue, &measurement_local, 0);

    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (xStatus == pdPASS) {
        queueError = QueueOK;
      } else {
        if (queue_size == 0) {
          queueError = QueueEmpty;
        } else {
          queueError = QueueCantRead;
        }
      }
      flag_local = queueError;
      xSemaphoreGive(mutex);
    }

    printf("time: %5lu, measured value: %4u, queue size %2u, error %u\r\n",
           xTaskGetTickCount(), 
           measurement_local, 
           (unsigned int)queue_size, 
           flag_local);

    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(histeresis));
  }
}

void commTask_1(void *args) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    queue_data_t local_data;
    uint32_t last_seen_counter = 0;

    for (;;) {
        if (xQueuePeek(mailbox, &local_data, 0) == pdPASS) {
            
            if (local_data.counter > last_seen_counter) {
                printf("time: %5lu, measured value: %4u, counter: %3lu\r\n",
                       xTaskGetTickCount(), local_data.measurement, local_data.counter);
                last_seen_counter = local_data.counter;
            } else {
                printf("No new data\r\n");
            }
            
        } else {
            printf("Queue empty\r\n");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(400));
    }
}

int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(&huart2, (uint8_t *) ptr, len, 50);
    return len;
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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

	// --> start TIM1 to generate PWM signal on TIMER3 connector
	// --> start TIM6 in interrupt
	// --> start ADC1 in DMA mode
	// --> create a mutex
	// --> create a queue
	// --> create all necessary tasks
	printf("Starting!\r\n");

	// --> start FreeRTOS scheduler

  mutex = xSemaphoreCreateMutex();
  queue = xQueueCreate(15, sizeof(uint16_t));
  mailbox = xQueueCreate(1, sizeof(queue_data_t));

  // xTaskCreate(measureTask, "measure", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
  // xTaskCreate(commTask, "comm", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);

  xTaskCreate(measureTask_1, "measure", configMINIMAL_STACK_SIZE * 2, NULL, 2, NULL);
  xTaskCreate(commTask_1, "comm", configMINIMAL_STACK_SIZE * 2, NULL, 1, NULL);

  /* USER CODE END 2 */

  /* Init scheduler */

  /* Call init function for freertos objects (in cmsis_os2.c) */

  /* Start scheduler */
  

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_Base_Start_IT(&htim6);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&measurement, 1);

  vTaskStartScheduler();

	while (1) {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    printf("Measured value: %u, time: %lu\r\n", measurement, HAL_GetTick());

    HAL_Delay(1000);
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
