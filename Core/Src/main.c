/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include"queue.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* Definitions for TaskBlinkShort */
osThreadId_t TaskBlinkShortHandle;
uint32_t TaskBlinkShortBuffer[ 128 ];
osStaticThreadDef_t TaskBlinkShortControlBlock;
const osThreadAttr_t TaskBlinkShort_attributes = {
  .name = "TaskBlinkShort",
  .cb_mem = &TaskBlinkShortControlBlock,
  .cb_size = sizeof(TaskBlinkShortControlBlock),
  .stack_mem = &TaskBlinkShortBuffer[0],
  .stack_size = sizeof(TaskBlinkShortBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for TaskBlinkLong */
osThreadId_t TaskBlinkLongHandle;
uint32_t TaskBlinkLongBuffer[ 128 ];
osStaticThreadDef_t TaskBlinkLongControlBlock;
const osThreadAttr_t TaskBlinkLong_attributes = {
  .name = "TaskBlinkLong",
  .cb_mem = &TaskBlinkLongControlBlock,
  .cb_size = sizeof(TaskBlinkLongControlBlock),
  .stack_mem = &TaskBlinkLongBuffer[0],
  .stack_size = sizeof(TaskBlinkLongBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal1,
};
/* Definitions for TaskRealBlink */
osThreadId_t TaskRealBlinkHandle;
uint32_t TaskRealBlinkBuffer[ 128 ];
osStaticThreadDef_t TaskRealBlinkControlBlock;
const osThreadAttr_t TaskRealBlink_attributes = {
  .name = "TaskRealBlink",
  .cb_mem = &TaskRealBlinkControlBlock,
  .cb_size = sizeof(TaskRealBlinkControlBlock),
  .stack_mem = &TaskRealBlinkBuffer[0],
  .stack_size = sizeof(TaskRealBlinkBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for TaskHandlerFSM */
osThreadId_t TaskHandlerFSMHandle;
uint32_t TaskHandlerFSMBuffer[ 128 ];
osStaticThreadDef_t TaskHandlerFSMControlBlock;
const osThreadAttr_t TaskHandlerFSM_attributes = {
  .name = "TaskHandlerFSM",
  .cb_mem = &TaskHandlerFSMControlBlock,
  .cb_size = sizeof(TaskHandlerFSMControlBlock),
  .stack_mem = &TaskHandlerFSMBuffer[0],
  .stack_size = sizeof(TaskHandlerFSMBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for TaskDelayBlink */
osThreadId_t TaskDelayBlinkHandle;
uint32_t TaskDelayBlinkBuffer[ 128 ];
osStaticThreadDef_t TaskDelayBlinkControlBlock;
const osThreadAttr_t TaskDelayBlink_attributes = {
  .name = "TaskDelayBlink",
  .cb_mem = &TaskDelayBlinkControlBlock,
  .cb_size = sizeof(TaskDelayBlinkControlBlock),
  .stack_mem = &TaskDelayBlinkBuffer[0],
  .stack_size = sizeof(TaskDelayBlinkBuffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for stateQueue */
osMessageQueueId_t stateQueueHandle;
uint8_t stateQueueBuffer[ 16 * sizeof( uint16_t ) ];
osStaticMessageQDef_t stateQueueControlBlock;
const osMessageQueueAttr_t stateQueue_attributes = {
  .name = "stateQueue",
  .cb_mem = &stateQueueControlBlock,
  .cb_size = sizeof(stateQueueControlBlock),
  .mq_mem = &stateQueueBuffer,
  .mq_size = sizeof(stateQueueBuffer)
};
/* USER CODE BEGIN PV */
// Определение перечисления для состояний конечного автомата
typedef enum {
    STATE_1,
    STATE_2,
    STATE_3
} State_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void fTaskBlinkShort(void *argument);
void fTaskBlinkLong(void *argument);
void fTaskRealBlink(void *argument);
void fTaskHandlerFSM(void *argument);
void fTaskDelayBlink(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of stateQueue */
  stateQueueHandle = osMessageQueueNew (16, sizeof(uint16_t), &stateQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of TaskBlinkShort */
  TaskBlinkShortHandle = osThreadNew(fTaskBlinkShort, NULL, &TaskBlinkShort_attributes);

  /* creation of TaskBlinkLong */
  TaskBlinkLongHandle = osThreadNew(fTaskBlinkLong, NULL, &TaskBlinkLong_attributes);

  /* creation of TaskRealBlink */
  TaskRealBlinkHandle = osThreadNew(fTaskRealBlink, NULL, &TaskRealBlink_attributes);

  /* creation of TaskHandlerFSM */
  TaskHandlerFSMHandle = osThreadNew(fTaskHandlerFSM, NULL, &TaskHandlerFSM_attributes);

  /* creation of TaskDelayBlink */
  TaskDelayBlinkHandle = osThreadNew(fTaskDelayBlink, NULL, &TaskDelayBlink_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* Удаление TaskBlinkShort */
  osThreadTerminate(TaskBlinkShortHandle);

  /* Удаление TaskBlinkLong */
  osThreadTerminate(TaskBlinkLongHandle);

  /* Удаление TaskRealBlink */
  osThreadTerminate(TaskRealBlinkHandle);

  /* Удаление TaskDelayBlink */
  osThreadTerminate(TaskDelayBlinkHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
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
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_fTaskBlinkShort */
/**
  * @brief  Function implementing the TaskBlinkShort thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_fTaskBlinkShort */
void fTaskBlinkShort(void *argument)
{
  /* USER CODE BEGIN 5 */
	const uint32_t xFrequency = 500; // 500 миллисекунд
	    const TickType_t xTransitionTime = pdMS_TO_TICKS(6000); // 6 секунд

	    TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Выполнение действий для состояния 1
	         osDelay(xFrequency); // Задержка в тиках времени FreeRTOS
	         if ((xTaskGetTickCount() - xLastWakeTime) >= xTransitionTime) {
	             // Отправка сообщения о переходе в состояние 1
	        	 State_t currentState = STATE_2;
	             xQueueSend(stateQueueHandle, &currentState, portMAX_DELAY);
	             vTaskDelete(NULL); // Завершение задачи
	         }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_fTaskBlinkLong */
/**
* @brief Function implementing the TaskBlinkLong thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskBlinkLong */
void fTaskBlinkLong(void *argument)
{
  /* USER CODE BEGIN fTaskBlinkLong */
	const uint32_t xFrequency = 1000; // 500 миллисекунд
	    const TickType_t xTransitionTime = pdMS_TO_TICKS(6000); // 6 секунд

	    TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Выполнение действий для состояния 1
	         osDelay(xFrequency); // Задержка в тиках времени FreeRTOS
	         if ((xTaskGetTickCount() - xLastWakeTime) >= xTransitionTime) {
	             // Отправка сообщения о переходе в состояние 1
	        	 State_t currentState = STATE_1;
	        	 	             xQueueSend(stateQueueHandle, &currentState, portMAX_DELAY);
	             vTaskDelete(NULL); // Завершение задачи
	         }
  }
  /* USER CODE END fTaskBlinkLong */
}

/* USER CODE BEGIN Header_fTaskRealBlink */
/**
* @brief Function implementing the TaskRealBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskRealBlink */
void fTaskRealBlink(void *argument)
{
  /* USER CODE BEGIN fTaskRealBlink */
	const uint32_t xFrequency = 100; // 500 миллисекунд
	    const TickType_t xTransitionTime = pdMS_TO_TICKS(2000); // 6 секунд

	    TickType_t xLastWakeTime = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13); // Выполнение действий для состояния 1
	         osDelay(xFrequency); // Задержка в тиках времени FreeRTOS
	         if ((xTaskGetTickCount() - xLastWakeTime) >= xTransitionTime) {
	             // Отправка сообщения о переходе в состояние 1
	            // xQueueSend(stateQueueHandle, &STATE_1, portMAX_DELAY);
	             vTaskDelete(NULL); // Завершение задачи
	         }
  }
  /* USER CODE END fTaskRealBlink */
}

/* USER CODE BEGIN Header_fTaskHandlerFSM */
/**
* @brief Function implementing the TaskHandlerFSM thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskHandlerFSM */
void fTaskHandlerFSM(void *argument)
{
  /* USER CODE BEGIN fTaskHandlerFSM */
	TaskDelayBlinkHandle = osThreadNew(fTaskDelayBlink, NULL, &TaskDelayBlink_attributes);
	TaskBlinkShortHandle = osThreadNew(fTaskBlinkShort, NULL, &TaskBlinkShort_attributes);
	State_t currentState = STATE_1;
	//xTaskCreate(fTaskDelayBlink, "StateDelayBlink", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY + 1, NULL);
  /* Infinite loop */
  for(;;)
  {
	  // Ожидание сообщения о переходе в другое состояние
	         xQueueReceive(stateQueueHandle, &currentState, portMAX_DELAY);
	                // Выбор и запуск задачи для нового состояния
	                switch (currentState) {
	                    case STATE_1:
	                    	TaskBlinkShortHandle = osThreadNew(fTaskBlinkShort, NULL, &TaskBlinkShort_attributes);
	                        break;
	                    case STATE_2:
	                    	TaskBlinkLongHandle = osThreadNew(fTaskBlinkLong, NULL, &TaskBlinkLong_attributes);
	                        break;
	                    case STATE_3:
	                    	TaskRealBlinkHandle = osThreadNew(fTaskRealBlink, NULL, &TaskRealBlink_attributes);
	                        break;
	                    default:
	                        break;
	                }
  }
  /* USER CODE END fTaskHandlerFSM */
}

/* USER CODE BEGIN Header_fTaskDelayBlink */
/**
* @brief Function implementing the TaskDelayBlink thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_fTaskDelayBlink */
void fTaskDelayBlink(void *argument)
{
  /* USER CODE BEGIN fTaskDelayBlink */
    const uint32_t xTaskDelayTime = 8000; // 8 секунд
  /* Infinite loop */
  for(;;)
  {
	 osDelay(xTaskDelayTime);
	 State_t currentState = STATE_3;
	 	             xQueueSend(stateQueueHandle, &currentState, portMAX_DELAY);
  }
  /* USER CODE END fTaskDelayBlink */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
