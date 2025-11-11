/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * File Name          : freertos.c
 * Description        : Code for freertos applications
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "fdcan1_task.h"
#include "fdcan2_task.h"
#include "gpio.h"
#include "observe_task.h"
#include "tim.h"
#include "vbus_check.h"
#include <stdio.h>

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for FDCAN1_TASK */
osThreadId_t FDCAN1_TASKHandle;
const osThreadAttr_t FDCAN1_TASK_attributes = {
  .name = "FDCAN1_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for OBSERVE_TASK */
osThreadId_t OBSERVE_TASKHandle;
const osThreadAttr_t OBSERVE_TASK_attributes = {
  .name = "OBSERVE_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for FDCAN2_TASK */
osThreadId_t FDCAN2_TASKHandle;
const osThreadAttr_t FDCAN2_TASK_attributes = {
  .name = "FDCAN2_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for VBUS_CHECK_TASK */
osThreadId_t VBUS_CHECK_TASKHandle;
const osThreadAttr_t VBUS_CHECK_TASK_attributes = {
  .name = "VBUS_CHECK_TASK",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void fdcan1_task(void *argument);
void observe_task(void *argument);
void fdcan2_task(void *argument);
void VBUS_CheckTask(void *argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of FDCAN1_TASK */
  FDCAN1_TASKHandle = osThreadNew(fdcan1_task, NULL, &FDCAN1_TASK_attributes);

  /* creation of OBSERVE_TASK */
  OBSERVE_TASKHandle = osThreadNew(observe_task, NULL, &OBSERVE_TASK_attributes);

  /* creation of FDCAN2_TASK */
  FDCAN2_TASKHandle = osThreadNew(fdcan2_task, NULL, &FDCAN2_TASK_attributes);

  /* creation of VBUS_CHECK_TASK */
  VBUS_CHECK_TASKHandle = osThreadNew(VBUS_CheckTask, NULL, &VBUS_CHECK_TASK_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_fdcan1_task */
/**
 * @brief Function implementing the FDCAN1_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_fdcan1_task */
void fdcan1_task(void *argument)
{
  /* USER CODE BEGIN fdcan1_task */
  /* Infinite loop */
  for (;;) {
    fdcan1_task_();
  }
  /* USER CODE END fdcan1_task */
}

/* USER CODE BEGIN Header_observe_task */
/**
 * @brief Function implementing the OBSERVE_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_observe_task */
void observe_task(void *argument)
{
  /* USER CODE BEGIN observe_task */
  /* Infinite loop */
  for (;;) {
    observe_task_();
  }
  /* USER CODE END observe_task */
}

/* USER CODE BEGIN Header_fdcan2_task */
/**
 * @brief Function implementing the FDCAN2_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_fdcan2_task */
void fdcan2_task(void *argument)
{
  /* USER CODE BEGIN fdcan2_task */
  /* Infinite loop */
  for (;;) {
    fdcan2_task_();
  }
  /* USER CODE END fdcan2_task */
}

/* USER CODE BEGIN Header_VBUS_CheckTask */
/**
 * @brief Function implementing the VBUS_CHECK_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_VBUS_CheckTask */
void VBUS_CheckTask(void *argument)
{
  /* USER CODE BEGIN VBUS_CheckTask */
  /* Infinite loop */
  for (;;) {
    VBUS_Check_task();
  }
  /* USER CODE END VBUS_CheckTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

