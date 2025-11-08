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
#include "cmsis_os.h"
#include "main.h"
#include "task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "chassisL_task.h"
#include "chassisR_task.h"
#include "connect_task.h"
#include "gpio.h"
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
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for CHASSISR_TASK */
osThreadId_t CHASSISR_TASKHandle;
const osThreadAttr_t CHASSISR_TASK_attributes = {
    .name = "CHASSISR_TASK",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for CONNECT_TASK */
osThreadId_t CONNECT_TASKHandle;
const osThreadAttr_t CONNECT_TASK_attributes = {
    .name = "CONNECT_TASK",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for CHASSISL_TASK */
osThreadId_t CHASSISL_TASKHandle;
const osThreadAttr_t CHASSISL_TASK_attributes = {
    .name = "CHASSISL_TASK",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for VBUS_CHECK_TASK */
osThreadId_t VBUS_CHECK_TASKHandle;
const osThreadAttr_t VBUS_CHECK_TASK_attributes = {
    .name = "VBUS_CHECK_TASK",
    .stack_size = 1024 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ChassisR_Task(void *argument);
void CONNECT_Task(void *argument);
void ChassisL_Task(void *argument);
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
  defaultTaskHandle =
      osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CHASSISR_TASK */
  CHASSISR_TASKHandle =
      osThreadNew(ChassisR_Task, NULL, &CHASSISR_TASK_attributes);

  /* creation of CONNECT_TASK */
  CONNECT_TASKHandle =
      osThreadNew(CONNECT_Task, NULL, &CONNECT_TASK_attributes);

  /* creation of CHASSISL_TASK */
  CHASSISL_TASKHandle =
      osThreadNew(ChassisL_Task, NULL, &CHASSISL_TASK_attributes);

  /* creation of VBUS_CHECK_TASK */
  VBUS_CHECK_TASKHandle =
      osThreadNew(VBUS_CheckTask, NULL, &VBUS_CHECK_TASK_attributes);

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
void StartDefaultTask(void *argument) {
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for (;;) {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ChassisR_Task */
/**
 * @brief Function implementing the CHASSISR_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ChassisR_Task */
void ChassisR_Task(void *argument) {
  /* USER CODE BEGIN ChassisR_Task */
  /* Infinite loop */
  for (;;) {
    ChassisR_task();
  }
  /* USER CODE END ChassisR_Task */
}

/* USER CODE BEGIN Header_CONNECT_Task */
/**
 * @brief Function implementing the CONNECT_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_CONNECT_Task */
void CONNECT_Task(void *argument) {
  /* USER CODE BEGIN CONNECT_Task */
  /* Infinite loop */
  for (;;) {
    Connect_task();
  }
  /* USER CODE END CONNECT_Task */
}

/* USER CODE BEGIN Header_ChassisL_Task */
/**
 * @brief Function implementing the CHASSISL_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_ChassisL_Task */
void ChassisL_Task(void *argument) {
  /* USER CODE BEGIN ChassisL_Task */
  /* Infinite loop */
  for (;;) {
    ChassisL_task();
  }
  /* USER CODE END ChassisL_Task */
}

/* USER CODE BEGIN Header_VBUS_CheckTask */
/**
 * @brief Function implementing the VBUS_CHECK_TASK thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_VBUS_CheckTask */
void VBUS_CheckTask(void *argument) {
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
