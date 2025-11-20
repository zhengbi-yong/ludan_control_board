/**
 * @file fdcan1_task.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  FDCAN1 motor control task module.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This module implements the FreeRTOS task for controlling motors
 *         connected to FDCAN1 bus. It handles motor initialization, enabling,
 *         and periodic control command sending.
 *
 * Zhengbi Yong
 *
 */
#ifndef __FDCAN1_TASK_H
#define __FDCAN1_TASK_H

#include "fdcan_bus.h"
#include "main.h"
#include "motor_config.h"

/**
 * @brief  Initialize FDCAN1 bus and enable all motors.
 *
 * @param  bus Pointer to FDCAN1 bus structure.
 *
 * @retval None
 *
 * @note   This function initializes all motors on FDCAN1 bus and attempts
 *         to enable them with retry mechanism. It sets the start_flag when
 *         initialization is complete.
 */
extern void fdcan1_init(fdcan_bus_t *bus);

/**
 * @brief  FDCAN1 control task main function.
 *
 * @note   This is the FreeRTOS task function for FDCAN1 motor control.
 *         It should be registered as a task with high priority.
 *
 * @retval None
 *
 * @note   Task flow:
 *         1. Wait 500ms for system stabilization
 *         2. Initialize FDCAN1 bus and motors
 *         3. Initialize motor feedback data structures
 *         4. Enter control loop: send MIT commands to all motors periodically
 */
extern void fdcan1_task_(void);

#endif /* __FDCAN1_TASK_H */
