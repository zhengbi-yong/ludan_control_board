/**
 * @file fdcan2_task.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  FDCAN2 motor control task module.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This module implements the FreeRTOS task for controlling motors
 *         connected to FDCAN2 bus. It handles motor initialization, enabling,
 *         and periodic control command sending. This task runs in parallel
 *         with FDCAN1 task to control motors on two independent CAN buses.
 *
 * Zhengbi Yong
 *
 */
#ifndef __FDCAN2_TASK_H
#define __FDCAN2_TASK_H

#include "fdcan_bus.h"
#include "main.h"
#include "motor_config.h"

/**
 * @brief  Initialize FDCAN2 bus and enable all motors.
 *
 * @param  bus Pointer to FDCAN2 bus structure.
 *
 * @retval None
 *
 * @note   This function initializes all motors on FDCAN2 bus and attempts
 *         to enable them with retry mechanism. It sets the start_flag when
 *         initialization is complete.
 *
 * @note   This function is similar to fdcan1_init() but operates on FDCAN2
 *         bus. Both functions can run in parallel without resource conflicts.
 */
extern void fdcan2_init(fdcan_bus_t *bus);

/**
 * @brief  FDCAN2 control task main function.
 *
 * @note   This is the FreeRTOS task function for FDCAN2 motor control.
 *         It should be registered as a task with high priority to ensure
 *         real-time control performance (1kHz control frequency).
 *
 * @retval None
 *
 * @note   Task execution flow:
 *         1. Wait 500ms for system stabilization
 *         2. Initialize FDCAN2 bus and enable all motors
 *         3. Initialize motor feedback data structures (dm***_fbdata_init)
 *         4. Enter infinite control loop:
 *            - Send MIT control commands to all motors
 *            - Delay CHASSL_TIME milliseconds (default 1ms)
 *
 * @note   Control frequency: 1kHz (when CHASSL_TIME = 1ms)
 * @note   This task runs in parallel with FDCAN1 task without resource
 *         conflicts, as they operate on independent CAN buses.
 */
extern void fdcan2_task_(void);

#endif /* __FDCAN2_TASK_H */
