/**
 * @file fdcan2_task.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  FDCAN2 motor control task implementation.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This file implements the FreeRTOS task for controlling motors on
 *         FDCAN2 bus. It includes motor initialization, enabling with retry
 *         mechanism, and periodic control command sending. This task runs
 *         in parallel with FDCAN1 task to control motors on two independent
 *         CAN buses.
 *
 * Zhengbi Yong
 *
 */
#include "fdcan2_task.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "fdcan_bus.h"

/**
 * @brief  Maximum number of retry attempts for motor enable.
 *
 * @note   If a motor fails to enable after this many attempts, the
 *         initialization continues with the next motor.
 */
#define MOTOR_ENABLE_MAX_RETRY 20

/**
 * @brief  Delay interval between motor enable retry attempts (milliseconds).
 *
 * @note   This delay allows time for the motor to respond to the enable
 *         command before checking the feedback.
 */
#define MOTOR_ENABLE_INTERVAL_MS 25

/**
 * @brief  Control loop period for FDCAN2 (milliseconds).
 *
 * @note   This variable controls the period of the FDCAN2 motor control loop.
 *         Default value is 1ms (1kHz control frequency).
 *         Can be adjusted for different control frequencies.
 *
 * @note   This is separate from CHASSR_TIME (FDCAN1) to allow independent
 *         control frequency adjustment for each bus.
 */
uint32_t CHASSL_TIME = 1;

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
 * @note   This task runs at high priority to ensure real-time performance.
 * @note   This task runs in parallel with FDCAN1 task without resource
 *         conflicts, as they operate on independent CAN buses (FDCAN1 and
 *         FDCAN2) and separate motor arrays (fdcan1_bus and fdcan2_bus).
 */
void fdcan2_task_(void) {
  /* Step 1: Wait for system stabilization */
  osDelay(500);

  /* Step 2: Initialize FDCAN2 bus and enable all motors */
  fdcan2_init(&fdcan2_bus);

  /* Step 3: Initialize motor feedback data structures */
  /* Motor 0-2: DM6248P (high precision motors) */
  dm6248p_fbdata_init(&fdcan2_bus.motor[0]);
  dm6248p_fbdata_init(&fdcan2_bus.motor[1]);
  dm6248p_fbdata_init(&fdcan2_bus.motor[2]);

  /* Motor 3-15: DM4340 (standard motors) */
  dm4340_fbdata_init(&fdcan2_bus.motor[3]);
  dm4340_fbdata_init(&fdcan2_bus.motor[4]);
  dm4340_fbdata_init(&fdcan2_bus.motor[5]);
  dm4340_fbdata_init(&fdcan2_bus.motor[6]);
  dm4340_fbdata_init(&fdcan2_bus.motor[7]);
  dm4340_fbdata_init(&fdcan2_bus.motor[8]);
  dm4340_fbdata_init(&fdcan2_bus.motor[9]);
  dm4340_fbdata_init(&fdcan2_bus.motor[10]);
  dm4340_fbdata_init(&fdcan2_bus.motor[11]);
  dm4340_fbdata_init(&fdcan2_bus.motor[12]);
  dm4340_fbdata_init(&fdcan2_bus.motor[13]);
  dm4340_fbdata_init(&fdcan2_bus.motor[14]);
  dm4340_fbdata_init(&fdcan2_bus.motor[15]);

  /* Set start flag (already set in fdcan2_init, but ensure it's set) */
  fdcan2_bus.start_flag = 1;

  /* Step 4: Enter control loop */
  while (1) {
    /* Send MIT control commands to all motors */
    for (int i = 0; i < fdcan2_bus.motor_count; i++) {
      mit_ctrl_test(fdcan2_bus.hfdcan, i + 1, &fdcan2_bus.motor[i]);
    }

    /* Delay to maintain control frequency (1ms for 1kHz) */
    osDelay(CHASSL_TIME);
  }
}

/**
 * @brief  Initialize FDCAN2 bus and enable all motors.
 *
 * @note   This function performs the following steps:
 *         1. Initialize all motor structures with joint_motor_init()
 *         2. Attempt to enable each motor with retry mechanism
 *         3. Set start_flag when initialization is complete
 *
 * @param  bus Pointer to FDCAN2 bus structure (must not be NULL).
 *
 * @retval None
 *
 * @note   Motor enable retry mechanism:
 *         - Maximum retry attempts: MOTOR_ENABLE_MAX_RETRY (20)
 *         - Retry interval: MOTOR_ENABLE_INTERVAL_MS (25ms)
 *         - Success condition: motor->para.enabled == 1 or state == 1
 *         - Delay 10ms between motors to prevent CAN bus congestion
 *
 * @note   If a motor fails to enable after maximum retries, initialization
 *         continues with the next motor. Failed motors can be identified
 *         by checking motor->para.enabled after initialization.
 *
 * @note   This function is similar to fdcan1_init() but operates on FDCAN2
 *         bus. Both functions can run in parallel without resource conflicts
 *         as they use independent CAN buses and motor arrays.
 */
void fdcan2_init(fdcan_bus_t *bus) {
  if (bus == NULL || bus->hfdcan == NULL) {
    return; /* Safety check */
  }

  /* Step 1: Initialize all motor parameters */
  for (int i = 0; i < MAX_MOTORS_PER_BUS; i++) {
    joint_motor_init(&bus->motor[i], i + 1, MIT_MODE);
  }

  /* Step 2: Enable all motors with retry mechanism */
  for (int i = 0; i < MAX_MOTORS_PER_BUS; i++) {
    Joint_Motor_t *motor = &bus->motor[i];
    uint8_t enable_ok = 0;
    uint32_t retry_count = 0;

    /* Retry loop: attempt to enable motor up to MAX_RETRY times */
    while (!enable_ok && retry_count < MOTOR_ENABLE_MAX_RETRY) {
      /* Send enable command */
      enable_motor_mode(bus->hfdcan, motor->para.id, motor->mode);
      osDelay(MOTOR_ENABLE_INTERVAL_MS);

      /* Check if motor is enabled (feedback updated by FDCAN callback) */
      if (motor->para.enabled == 1 || motor->para.state == 1) {
        enable_ok = 1; /* Motor enabled successfully */
      } else {
        retry_count++; /* Retry if not enabled */
      }
    }

    /* Delay between motors to prevent CAN bus congestion */
    osDelay(10);
  }

  /* Step 3: Set start flag to indicate initialization complete */
  bus->start_flag = 1;
}