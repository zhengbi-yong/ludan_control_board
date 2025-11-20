/**
 * @file fdcan_bus.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  CAN bus management module for motor control.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This module provides data structures and global objects for managing
 *         multiple motors on CAN buses. It supports up to two independent CAN
 *         buses, each capable of controlling up to 16 motors.
 *
 * Zhengbi Yong
 *
 */
#ifndef __FDCAN_BUS_H
#define __FDCAN_BUS_H

#include "fdcan.h"
#include "motor_config.h"

/**
 * @brief  Maximum number of motors per CAN bus.
 *
 * @note   This value defines the maximum number of motors that can be
 *         controlled on a single CAN bus. The actual number of motors
 *         is specified by motor_count in fdcan_bus_t structure.
 */
#define MAX_MOTORS_PER_BUS 16

/**
 * @brief  CAN bus management structure.
 *
 * @note   This structure encapsulates all information needed to manage a
 *         CAN bus and its associated motors. It is used to organize motor
 *         control tasks and track bus status.
 *
 * @note   Thread safety: This structure is accessed by both CAN tasks and
 *         interrupt handlers. Care must be taken when modifying motor_count
 *         and start_flag to avoid race conditions.
 */
typedef struct {
  FDCAN_HandleTypeDef *hfdcan; /**< Pointer to FDCAN handle (bound to hfdcan1 or hfdcan2) */
  Joint_Motor_t motor[MAX_MOTORS_PER_BUS]; /**< Array of motor structures (max 16 motors) */
  uint8_t motor_count; /**< Current number of motors on this bus (0-16) */
  uint8_t start_flag; /**< Bus start flag (0: not started, 1: started and ready) */
} fdcan_bus_t;

/**
 * @brief  Global CAN bus object for FDCAN1.
 *
 * @note   This object manages all motors connected to FDCAN1 bus.
 *         It is initialized in fdcan_bus.c and used by FDCAN1_TASK.
 */
extern fdcan_bus_t fdcan1_bus;

/**
 * @brief  Global CAN bus object for FDCAN2.
 *
 * @note   This object manages all motors connected to FDCAN2 bus.
 *         It is initialized in fdcan_bus.c and used by FDCAN2_TASK.
 */
extern fdcan_bus_t fdcan2_bus;

#endif /* __FDCAN_BUS_H */
