/**
 * @file observe_task.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Data observation and transmission task module.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This module implements the FreeRTOS task for collecting motor data
 *         from both FDCAN1 and FDCAN2 buses, packing it into a frame, and
 *         sending it via USB CDC or UART for monitoring and debugging.
 *
 * Zhengbi Yong
 *
 */
#ifndef __OBSERVE_TASK_H
#define __OBSERVE_TASK_H

#include "fdcan2_task.h"
#include "main.h"
#include "stdint.h"

/**
 * @brief  Data observation task main function.
 *
 * @note   This is the FreeRTOS task function for collecting and transmitting
 *         motor data. It packs data from all motors on both CAN buses into
 *         a frame and sends it via USB CDC.
 *
 * @retval None
 *
 * @note   Task execution flow:
 *         1. Pack motor data from FDCAN1 and FDCAN2 buses
 *         2. Calculate checksum
 *         3. Send frame via USB CDC
 *         4. Delay OBSERVE_TIME milliseconds
 *
 * @note   Frame format:
 *         - Frame header: FRAME_HEADER (0x7B)
 *         - Motor data: 5 bytes per motor (position, velocity, torque)
 *         - Checksum: 1 byte at the end
 *         - Total length: 152 bytes (FRAME_LENGTH)
 *
 * @note   Sampling frequency:
 *         - OBSERVE_TIME = 1: 640 Hz
 *         - OBSERVE_TIME = 2: 320 Hz
 *         - OBSERVE_TIME = 5: 160 Hz
 */
extern void observe_task_(void);

#endif /* __OBSERVE_TASK_H */
