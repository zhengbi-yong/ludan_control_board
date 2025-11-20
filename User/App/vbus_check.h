/**
 * @file vbus_check.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Voltage monitoring and protection task module.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This module implements the FreeRTOS task for real-time voltage
 *         monitoring and protection. It continuously monitors the system
 *         voltage and triggers alarms or protection actions when voltage
 *         falls below safe thresholds.
 *
 * Zhengbi Yong
 *
 */
#ifndef __VBUS_CHECK_H
#define __VBUS_CHECK_H

#include "stm32h7xx_hal.h"

/**
 * @brief  Voltage loss flag (global).
 *
 * @note   This flag is set to 1 when voltage falls below the disable
 *         threshold (vbus_threhold_disable). It can be checked by other
 *         tasks to implement voltage-aware behavior.
 */
extern uint8_t loss_voltage;

/**
 * @brief  Voltage monitoring task main function.
 *
 * @note   This is the FreeRTOS task function for voltage monitoring and
 *         protection. It should be registered as a task with normal priority.
 *
 * @retval None
 *
 * @note   Task execution flow:
 *         1. Initialize ADC calibration
 *         2. Start ADC DMA for continuous voltage sampling
 *         3. Enter monitoring loop:
 *            - Calculate voltage from ADC value
 *            - Trigger buzzer alarm if voltage is low
 *            - Disable power outputs and motors if voltage is critically low
 *            - Delay 10ms (100Hz monitoring frequency)
 *
 * @note   Voltage thresholds:
 *         - vbus_threhold_call (22.6V): Buzzer alarm threshold
 *         - vbus_threhold_disable (22.2V): Power disable threshold
 *         - Minimum valid voltage: 6.0V
 */
extern void VBUS_Check_task(void);

#endif /* __VBUS_CHECK_H */

