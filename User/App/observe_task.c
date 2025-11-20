/**
 * @file observe_task.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Data observation and transmission task implementation.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This file implements the FreeRTOS task for collecting motor data
 *         from both FDCAN1 and FDCAN2 buses, packing it into a frame format,
 *         and sending it via USB CDC for real-time monitoring and debugging.
 *
 * Zhengbi Yong
 *
 */
#include "observe_task.h"

#include "bsp_usart1.h"
#include "cmsis_os.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"

/* External declarations */
extern UART_HandleTypeDef huart1;
extern send_data_t send_data;
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;

/**
 * @brief  Total frame length in bytes.
 *
 * @note   Frame structure:
 *         - 1 byte: Frame header (FRAME_HEADER = 0x7B)
 *         - 150 bytes: Motor data (30 motors × 5 bytes/motor)
 *         - 1 byte: Checksum
 *         - Total: 152 bytes
 */
#define FRAME_LENGTH 152

/**
 * @brief  Data length for checksum calculation (excluding checksum byte).
 *
 * @note   Checksum is calculated over the first LOAD_LENGTH bytes
 *         (frame header + motor data).
 */
#define LOAD_LENGTH 151

/**
 * @brief  Observation task delay time (milliseconds).
 *
 * @note   This variable controls the sampling frequency of the observation
 *         task. The actual frequency depends on the value:
 *         - OBSERVE_TIME = 1: 640 Hz (1ms period)
 *         - OBSERVE_TIME = 2: 320 Hz (2ms period)
 *         - OBSERVE_TIME = 5: 160 Hz (5ms period)
 *
 * @note   Default value is 1ms for maximum sampling rate (640 Hz).
 */
uint32_t OBSERVE_TIME = 1;

/**
 * @brief  Data observation task main function.
 *
 * @note   This function collects motor data from both FDCAN1 and FDCAN2 buses,
 *         packs it into a frame format, and sends it via USB CDC for real-time
 *         monitoring and debugging.
 *
 * @retval None
 *
 * @note   Frame format (152 bytes total):
 *         - Byte 0: Frame header (FRAME_HEADER = 0x7B)
 *         - Bytes 1-150: Motor data (30 motors × 5 bytes/motor)
 *           - Each motor: 5 bytes
 *             - Byte 0-1: Position (16 bits, big-endian)
 *             - Byte 2: Velocity high 8 bits
 *             - Byte 3: Velocity low 4 bits | Torque high 4 bits
 *             - Byte 4: Torque low 8 bits
 *         - Byte 151: Checksum (sum of first 151 bytes)
 *
 * @note   Data compression:
 *         - Velocity: 12 bits compressed to 1.5 bytes
 *         - Torque: 12 bits compressed to 1.5 bytes
 *         - Position: 16 bits (2 bytes, no compression)
 *
 * @note   Motor order:
 *         - First: All motors from FDCAN1 bus (motor_count motors)
 *         - Then: All motors from FDCAN2 bus (motor_count motors)
 *         - Total: up to 30 motors (15 per bus)
 *
 * @note   Sampling frequency:
 *         - Controlled by OBSERVE_TIME variable
 *         - Default: 640 Hz (OBSERVE_TIME = 1ms)
 */
void observe_task_(void) {
  while (1) {
    /* Step 1: Set frame header */
    send_data.tx[0] = FRAME_HEADER;

    /* Step 2: Pack motor data from FDCAN1 and FDCAN2 buses */
    int idx = 0; /* Global motor index for frame packing offset */

    /* Pack FDCAN1 bus motor data */
    for (int i = 0; i < fdcan1_bus.motor_count; i++) {
      /* Position: 16 bits (2 bytes, big-endian) */
      send_data.tx[1 + idx * 5] = fdcan1_bus.motor[i].para.p_int >> 8;  /* Position high byte */
      send_data.tx[2 + idx * 5] = fdcan1_bus.motor[i].para.p_int;       /* Position low byte */

      /* Velocity: 12 bits compressed to 1.5 bytes */
      send_data.tx[3 + idx * 5] = fdcan1_bus.motor[i].para.v_int >> 4;  /* Velocity high 8 bits */

      /* Velocity low 4 bits | Torque high 4 bits */
      send_data.tx[4 + idx * 5] =
          ((fdcan1_bus.motor[i].para.v_int & 0x0F) << 4) |
          (fdcan1_bus.motor[i].para.t_int >> 8);

      /* Torque: 12 bits compressed to 1.5 bytes */
      send_data.tx[5 + idx * 5] = fdcan1_bus.motor[i].para.t_int;       /* Torque low 8 bits */

      idx++; /* Increment global motor index */
    }

    /* Pack FDCAN2 bus motor data */
    for (int i = 0; i < fdcan2_bus.motor_count; i++) {
      /* Position: 16 bits (2 bytes, big-endian) */
      send_data.tx[1 + idx * 5] = fdcan2_bus.motor[i].para.p_int >> 8;  /* Position high byte */
      send_data.tx[2 + idx * 5] = fdcan2_bus.motor[i].para.p_int;       /* Position low byte */

      /* Velocity: 12 bits compressed to 1.5 bytes */
      send_data.tx[3 + idx * 5] = fdcan2_bus.motor[i].para.v_int >> 4;  /* Velocity high 8 bits */

      /* Velocity low 4 bits | Torque high 4 bits */
      send_data.tx[4 + idx * 5] =
          ((fdcan2_bus.motor[i].para.v_int & 0x0F) << 4) |
          (fdcan2_bus.motor[i].para.t_int >> 8);

      /* Torque: 12 bits compressed to 1.5 bytes */
      send_data.tx[5 + idx * 5] = fdcan2_bus.motor[i].para.t_int;       /* Torque low 8 bits */

      idx++; /* Increment global motor index */
    }

    /* Step 3: Calculate and set checksum */
    send_data.tx[LOAD_LENGTH] = Check_Sum(LOAD_LENGTH, send_data.tx);

    /* Step 4: Send frame via USB CDC */
    CDC_Transmit_HS((uint8_t *)send_data.tx, FRAME_LENGTH);

    /* Step 5: Delay to maintain sampling frequency */
    osDelay(OBSERVE_TIME);
  }
}
