/**
 * @file motor_cmd.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Motor command processing module for USB CDC communication.
 * @version 0.1
 * @date 2025-01-20
 *
 * @note   This module implements command processing for motor control commands
 *         received via USB CDC. It supports single motor and batch operations
 *         for enable, disable, save zero point, and clear error commands.
 *
 * Zhengbi Yong
 *
 */
#ifndef __MOTOR_CMD_H
#define __MOTOR_CMD_H

#include <stdint.h>
#include "stm32h7xx_hal.h"

/**
 * @brief  Command frame header identifier.
 */
#define CMD_FRAME_HEADER 0x7C

/**
 * @brief  Response frame header identifier.
 */
#define RESP_FRAME_HEADER 0x7D

/**
 * @brief  Command frame length in bytes.
 */
#define CMD_FRAME_LENGTH 16

/**
 * @brief  Response frame length in bytes.
 */
#define RESP_FRAME_LENGTH 8

/**
 * @brief  Command ID definitions.
 */
#define CMD_ENABLE_MOTOR  0x01  /**< Enable motor command */
#define CMD_DISABLE_MOTOR 0x02  /**< Disable motor command */
#define CMD_SAVE_ZERO     0x03  /**< Save position zero point command */
#define CMD_CLEAR_ERROR   0x04  /**< Clear motor error command */

/**
 * @brief  Operation type definitions.
 */
#define OP_TYPE_SINGLE 0x00  /**< Single motor operation */
#define OP_TYPE_ALL    0xFF  /**< All motors operation */

/**
 * @brief  Status code definitions.
 */
#define STATUS_OK               0x00  /**< Success */
#define STATUS_INVALID_CMD      0x01  /**< Invalid command ID */
#define STATUS_INVALID_MOTOR_ID 0x02  /**< Invalid motor ID */
#define STATUS_CHECKSUM_ERROR   0x03  /**< Checksum error */
#define STATUS_EXEC_FAIL        0x04  /**< Execution failed */

/**
 * @brief  Command frame structure.
 *
 * @note   Fixed 16-byte frame format:
 *         - Byte 0: Frame header (0x7C)
 *         - Byte 1: Command ID
 *         - Byte 2: Operation type (0x00=single, 0xFF=all)
 *         - Byte 3: Motor ID (1-30, valid for single operation)
 *         - Bytes 4-14: Reserved (all zeros)
 *         - Byte 15: Checksum (sum of first 15 bytes, low 8 bits)
 */
typedef struct __packed {
  uint8_t header;      /**< Frame header (0x7C) */
  uint8_t cmd_id;      /**< Command ID */
  uint8_t op_type;     /**< Operation type (0x00=single, 0xFF=all) */
  uint8_t motor_id;    /**< Motor ID (1-30, valid for single operation) */
  uint8_t reserved[11]; /**< Reserved bytes (all zeros) */
  uint8_t checksum;    /**< Checksum (sum of first 15 bytes) */
} motor_cmd_frame_t;

/**
 * @brief  Response frame structure.
 *
 * @note   Fixed 8-byte frame format:
 *         - Byte 0: Frame header (0x7D)
 *         - Byte 1: Command ID (echo from request)
 *         - Byte 2: Status code
 *         - Byte 3: Operation type
 *         - Byte 4: Motor ID (0xFF for all motors)
 *         - Bytes 5-6: Reserved (all zeros)
 *         - Byte 7: Checksum (sum of first 7 bytes, low 8 bits)
 */
typedef struct __packed {
  uint8_t header;     /**< Frame header (0x7D) */
  uint8_t cmd_id;     /**< Command ID (echo from request) */
  uint8_t status;     /**< Status code */
  uint8_t op_type;    /**< Operation type */
  uint8_t motor_id;   /**< Motor ID (0xFF for all motors) */
  uint8_t reserved[2]; /**< Reserved bytes (all zeros) */
  uint8_t checksum;   /**< Checksum (sum of first 7 bytes) */
} motor_resp_frame_t;

/**
 * @brief  Motor command processing task main function.
 *
 * @note   This is the FreeRTOS task function for processing motor commands
 *         from USB CDC. It should be registered as a task with normal priority.
 *
 * @retval None
 */
extern void motor_cmd_task_(void);

#endif /* __MOTOR_CMD_H */

