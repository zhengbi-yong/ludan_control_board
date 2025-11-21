/**
 * @file motor_cmd.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Motor command processing module implementation.
 * @version 0.1
 * @date 2025-01-20
 *
 * @note   This module implements command processing for motor control commands
 *         received via USB CDC. It supports single motor and batch operations.
 *
 * Zhengbi Yong
 *
 */
#include "motor_cmd.h"
#include "fdcan_bus.h"
#include "motor_config.h"
#include "cmsis_os2.h"
#include "usbd_cdc_if.h"
#include <string.h>

/* External variables */
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;

/* Command queue handle (defined in freertos.c) */
extern osMessageQueueId_t motor_cmd_queue;

/**
 * @brief  Calculate checksum for command/response frame.
 *
 * @param  data  Data pointer
 * @param  len   Data length (excluding checksum byte)
 * @retval Checksum value (sum of all bytes, low 8 bits)
 */
uint8_t motor_cmd_calculate_checksum(const uint8_t *data, uint32_t len) {
  uint32_t sum = 0;
  for (uint32_t i = 0; i < len; i++) {
    sum += data[i];
  }
  return (uint8_t)(sum & 0xFF);
}

/**
 * @brief  Parse and validate command frame.
 *
 * @param  frame    Command frame pointer (16 bytes)
 * @param  len      Data length
 * @param  cmd_id   Output: Command ID
 * @param  op_type  Output: Operation type
 * @param  motor_id Output: Motor ID
 * @retval 0: Valid, non-zero: Invalid
 */
uint8_t motor_cmd_parse(const uint8_t *frame, uint32_t len, uint8_t *cmd_id,
                        uint8_t *op_type, uint8_t *motor_id) {
  /* Validate input parameters */
  if (frame == NULL || cmd_id == NULL || op_type == NULL ||
      motor_id == NULL) {
    return 1; /* Invalid parameters */
  }

  /* Check frame length */
  if (len != CMD_FRAME_LENGTH) {
    return 1; /* Invalid length */
  }

  /* Check frame header */
  if (frame[0] != CMD_FRAME_HEADER) {
    return 1; /* Invalid header */
  }

  /* Verify checksum */
  uint8_t calculated_checksum =
      motor_cmd_calculate_checksum(frame, CMD_FRAME_LENGTH - 1);
  if (calculated_checksum != frame[CMD_FRAME_LENGTH - 1]) {
    return 1; /* Checksum error */
  }

  /* Extract command parameters */
  *cmd_id = frame[1];
  *op_type = frame[2];
  *motor_id = frame[3];

  return 0; /* Valid frame */
}

/**
 * @brief  Get bus pointer and local ID from global motor ID.
 *
 * @param  global_id  Global motor ID (1-30)
 * @param  bus        Output: Bus pointer (fdcan1_bus or fdcan2_bus)
 * @param  local_id   Output: Local ID (0-14, motor array index)
 * @retval 0: Success, non-zero: Invalid ID
 *
 * @note   Mapping rule:
 *         - Motors 1-15:  fdcan1_bus, local_id = global_id - 1
 *         - Motors 16-30: fdcan2_bus, local_id = global_id - 16
 */
uint8_t motor_cmd_get_bus(uint8_t global_id, fdcan_bus_t **bus,
                           uint8_t *local_id) {
  if (global_id < 1 || global_id > 30) {
    return 1; /* Invalid global ID */
  }

  if (global_id <= 15) {
    /* FDCAN1 bus: motors 1-15 */
    *bus = &fdcan1_bus;
    *local_id = global_id - 1;
  } else {
    /* FDCAN2 bus: motors 16-30 */
    *bus = &fdcan2_bus;
    *local_id = global_id - 16;
  }

  return 0; /* Success */
}

/**
 * @brief  Execute single motor command.
 *
 * @param  cmd_id    Command ID
 * @param  motor_id  Global motor ID (1-30)
 * @retval Status code
 */
uint8_t motor_cmd_execute_single(uint8_t cmd_id, uint8_t motor_id) {
  fdcan_bus_t *bus = NULL;
  uint8_t local_id = 0;

  /* Get bus pointer and local ID */
  if (motor_cmd_get_bus(motor_id, &bus, &local_id) != 0) {
    return STATUS_INVALID_MOTOR_ID;
  }

  /* Check if bus is initialized */
  if (bus == NULL || bus->hfdcan == NULL) {
    return STATUS_EXEC_FAIL;
  }

  /* Check if motor ID is within bus motor count */
  if (local_id >= bus->motor_count) {
    return STATUS_INVALID_MOTOR_ID;
  }

  /* Execute command based on command ID */
  uint16_t can_motor_id = local_id + 1; /* CAN motor ID (1-16) */
  uint8_t result = STATUS_OK;

  switch (cmd_id) {
  case CMD_ENABLE_MOTOR:
    enable_motor_mode(bus->hfdcan, can_motor_id, MIT_MODE);
    break;

  case CMD_DISABLE_MOTOR:
    disable_motor_mode(bus->hfdcan, can_motor_id, MIT_MODE);
    break;

  case CMD_SAVE_ZERO:
    save_motor_zero(bus->hfdcan, can_motor_id, MIT_MODE);
    break;

  case CMD_CLEAR_ERROR:
    clear_motor_error(bus->hfdcan, can_motor_id, MIT_MODE);
    break;

  default:
    result = STATUS_INVALID_CMD;
    break;
  }

  return result;
}

/**
 * @brief  Execute command for all motors (batch operation).
 *
 * @param  cmd_id  Command ID
 * @retval Status code
 *
 * @note   Sequential execution: first all motors on fdcan1_bus,
 *         then all motors on fdcan2_bus. Each motor operation
 *         is followed by a 5ms delay to prevent CAN bus congestion.
 */
uint8_t motor_cmd_execute_all(uint8_t cmd_id) {
  /* Validate command ID */
  if (cmd_id < CMD_ENABLE_MOTOR || cmd_id > CMD_CLEAR_ERROR) {
    return STATUS_INVALID_CMD;
  }

  /* Execute command for all motors on FDCAN1 bus */
  for (uint8_t i = 0; i < fdcan1_bus.motor_count; i++) {
    uint8_t global_id = i + 1; /* Global ID: 1-15 */
    motor_cmd_execute_single(cmd_id, global_id);
    osDelay(5); /* Delay to prevent CAN bus congestion */
  }

  /* Execute command for all motors on FDCAN2 bus */
  for (uint8_t i = 0; i < fdcan2_bus.motor_count; i++) {
    uint8_t global_id = i + 16; /* Global ID: 16-30 */
    motor_cmd_execute_single(cmd_id, global_id);
    osDelay(5); /* Delay to prevent CAN bus congestion */
  }

  return STATUS_OK;
}

/**
 * @brief  Send response frame via USB CDC.
 *
 * @param  cmd_id    Command ID
 * @param  status    Status code
 * @param  op_type   Operation type
 * @param  motor_id  Motor ID (0xFF for all motors)
 * @retval 0: Success, non-zero: Failure
 */
uint8_t motor_cmd_send_response(uint8_t cmd_id, uint8_t status,
                                  uint8_t op_type, uint8_t motor_id) {
  motor_resp_frame_t resp_frame;

  /* Fill response frame */
  resp_frame.header = RESP_FRAME_HEADER;
  resp_frame.cmd_id = cmd_id;
  resp_frame.status = status;
  resp_frame.op_type = op_type;
  resp_frame.motor_id = motor_id;
  resp_frame.reserved[0] = 0;
  resp_frame.reserved[1] = 0;

  /* Calculate checksum */
  resp_frame.checksum =
      motor_cmd_calculate_checksum((uint8_t *)&resp_frame, RESP_FRAME_LENGTH - 1);

  /* Send response via USB CDC */
  if (CDC_Transmit_HS((uint8_t *)&resp_frame, RESP_FRAME_LENGTH) != USBD_OK) {
    return 1; /* Send failed */
  }

  return 0; /* Success */
}

/**
 * @brief  Motor command processing task main function.
 *
 * @note   This FreeRTOS task processes motor commands from the command queue.
 *         It parses commands, executes them, and sends responses.
 *
 * @retval None
 *
 * @note   Task flow:
 *         1. Wait for command from queue (blocking)
 *         2. Parse command frame
 *         3. Validate command parameters
 *         4. Execute command (single or batch)
 *         5. Send response frame
 */
void motor_cmd_task_(void) {
  uint8_t cmd_buffer[CMD_FRAME_LENGTH];
  uint8_t cmd_id, op_type, motor_id;
  uint8_t status;

  while (1) {
    /* Wait for command from queue (blocking) */
    if (osMessageQueueGet(motor_cmd_queue, cmd_buffer, NULL,
                          osWaitForever) == osOK) {
      /* Parse command frame */
      if (motor_cmd_parse(cmd_buffer, CMD_FRAME_LENGTH, &cmd_id, &op_type,
                          &motor_id) == 0) {
        /* Validate command ID */
        if (cmd_id < CMD_ENABLE_MOTOR || cmd_id > CMD_CLEAR_ERROR) {
          status = STATUS_INVALID_CMD;
          motor_cmd_send_response(cmd_id, status, op_type, motor_id);
          continue;
        }

        /* Validate operation type */
        if (op_type != OP_TYPE_SINGLE && op_type != OP_TYPE_ALL) {
          status = STATUS_INVALID_CMD;
          motor_cmd_send_response(cmd_id, status, op_type, motor_id);
          continue;
        }

        /* Execute command */
        if (op_type == OP_TYPE_SINGLE) {
          /* Single motor operation */
          if (motor_id < 1 || motor_id > 30) {
            status = STATUS_INVALID_MOTOR_ID;
          } else {
            status = motor_cmd_execute_single(cmd_id, motor_id);
          }
          motor_cmd_send_response(cmd_id, status, op_type, motor_id);
        } else {
          /* All motors operation */
          status = motor_cmd_execute_all(cmd_id);
          motor_cmd_send_response(cmd_id, status, op_type, 0xFF);
        }
      } else {
        /* Parse failed - send checksum error response */
        /* Try to extract cmd_id for response (may be invalid) */
        cmd_id = (cmd_buffer[1] < CMD_ENABLE_MOTOR ||
                  cmd_buffer[1] > CMD_CLEAR_ERROR)
                     ? CMD_ENABLE_MOTOR
                     : cmd_buffer[1];
        op_type = cmd_buffer[2];
        motor_id = cmd_buffer[3];
        motor_cmd_send_response(cmd_id, STATUS_CHECKSUM_ERROR, op_type,
                                motor_id);
      }
    }
  }
}

