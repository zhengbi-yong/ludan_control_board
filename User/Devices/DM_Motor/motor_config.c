/**
 * @file motor_config.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief
 * @version 0.1
 * @date 2025-11-18
 *
 * Zhengbi Yong
 *
 */
#include "motor_config.h"
#include "arm_math.h"
#include "fdcan.h"

float Hex_To_Float(uint32_t *Byte, int num) { return *((float *)Byte); }

uint32_t FloatTohex(float HEX) { return *(uint32_t *)&HEX; }

/**
 * @brief  Convert float value to unsigned integer with quantization.
 *
 * @note   This function quantizes a float value to an integer within the
 *         specified range and bit width. Input values outside the range are
 *         clamped to the boundaries.
 *
 * @param  x_float Input float value to quantize.
 * @param  x_min   Minimum value of the range.
 * @param  x_max   Maximum value of the range.
 * @param  bits    Number of bits for quantization (determines resolution).
 *
 * @return Quantized integer value (0 to (2^bits - 1)).
 *
 * @note   Formula: int = (x_float - x_min) * (2^bits - 1) / (x_max - x_min)
 * @note   Input values are automatically clamped to [x_min, x_max] range.
 */
int float_to_uint(float x_float, float x_min, float x_max, int bits) {
  /* Clamp input value to valid range */
  if (x_float < x_min) {
    x_float = x_min;
  } else if (x_float > x_max) {
    x_float = x_max;
  }

  /* Convert float to unsigned int, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
 * @brief  Convert unsigned integer to float value with dequantization.
 *
 * @note   This function converts a quantized integer back to a float value
 *         within the specified range. This is the inverse operation of
 *         float_to_uint().
 *
 * @param  x_int  Input integer value to dequantize (0 to (2^bits - 1)).
 * @param  x_min  Minimum value of the output range.
 * @param  x_max  Maximum value of the output range.
 * @param  bits   Number of bits used for quantization (must match encoding).
 *
 * @return Dequantized float value within [x_min, x_max] range.
 *
 * @note   Formula: float = x_int * (x_max - x_min) / (2^bits - 1) + x_min
 * @note   This function is the inverse of float_to_uint().
 */
float uint_to_float(int x_int, float x_min, float x_max, int bits) {
  /* Converts unsigned int to float, given range and number of bits */
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/**
 * @brief  Initialize joint motor structure with ID and control mode.
 *
 * @note   This function initializes a Joint_Motor_t structure with the
 *         specified motor ID and control mode. All other parameters are
 *         set to zero/default values.
 *
 * @param  motor Pointer to Joint_Motor_t structure to initialize.
 * @param  id    Motor ID (typically 1-16, corresponding to CAN ID 0x11-0x1F).
 * @param  mode  Control mode (MIT_MODE, POS_MODE, or SPEED_MODE).
 *
 * @retval None
 *
 * @note   This function should be called before using the motor for the
 *         first time or when resetting motor parameters.
 */
void joint_motor_init(Joint_Motor_t *motor, uint16_t id, uint16_t mode) {
  if (motor == NULL) {
    return; /* Safety check */
  }

  motor->mode = mode;
  motor->para.id = id;
  motor->para.state = 0;
  motor->para.enabled = 0;

  /* Initialize all parameters to zero */
  motor->para.p_int = 0;
  motor->para.v_int = 0;
  motor->para.t_int = 0;
  motor->para.kp_int = 0;
  motor->para.kd_int = 0;

  motor->para.pos = 0.0f;
  motor->para.vel = 0.0f;
  motor->para.tor = 0.0f;
  motor->para.kp = 0.0f;
  motor->para.kd = 0.0f;

  motor->para.Tmos = 0.0f;
  motor->para.Tcoil = 0.0f;
}

void wheel_motor_init(Wheel_Motor_t *motor, uint16_t id, uint16_t mode) {
  motor->mode = mode;
  motor->para.id = id;
}

/**
 * @brief  Parse DM4310 motor feedback data.
 *
 * @note   This function parses the 8-byte feedback frame from DM4310 motor
 *         and updates the motor parameter structure. The feedback includes
 *         position, velocity, torque, temperature, and status information.
 *
 * @param  motor    Pointer to Joint_Motor_t structure to update.
 * @param  rx_data  Pointer to received CAN data buffer (must be 8 bytes).
 * @param  len      Data length (must be FDCAN_DLC_BYTES_8).
 *
 * @retval None
 *
 * @note   Data frame format (8 bytes):
 *         - Byte 0: [Error Code (4 bits)] [Motor ID (4 bits)]
 *         - Byte 1-2: Position (16 bits, signed)
 *         - Byte 3: Velocity high 8 bits
 *         - Byte 4: Velocity low 4 bits | Torque high 4 bits
 *         - Byte 5: Torque low 8 bits
 *         - Byte 6: MOS temperature (°C)
 *         - Byte 7: Coil temperature (°C)
 *
 * @note   If data length is not 8 bytes, the function returns without
 *         updating motor parameters.
 *
 * @note   Parameter ranges for DM4310:
 *         - Position: P_MIN1 to P_MAX1 (-12.5 to 12.5 rad)
 *         - Velocity: V_MIN1 to V_MAX1 (-30.0 to 30.0 rad/s)
 *         - Torque: T_MIN1 to T_MAX1 (-10.0 to 10.0 N·m)
 */
void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t len) {
  /* Validate input parameters */
  if (motor == NULL || rx_data == NULL) {
    return; /* Safety check */
  }

  /* Validate data length - must be exactly 8 bytes */
  if (len != FDCAN_DLC_BYTES_8) {
    return;
  }

  /* Step 1: Parse motor ID and error code from byte 0 */
  uint8_t id_low = rx_data[0] & 0x0F;
  uint8_t err_code = rx_data[0] >> 4;

  motor->para.id = id_low;
  motor->para.state = err_code;

  /* Step 2: Parse raw integer data from CAN frame */
  motor->para.p_int = (rx_data[1] << 8) | rx_data[2]; /* Position (16 bits) */
  motor->para.v_int =
      (rx_data[3] << 4) | (rx_data[4] >> 4); /* Velocity (12 bits) */
  motor->para.t_int =
      ((rx_data[4] & 0x0F) << 8) | rx_data[5]; /* Torque (12 bits) */

  /* Step 3: Convert integer values to float using uint_to_float */
  motor->para.pos = uint_to_float(motor->para.p_int, P_MIN1, P_MAX1, 16);
  motor->para.vel = uint_to_float(motor->para.v_int, V_MIN1, V_MAX1, 12);
  motor->para.tor = uint_to_float(motor->para.t_int, T_MIN1, T_MAX1, 12);

  /* Step 4: Parse temperature information */
  motor->para.Tmos = (float)rx_data[6];  /* MOS temperature (°C) */
  motor->para.Tcoil = (float)rx_data[7]; /* Coil temperature (°C) */

  /* Step 5: Update motor enabled status based on state code */
  if (err_code == 0) {
    motor->para.enabled = 1;
  } else {
    motor->para.enabled = 0;
  }
}

void dm4340_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN2, P_MAX2, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN2, V_MAX2, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN2, T_MAX2, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm6006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN3, P_MAX3, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN3, V_MAX3, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN3, T_MAX3, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm8006_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN4, P_MAX4, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN4, V_MAX4, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN4, T_MAX4, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm3507_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN5, P_MAX5, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN5, V_MAX5, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN5, T_MAX5, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm10010l_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,
                     uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN6, P_MAX6, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN6, V_MAX6, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN6, T_MAX6, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm6248p_fbdata(Joint_Motor_t *motor, uint8_t *rx_data, uint32_t data_len) {
  if (data_len == FDCAN_DLC_BYTES_8) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, P_MIN7, P_MAX7, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, V_MIN7, V_MAX7, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, T_MIN7, T_MAX7, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
  }
}

void dm4310_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN1, P_MAX1, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN1, V_MAX1, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN1, T_MAX1, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN1, KP_MAX1, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN1, KD_MAX1, 12);
}

void dm4340_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN2, P_MAX2, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN2, V_MAX2, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN2, T_MAX2, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN2, KP_MAX2, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN2, KD_MAX2, 12);
}

void dm6006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN3, P_MAX3, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN3, V_MAX3, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN3, T_MAX3, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN3, KP_MAX3, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN3, KD_MAX3, 12);
}

void dm8006_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN4, P_MAX4, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN4, V_MAX4, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN4, T_MAX4, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN4, KP_MAX4, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN4, KD_MAX4, 12);
}

void dm10010l_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN6, P_MAX6, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN6, V_MAX6, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN6, T_MAX6, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN6, KP_MAX6, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN6, KD_MAX6, 12);
}

void dm6248p_fbdata_test(Joint_Motor_t *motor, uint8_t *rx_data) {
  motor->para.p_int_test = (rx_data[2] << 8) | rx_data[3];

  motor->para.v_int_test =
      (((int16_t)rx_data[4]) << 4) | ((rx_data[5] & 0xF0) >> 4);

  motor->para.kp_int_test = (((int16_t)(rx_data[5] & 0x0F)) << 8) | rx_data[6];

  motor->para.kd_int_test =
      (((int16_t)rx_data[7]) << 4) | ((rx_data[8] & 0xF0) >> 4);

  motor->para.t_int_test = (((int16_t)(rx_data[8] & 0x0F)) << 8) | rx_data[9];

  motor->para.pos_set =
      uint_to_float(motor->para.p_int_test, P_MIN7, P_MAX7, 16);
  motor->para.vel_set =
      uint_to_float(motor->para.v_int_test, V_MIN7, V_MAX7, 12);
  motor->para.tor_set =
      uint_to_float(motor->para.t_int_test, T_MIN7, T_MAX7, 12);
  motor->para.kp_test =
      uint_to_float(motor->para.kp_int_test, KP_MIN7, KP_MAX7, 12);
  motor->para.kd_test =
      uint_to_float(motor->para.kd_int_test, KD_MIN7, KD_MAX7, 12);
}

/**
 * @brief  Enable motor and enter specified control mode.
 *
 * @note   This function sends an enable command to the motor to enter the
 *         specified control mode (MIT_MODE, POS_MODE, or SPEED_MODE).
 *         The motor will respond with feedback data once enabled.
 *
 * @param  hcan     Pointer to FDCAN handle.
 * @param  motor_id Motor ID (1-16, corresponding to CAN ID 0x11-0x1F).
 * @param  mode_id  Control mode (MIT_MODE, POS_MODE, or SPEED_MODE).
 *
 * @retval None
 *
 * @note   Enable command format: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFC
 * @note   CAN ID = motor_id + mode_id (e.g., motor_id=1, mode_id=MIT_MODE
 *         results in CAN ID = 0x001)
 * @note   The function does not wait for motor confirmation. Check motor
 *         feedback to verify enable status.
 */
void enable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  if (hcan == NULL) {
    return; /* Safety check */
  }

  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  /* Enable command: all 0xFF except last byte 0xFC */
  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFC;

  /* Send enable command via CAN */
  canx_send_data(hcan, id, data, 8);
}

void save_motor_zero(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  /* Disable command: all 0xFF except last byte 0xFE */
  if (hcan == NULL) {
    return; /* Safety check */
  }

  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFE;

  /* Send disable command via CAN */
  canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Disable motor and exit control mode.
 *
 * @note   This function sends a disable command to the motor to exit the
 *         current control mode. The motor will stop responding to control
 *         commands after being disabled.
 *
 * @param  hcan     Pointer to FDCAN handle.
 * @param  motor_id Motor ID (1-16, corresponding to CAN ID 0x11-0x1F).
 * @param  mode_id  Control mode (MIT_MODE, POS_MODE, or SPEED_MODE).
 *
 * @retval None
 *
 * @note   Disable command format: 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFF 0xFD
 * @note   CAN ID = motor_id + mode_id
 * @note   After disabling, the motor will stop responding to control commands.
 */
void disable_motor_mode(hcan_t *hcan, uint16_t motor_id, uint16_t mode_id) {
  if (hcan == NULL) {
    return; /* Safety check */
  }

  uint8_t data[8];
  uint16_t id = motor_id + mode_id;

  data[0] = 0xFF;
  data[1] = 0xFF;
  data[2] = 0xFF;
  data[3] = 0xFF;
  data[4] = 0xFF;
  data[5] = 0xFF;
  data[6] = 0xFF;
  data[7] = 0xFD;

  canx_send_data(hcan, id, data, 8);
}

/**
 * @brief  Send MIT mode control command for DM4310 motor.
 *
 * @note   This function sends a MIT (Mixed Control) mode command to DM4310
 *         motor. MIT mode allows simultaneous control of position, velocity,
 *         and torque with configurable stiffness (Kp) and damping (Kd).
 *
 * @param  hcan     Pointer to FDCAN handle.
 * @param  motor_id Motor ID (1-16, corresponding to CAN ID 0x11-0x1F).
 * @param  pos      Target position in radians (range: P_MIN1 to P_MAX1,
 *                  typically -12.5 to 12.5 rad).
 * @param  vel      Target velocity in rad/s (range: V_MIN1 to V_MAX1,
 *                  typically -30.0 to 30.0 rad/s).
 * @param  kp       Position stiffness coefficient (range: KP_MIN1 to KP_MAX1,
 *                  typically 0.0 to 500.0).
 * @param  kd       Velocity damping coefficient (range: KD_MIN1 to KD_MAX1,
 *                  typically 0.0 to 5.0).
 * @param  torq     Target torque in N·m (range: T_MIN1 to T_MAX1,
 *                  typically -10.0 to 10.0 N·m).
 *
 * @retval None
 *
 * @note   CAN ID = motor_id + MIT_MODE (e.g., motor_id=1 results in
 *         CAN ID = 0x001)
 * @note   Parameters are automatically clamped to valid ranges by
 *         float_to_uint() function.
 * @note   Data frame format (8 bytes):
 *         - Byte 0-1: Position (16 bits)
 *         - Byte 2: Velocity high 8 bits
 *         - Byte 3: Velocity low 4 bits | Kp high 4 bits
 *         - Byte 4: Kp low 8 bits
 *         - Byte 5: Kd high 4 bits
 *         - Byte 6: Kd low 4 bits | Torque high 4 bits
 *         - Byte 7: Torque low 8 bits
 */
void mit_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
              float kd, float torq) {
  if (hcan == NULL) {
    return; /* Safety check */
  }

  uint8_t data[8] = {0};
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  /* Step 1: Quantize float parameters to integers */
  pos_tmp = float_to_uint(pos, P_MIN1, P_MAX1, 16);
  vel_tmp = float_to_uint(vel, V_MIN1, V_MAX1, 12);
  kp_tmp = float_to_uint(kp, KP_MIN1, KP_MAX1, 12);
  kd_tmp = float_to_uint(kd, KD_MIN1, KD_MAX1, 12);
  tor_tmp = float_to_uint(torq, T_MIN1, T_MAX1, 12);

  /* Step 2: Pack data into 8-byte frame */
  data[0] = (pos_tmp >> 8); /* Position high byte */
  data[1] = pos_tmp;        /* Position low byte */
  data[2] = (vel_tmp >> 4); /* Velocity high 8 bits */
  data[3] = ((vel_tmp & 0xF) << 4) |
            (kp_tmp >> 8); /* Velocity low 4 bits | Kp high 4 bits */
  data[4] = kp_tmp;        /* Kp low 8 bits */
  data[5] = (kd_tmp >> 4); /* Kd high 4 bits */
  data[6] = ((kd_tmp & 0xF) << 4) |
            (tor_tmp >> 8); /* Kd low 4 bits | Torque high 4 bits */
  data[7] = tor_tmp;        /* Torque low 8 bits */

  /* Step 3: Send control command via CAN */
  canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	pos_speed_ctrl:
* @param[in]:   hcan:
* @param[in]:   motor_id:
* @param[in]:   vel:
* @retval:     	void
* @details:
************************************************************************
**/
void pos_speed_ctrl(hcan_t *hcan, uint16_t motor_id, float pos, float vel) {
  uint16_t id;
  uint8_t *pbuf, *vbuf;
  uint8_t data[8];

  id = motor_id + POS_MODE;
  pbuf = (uint8_t *)&pos;
  vbuf = (uint8_t *)&vel;

  data[0] = *pbuf;
  data[1] = *(pbuf + 1);
  data[2] = *(pbuf + 2);
  data[3] = *(pbuf + 3);

  data[4] = *vbuf;
  data[5] = *(vbuf + 1);
  data[6] = *(vbuf + 2);
  data[7] = *(vbuf + 3);

  canx_send_data(hcan, id, data, 8);
}
/**
************************************************************************
* @brief:      	speed_ctrl:
* @param[in]:   hcan:
* @param[in]:   motor_id:
* @param[in]:   vel:
* @retval:     	void
* @details:
************************************************************************
**/
void speed_ctrl(hcan_t *hcan, uint16_t motor_id, float vel) {
  uint16_t id;
  uint8_t *vbuf;
  uint8_t data[4];

  id = motor_id + SPEED_MODE;
  vbuf = (uint8_t *)&vel;

  data[0] = *vbuf;
  data[1] = *(vbuf + 1);
  data[2] = *(vbuf + 2);
  data[3] = *(vbuf + 3);

  canx_send_data(hcan, id, data, 4);
}

// 4340
void mit_ctrl2(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN2, P_MAX2, 16);
  vel_tmp = float_to_uint(vel, V_MIN2, V_MAX2, 12);
  kp_tmp = float_to_uint(kp, KP_MIN2, KP_MAX2, 12);
  kd_tmp = float_to_uint(kd, KD_MIN2, KD_MAX2, 12);
  tor_tmp = float_to_uint(torq, T_MIN2, T_MAX2, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 6006
void mit_ctrl3(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN3, P_MAX3, 16);
  vel_tmp = float_to_uint(vel, V_MIN3, V_MAX3, 12);
  kp_tmp = float_to_uint(kp, KP_MIN3, KP_MAX3, 12);
  kd_tmp = float_to_uint(kd, KD_MIN3, KD_MAX3, 12);
  tor_tmp = float_to_uint(torq, T_MIN3, T_MAX3, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 8006
void mit_ctrl4(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN4, P_MAX4, 16);
  vel_tmp = float_to_uint(vel, V_MIN4, V_MAX4, 12);
  kp_tmp = float_to_uint(kp, KP_MIN4, KP_MAX4, 12);
  kd_tmp = float_to_uint(kd, KD_MIN4, KD_MAX4, 12);
  tor_tmp = float_to_uint(torq, T_MIN4, T_MAX4, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

// 3507
void mit_ctrl5(hcan_t *hcan, uint16_t motor_id, float pos, float vel, float kp,
               float kd, float torq) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = float_to_uint(pos, P_MIN5, P_MAX5, 16);
  vel_tmp = float_to_uint(vel, V_MIN5, V_MAX5, 12);
  kp_tmp = float_to_uint(kp, KP_MIN5, KP_MAX5, 12);
  kd_tmp = float_to_uint(kd, KD_MIN5, KD_MAX5, 12);
  tor_tmp = float_to_uint(torq, T_MIN5, T_MAX5, 12);

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

void mit_ctrl_test(hcan_t *hcan, uint16_t motor_id, Joint_Motor_t *motor) {
  uint8_t data[8];
  uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
  uint16_t id = motor_id + MIT_MODE;

  pos_tmp = motor->para.p_int_test;
  vel_tmp = motor->para.v_int_test;
  kp_tmp = motor->para.kp_int_test;
  kd_tmp = motor->para.kd_int_test;
  tor_tmp = motor->para.t_int_test;

  data[0] = (pos_tmp >> 8);
  data[1] = pos_tmp;
  data[2] = (vel_tmp >> 4);
  data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
  data[4] = kp_tmp;
  data[5] = (kd_tmp >> 4);
  data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
  data[7] = tor_tmp;

  canx_send_data(hcan, id, data, 8);
}

void dm4310_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN1, P_MAX1, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN1, V_MAX1, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN1, KP_MAX1, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN1, KD_MAX1, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN1, T_MAX1, 12);
}

void dm4340_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN2, P_MAX2, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN2, V_MAX2, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN2, KP_MAX2, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN2, KD_MAX2, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN2, T_MAX2, 12);
}

void dm6006_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN3, P_MAX3, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN3, V_MAX3, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN3, KP_MAX3, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN3, KD_MAX3, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN3, T_MAX3, 12);
}

void dm8006_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN4, P_MAX4, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN4, V_MAX4, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN4, KP_MAX4, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN4, KD_MAX4, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN4, T_MAX4, 12);
}

void dm10010l_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN6, P_MAX6, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN6, V_MAX6, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN6, KP_MAX6, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN6, KD_MAX6, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN6, T_MAX6, 12);
}

void dm6248p_fbdata_init(Joint_Motor_t *motor) {
  motor->para.p_int_test = float_to_uint(0.0f, P_MIN7, P_MAX7, 16);
  motor->para.v_int_test = float_to_uint(0.0f, V_MIN7, V_MAX7, 12);
  motor->para.kp_int_test = float_to_uint(0.0f, KP_MIN7, KP_MAX7, 12);
  motor->para.kd_int_test = float_to_uint(0.0f, KD_MIN7, KD_MAX7, 12);
  motor->para.t_int_test = float_to_uint(0.0f, T_MIN7, T_MAX7, 12);
}
