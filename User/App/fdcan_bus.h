#ifndef __FDCAN_BUS_H
#define __FDCAN_BUS_H

#include "fdcan.h"
#include "motor_config.h"

#define MAX_MOTORS_PER_BUS 16 // 每条总线最多电机数量，可按需求调整

typedef struct {
  FDCAN_HandleTypeDef *hfdcan; // 绑定的FDCAN句柄
  Joint_Motor_t motor[MAX_MOTORS_PER_BUS];
  uint8_t motor_count; // 当前电机数量
  uint8_t start_flag;
} fdcan_bus_t;

// 全局总线对象（两条总线）
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;

#endif
