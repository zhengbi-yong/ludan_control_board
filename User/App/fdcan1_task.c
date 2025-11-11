/**
  *********************************************************************
  * @file      fdcan1_task.c/h
  * @brief     该任务控制can1总线上的电机
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "fdcan1_task.h"
#include "cmsis_os2.h"
#include "fdcan.h"
#include "fdcan_bus.h"

#define MOTOR_ENABLE_MAX_RETRY 20   // 最多重试次数
#define MOTOR_ENABLE_INTERVAL_MS 25 // 每次重试间隔
// chassis_t chassis_move;

uint32_t CHASSR_TIME = 1;

void fdcan1_task_(void) {
  osDelay(500);
  fdcan1_init(&fdcan1_bus);

  dm6248p_fbdata_init(&fdcan1_bus.motor[0]);
  dm6248p_fbdata_init(&fdcan1_bus.motor[1]);
  dm6248p_fbdata_init(&fdcan1_bus.motor[2]);
  dm4340_fbdata_init(&fdcan1_bus.motor[3]);
  dm4340_fbdata_init(&fdcan1_bus.motor[4]);
  dm4340_fbdata_init(&fdcan1_bus.motor[5]);
  dm4340_fbdata_init(&fdcan1_bus.motor[6]);
  fdcan1_bus.start_flag = 1;
  while (1) {
    for (int i = 0; i < fdcan1_bus.motor_count; i++) {
      mit_ctrl_test(fdcan1_bus.hfdcan, i + 1, &fdcan1_bus.motor[i]);
    }
    osDelay(CHASSR_TIME);
  }
}

void fdcan1_init(fdcan_bus_t *bus) {
  // 初始化电机参数
  for (int i = 0; i < 7; i++) {
    joint_motor_init(&bus->motor[i], i + 1, MIT_MODE);
  }

  osDelay(100);

  // 遍历所有电机，逐个尝试使能
  for (int i = 0; i < 7; i++) {
    Joint_Motor_t *motor = &bus->motor[i];
    uint8_t enable_ok = 0;
    uint32_t retry_count = 0;

    while (!enable_ok && retry_count < MOTOR_ENABLE_MAX_RETRY) {
      enable_motor_mode(bus->hfdcan, motor->para.id, motor->mode);
      osDelay(MOTOR_ENABLE_INTERVAL_MS);

      // 反馈帧由 FDCAN 回调更新 motor->para.enabled / state
      if (motor->para.enabled == 1 || motor->para.state == 1) {
        enable_ok = 1;
      } else {
        retry_count++;
      }
    }

    // 若超出最大次数仍未成功，可在此处添加故障处理逻辑（如上报错误）
    osDelay(10); // 防止CAN总线拥堵
  }

  bus->start_flag = 1;
}