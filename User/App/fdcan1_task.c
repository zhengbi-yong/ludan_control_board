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
#define MOTOR_ENABLE_MAX_RETRY 20   // 最多重试次数
#define MOTOR_ENABLE_INTERVAL_MS 25 // 每次重试间隔
chassis_t chassis_move;

uint32_t CHASSR_TIME = 1;

void fdcan1_task_(void) {
  osDelay(500);
  fdcan1_init(&chassis_move);

  dm6248p_fbdata_init(&chassis_move.joint_motor[0]);
  dm6248p_fbdata_init(&chassis_move.joint_motor[1]);
  dm6248p_fbdata_init(&chassis_move.joint_motor[2]);
  dm4340_fbdata_init(&chassis_move.joint_motor[3]);
  dm4340_fbdata_init(&chassis_move.joint_motor[4]);
  dm4340_fbdata_init(&chassis_move.joint_motor[5]);
  dm4340_fbdata_init(&chassis_move.joint_motor[6]);
  chassis_move.start_flag = 1;
  while (1) {
    mit_ctrl_test(&hfdcan1, 0x01, &chassis_move.joint_motor[0]);
    mit_ctrl_test(&hfdcan1, 0x02, &chassis_move.joint_motor[1]);
    mit_ctrl_test(&hfdcan1, 0x03, &chassis_move.joint_motor[2]);
    mit_ctrl_test(&hfdcan1, 0x04, &chassis_move.joint_motor[3]);
    mit_ctrl_test(&hfdcan1, 0x05, &chassis_move.joint_motor[4]);
    mit_ctrl_test(&hfdcan1, 0x06, &chassis_move.joint_motor[5]);
    mit_ctrl_test(&hfdcan1, 0x07, &chassis_move.joint_motor[6]);
    osDelay(CHASSR_TIME);
  }
}

void fdcan1_init(chassis_t *chassis) {
  // 初始化电机参数
  for (int i = 0; i < 7; i++) {
    joint_motor_init(&chassis->joint_motor[i], i + 1, MIT_MODE);
  }

  osDelay(100);

  // 遍历所有电机，逐个尝试使能
  for (int i = 0; i < 7; i++) {
    Joint_Motor_t *motor = &chassis->joint_motor[i];
    uint8_t enable_ok = 0;
    uint32_t retry_count = 0;

    while (!enable_ok && retry_count < MOTOR_ENABLE_MAX_RETRY) {
      enable_motor_mode(&hfdcan1, motor->para.id, motor->mode);
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

  chassis->start_flag = 1;
}