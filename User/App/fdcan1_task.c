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
  joint_motor_init(&chassis->joint_motor[0], 1, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[1], 2, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[2], 3, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[3], 4, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[4], 5, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[5], 6, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[6], 7, MIT_MODE);
  osDelay(200);
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[0].para.id,
                      chassis->joint_motor[0].mode);
    osDelay(50);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[1].para.id,
                      chassis->joint_motor[1].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[2].para.id,
                      chassis->joint_motor[2].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[3].para.id,
                      chassis->joint_motor[3].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[4].para.id,
                      chassis->joint_motor[4].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[5].para.id,
                      chassis->joint_motor[5].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan1, chassis->joint_motor[6].para.id,
                      chassis->joint_motor[6].mode);
    osDelay(25);
  }
}