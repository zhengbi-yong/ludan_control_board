/**
  *********************************************************************
  * @file      fdcan2_task.c/h
  * @brief     该任务控制can2总线上的电机
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "fdcan2_task.h"
#include "cmsis_os2.h"
#include "fdcan.h"

extern chassis_t chassis_move;

uint32_t CHASSL_TIME = 1;

void fdcan2_task_(void) {
  chassis_move.start_flag = 1;
  osDelay(500);
  fdcan2_init(&chassis_move);

  dm6248p_fbdata_init(&chassis_move.joint_motor[7]);
  dm6248p_fbdata_init(&chassis_move.joint_motor[8]);
  dm6248p_fbdata_init(&chassis_move.joint_motor[9]);
  dm4340_fbdata_init(&chassis_move.joint_motor[10]);
  dm4340_fbdata_init(&chassis_move.joint_motor[11]);
  dm4340_fbdata_init(&chassis_move.joint_motor[12]);
  dm4340_fbdata_init(&chassis_move.joint_motor[13]);

  while (1) {
    if (chassis_move.start_flag == 1) {
      mit_ctrl_test(&hfdcan2, 0x01, &chassis_move.joint_motor[7]);
      mit_ctrl_test(&hfdcan2, 0x02, &chassis_move.joint_motor[8]);
      mit_ctrl_test(&hfdcan2, 0x03, &chassis_move.joint_motor[9]);
      mit_ctrl_test(&hfdcan2, 0x04, &chassis_move.joint_motor[10]);
      mit_ctrl_test(&hfdcan2, 0x05, &chassis_move.joint_motor[11]);
      mit_ctrl_test(&hfdcan2, 0x06, &chassis_move.joint_motor[12]);
      mit_ctrl_test(&hfdcan2, 0x07, &chassis_move.joint_motor[13]);
    }
    osDelay(CHASSL_TIME);
  }
}

void fdcan2_init(chassis_t *chassis) {
  joint_motor_init(&chassis->joint_motor[7], 1, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[8], 2, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[9], 3, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[10], 4, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[11], 5, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[12], 6, MIT_MODE);
  joint_motor_init(&chassis->joint_motor[13], 7, MIT_MODE);

  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[7].para.id,
                      chassis->joint_motor[7].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[8].para.id,
                      chassis->joint_motor[8].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[9].para.id,
                      chassis->joint_motor[9].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[10].para.id,
                      chassis->joint_motor[10].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[11].para.id,
                      chassis->joint_motor[11].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[12].para.id,
                      chassis->joint_motor[12].mode);
    osDelay(25);
  }
  for (int j = 0; j < 10; j++) {
    enable_motor_mode(&hfdcan2, chassis->joint_motor[13].para.id,
                      chassis->joint_motor[13].mode);
    osDelay(25);
  }
}
