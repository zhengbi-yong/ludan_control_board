#ifndef __FDCAN1_TASK_H
#define __FDCAN1_TASK_H

#include "main.h"
#include "motor_config.h"

typedef struct {
  Joint_Motor_t joint_motor[14];

  uint8_t start_flag;

} chassis_t;

extern void fdcan1_init(chassis_t *chassis);
extern void fdcan1_task_(void);

#endif
