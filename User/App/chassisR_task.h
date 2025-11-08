#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "motor_config.h"

typedef struct {
  Joint_Motor_t joint_motor[14];

  uint8_t start_flag; // ������־

} chassis_t;

extern void ChassisR_init(chassis_t *chassis);
extern void ChassisR_task(void);

extern void mySaturate(float *in, float min, float max);

#endif
