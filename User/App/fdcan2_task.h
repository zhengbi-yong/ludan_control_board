#ifndef __FDCAN2_TASK_H
#define __FDCAN2_TASK_H

#include "fdcan1_task.h"
#include "main.h"
#include "motor_config.h"

extern void fdcan2_task_(void);

extern void fdcan2_init(chassis_t *chassis);

#endif
