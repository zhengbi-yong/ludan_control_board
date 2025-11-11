#ifndef __FDCAN1_TASK_H
#define __FDCAN1_TASK_H

#include "fdcan_bus.h"
#include "main.h"
#include "motor_config.h"

extern void fdcan1_init(fdcan_bus_t *bus);
extern void fdcan1_task_(void);

#endif
