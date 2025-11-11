#ifndef __FDCAN2_TASK_H
#define __FDCAN2_TASK_H

#include "fdcan_bus.h"
#include "main.h"
#include "motor_config.h"

extern void fdcan2_init(fdcan_bus_t *bus);
extern void fdcan2_task_(void);

#endif
