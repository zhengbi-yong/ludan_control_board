/**
 * @file fdcan2_task.h
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-18
 * 
 * Zhengbi Yong
 * 
 */
#ifndef __FDCAN2_TASK_H
#define __FDCAN2_TASK_H

#include "fdcan_bus.h"
#include "main.h"
#include "motor_config.h"

extern void fdcan2_init(fdcan_bus_t *bus);
extern void fdcan2_task_(void);

#endif
