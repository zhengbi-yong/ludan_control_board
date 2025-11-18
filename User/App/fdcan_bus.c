/**
 * @file fdcan_bus.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-18
 * 
 * Zhengbi Yong
 * 
 */
#include "fdcan_bus.h"

// 声明外部FDCAN句柄
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

// 定义两条总线的全局对象
fdcan_bus_t fdcan1_bus = {
    .hfdcan = &hfdcan1,
    .motor_count = 15,
    .start_flag = 0,
};

fdcan_bus_t fdcan2_bus = {
    .hfdcan = &hfdcan2,
    .motor_count = 15,
    .start_flag = 0,
};
