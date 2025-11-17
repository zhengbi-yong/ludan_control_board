/**
  *********************************************************************
  * @file      observe_task.c/h
  * @brief
  * @note
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */

#include "observe_task.h"

#include "bsp_usart1.h"
#include "cmsis_os.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "usbd_core.h"
extern UART_HandleTypeDef huart1;
extern send_data_t send_data;

#define FRAME_LENGTH 152
#define LOAD_LENGTH 151
// extern chassis_t chassis_move;
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;

uint32_t OBSERVE_TIME = 5;

void observe_task_(void) {
  while (1) {
    send_data.tx[0] = FRAME_HEADER;
    int idx = 0; // 记录全局电机索引，用于帧打包偏移

    // 遍历 FDCAN1 上的电机
    for (int i = 0; i < fdcan1_bus.motor_count; i++) {
      send_data.tx[1 + idx * 5] = fdcan1_bus.motor[i].para.p_int >> 8;
      send_data.tx[2 + idx * 5] = fdcan1_bus.motor[i].para.p_int;
      send_data.tx[3 + idx * 5] = fdcan1_bus.motor[i].para.v_int >> 4;
      send_data.tx[4 + idx * 5] =
          ((fdcan1_bus.motor[i].para.v_int & 0x0F) << 4) |
          (fdcan1_bus.motor[i].para.t_int >> 8);
      send_data.tx[5 + idx * 5] = fdcan1_bus.motor[i].para.t_int;
      idx++;
    }

    // 遍历 FDCAN2 上的电机
    for (int i = 0; i < fdcan2_bus.motor_count; i++) {
      send_data.tx[1 + idx * 5] = fdcan2_bus.motor[i].para.p_int >> 8;
      send_data.tx[2 + idx * 5] = fdcan2_bus.motor[i].para.p_int;
      send_data.tx[3 + idx * 5] = fdcan2_bus.motor[i].para.v_int >> 4;
      send_data.tx[4 + idx * 5] =
          ((fdcan2_bus.motor[i].para.v_int & 0x0F) << 4) |
          (fdcan2_bus.motor[i].para.t_int >> 8);
      send_data.tx[5 + idx * 5] = fdcan2_bus.motor[i].para.t_int;
      idx++;
    }

    send_data.tx[LOAD_LENGTH] = Check_Sum(LOAD_LENGTH, send_data.tx);
    CDC_Transmit_HS((uint8_t *)send_data.tx, FRAME_LENGTH);

    osDelay(OBSERVE_TIME);
  }
}
