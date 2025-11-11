/**
  *********************************************************************
  * @file      VBUS_Check_task.c/h
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
#include "vbus_check.h"
#include "adc.h"
#include "cmsis_os.h"
#include "gpio.h"

#include "cmsis_os.h"
#include "fdcan.h"
#include "fdcan1_task.h"
#include "fdcan_bus.h"
#include "tim.h"
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;

static uint16_t adc_val[2];
static float vbus;

static uint16_t calibration_value = 378;

uint8_t loss_voltage = 0;

#define vbus_threhold_disable (22.2f)

#define vbus_threhold_call (22.6f)

void VBUS_Check_task(void) {

  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2);

  while (1) {
    vbus = ((adc_val[1] + calibration_value) * 3.3f / 65535) * 11.0f;

    if (6.0f < vbus && vbus < vbus_threhold_call) {
      Buzzer_ON();
    } else {
      Buzzer_OFF();
    }
    if (6.0f < vbus && vbus < vbus_threhold_disable) {
      loss_voltage = 1;
      Power_OUT2_OFF();
      Power_OUT1_OFF();

      for (int j = 0; j < 7; j++) {
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[0].para.id,
                           fdcan1_bus.motor[0].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[1].para.id,
                           fdcan1_bus.motor[1].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[2].para.id,
                           fdcan1_bus.motor[2].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[3].para.id,
                           fdcan1_bus.motor[3].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[4].para.id,
                           fdcan1_bus.motor[4].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[5].para.id,
                           fdcan1_bus.motor[5].mode);
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[6].para.id,
                           fdcan1_bus.motor[6].mode);
        osDelay(5);
      }
      for (int j = 0; j < 7; j++) {
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[0].para.id,
                           fdcan2_bus.motor[0].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[1].para.id,
                           fdcan2_bus.motor[1].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[2].para.id,
                           fdcan2_bus.motor[2].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[3].para.id,
                           fdcan2_bus.motor[3].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[4].para.id,
                           fdcan2_bus.motor[4].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[5].para.id,
                           fdcan2_bus.motor[5].mode);
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[6].para.id,
                           fdcan2_bus.motor[6].mode);
        osDelay(5);
      }
      for (int j = 0; j < 7; j++) {
        // disable_motor_mode(&hfdcan3, robot_body.loin_motor.para.id,
        //                    robot_body.loin_motor.mode);
        osDelay(5);
      }
    } else {
      Power_OUT2_ON();
      Power_OUT1_ON();

      loss_voltage = 0;
    }

    osDelay(100);
  }
}
