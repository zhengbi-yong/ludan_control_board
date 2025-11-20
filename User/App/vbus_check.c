/**
 * @file vbus_check.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief  Voltage monitoring and protection task implementation.
 * @version 0.1
 * @date 2025-11-18
 *
 * @note   This file implements the FreeRTOS task for real-time voltage
 *         monitoring and protection. It uses ADC with DMA to continuously
 *         sample voltage and triggers protection actions when voltage falls
 *         below safe thresholds.
 *
 * Zhengbi Yong
 *
 */
#include "vbus_check.h"
#include "adc.h"
#include "cmsis_os.h"
#include "fdcan.h"
#include "fdcan_bus.h"
#include "gpio.h"
#include "motor_config.h"
#include "tim.h" /* Must be before gpio.h for htim12 declaration */

/* External declarations */
extern fdcan_bus_t fdcan1_bus;
extern fdcan_bus_t fdcan2_bus;
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;

/* Static variables */
static uint16_t adc_val[2] = {0}; /* ADC DMA buffer (2 channels) */
static float vbus = 0.0f;         /* Calculated bus voltage */

/**
 * @brief  ADC calibration value for voltage calculation.
 *
 * @note   This value compensates for ADC offset and is determined through
 *         calibration. Default value is 378.
 */
static uint16_t calibration_value = 378;

/**
 * @brief  Global voltage loss flag.
 *
 * @note   Set to 1 when voltage falls below vbus_threhold_disable.
 *         Other tasks can check this flag to implement voltage-aware behavior.
 */
uint8_t loss_voltage = 0;

/**
 * @brief  Voltage threshold for power disable (volts).
 *
 * @note   When voltage falls below this threshold, power outputs are
 *         disabled and all motors are stopped for safety.
 */
#define vbus_threhold_disable (22.2f)

/**
 * @brief  Voltage threshold for buzzer alarm (volts).
 *
 * @note   When voltage falls below this threshold but above disable
 *         threshold, buzzer is activated to alert the user.
 */
#define vbus_threhold_call (22.6f)

/**
 * @brief  Minimum valid voltage (volts).
 *
 * @note   Voltage below this value is considered invalid and protection
 *         actions are not triggered.
 */
#define VBUS_MIN_VALID (6.0f)

/**
 * @brief  Voltage monitoring task main function.
 *
 * @note   This function implements the voltage monitoring and protection
 *         task. It continuously monitors system voltage and triggers protection
 *         actions when voltage falls below safe thresholds.
 *
 * @retval None
 *
 * @note   Voltage calculation formula:
 *         vbus = ((adc_val[1] + calibration_value) * 3.3V / 65535) * 11.0
 *         - adc_val[1]: ADC channel 1 raw value (voltage divider input)
 *         - calibration_value: ADC offset calibration (default: 378)
 *         - 3.3V: ADC reference voltage
 *         - 65535: Maximum ADC value (16-bit)
 *         - 11.0: Voltage divider ratio
 *
 * @note   Protection logic:
 *         - 6.0V < vbus < 22.6V: Buzzer alarm (low voltage warning)
 *         - 6.0V < vbus < 22.2V: Critical low voltage
 *           - Set loss_voltage = 1
 *           - Disable power outputs (Power_OUT1_OFF, Power_OUT2_OFF)
 *           - Disable all motors on FDCAN1 and FDCAN2
 *         - vbus >= 22.2V: Normal operation
 *           - Clear loss_voltage = 0
 *           - Enable power outputs (Power_OUT1_ON, Power_OUT2_ON)
 *
 * @note   Monitoring frequency: 100Hz (10ms period)
 */
void VBUS_Check_task(void) {
  /* Step 1: Initialize ADC calibration */
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

  /* Step 2: Start ADC DMA for continuous voltage sampling */
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_val, 2);

  /* Step 3: Enter voltage monitoring loop */
  while (1) {
    /* Calculate bus voltage from ADC value */
    /* Formula: vbus = ((ADC_raw + calibration) * Vref / ADC_max) *
     * divider_ratio */
    vbus = ((adc_val[1] + calibration_value) * 3.3f / 65535.0f) * 11.0f;

    /* Step 4: Check voltage thresholds and trigger protection actions */

    /* Low voltage alarm: 6.0V < vbus < 22.6V */
    if (VBUS_MIN_VALID < vbus && vbus < vbus_threhold_call) {
      Buzzer_ON(); /* Activate buzzer alarm */
    } else {
      Buzzer_OFF(); /* Normal voltage, turn off buzzer */
    }

    /* Critical low voltage protection: 6.0V < vbus < 22.2V */
    if (VBUS_MIN_VALID < vbus && vbus < vbus_threhold_disable) {
      /* Set voltage loss flag */
      loss_voltage = 1;

      /* Disable power outputs for safety */
      Power_OUT2_OFF();
      Power_OUT1_OFF();

      /* Disable all motors on FDCAN1 bus */
      for (int i = 0; i < fdcan1_bus.motor_count; i++) {
        disable_motor_mode(&hfdcan1, fdcan1_bus.motor[i].para.id,
                           fdcan1_bus.motor[i].mode);
        osDelay(5); /* Delay between motors to prevent CAN bus congestion */
      }

      /* Disable all motors on FDCAN2 bus */
      for (int i = 0; i < fdcan2_bus.motor_count; i++) {
        disable_motor_mode(&hfdcan2, fdcan2_bus.motor[i].para.id,
                           fdcan2_bus.motor[i].mode);
        osDelay(5); /* Delay between motors to prevent CAN bus congestion */
      }
    } else {
      /* Normal voltage: vbus >= 22.2V */
      /* Enable power outputs */
      Power_OUT2_ON();
      Power_OUT1_ON();

      /* Clear voltage loss flag */
      loss_voltage = 0;
    }

    /* Delay to maintain 100Hz monitoring frequency (10ms period) */
    osDelay(10);
  }
}
