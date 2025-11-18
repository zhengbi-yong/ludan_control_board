/**
 * @file bsp_PWM.c
 * @author Zhengbi Yong (zhengbi.yong@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2025-11-18
 * 
 * Zhengbi Yong
 * 
 */
#include "bsp_PWM.h"

void TIM_Set_PWM(TIM_HandleTypeDef *tim_pwmHandle, uint8_t Channel, uint16_t value)
{
    if (value > tim_pwmHandle->Instance->ARR)
        value = tim_pwmHandle->Instance->ARR;

    switch (Channel)
    {
    case TIM_CHANNEL_1:
        tim_pwmHandle->Instance->CCR1 = value;
        break;
    case TIM_CHANNEL_2:
        tim_pwmHandle->Instance->CCR2 = value;
        break;
    case TIM_CHANNEL_3:
        tim_pwmHandle->Instance->CCR3 = value;
        break;
    case TIM_CHANNEL_4:
        tim_pwmHandle->Instance->CCR4 = value;
        break;
    }
}
