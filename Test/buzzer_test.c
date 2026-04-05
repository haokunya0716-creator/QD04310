//
// Created by gaoxu on 2026/4/1.
//

#include "buzzer_test.h"

#include <stdint.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"
#include "tim.h"

void Buzzer_Test(void) {
    uint32_t arr_buzzer = __HAL_TIM_GET_AUTORELOAD(&htim4);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, (arr_buzzer+1)/5.0);
    HAL_Delay(200);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);

}
