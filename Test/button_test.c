//
// Created by gaoxu on 2026/4/1.
//

#include "button_test.h"

#include "tim.h"
extern volatile uint8_t key_flag;
void Button_Test(void) {
    ///////////////////////开启三色灯pwm
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);//蓝
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);//绿
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);//红
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, 0);
    uint32_t arr_pwm = __HAL_TIM_GET_AUTORELOAD(&htim5) ;
    while(1) {
        if (key_flag == 1) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,arr_pwm / 5.0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,0);
        }else if(key_flag == 2) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,arr_pwm / 5.0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,0);
        }else if(key_flag == 3) {
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3,arr_pwm / 5.0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,0 );
            __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,0);
        }
    }
}
