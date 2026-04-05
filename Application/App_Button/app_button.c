/*
* app_button.c
 *
 *  Created on: May 6, 2025
 *      Author: gaoxi
 */

#include "app_button.h"
#include "button.h"
//用定时器12中断来开启按键扫描（10ms一次），相关回调函数在delay.c中
static Button_TypeDef userKey; // 用户按钮


volatile uint8_t key_flag = 0;//


static void OnUserKey_Clicked1(uint8_t clicks);

//
// @简介：按钮初始化
//
void App_Button_Init(void)
{
    Button_InitTypeDef Button_InitStruct = {0};

    Button_InitStruct.GPIOx = GPIOA;
    Button_InitStruct.GPIO_Pin = GPIO_PIN_0;

    My_Button_Init(&userKey, &Button_InitStruct);

    My_Button_SetClickCb(&userKey, OnUserKey_Clicked1);
    /////////////////////////////////////////////////
}

void App_Button_Proc(void)
{
    My_Button_Proc(&userKey);

}
//
// @简介：按钮点击的回调函数
//
static void OnUserKey_Clicked1(uint8_t clicks)
{
    if(clicks == 1)
    {
        key_flag = 1;
    }else if (clicks == 2) {
        key_flag = 2;
    }else if (clicks == 3) {
        key_flag = 3;
    }

}