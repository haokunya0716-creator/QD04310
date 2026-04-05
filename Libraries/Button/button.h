/**
  ******************************************************************************
  * @file    button.h
  * @author  铁头山羊
  * @version V 1.0.0
  * @date    2022年9月7日
  * @brief   按钮驱动程序
  ******************************************************************************
  */

#ifndef _BUTTON_H_
#define _BUTTON_H_

#include "main.h"

typedef struct
{
	GPIO_TypeDef *GPIOx;   /* 按钮的端口号。
	                              可以取GPIOA..D中的一个*/

	uint32_t GPIO_Pin;     /* 按钮的引脚编号。
	                              可以取GPIO_PIN_0..15中的一个 */
} Button_InitTypeDef;

typedef struct
{
	/* 初始化参数 */
	GPIO_TypeDef *GPIOx;
	uint16_t GPIO_Pin;
	void (*button_pressed_cb)(void);
	void (*button_released_cb)(void);
	void (*button_clicked_cb)(uint8_t clicks);
	void (*button_long_pressed_cb)(uint8_t ticks);
	uint32_t LongPressThreshold;
	uint32_t LongPressTickInterval;
	uint32_t ClickInterval;

	uint8_t  LastState;     // 按钮上次的状态，0 - 松开，1 - 按下
	uint8_t  ChangePending; // 按钮的状态是否正在发生改变
	uint32_t PendingTime;   // 按钮状态开始变化的时间

	uint32_t LastPressedTime;  // 按钮上次按下的时间
	uint32_t LastReleasedTime; // 按钮上次松开的时间

	uint8_t LongPressTicks;
	uint32_t LastLongPressTickTime;

	uint8_t ClickCnt;

} Button_TypeDef;

void My_Button_Init(Button_TypeDef *Button, Button_InitTypeDef *Button_InistStruct);
void My_Button_Proc(Button_TypeDef *Button);
uint8_t MyButton_GetState(Button_TypeDef *Button);
//设置长按触发的函数
void My_Button_SetLongPressCb(Button_TypeDef *Button, void (*LongPressCb)(uint8_t ticks));
/*长按：ticks 代表的是“持续时间”，而不是“长按了第几次”
对于长按（button_long_pressed_cb）
含义：ticks 并不是说你“长按了几次”，是指在同一次长按动作中，时间流逝的刻度。
逻辑：
你按住按钮不松开。
达到 LongPressThreshold（如1秒）后，触发第一次回调，此时 ticks = 1。
如果你继续按住不松手，每隔 LongPressTickInterval（如100ms），回调函数就会再次被调用，ticks 变成 2, 3, 4, 5...
一旦你松开手，ticks 立即清零。
例子：
你按住不放持续了 2 秒，回调函数可能会被触发 10 次，ticks 从 1 增加到 10。
用途：常用于连续加减。比如调时钟或音量，按住加号键不放，数值会随着 ticks 的增加而快速增长（1, 2, 3, 4...）。
*/

//设置按下瞬间触发的函数
void My_Button_SetPressCb(Button_TypeDef *Button, void (*button_pressed_cb)(void));

//设置松开瞬间触发的函数
void My_Button_SetReleaseCb(Button_TypeDef *Button, void (*button_released_cb)(void));

//设置单击/连击触发的函数
void My_Button_SetClickCb(Button_TypeDef *Button, void (*button_clicked_cb)(uint8_t clicks));

void My_Button_ClickIntervalConfig(Button_TypeDef *Button, uint32_t Interval);
void My_Button_LongPressConfig(Button_TypeDef *Button, uint32_t Throshold, uint32_t TickInterval);


#endif
