/*
* app_usart2.c
 *
 *  Created on: May 5, 2025
 *      Author: gaoxi
 */

#include <stdarg.h>
#include "main.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;

void App_USART1_Printf(const char* Format, ...)
{
    char format_buffer[128];

    va_list argptr;

    va_start(argptr, Format);

    vsprintf(format_buffer, Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart1, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}

//
// @简介：使用串口2打印格式化字符串
//用法与printf差不多
//
void App_USART2_Printf(const char* Format, ...)
{
    char format_buffer[128];

    va_list argptr;

    va_start(argptr, Format);

    vsprintf(format_buffer, Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart2, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}

void App_USART6_Printf(const char* Format, ...)
{
    char format_buffer[128];

    va_list argptr;

    va_start(argptr, Format);

    vsprintf(format_buffer, Format, argptr);

    va_end(argptr);

    HAL_UART_Transmit(&huart6, (uint8_t *)format_buffer, strlen(format_buffer), HAL_MAX_DELAY);
}
