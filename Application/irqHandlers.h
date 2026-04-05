//
// Created by ASUS on 2026/3/28.
//

#ifndef QD4310_IRQHANDLERS_H
#define QD4310_IRQHANDLERS_H
#include <stdint.h>

extern uint8_t task1_flag;
void CAN_InterfaceInit();
void DebugTask_Init();
void DebugTask_Run();

#endif //QD4310_IRQHANDLERS_H