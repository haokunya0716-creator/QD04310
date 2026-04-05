//
// Created by gaoxu on 2026/4/5.
//

#ifndef QD4310_APP_QD4310_PID_H
#define QD4310_APP_QD4310_PID_H
#include "main.h"
#include "pid.h"
#include "delay.h"
void QD4310_PID_Init(void);
void QD4310_PID_Pro(void);
void QD4310_PID_Update_yaw(PID_TypeDef *PID);
void QD4310_PID_Update_pitch(PID_TypeDef *PID);
#endif //QD4310_APP_QD4310_PID_H