//
// Created by gaoxu on 2026/4/5.
//

#include "app_QD4310_PID.h"
#include "task.h"
#include "QD4310.h"
#include "vision_protocol.h"

uint8_t valid = 0;
volatile float laser_current_yaw = 0;//激光笔现在的水平坐标
volatile float laser_current_yaw_target = 0;//设定的水平目标
volatile float laser_current_pitch = 0;//现在的竖直位置
volatile float laser_current_pitch_target = 0;//设定的竖直位置

static PID_TypeDef pid_motor_pitch; // pitch电机调速系统的PID控制器
static PID_TypeDef pid_motor_yaw; // yaw调速系统的PID控制器

void QD4310_PID_Init(void) {

    //////////////////////5    CAN_InterfaceInit(); //CAN初始化
    QD4310_Init(&YawMotor, &hcan1, 0x00); // 初始化云台yaw电机
    QD4310_Init(&PitchMotor, &hcan1, 0x01);//初始化pitch轴
    HAL_Delay(100);
    // QD4310_Enable(&YawMotor);// 使能电机
    // QD4310_Enable(&PitchMotor);
    HAL_Delay(20);

    PID_Init(&pid_motor_pitch, 0, 0, 0);
    PID_LimitConfig(&pid_motor_pitch, +1000.0f, -1000.0f);
    PID_Init(&pid_motor_yaw, 00,00 , 0);
    PID_LimitConfig(&pid_motor_yaw, +1000.0f, -1000.0f);
}
void QD4310_PID_Pro(void) {
    PERIODIC_START(QD4310PID,2)//执行周期为2ms，500Hz
    QD4310_Vaild_Update();
    QD4310_PID_Update_yaw(&pid_motor_yaw);//先更新yaw轴和pitch轴的数据
    QD4310_PID_Update_pitch(&pid_motor_pitch);
    //再进行pid计算
    if (valid == 0) {
        QD4310_SetSpeed(&YawMotor,0);
        QD4310_SetSpeed(&PitchMotor,0);
        PID_Reset(&pid_motor_yaw);
        PID_Reset(&pid_motor_pitch);
    }else {
        float yaw_speed = PID_Compute(&pid_motor_yaw, laser_current_yaw);
        QD4310_SetSpeed(&YawMotor,yaw_speed);

        float pitch_speed = PID_Compute(&pid_motor_pitch, laser_current_pitch);
        QD4310_SetSpeed(&PitchMotor,pitch_speed);
    }
    PERIODIC_END
}



void QD4310_Vaild_Update(void) {
    __disable_irq();
    valid = g_vision.valid;//有效值，0代表相机未识别到图形
    __enable_irq();
}
void QD4310_PID_Update_yaw(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_yaw = g_vision.dx;
    __enable_irq();
    laser_current_yaw_target = 0.0f;//视觉发送的坐标

    PID->SP = laser_current_yaw_target;
}
void QD4310_PID_Update_pitch(PID_TypeDef *PID) {
    __disable_irq();
    laser_current_pitch = g_vision.dy;
    __enable_irq();

    laser_current_pitch_target = 0.0f;

    PID->SP = laser_current_pitch_target;
}


