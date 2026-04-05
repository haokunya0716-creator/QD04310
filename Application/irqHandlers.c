/**
* @param 本次工程用大疆C板的USART1来和串口屏通信，USART6来和相机通信，VOFA+可以在两者中有一个不用时做调试作用
* 而且好像USB协议可以用虚拟串口来和PC连接实现调试，以后可以试试
* @param 回调函数放在这里
*/

#include "irqHandlers.h"
#include <string.h>
#include "QD4310.h"
#include "task.h"
#include "usart.h"
#include "vofa.h"
#include "HMI.h"
#include "mytask.h"
#include "vision_protocol.h"
#include "app_button.h"
#include "app_QD4310_PID.h"
#include "tim.h"
#include "App_QD4310_PID/app_QD4310_PID.h"

//把任务状态枚举放到前面，供全局使用，当串口屏的按键按下时，会对应到相应的状态，从而执行不同的任务
typedef enum {
    SYS_IDLE = 0,     // 待机/急停
    SYS_TASK1,        // 1. 目标捕获与稳定跟踪
    SYS_TASK2,        // 2. 多目标顺序打靶
    SYS_TASK4,        // 4. 动态靶心追踪 (发挥1)
    SYS_TASK3,        // 3. 指定图形绘制
    SYS_TASK5,        // 5. 动态轨迹同步绘制 (发挥2)
    SYS_DISABLE,
    SYS_ENABLE,
} SystemState_t;
volatile float pid_p = 0;
SystemState_t sys_state = SYS_IDLE; // 默认开机待机
//声明外部的屏幕接收缓存 (在 HMI.c 里定义)
extern uint8_t hmi_rx_buf[64];

//接口初始化函数
void DebugTask_Init() {
    //VOFA_Init(&huart6); // 初始化VOFA+协议，绑定USART6
    QD4310_PID_Init();//初始化无刷电机
    HMI_Init();         //串口屏初始化

    App_Button_Init();//初始化按键
    HAL_TIM_Base_Start_IT(&htim12);//开启定时器12中断，用来扫描按键
    HAL_TIM_Base_Start_IT(&htim11);//开启定时器11中断，用来微秒级计时

    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);//开启蜂鸣器pwm
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);//把蜂鸣器占空比设为0


}
/**
 * @简介： 核心任务代码,记得之后把所有的任务封装成函数（函数的编写可以单独创建一个mytask.c/.h文件）写在这里的case里
 * 记住不要命名为task.c/.h，有命名风险,嘿嘿我已经建了
 * 还有在串口屏显示数据的代码不要放在10HZ频率的任务里，放在100Hz的任务里就好怕卡屏
 */
void DebugTask_Run() {
        //电机控制任务 (100Hz),根据屏幕按键的状态来决定电机干嘛
        PERIODIC_START(MotorTask, 10)
            switch (sys_state) {
            case SYS_IDLE:
                    // 待机或急停：电机速度设为0
                    break;
            case SYS_TASK1:
                    // 自动识别并对准A4纸中心，保持5s
                     //自己看任务编写
                    QD4310_SetLowSpeed(&YawMotor, 10);
                    break;
            case SYS_TASK2:
                    // 顺序打靶逻辑 (圆形->方形->星形)
                    QD4310_SetSpeed(&YawMotor, 10);//写这个是为了验证串口屏和电机能正常使用，不是任务！
                    break;
            case SYS_TASK3:
                    // 沿胶带顺时针走一周 (30s)
                    break;
            case SYS_TASK4:
                    // 动态靶心追踪 (加入前馈/预测算法)
                    break;
            case SYS_TASK5:
                    // 动态画圆 (同步 6cm 半径)
                    break;
            case SYS_DISABLE:
                    QD4310_Disable(&YawMotor);
                    QD4310_Disable(&PitchMotor);
                    HAL_Delay(20);
                    break;
            case SYS_ENABLE:
                    QD4310_Enable(&YawMotor);
                    QD4310_Enable(&PitchMotor);
                    HAL_Delay(20);
                    break;
            }
        PERIODIC_END

    //VOFA+显示波形（记住要开justfloat协议以及浮点数输出）
    //当然这里已经开了
    PERIODIC_START(VofaTask, 20)
        // 参数1: 当前角度 (乘以 RAD 转回 0~360度，VOFA上看波形更直观)
        // 参数2: 当前转速 (rpm)
        // 参数3: 当前电流 (A)
    VOFA_Send_JustFloat(YawMotor.angle * RAD, YawMotor.speed, YawMotor.current);
    PERIODIC_END

    //串口屏(USART1)显示数据
    PERIODIC_START(HMITask, 100)
    // 获取电机的当前角度，并转为度数显示
    HMI_SetFloat("Yaw_Angle", YawMotor.angle * RAD);// 更新 Yaw_Angle，把弧度制改成角度单位，人性化
    HMI_SetFloat("Pitch_Angle", PitchMotor.angle * RAD);
    //HMI_SetFloat("t2", pitch_deg); // 更新 Pitch_Angle
    PERIODIC_END
}
//初始化CAN配置
void CAN_InterfaceInit() {
    //CAN滤波器配置
    CAN_FilterTypeDef sFilterConfig;
    sFilterConfig.FilterBank = 0;
    sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdLow = 0x0000;
    sFilterConfig.FilterIdHigh = 0x0000;
    sFilterConfig.FilterMaskIdLow = 0x0000;
    sFilterConfig.FilterMaskIdHigh = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation = ENABLE;
    sFilterConfig.SlaveStartFilterBank = 14;
    //这里已经开启了滤波器配置
    if (HAL_CAN_ConfigFilter(&hcan1, &sFilterConfig) != HAL_OK) Error_Handler();
    //这里已经开启了CAN
    if (HAL_CAN_Start(&hcan1) != HAL_OK) Error_Handler();
    //这里已经开启了CAN的接收中断
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) Error_Handler();
}
//CAN接收回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    if (hcan == &hcan1) {
        CAN_RxHeaderTypeDef rx_header;
        uint8_t rx_data[8];
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
        //确认是正确的ID
        if (rx_header.StdId >= 0x500 && rx_header.StdId <= 0x508) {
            if (rx_header.StdId == 0x500) {
                QD4310_Update(&YawMotor, rx_data); // 更新云台yaw轴的状态
            } else if (rx_header.StdId == 0x501) {
                //这里则是更新云台pitch的状态
                QD4310_Update(&PitchMotor, rx_data); // 更新云台pitch轴的状态
            }
        }
    }
}


//串口空闲中断回调：在这里把屏幕的字符串变成“普通按键”来触发状态切换
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart1) {
        hmi_rx_buf[Size] = '\0';
        char* buf = (char*)hmi_rx_buf;

        if (strstr(buf, "CMD_T1"))      { sys_state = SYS_TASK1; HMI_SetText("State", "Task 1"); }
        else if (strstr(buf, "CMD_T2")) { sys_state = SYS_TASK2; HMI_SetText("State", "Task 2"); }
        else if (strstr(buf, "CMD_T3")) { sys_state = SYS_TASK3; HMI_SetText("State", "Task 3"); }
        else if (strstr(buf, "CMD_T4")) { sys_state = SYS_TASK4; HMI_SetText("State", "Task 4"); }
        else if (strstr(buf, "CMD_T5")) { sys_state = SYS_TASK5; HMI_SetText("State", "Task 5"); }
        else if (strstr(buf, "CMD_DISABLE")) { sys_state = SYS_DISABLE; HMI_SetText("State", "DISABLE"); }
        else if (strstr(buf, "CMD_ENABLE")) { sys_state = SYS_DISABLE; HMI_SetText("State", "ENABLE"); }
        else if (strstr(buf, "CMD_STOP")) {
            sys_state = SYS_IDLE;
            QD4310_SetSpeed(&YawMotor, 0);
            QD4310_SetSpeed(&PitchMotor, 0);
            HMI_SetText("State", "EMERGENCY STOP");
        }

        HAL_UARTEx_ReceiveToIdle_DMA(&huart1, hmi_rx_buf, sizeof(hmi_rx_buf));
    }
    else if (huart == &huart6) {  //相机数据
        Vision_RxCallback(huart, Size);  // 调用协议栈处理
    }
}







