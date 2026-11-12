#ifndef _FOC_H_
#define _FOC_H_
#include "main.h"
#include <math.h>
#include "SVPWM.h"

#define FOC_VOLTAGE_LIMIT 1.5f    /* dq 电压矢量的幅值上限（V） */

/* 同一个矢量在 abc、αβ、dq 三种坐标系下的分量，电压计算时单位为 V。 */
typedef struct FOC_Vector
{
    float a, b, c;
    float alpha, beta;
    float d, q;
} FOC_Vector_t;

/* 控制模式：位置使用累计机械角度（rad），速度使用机械角速度（rad/s）。 */
typedef enum
{
    FOC_MODE_VOLTAGE = 1,    /* 直接输出 q 轴目标电压 */
    FOC_MODE_SPEED,          /* 速度 PI 输出 q 轴电压 */
    FOC_MODE_POSITION        /* 位置 P 与速度 PI 串级 */
} FOC_ControlMode_t;

typedef enum
{
    FOC_STATE_BOOT = 0,    /* 尚未启动 */
    FOC_STATE_STOPPED = 1,    /* 主动停止 */
    FOC_STATE_ALIGNING = 2,    /* 正在等待或施加对齐电压 */
    FOC_STATE_RUNNING = 3,    /* 正常运行 */
    FOC_STATE_SENSOR_FAULT = 4    /* 传感器读取故障 */
} FOC_State_t;

/* 显示与通信使用的反馈快照，由 FOC_GetFeedback 一次性复制。 */
typedef struct
{
    float speed_reference;    /* 当前生效的机械速度参考（rad/s）；电压模式为 0 */
    float position_reference;    /* 绝对位置目标（累计机械 rad） */
    float applied_uq;    /* 实际采用的 q 轴电压（V，软件限幅后） */
    float applied_ud;    /* 实际采用的 d 轴电压（V，软件限幅后） */
    FOC_ControlMode_t mode;    /* 当前控制模式 */
    float mechanical_angle;    /* 编码器累计机械角度（rad） */
    float mechanical_speed;    /* 编码器机械角速度（rad/s） */
    float mechanical_acceleration;    /* 滤波速度差分得到的机械角加速度（rad/s²） */
    float electrical_angle;    /* 用于换相的电角度（rad，范围 [0, 2π)） */
    FOC_State_t state;    /* 启动、运行或故障状态，与 VOFA I8 对应 */
    uint32_t sensor_fail_count;    /* 累计读取失败次数，与 VOFA I9 对应 */
    uint32_t sensor_status;         /* HAL 返回值：0=OK，1=ERROR，2=BUSY，3=TIMEOUT */
    uint32_t sensor_error;          /* I2C 错误位掩码，故障时保存 */
    uint32_t sensor_fault_stage;    /* 0=无，1=首次读取，2=对齐后，3=运行中 */
} FOC_Feedback_t;

/********************************************
	FOC 对外接口使用约定
	功能：电机任务负责调用启动、周期更新、停止和目标设置接口
	输入：now_ms 为同一 uint32_t 毫秒时钟，如 HAL_GetTick()
	输出：显示与通信任务通过 FOC_GetFeedback 读取反馈
	说明：FOC_Start 非阻塞；FOC_Update 返回 1 才表示完成一个正常控制周期
*******************************************/
void FOC_Start(uint32_t now_ms);
uint8_t FOC_Update(uint32_t now_ms);
void FOC_Stop(void);
void FOC_GetFeedback(FOC_Feedback_t *feedback);
void FOC_Init(void);
void FOC_SetControlMode(FOC_ControlMode_t mode);
void FOC_SetTargetVoltage(float voltage);
void FOC_SetTargetSpeed(float speed_rad_s);
void FOC_SetTargetPosition(float position_rad);
void FOC_MotorControl(float theta_e, float mechanical_angle,
                      float mechanical_speed, float dt);

void Inv_Clark_transformer(FOC_Vector_t *cu);
void Inv_Pake_transformer(FOC_Vector_t *cu, float theta_e);
void FOC_SetVoltage(float ud, float uq, float theta_e);
void FOC_SetAlignVoltage(float ud, float theta);
void FOC_CalibrateZero(float mechanical_angle);
#endif
