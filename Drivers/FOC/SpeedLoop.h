#pragma once

/* 输出低通时间常数（s）：先用 2 ms；设为 0 可旁路滤波。 */
#define SPEED_LOOP_OUTPUT_FILTER_TAU_S 0.004f

/* Speed PI: rad/s feedback, volts output. */
typedef struct
{
    float kp;
    float ki;
    float voltage_limit;
    float integral;
    float filtered_voltage; /* 上一周期滤波后的 Uq（V） */
} SpeedLoop_t;

void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki, float voltage_limit);
void SpeedLoop_Reset(SpeedLoop_t *loop);
float SpeedLoop_Update(SpeedLoop_t *loop, float target_speed,
                       float measured_speed, float dt);
