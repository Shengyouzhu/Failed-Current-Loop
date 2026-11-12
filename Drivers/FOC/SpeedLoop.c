#include "SpeedLoop.h"

static float SpeedLoop_Clamp(float value, float limit)
{
    if (value > limit) return limit;
    if (value < -limit) return -limit;
    return value;
}

void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki, float voltage_limit)
{
    loop->kp = kp;
    loop->ki = ki;
    loop->voltage_limit = voltage_limit;
    SpeedLoop_Reset(loop);
}

void SpeedLoop_Reset(SpeedLoop_t *loop)
{
    loop->integral = 0.0f;
    /* 启动、停机及模式切换时，不保留上一模式的输出。 */
    loop->filtered_voltage = 0.0f;
}

float SpeedLoop_Update(SpeedLoop_t *loop, float target_speed,
                       float measured_speed, float dt)
{
    float error = target_speed - measured_speed;
    float proposed_integral;
    float proposed_voltage;
    float voltage;
    float alpha;

    if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;
    proposed_integral = SpeedLoop_Clamp(loop->integral + loop->ki * error * dt,
                                        loop->voltage_limit);
    proposed_voltage = loop->kp * error + proposed_integral;
    /* Do not accumulate error that would push further into saturation. */
    if (!((proposed_voltage > loop->voltage_limit && error > 0.0f) ||
          (proposed_voltage < -loop->voltage_limit && error < 0.0f)))
        loop->integral = proposed_integral;

    /* 先限幅，再对最终 Uq 做一阶低通；速度和位置模式共用此输出。
     * alpha = dt / (tau + dt)，随实际周期换算滤波系数。
     * tau 为 2 ms 时，1 ms 周期下 alpha 约为 0.333。
     * 时间常数过大会增加控制延迟，先用小滤波观察静态抖动。
     */
    voltage = SpeedLoop_Clamp(loop->kp * error + loop->integral, loop->voltage_limit);
    alpha = dt / (SPEED_LOOP_OUTPUT_FILTER_TAU_S + dt);
    loop->filtered_voltage += alpha * (voltage - loop->filtered_voltage);
    return loop->filtered_voltage;
}
