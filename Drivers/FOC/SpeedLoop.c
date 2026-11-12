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
}

float SpeedLoop_Update(SpeedLoop_t *loop, float target_speed,
                       float measured_speed, float dt)
{
    float error = target_speed - measured_speed;
    float proposed_integral;
    float proposed_voltage;

    if (dt <= 0.0f || dt > 0.01f) dt = 0.001f;
    proposed_integral = SpeedLoop_Clamp(loop->integral + loop->ki * error * dt,
                                        loop->voltage_limit);
    proposed_voltage = loop->kp * error + proposed_integral;
    /* Do not accumulate error that would push further into saturation. */
    if (!((proposed_voltage > loop->voltage_limit && error > 0.0f) ||
          (proposed_voltage < -loop->voltage_limit && error < 0.0f)))
        loop->integral = proposed_integral;

    return SpeedLoop_Clamp(loop->kp * error + loop->integral, loop->voltage_limit);
}
