#pragma once

/* Speed PI: rad/s feedback, volts output. */
typedef struct
{
    float kp;
    float ki;
    float voltage_limit;
    float integral;
} SpeedLoop_t;

void SpeedLoop_Init(SpeedLoop_t *loop, float kp, float ki, float voltage_limit);
void SpeedLoop_Reset(SpeedLoop_t *loop);
float SpeedLoop_Update(SpeedLoop_t *loop, float target_speed,
                       float measured_speed, float dt);
