#pragma once

/* Position P: unwrapped mechanical radians input, rad/s output. */
typedef struct
{
    float kp;
    float speed_limit;
} PositionLoop_t;

void PositionLoop_Init(PositionLoop_t *loop, float kp, float speed_limit);
float PositionLoop_Update(const PositionLoop_t *loop,
                          float target_position, float measured_position);
