#include "PositionLoop.h"

void PositionLoop_Init(PositionLoop_t *loop, float kp, float speed_limit)
{
    loop->kp = kp;
    loop->speed_limit = speed_limit;
}

float PositionLoop_Update(const PositionLoop_t *loop,
                          float target_position, float measured_position)
{
    float target_speed = loop->kp * (target_position - measured_position);
    if (target_speed > loop->speed_limit) return loop->speed_limit;
    if (target_speed < -loop->speed_limit) return -loop->speed_limit;
    return target_speed;
}
