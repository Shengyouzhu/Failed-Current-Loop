#ifndef _HAPTIC_H_
#define _HAPTIC_H_
#include "foc.h"

/* 速度单位 rad/s；dt 为实际调用周期，单位 s。 */
void Roller(const FOC_Feedback_t *feedback, float dt);
#endif
