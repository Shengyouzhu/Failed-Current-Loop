#ifndef  _AS5600_H_
#define  _AS5600_H_

#include "main.h"
#include "i2c.h"

#define AS5600_ADDR       (0x36 << 1)
#define AS5600_ANGLE_REG  0x0E

uint8_t as5600_read_byte(float *angle ,float *Mechanical_perspective,float dt);
/* Bounded, lightly filtered single-turn angle for commutation. */
float AS5600_GetWrappedAngle(void);
/* 最近一次成功采样的机械角加速度（rad/s²）。 */
float AS5600_GetAcceleration(void);
float FOC_GetElectricalAngle(float mechanical_angle);

#endif
