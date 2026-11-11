#ifndef  _AS5600_H_
#define  _AS5600_H_

#include "main.h"
#include "i2c.h"

#define AS5600_ADDR       (0x36 << 1)
#define AS5600_ANGLE_REG  0x0E

uint8_t as5600_read_byte(float *angle ,float *Mechanical_perspective,float dt);
float FOC_GetElectricalAngle(float mechanical_angle);

#endif
