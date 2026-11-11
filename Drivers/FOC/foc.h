#ifndef  _FOC_H_
#define  _FOC_H_
#include "main.h"
#include <math.h>
#include "SVPWM.h"

typedef struct FOC_Vector
{
    float a;
    float b;
    float c;

    float alpha;
    float beta;

    float d;
    float q;

} FOC_Vector_t;


void Inv_Clark_transformer(FOC_Vector_t *cu );
void Clark_transformer(FOC_Vector_t *cu );
void Pake_transformer(FOC_Vector_t *cu,float theta_e);
void Inv_Pake_transformer(FOC_Vector_t *cu,float theta_e);

void FOC_MotorControl(float ia, float ib, float ic,
                      float theta_e, float dt);
void FOC_Init(void);
void FOC_CalibrateZero(float mechanical_angle);
void FOC_SetAlignVoltage(float ud, float theta);

#endif

