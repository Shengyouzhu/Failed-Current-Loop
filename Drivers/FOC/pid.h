#ifndef  _PID_H_
#define  _PID_H_
#include "main.h"
#include <stdbool.h>
#include <math.h>
#include <stddef.h>


typedef struct{
    float kp;
    float ki;
    float kd;

    float out_min;
    float out_max;

    float integral_min;
    float integral_max;

    float d_filter_tau;
}PID_Config_t;

typedef struct
{
    PID_Config_t cfg;

    float integral;
    float prev_measurement;
    float filtered_d_meas;
    float last_output;

    bool has_prev_measurement;

} PID_t;

void PID_Init(PID_t *pid,const PID_Config_t *config);
void PID_Update(PID_t *pid,float target,float measurement,float dt,float *output);


#endif
