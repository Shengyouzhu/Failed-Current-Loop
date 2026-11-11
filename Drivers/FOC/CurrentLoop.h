#ifndef  _CURRENTLOOP_H_
#define  _CURRENTLOOP_H_

#include "main.h"
#include "adc.h"
#include "tim.h"
#include "foc.h"

#define ADC_VREF    3.3f
#define ADC_MAX     4095.0f
#define CSA_GAIN    0.15f

#define CURRENT_LOOP_KP             5.0f
#define CURRENT_LOOP_KI             0.1f


#define CURRENT_LOOP_VOLTAGE_LIMIT  1.5f
#define CURRENT_ADC_SAMPLES_PER_CONTROL 10U
#define CURRENT_ADC_SAMPLE_DT_S 0.00005f
#define CURRENT_PHASE_AVG_SAMPLES 5U
#define CURRENT_DQ_LPF_ALPHA 0.15f

typedef struct
{
    uint16_t adc_a;
    uint16_t adc_b;
    uint16_t adc_c;

		float offset_a;
    float offset_b;
    float offset_c;

    float ia;
    float ib;
    float ic;

    uint8_t offset_ready;
    uint32_t sample_count;


} CurrentSense_t;



extern volatile CurrentSense_t motor_current;

void CurrentSense_Start(void);

void CurrentSense_BeginCalibration(void);

void CurrentLoop_Init(void);

void CurrentLoop_Update(float ia, float ib, float ic, float theta_e, float dt, FOC_Vector_t *voltage);

void CurrentLoop_SetTarget(float id, float iq);
float CurrentLoop_GetTargetIq(void);
void CurrentLoop_SetEnabled(uint8_t enabled);

#endif

