#include "CurrentLoop.h"
#include "pid.h"

#define CURRENT_SCALE (ADC_VREF / (ADC_MAX * CSA_GAIN))

#define CALIB_SAMPLES 256

extern volatile float theta_e;

volatile CurrentSense_t motor_current = {0};
static uint32_t sum_a = 0;
static uint32_t sum_b = 0;
static uint32_t sum_c = 0;

static uint16_t calib_count = 0;
static uint8_t calibrating = 0;
static volatile uint8_t control_enabled = 0;
/* Vendor-style five-sample moving average for each phase. */
typedef struct { float a, b, c; } CurrentTriplet_t;
static CurrentTriplet_t current_history[CURRENT_PHASE_AVG_SAMPLES];
static uint8_t current_history_count = 0;
static uint8_t current_history_next = 0;
static uint32_t last_control_sample_count = 0;
static PID_t pid_id;
static PID_t pid_iq;
static float id_filtered = 0.0f;
static float iq_filtered = 0.0f;
static uint8_t dq_filter_ready = 0;

// 目标电流，单位A
static float target_id = 0.0f;
static volatile float target_iq = 0.0f;

// 启动电流采样
void CurrentSense_Start(void)
{
    // 启动 ADC 注入组
    HAL_ADCEx_InjectedStart_IT(&hadc1);

    // 启动 TIM1_CH4 硬件触发
    HAL_TIM_OC_Start(&htim1, TIM_CHANNEL_4);
}


// 启动零点校准
void CurrentSense_BeginCalibration(void)
{
    sum_a = 0;
    sum_b = 0;
    sum_c = 0;

    calib_count = 0;
    current_history_count = 0;
    current_history_next = 0;
    motor_current.offset_ready = 0;

    calibrating = 1;
}


// ADC 转换完成回调
void HAL_ADCEx_InjectedConvCpltCallback(
    ADC_HandleTypeDef *hadc)
{
		static uint8_t count = 0;
    if (hadc->Instance != ADC1)
        return;

    // 1. 读取 ADC 原始值
    motor_current.adc_a =
        HAL_ADCEx_InjectedGetValue(
            hadc, ADC_INJECTED_RANK_1);

    motor_current.adc_b =
        HAL_ADCEx_InjectedGetValue(
            hadc, ADC_INJECTED_RANK_2);

    motor_current.adc_c =
        HAL_ADCEx_InjectedGetValue(
            hadc, ADC_INJECTED_RANK_3);

    motor_current.sample_count++;


    // 2. 零点校准
    if (calibrating)
    {
        sum_a += motor_current.adc_a;
        sum_b += motor_current.adc_b;
        sum_c += motor_current.adc_c;

        calib_count++;

        if (calib_count >= CALIB_SAMPLES)
        {
            motor_current.offset_a =
                sum_a / (float)CALIB_SAMPLES;

            motor_current.offset_b =
                sum_b / (float)CALIB_SAMPLES;

            motor_current.offset_c =
                sum_c / (float)CALIB_SAMPLES;

            calibrating = 0;
            motor_current.offset_ready = 1;
        }

        return;
    }


    // 3. 换算相电流
    if (motor_current.offset_ready)
    {
        motor_current.ia =
            (motor_current.adc_a -
             motor_current.offset_a) * CURRENT_SCALE;

        motor_current.ib =
            (motor_current.adc_b -
             motor_current.offset_b) * CURRENT_SCALE;

        motor_current.ic =
            (motor_current.adc_c -
             motor_current.offset_c) * CURRENT_SCALE;
    }

    if (!motor_current.offset_ready || !control_enabled)
    {
        count = 0;
        current_history_count = 0;
        current_history_next = 0;
        last_control_sample_count = motor_current.sample_count;
        return;
    }

    current_history[current_history_next].a = motor_current.ia;
    current_history[current_history_next].b = motor_current.ib;
    current_history[current_history_next].c = motor_current.ic;
    current_history_next =
        (uint8_t)((current_history_next + 1U) % CURRENT_PHASE_AVG_SAMPLES);
    if (current_history_count < CURRENT_PHASE_AVG_SAMPLES)
        current_history_count++;

    if (++count < CURRENT_ADC_SAMPLES_PER_CONTROL)
        return;
    count = 0;

    float ia = 0.0f;
    float ib = 0.0f;
    float ic = 0.0f;
    for (uint8_t i = 0; i < current_history_count; ++i)
    {
        ia += current_history[i].a;
        ib += current_history[i].b;
        ic += current_history[i].c;
    }
    ia /= (float)current_history_count;
    ib /= (float)current_history_count;
    ic /= (float)current_history_count;
    uint32_t elapsed_samples = motor_current.sample_count - last_control_sample_count;
    last_control_sample_count = motor_current.sample_count;
    FOC_MotorControl(ia, ib, ic, theta_e,
                     (float)elapsed_samples * CURRENT_ADC_SAMPLE_DT_S);
}


void CurrentLoop_Init(void)
{
    PID_Config_t config = {
        .kp = CURRENT_LOOP_KP,
        .ki = CURRENT_LOOP_KI,
        .kd = 0.0f,

        .out_min = -CURRENT_LOOP_VOLTAGE_LIMIT,
        .out_max = CURRENT_LOOP_VOLTAGE_LIMIT,

        .integral_min = -CURRENT_LOOP_VOLTAGE_LIMIT,
        .integral_max = CURRENT_LOOP_VOLTAGE_LIMIT,

        .d_filter_tau = 0.0f
    };

    PID_Init(&pid_id, &config);
    PID_Init(&pid_iq, &config);
}


void CurrentLoop_SetEnabled(uint8_t enabled)
{
    if (!enabled || !control_enabled)
        dq_filter_ready = 0;
    control_enabled = enabled ? 1U : 0U;
}

void CurrentLoop_SetTarget(float id, float iq)
{
    target_id = id;
    target_iq = iq;
}

float CurrentLoop_GetTargetIq(void)
{
    return target_iq;
}
////////
volatile float debug_i = 0.0f;
volatile float debug_id = 0.0f;
volatile float debug_i_raw = 0.0f;
/////////
void CurrentLoop_Update(float ia,
                        float ib,
                        float ic,
                        float theta_e,
                        float dt,
                        FOC_Vector_t *voltage)
{
    FOC_Vector_t current = {0};

    // 三相电流
    current.a = ia;
    current.b = ib;
    current.c = ic;

    // 坐标变换
    Clark_transformer(&current);
    Pake_transformer(&current, theta_e);
    debug_i_raw = current.q;
    if (!dq_filter_ready)
    {
        id_filtered = current.d;
        iq_filtered = current.q;
        dq_filter_ready = 1;
    }
    else
    {
        id_filtered += CURRENT_DQ_LPF_ALPHA * (current.d - id_filtered);
        iq_filtered += CURRENT_DQ_LPF_ALPHA * (current.q - iq_filtered);
    }
    debug_i = iq_filtered;
    debug_id = id_filtered;
    // 电流 PI
     PID_Update(
        &pid_id, target_id, id_filtered, dt,&voltage->d
    );

     PID_Update(
        &pid_iq, target_iq, iq_filtered, dt,&voltage->q
    );
}











