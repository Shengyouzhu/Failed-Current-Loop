#include "main.h"
#include <math.h>
#include "foc.h"

extern TIM_HandleTypeDef htim1;

/********************************************
	SVPWM生成
	功能：将电压信号变为驱动的SVPWM波
	输入：ua,ub,uc,vbus（驱动板电压供电）
	输出：TIM1定时器的三个CRR寄存器
*******************************************/

void SVPWM_SetPhaseVoltages(float ua,
                            float ub,
                            float uc,
                            float vbus)
{
    if (vbus <= 0.0f)
        return;

    // 1. 找出三相最大、最小电压
    float u_max = fmaxf(ua, fmaxf(ub, uc));
    float u_min = fminf(ua, fminf(ub, uc));

    // 2. 超出线性调制范围时按比例缩放
    float span = u_max - u_min;

    if (span > vbus)
    {
        float scale = vbus / span;

        ua *= scale;
        ub *= scale;
        uc *= scale;

        u_max *= scale;
        u_min *= scale;
    }

    // 3. 计算公共电压偏移量
    float u_offset = -(u_max + u_min) * 0.5f;

    // 4. 转换成三相占空比
    float duty_a = 0.5f + (ua + u_offset) / vbus;
    float duty_b = 0.5f + (ub + u_offset) / vbus;
    float duty_c = 0.5f + (uc + u_offset) / vbus;

    // 5. 获取 TIM1 ARR
    uint32_t arr =
        __HAL_TIM_GET_AUTORELOAD(&htim1);

    // 6. 更新 PWM
    __HAL_TIM_SET_COMPARE(
        &htim1, TIM_CHANNEL_1,
        (uint32_t)(duty_a * arr)
    );

    __HAL_TIM_SET_COMPARE(
        &htim1, TIM_CHANNEL_2,
        (uint32_t)(duty_b * arr)
    );

    __HAL_TIM_SET_COMPARE(
        &htim1, TIM_CHANNEL_3,
        (uint32_t)(duty_c * arr)
    );
}

