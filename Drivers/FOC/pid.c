#include "pid.h"


/********************************************
	PID限幅
	功能：设定输入值的最大和最小值
	输入：当前值，最小值，最大值
*******************************************/

static float pid_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) return minimum;
    if (value > maximum) return maximum;
    return value;
}

/********************************************
	PID初始化
	功能：将PID参数配置为PID_Config_t参数
	输入：PID_T结构体，PID_Config_t结构体
*******************************************/
void PID_Init(PID_t *pid,const PID_Config_t *config)
{
	pid->cfg = *config;
	pid->integral = 0.0f;
  pid->prev_measurement = 0.0f;
  pid->filtered_d_meas = 0.0f;
  pid->last_output = 0.0f;
  pid->has_prev_measurement = false;
}

/********************************************
	PID控制器
	功能：通过比例积分微分算法输出控制
	输入：PID_T结构体，误差R
	输出：调节结果
*******************************************/

void PID_Update(PID_t *pid,float target,float measurement,float dt,float *output)
{
	const PID_Config_t *cfg = &pid->cfg;
	const float error = target - measurement;

	float p = cfg->kp * error;
	
	float derivative = 0.0f;

	if(pid->has_prev_measurement)
		derivative = -(measurement - pid->prev_measurement) / dt;
//滤波器
	float alpha = dt / (cfg->d_filter_tau + dt);
	pid->filtered_d_meas += alpha * (derivative - pid->filtered_d_meas);

	float d = cfg->kd * pid->filtered_d_meas;

	float i = pid_clamp(pid->integral + cfg->ki *error *dt,cfg->integral_min,cfg->integral_max);

	float proposed = p + i + d;

	if (!((proposed > cfg->out_max && error > 0.0f) ||(proposed < cfg->out_min && error < 0.0f)))
	{
		pid->integral = i;
	}
	
	 pid->last_output = pid_clamp(
        p + pid->integral + d,
        cfg->out_min,
        cfg->out_max
    );

		pid->prev_measurement = measurement;
    pid->has_prev_measurement = true;

		*output = pid->last_output;
}






